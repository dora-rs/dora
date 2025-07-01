use std::{
    collections::{BTreeMap, HashMap, VecDeque},
    pin::pin,
    sync::Arc,
    time::Duration,
};

use dora_message::{
    daemon_to_node::{DaemonCommunication, DaemonReply, DataMessage, NodeEvent},
    id::DataId,
    node_to_daemon::{DaemonRequest, Timestamped},
    DataflowId,
};
pub use event::{Event, StopCause};
use futures::{
    future::{select, Either},
    Stream, StreamExt,
};
use futures_timer::Delay;
use scheduler::{Scheduler, NON_INPUT_EVENT};

use self::thread::{EventItem, EventStreamThreadHandle};
use crate::{
    daemon_connection::DaemonChannel,
    event_stream::data_conversion::{MappedInputData, RawData, SharedMemoryData},
};
use dora_core::{
    config::{Input, NodeId},
    uhlc,
};
use eyre::{eyre, Context};

mod data_conversion;
mod event;
pub mod merged;
mod scheduler;
mod thread;

pub struct EventStream {
    node_id: NodeId,
    receiver: flume::r#async::RecvStream<'static, EventItem>,
    _thread_handle: EventStreamThreadHandle,
    close_channel: DaemonChannel,
    clock: Arc<uhlc::HLC>,
    scheduler: Scheduler,
}

impl EventStream {
    #[tracing::instrument(level = "trace", skip(clock))]
    pub(crate) fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        daemon_communication: &DaemonCommunication,
        input_config: BTreeMap<DataId, Input>,
        clock: Arc<uhlc::HLC>,
    ) -> eyre::Result<Self> {
        let channel = match daemon_communication {
            DaemonCommunication::Shmem {
                daemon_events_region_id,
                ..
            } => unsafe { DaemonChannel::new_shmem(daemon_events_region_id) }.wrap_err_with(
                || format!("failed to create shmem event stream for node `{node_id}`"),
            )?,
            DaemonCommunication::Tcp { socket_addr } => DaemonChannel::new_tcp(*socket_addr)
                .wrap_err_with(|| format!("failed to connect event stream for node `{node_id}`"))?,
            #[cfg(unix)]
            DaemonCommunication::UnixDomain { socket_file } => {
                DaemonChannel::new_unix_socket(socket_file).wrap_err_with(|| {
                    format!("failed to connect event stream for node `{node_id}`")
                })?
            }
        };

        let close_channel = match daemon_communication {
            DaemonCommunication::Shmem {
                daemon_events_close_region_id,
                ..
            } => unsafe { DaemonChannel::new_shmem(daemon_events_close_region_id) }.wrap_err_with(
                || format!("failed to create shmem event close channel for node `{node_id}`"),
            )?,
            DaemonCommunication::Tcp { socket_addr } => DaemonChannel::new_tcp(*socket_addr)
                .wrap_err_with(|| {
                    format!("failed to connect event close channel for node `{node_id}`")
                })?,
            #[cfg(unix)]
            DaemonCommunication::UnixDomain { socket_file } => {
                DaemonChannel::new_unix_socket(socket_file).wrap_err_with(|| {
                    format!("failed to connect event close channel for node `{node_id}`")
                })?
            }
        };

        let mut queue_size_limit: HashMap<DataId, (usize, VecDeque<EventItem>)> = input_config
            .iter()
            .map(|(input, config)| {
                (
                    input.clone(),
                    (config.queue_size.unwrap_or(1), VecDeque::new()),
                )
            })
            .collect();

        queue_size_limit.insert(
            DataId::from(NON_INPUT_EVENT.to_string()),
            (1_000, VecDeque::new()),
        );

        let scheduler = Scheduler::new(queue_size_limit);

        Self::init_on_channel(
            dataflow_id,
            node_id,
            channel,
            close_channel,
            clock,
            scheduler,
        )
    }

    pub(crate) fn init_on_channel(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        mut channel: DaemonChannel,
        mut close_channel: DaemonChannel,
        clock: Arc<uhlc::HLC>,
        scheduler: Scheduler,
    ) -> eyre::Result<Self> {
        channel.register(dataflow_id, node_id.clone(), clock.new_timestamp())?;
        let reply = channel
            .request(&Timestamped {
                inner: DaemonRequest::Subscribe,
                timestamp: clock.new_timestamp(),
            })
            .map_err(|e| eyre!(e))
            .wrap_err("failed to create subscription with dora-daemon")?;

        match reply {
            DaemonReply::Result(Ok(())) => {}
            DaemonReply::Result(Err(err)) => {
                eyre::bail!("subscribe failed: {err}")
            }
            other => eyre::bail!("unexpected subscribe reply: {other:?}"),
        }

        close_channel.register(dataflow_id, node_id.clone(), clock.new_timestamp())?;

        let (tx, rx) = flume::bounded(100_000_000);

        let thread_handle = thread::init(node_id.clone(), tx, channel, clock.clone())?;

        Ok(EventStream {
            node_id: node_id.clone(),
            receiver: rx.into_stream(),
            _thread_handle: thread_handle,
            close_channel,
            clock,
            scheduler,
        })
    }

    /// wait for the next event on the events stream.
    pub fn recv(&mut self) -> Option<Event> {
        futures::executor::block_on(self.recv_async())
    }

    /// wait for the next event on the events stream until timeout
    pub fn recv_timeout(&mut self, dur: Duration) -> Option<Event> {
        futures::executor::block_on(self.recv_async_timeout(dur))
    }

    pub async fn recv_async(&mut self) -> Option<Event> {
        loop {
            if self.scheduler.is_empty() {
                if let Some(event) = self.receiver.next().await {
                    self.scheduler.add_event(event);
                } else {
                    break;
                }
            } else {
                match select(Delay::new(Duration::from_micros(300)), self.receiver.next()).await {
                    Either::Left((_elapsed, _)) => break,
                    Either::Right((Some(event), _)) => self.scheduler.add_event(event),
                    Either::Right((None, _)) => break,
                };
            }
        }
        let event = self.scheduler.next();
        event.map(Self::convert_event_item)
    }

    pub async fn recv_async_timeout(&mut self, dur: Duration) -> Option<Event> {
        match select(Delay::new(dur), pin!(self.recv_async())).await {
            Either::Left((_elapsed, _)) => Some(Self::convert_event_item(EventItem::TimeoutError(
                eyre!("Receiver timed out"),
            ))),
            Either::Right((event, _)) => event,
        }
    }

    fn convert_event_item(item: EventItem) -> Event {
        match item {
            EventItem::NodeEvent { event, ack_channel } => match event {
                NodeEvent::Stop => Event::Stop(event::StopCause::Manual),
                NodeEvent::Reload { operator_id } => Event::Reload { operator_id },
                NodeEvent::InputClosed { id } => Event::InputClosed { id },
                NodeEvent::Input { id, metadata, data } => {
                    let data = match data {
                        None => Ok(None),
                        Some(DataMessage::Vec(v)) => Ok(Some(RawData::Vec(v))),
                        Some(DataMessage::SharedMemory {
                            shared_memory_id,
                            len,
                            drop_token: _, // handled in `event_stream_loop`
                        }) => unsafe {
                            MappedInputData::map(&shared_memory_id, len).map(|data| {
                                Some(RawData::SharedMemory(SharedMemoryData {
                                    data,
                                    _drop: ack_channel,
                                }))
                            })
                        },
                    };
                    let data = data.and_then(|data| {
                        let raw_data = data.unwrap_or(RawData::Empty);
                        raw_data
                            .into_arrow_array(&metadata.type_info)
                            .map(arrow::array::make_array)
                    });
                    match data {
                        Ok(data) => Event::Input {
                            id,
                            metadata,
                            data: data.into(),
                        },
                        Err(err) => Event::Error(format!("{err:?}")),
                    }
                }
                NodeEvent::AllInputsClosed => Event::Stop(event::StopCause::AllInputsClosed),
            },

            EventItem::FatalError(err) => {
                Event::Error(format!("fatal event stream error: {err:?}"))
            }
            EventItem::TimeoutError(err) => {
                Event::Error(format!("Timeout event stream error: {err:?}"))
            }
        }
    }
}

impl Stream for EventStream {
    type Item = Event;

    fn poll_next(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Option<Self::Item>> {
        self.receiver
            .poll_next_unpin(cx)
            .map(|item| item.map(Self::convert_event_item))
    }
}

impl Drop for EventStream {
    #[tracing::instrument(skip(self), fields(%self.node_id))]
    fn drop(&mut self) {
        let request = Timestamped {
            inner: DaemonRequest::EventStreamDropped,
            timestamp: self.clock.new_timestamp(),
        };
        let result = self
            .close_channel
            .request(&request)
            .map_err(|e| eyre!(e))
            .wrap_err("failed to signal event stream closure to dora-daemon")
            .and_then(|r| match r {
                DaemonReply::Result(Ok(())) => Ok(()),
                DaemonReply::Result(Err(err)) => Err(eyre!("EventStreamClosed failed: {err}")),
                other => Err(eyre!("unexpected EventStreamClosed reply: {other:?}")),
            });
        if let Err(err) = result {
            tracing::warn!("{err:?}")
        }
    }
}
