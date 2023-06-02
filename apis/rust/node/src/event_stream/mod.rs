use std::sync::Arc;

pub use event::{Data, Event, MappedInputData};

use self::thread::{EventItem, EventStreamThreadHandle};
use crate::daemon_connection::DaemonChannel;
use dora_core::{
    config::NodeId,
    daemon_messages::{self, DaemonCommunication, DaemonRequest, DataflowId, NodeEvent},
    message::uhlc,
};
use eyre::{eyre, Context};

mod event;
mod thread;

pub struct EventStream {
    node_id: NodeId,
    receiver: flume::Receiver<EventItem>,
    _thread_handle: EventStreamThreadHandle,
    close_channel: DaemonChannel,
}

impl EventStream {
    #[tracing::instrument(level = "trace", skip(hlc))]
    pub(crate) fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        daemon_communication: &DaemonCommunication,
        hlc: Arc<uhlc::HLC>,
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
        };

        Self::init_on_channel(dataflow_id, node_id, channel, close_channel, hlc)
    }

    pub(crate) fn init_on_channel(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        mut channel: DaemonChannel,
        mut close_channel: DaemonChannel,
        hlc: Arc<uhlc::HLC>,
    ) -> eyre::Result<Self> {
        channel.register(dataflow_id, node_id.clone())?;
        let reply = channel
            .request(&DaemonRequest::Subscribe)
            .map_err(|e| eyre!(e))
            .wrap_err("failed to create subscription with dora-daemon")?;

        match reply {
            daemon_messages::DaemonReply::Result(Ok(())) => {}
            daemon_messages::DaemonReply::Result(Err(err)) => {
                eyre::bail!("subscribe failed: {err}")
            }
            other => eyre::bail!("unexpected subscribe reply: {other:?}"),
        }

        close_channel.register(dataflow_id, node_id.clone())?;

        let (tx, rx) = flume::bounded(0);
        let thread_handle = thread::init(node_id.clone(), tx, channel, hlc)?;

        Ok(EventStream {
            node_id: node_id.clone(),
            receiver: rx,
            _thread_handle: thread_handle,
            close_channel,
        })
    }

    /// wait for the next event on the events stream.
    pub fn recv(&mut self) -> Option<Event> {
        let event = self.receiver.recv();
        self.recv_common(event)
    }

    pub async fn recv_async(&mut self) -> Option<Event> {
        let event = self.receiver.recv_async().await;
        self.recv_common(event)
    }

    #[tracing::instrument(skip(self), fields(%self.node_id))]
    fn recv_common(&mut self, event: Result<EventItem, flume::RecvError>) -> Option<Event> {
        let event = match event {
            Ok(event) => event,
            Err(flume::RecvError::Disconnected) => {
                tracing::trace!("event channel disconnected");
                return None;
            }
        };
        let event = match event {
            EventItem::NodeEvent { event, ack_channel } => match event {
                NodeEvent::Stop => Event::Stop,
                NodeEvent::Reload { operator_id } => Event::Reload { operator_id },
                NodeEvent::InputClosed { id } => Event::InputClosed { id },
                NodeEvent::Input { id, metadata, data } => {
                    let data = match data {
                        None => Ok(None),
                        Some(daemon_messages::Data::Vec(v)) => Ok(Some(Data::Vec(v))),
                        Some(daemon_messages::Data::SharedMemory {
                            shared_memory_id,
                            len,
                            drop_token: _, // handled in `event_stream_loop`
                        }) => unsafe {
                            MappedInputData::map(&shared_memory_id, len).map(|data| {
                                Some(Data::SharedMemory {
                                    data,
                                    _drop: ack_channel,
                                })
                            })
                        },
                    };
                    match data {
                        Ok(data) => Event::Input { id, metadata, data },
                        Err(err) => Event::Error(format!("{err:?}")),
                    }
                }
                NodeEvent::AllInputsClosed => {
                    let err = eyre!(
                        "received `AllInputsClosed` event, which should be handled by background task"
                    );
                    tracing::error!("{err:?}");
                    Event::Error(err.wrap_err("internal error").to_string())
                }
            },

            EventItem::FatalError(err) => {
                Event::Error(format!("fatal event stream error: {err:?}"))
            }
        };

        Some(event)
    }
}

impl Drop for EventStream {
    #[tracing::instrument(skip(self), fields(%self.node_id))]
    fn drop(&mut self) {
        let result = self
            .close_channel
            .request(&DaemonRequest::EventStreamDropped)
            .map_err(|e| eyre!(e))
            .wrap_err("failed to signal event stream closure to dora-daemon")
            .and_then(|r| match r {
                daemon_messages::DaemonReply::Result(Ok(())) => Ok(()),
                daemon_messages::DaemonReply::Result(Err(err)) => {
                    Err(eyre!("EventStreamClosed failed: {err}"))
                }
                other => Err(eyre!("unexpected EventStreamClosed reply: {other:?}")),
            });
        if let Err(err) = result {
            tracing::warn!("{err:?}")
        }
    }
}
