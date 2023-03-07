use dora_core::{
    config::{DataId, NodeId},
    daemon_messages::{DaemonCommunication, DaemonReply, DaemonRequest, DataflowId, NodeEvent},
    message::Metadata,
};
use eyre::{bail, eyre, Context};
use flume::RecvTimeoutError;
use shared_memory_server::{Shmem, ShmemClient, ShmemConf};
use std::{marker::PhantomData, net::TcpStream, sync::Arc, time::Duration};

mod tcp;

pub(crate) struct DaemonConnection {
    pub control_channel: ControlChannel,
    pub event_stream: EventStream,
}

impl DaemonConnection {
    pub(crate) fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        daemon_communication: &DaemonCommunication,
    ) -> eyre::Result<Self> {
        let (control, events) = match daemon_communication {
            DaemonCommunication::Shmem {
                daemon_control_region_id,
                daemon_events_region_id,
            } => {
                let control = unsafe { DaemonChannel::new_shmem(daemon_control_region_id) }
                    .wrap_err("failed to create shmem control channel")?;
                let events = unsafe { DaemonChannel::new_shmem(daemon_events_region_id) }
                    .wrap_err("failed to create shmem event channel")?;
                (control, events)
            }
            DaemonCommunication::Tcp { socket_addr } => {
                let control = DaemonChannel::new_tcp(
                    TcpStream::connect(socket_addr).wrap_err("failed to connect control stream")?,
                )?;
                let events = DaemonChannel::new_tcp(
                    TcpStream::connect(socket_addr).wrap_err("failed to connect event stream")?,
                )?;
                (control, events)
            }
        };

        let mut control_channel = ControlChannel::init(dataflow_id, node_id, control)
            .wrap_err("failed to init control stream")?;

        let (event_stream, event_stream_thread_handle) =
            EventStream::init(dataflow_id, node_id, events)
                .wrap_err("failed to init event stream")?;

        control_channel.event_stream_thread_handle = Some(event_stream_thread_handle);

        Ok(Self {
            control_channel,
            event_stream,
        })
    }
}

pub(crate) struct ControlChannel {
    channel: DaemonChannel,
    event_stream_thread_handle: Option<Arc<EventStreamThreadHandle>>,
}

impl ControlChannel {
    #[tracing::instrument(skip(channel))]
    fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        mut channel: DaemonChannel,
    ) -> eyre::Result<Self> {
        register(dataflow_id, node_id.clone(), &mut channel)?;

        Ok(Self {
            channel,
            event_stream_thread_handle: None,
        })
    }

    pub fn report_stop(&mut self) -> eyre::Result<()> {
        let reply = self
            .channel
            .request(&DaemonRequest::Stopped)
            .wrap_err("failed to report stopped to dora-daemon")?;
        match reply {
            dora_core::daemon_messages::DaemonReply::Result(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to report stop event to dora-daemon")?,
            other => bail!("unexpected stopped reply: {other:?}"),
        }
        Ok(())
    }

    pub fn report_closed_outputs(&mut self, outputs: Vec<DataId>) -> eyre::Result<()> {
        let reply = self
            .channel
            .request(&DaemonRequest::CloseOutputs(outputs))
            .wrap_err("failed to report closed outputs to dora-daemon")?;
        match reply {
            dora_core::daemon_messages::DaemonReply::Result(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to receive closed outputs reply from dora-daemon")?,
            other => bail!("unexpected closed outputs reply: {other:?}"),
        }
        Ok(())
    }

    pub fn prepare_message(
        &mut self,
        output_id: DataId,
        metadata: Metadata<'static>,
        data_len: usize,
    ) -> eyre::Result<MessageSample> {
        let reply = self
            .channel
            .request(&DaemonRequest::PrepareOutputMessage {
                output_id,
                metadata,
                data_len,
            })
            .wrap_err("failed to send PrepareOutputMessage request to dora-daemon")?;
        match reply {
            dora_core::daemon_messages::DaemonReply::PreparedMessage {
                shared_memory_id: id,
            } => Ok(MessageSample { id }),
            dora_core::daemon_messages::DaemonReply::Result(Err(err)) => {
                Err(eyre!(err).wrap_err("failed to report stop event to dora-daemon"))
            }
            other => bail!("unexpected PrepareOutputMessage reply: {other:?}"),
        }
    }

    pub fn send_prepared_message(&mut self, sample: MessageSample) -> eyre::Result<()> {
        let reply = self
            .channel
            .request(&DaemonRequest::SendPreparedMessage { id: sample.id })
            .wrap_err("failed to send SendOutMessage request to dora-daemon")?;
        match reply {
            dora_core::daemon_messages::DaemonReply::Result(result) => {
                result.map_err(|err| eyre!(err))
            }
            other => bail!("unexpected SendOutMessage reply: {other:?}"),
        }
    }

    pub fn send_message(
        &mut self,
        output_id: DataId,
        metadata: Metadata<'static>,
        data: Vec<u8>,
    ) -> eyre::Result<()> {
        let request = DaemonRequest::SendMessage {
            output_id,
            metadata,
            data,
        };
        let reply = self
            .channel
            .request(&request)
            .wrap_err("failed to send SendMessage request to dora-daemon")?;
        match reply {
            dora_core::daemon_messages::DaemonReply::Empty => Ok(()),
            other => bail!("unexpected SendMessage reply: {other:?}"),
        }
    }
}

enum DaemonChannel {
    Shmem(ShmemClient<DaemonRequest, DaemonReply>),
    Tcp(TcpStream),
}

impl DaemonChannel {
    #[tracing::instrument]
    fn new_tcp(stream: TcpStream) -> eyre::Result<Self> {
        stream.set_nodelay(true).context("failed to set nodelay")?;
        Ok(DaemonChannel::Tcp(stream))
    }

    #[tracing::instrument]
    unsafe fn new_shmem(daemon_control_region_id: &str) -> eyre::Result<Self> {
        let daemon_events_region = ShmemConf::new()
            .os_id(daemon_control_region_id)
            .open()
            .wrap_err("failed to connect to dora-daemon")?;
        let channel = DaemonChannel::Shmem(
            unsafe { ShmemClient::new(daemon_events_region, Some(Duration::from_secs(5))) }
                .wrap_err("failed to create ShmemChannel")?,
        );
        Ok(channel)
    }

    fn request(&mut self, request: &DaemonRequest) -> eyre::Result<DaemonReply> {
        match self {
            DaemonChannel::Shmem(client) => client.request(request),
            DaemonChannel::Tcp(stream) => tcp::request(stream, request),
        }
    }
}

fn register(
    dataflow_id: DataflowId,
    node_id: NodeId,
    channel: &mut DaemonChannel,
) -> eyre::Result<()> {
    let msg = DaemonRequest::Register {
        dataflow_id,
        node_id,
    };
    let reply = channel
        .request(&msg)
        .wrap_err("failed to send register request to dora-daemon")?;

    match reply {
        dora_core::daemon_messages::DaemonReply::Result(result) => result
            .map_err(|e| eyre!(e))
            .wrap_err("failed to register node with dora-daemon")?,
        other => bail!("unexpected register reply: {other:?}"),
    }
    Ok(())
}

enum EventItem {
    NodeEvent {
        event: NodeEvent,
        ack_channel: std::sync::mpsc::Sender<()>,
    },
    FatalError(eyre::Report),
}

pub struct EventStream {
    receiver: flume::Receiver<EventItem>,
    _thread_handle: Arc<EventStreamThreadHandle>,
}

impl EventStream {
    fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        mut channel: DaemonChannel,
    ) -> eyre::Result<(Self, Arc<EventStreamThreadHandle>)> {
        register(dataflow_id, node_id.clone(), &mut channel)?;

        channel
            .request(&DaemonRequest::Subscribe)
            .map_err(|e| eyre!(e))
            .wrap_err("failed to create subscription with dora-daemon")?;

        let (tx, rx) = flume::bounded(0);
        let mut drop_tokens = Vec::new();
        let node_id = node_id.clone();
        let join_handle = std::thread::spawn(move || {
            let result = 'outer: loop {
                let daemon_request = DaemonRequest::NextEvent {
                    drop_tokens: std::mem::take(&mut drop_tokens),
                };
                let events = match channel.request(&daemon_request) {
                    Ok(DaemonReply::NextEvents(events)) if events.is_empty() => {
                        tracing::debug!("Event stream closed for node ID `{node_id}`");
                        break Ok(());
                    }
                    Ok(DaemonReply::NextEvents(events)) => events,
                    Ok(other) => {
                        let err = eyre!("unexpected control reply: {other:?}");
                        tracing::warn!("{err:?}");
                        continue;
                    }
                    Err(err) => {
                        let err = eyre!(err).wrap_err("failed to receive incoming event");
                        tracing::warn!("{err:?}");
                        continue;
                    }
                };
                for event in events {
                    let drop_token = match &event {
                        NodeEvent::Input {
                            data: Some(data), ..
                        } => data.drop_token(),
                        NodeEvent::Stop
                        | NodeEvent::InputClosed { .. }
                        | NodeEvent::Input { data: None, .. } => None,
                    };

                    let (drop_tx, drop_rx) = std::sync::mpsc::channel();
                    match tx.send(EventItem::NodeEvent {
                        event,
                        ack_channel: drop_tx,
                    }) {
                        Ok(()) => {}
                        Err(_) => {
                            // receiving end of channel was closed
                            break 'outer Ok(());
                        }
                    }

                    let timeout = Duration::from_secs(30);
                    match drop_rx.recv_timeout(timeout) {
                        Ok(()) => {
                            break 'outer Err(eyre!(
                                "Node API should not send anything on ACK channel"
                            ))
                        }
                        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                            tracing::warn!("timeout: event was not dropped after {timeout:?}");
                        }
                        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {} // expected result
                    }

                    if let Some(token) = drop_token {
                        drop_tokens.push(token);
                    }
                }
            };
            if let Err(err) = result {
                if let Err(flume::SendError(item)) = tx.send(EventItem::FatalError(err)) {
                    let err = match item {
                        EventItem::FatalError(err) => err,
                        _ => unreachable!(),
                    };
                    tracing::error!("failed to report fatal EventStream error: {err:?}");
                }
            }
        });

        let thread_handle = EventStreamThreadHandle::new(join_handle);

        Ok((
            EventStream {
                receiver: rx,
                _thread_handle: thread_handle.clone(),
            },
            thread_handle,
        ))
    }

    pub fn recv(&mut self) -> Option<Event> {
        let event = self.receiver.recv();
        self.recv_common(event)
    }

    pub async fn recv_async(&mut self) -> Option<Event> {
        let event = self.receiver.recv_async().await;
        self.recv_common(event)
    }

    fn recv_common(&mut self, event: Result<EventItem, flume::RecvError>) -> Option<Event> {
        let event = match event {
            Ok(event) => event,
            Err(flume::RecvError::Disconnected) => {
                tracing::info!("event channel disconnected");
                return None;
            }
        };
        let event = match event {
            EventItem::NodeEvent { event, ack_channel } => match event {
                NodeEvent::Stop => Event::Stop,
                NodeEvent::InputClosed { id } => Event::InputClosed { id },
                NodeEvent::Input { id, metadata, data } => {
                    let data = data
                        .map(|data| match data {
                            dora_core::daemon_messages::InputData::Vec(d) => Ok(Data::Vec(d)),
                            dora_core::daemon_messages::InputData::SharedMemory(d) => unsafe {
                                MappedInputData::map(&d.shared_memory_id, d.len).map(|data| {
                                    Data::SharedMemory {
                                        data,
                                        _drop: ack_channel,
                                    }
                                })
                            },
                        })
                        .transpose();
                    match data {
                        Ok(data) => Event::Input { id, metadata, data },
                        Err(err) => Event::Error(format!("{err:?}")),
                    }
                }
            },
            EventItem::FatalError(err) => {
                Event::Error(format!("fatal event stream error: {err:?}"))
            }
        };

        Some(event)
    }
}

pub struct MessageSample {
    pub id: String,
}

#[derive(Debug)]
#[non_exhaustive]
pub enum Event<'a> {
    Stop,
    Input {
        id: DataId,
        metadata: Metadata<'static>,
        data: Option<Data<'a>>,
    },
    InputClosed {
        id: DataId,
    },
    Error(String),
}

pub enum Data<'a> {
    Vec(Vec<u8>),
    SharedMemory {
        data: MappedInputData<'a>,
        _drop: std::sync::mpsc::Sender<()>,
    },
}

impl std::ops::Deref for Data<'_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        match self {
            Data::SharedMemory { data, .. } => data,
            Data::Vec(data) => data,
        }
    }
}

impl std::fmt::Debug for Data<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Data").finish_non_exhaustive()
    }
}

pub struct MappedInputData<'a> {
    memory: Shmem,
    len: usize,
    _data: PhantomData<&'a [u8]>,
}

impl MappedInputData<'_> {
    unsafe fn map(shared_memory_id: &str, len: usize) -> eyre::Result<Self> {
        let memory = ShmemConf::new()
            .os_id(shared_memory_id)
            .open()
            .wrap_err("failed to map shared memory input")?;
        Ok(MappedInputData {
            memory,
            len,
            _data: PhantomData,
        })
    }
}

impl std::ops::Deref for MappedInputData<'_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        unsafe { &self.memory.as_slice()[..self.len] }
    }
}

struct EventStreamThreadHandle(flume::Receiver<std::thread::Result<()>>);
impl EventStreamThreadHandle {
    fn new(join_handle: std::thread::JoinHandle<()>) -> Arc<Self> {
        let (tx, rx) = flume::bounded(1);
        std::thread::spawn(move || {
            let _ = tx.send(join_handle.join());
        });
        Arc::new(Self(rx))
    }
}

impl Drop for EventStreamThreadHandle {
    fn drop(&mut self) {
        match self.0.recv_timeout(Duration::from_secs(2)) {
            Ok(Ok(())) => {}
            Ok(Err(_)) => {
                tracing::error!("event stream thread panicked");
            }
            Err(RecvTimeoutError::Timeout) => {
                tracing::warn!("timeout while waiting for event stream thread");
            }
            Err(RecvTimeoutError::Disconnected) => {
                tracing::warn!("event stream thread result channel closed unexpectedly");
            }
        }
    }
}
