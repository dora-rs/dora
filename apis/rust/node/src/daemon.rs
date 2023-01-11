use dora_core::{
    config::{DataId, NodeId},
    daemon_messages::{ControlRequest, DataflowId, NodeEvent},
    shared_memory::ShmemClient,
};
use dora_message::Metadata;
use eyre::{bail, eyre, Context};
use shared_memory::{Shmem, ShmemConf};
use std::{marker::PhantomData, thread::JoinHandle, time::Duration};

pub struct DaemonConnection {
    pub control_channel: ControlChannel,
    pub event_stream: EventStream,
    pub(crate) event_stream_thread: JoinHandle<()>,
}

impl DaemonConnection {
    pub(crate) fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        daemon_control_region_id: &str,
        daemon_events_region_id: &str,
    ) -> eyre::Result<Self> {
        let control_channel = ControlChannel::init(dataflow_id, node_id, daemon_control_region_id)
            .wrap_err("failed to init control stream")?;

        let (event_stream, event_stream_thread) =
            EventStream::init(dataflow_id, node_id, daemon_events_region_id)
                .wrap_err("failed to init event stream")?;

        Ok(Self {
            control_channel,
            event_stream,
            event_stream_thread,
        })
    }
}

pub struct ControlChannel {
    channel: ShmemClient,
}

impl ControlChannel {
    #[tracing::instrument]
    fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        daemon_control_region_id: &str,
    ) -> eyre::Result<Self> {
        let daemon_events_region = ShmemConf::new()
            .os_id(daemon_control_region_id)
            .open()
            .wrap_err("failed to connect to dora-daemon")?;
        let mut channel =
            unsafe { ShmemClient::new(daemon_events_region, Some(Duration::from_secs(5))) }
                .wrap_err("failed to create ShmemChannel")?;

        let msg = ControlRequest::Register {
            dataflow_id,
            node_id: node_id.clone(),
        };
        let reply = channel
            .request(&msg)
            .wrap_err("failed to send register request to dora-daemon")?;

        match reply {
            dora_core::daemon_messages::ControlReply::Result(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to register node with dora-daemon")?,
            other => bail!("unexpected register reply: {other:?}"),
        }

        Ok(Self { channel })
    }

    pub fn report_stop(&mut self) -> eyre::Result<()> {
        let reply = self
            .channel
            .request(&ControlRequest::Stopped)
            .wrap_err("failed to report stopped to dora-daemon")?;
        match reply {
            dora_core::daemon_messages::ControlReply::Result(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to report stop event to dora-daemon")?,
            other => bail!("unexpected stopped reply: {other:?}"),
        }
        Ok(())
    }

    pub fn prepare_message(
        &mut self,
        output_id: DataId,
        metadata: dora_message::Metadata<'static>,
        data_len: usize,
    ) -> eyre::Result<MessageSample> {
        let reply = self
            .channel
            .request(&ControlRequest::PrepareOutputMessage {
                output_id,
                metadata,
                data_len,
            })
            .wrap_err("failed to send PrepareOutputMessage request to dora-daemon")?;
        match reply {
            dora_core::daemon_messages::ControlReply::PreparedMessage {
                shared_memory_id: id,
            } => Ok(MessageSample { id }),
            dora_core::daemon_messages::ControlReply::Result(Err(err)) => {
                Err(eyre!(err).wrap_err("failed to report stop event to dora-daemon"))
            }
            other => bail!("unexpected PrepareOutputMessage reply: {other:?}"),
        }
    }

    pub fn send_message(&mut self, sample: MessageSample) -> eyre::Result<()> {
        let reply = self
            .channel
            .request(&ControlRequest::SendOutMessage { id: sample.id })
            .wrap_err("failed to send SendOutMessage request to dora-daemon")?;
        match reply {
            dora_core::daemon_messages::ControlReply::Result(result) => {
                result.map_err(|err| eyre!(err))
            }
            other => bail!("unexpected SendOutMessage reply: {other:?}"),
        }
    }
}

pub struct EventStream {
    receiver: flume::Receiver<(NodeEvent, std::sync::mpsc::Sender<()>)>,
}

impl EventStream {
    fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        daemon_events_region_id: &str,
    ) -> eyre::Result<(Self, JoinHandle<()>)> {
        let daemon_events_region = ShmemConf::new()
            .os_id(daemon_events_region_id)
            .open()
            .wrap_err("failed to connect to dora-daemon")?;
        let mut channel =
            unsafe { ShmemClient::new(daemon_events_region, None) }
                .wrap_err("failed to create ShmemChannel")?;

        channel
            .request(&ControlRequest::Subscribe {
                dataflow_id,
                node_id: node_id.clone(),
            })
            .map_err(|e| eyre!(e))
            .wrap_err("failed to create subscription with dora-daemon")?;

        let (tx, rx) = flume::bounded(1);
        let mut drop_tokens = Vec::new();
        let thread = std::thread::spawn(move || loop {
            let event: NodeEvent = match channel.request(&ControlRequest::NextEvent {
                drop_tokens: std::mem::take(&mut drop_tokens),
            }) {
                Ok(event) => event,
                Err(err) => {
                    let err = eyre!(err).wrap_err("failed to receive incoming event");
                    tracing::warn!("{err:?}");
                    continue;
                }
            };
            let drop_token = match &event {
                NodeEvent::Input {
                    data: Some(data), ..
                } => Some(data.drop_token.clone()),
                NodeEvent::Stop
                | NodeEvent::InputClosed { .. }
                | NodeEvent::Input { data: None, .. } => None,
            };

            let (drop_tx, drop_rx) = std::sync::mpsc::channel();
            match tx.send((event, drop_tx)) {
                Ok(()) => {}
                Err(_) => {
                    // receiving end of channel was closed
                    break;
                }
            }

            match drop_rx.recv_timeout(Duration::from_secs(30)) {
                Ok(()) => panic!("Node API should not send anything on ACK channel"),
                Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                    tracing::warn!("timeout while waiting for input ACK");
                }
                Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {} // expected result
            }

            if let Some(token) = drop_token {
                drop_tokens.push(token);
            }
        });

        Ok((EventStream { receiver: rx }, thread))
    }

    pub fn recv(&mut self) -> Option<Event> {
        let (node_event, drop_sender) = match self.receiver.recv() {
            Ok(d) => d,
            Err(flume::RecvError::Disconnected) => return None,
        };
        let event = match node_event {
            NodeEvent::Stop => Event::Stop,
            NodeEvent::InputClosed { id } => Event::InputClosed { id },
            NodeEvent::Input { id, metadata, data } => {
                let mapped = data
                    .map(|d| unsafe { MappedInputData::map(&d.shared_memory_id, d.len) })
                    .transpose();
                match mapped {
                    Ok(mapped) => Event::Input {
                        id,
                        metadata,
                        data: mapped.map(|data| Data {
                            data,
                            _drop: drop_sender,
                        }),
                    },
                    Err(err) => Event::Error(format!("{err:?}")),
                }
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

pub struct Data<'a> {
    data: MappedInputData<'a>,
    _drop: std::sync::mpsc::Sender<()>,
}

impl std::ops::Deref for Data<'_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &self.data
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
