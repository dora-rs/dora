use std::{
    io::{ErrorKind, Read, Write},
    marker::PhantomData,
    net::{Ipv4Addr, SocketAddr, TcpStream},
    time::Duration,
};

use dora_core::{
    config::{DataId, NodeId},
    daemon_messages::{ControlRequest, DataflowId, DropEvent, NodeEvent},
    shm_channel::ShmemChannel,
};
use dora_message::Metadata;
use eyre::{bail, eyre, Context};
use shared_memory::{Shmem, ShmemConf};

pub struct DaemonConnection {
    pub control_channel: ControlChannel,
    pub event_stream: EventStream,
}

impl DaemonConnection {
    pub fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        daemon_port: u16,
        daemon_events_region_id: &str,
    ) -> eyre::Result<Self> {
        let control_channel = ControlChannel::init(dataflow_id, node_id, daemon_events_region_id)
            .wrap_err("failed to init control stream")?;

        let daemon_addr = (Ipv4Addr::new(127, 0, 0, 1), daemon_port).into();
        let event_stream = EventStream::init(daemon_addr, dataflow_id, node_id)
            .wrap_err("failed to init event stream")?;

        Ok(Self {
            control_channel,
            event_stream,
        })
    }
}

pub struct ControlChannel {
    channel: ShmemChannel,
}

impl ControlChannel {
    #[tracing::instrument]
    fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        daemon_events_region_id: &str,
    ) -> eyre::Result<Self> {
        let daemon_events_region = ShmemConf::new()
            .os_id(daemon_events_region_id)
            .open()
            .wrap_err("failed to connect to dora-daemon")?;
        let mut channel = unsafe { ShmemChannel::new_client(daemon_events_region) }
            .wrap_err("failed to create ShmemChannel")?;

        let msg = ControlRequest::Register {
            dataflow_id,
            node_id: node_id.clone(),
        };
        channel
            .send(&msg)
            .wrap_err("failed to send register request to dora-daemon")?;

        // wait for reply
        let reply = channel
            .receive()
            .wrap_err("failed to wait for receive register reply from dora-daemon")?;
        match reply {
            dora_core::daemon_messages::ControlReply::Result(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to register node with dora-daemon")?,
            other => bail!("unexpected register reply: {other:?}"),
        }

        Ok(Self { channel })
    }

    pub fn report_stop(&mut self) -> eyre::Result<()> {
        self.channel
            .send(&ControlRequest::Stopped)
            .wrap_err("failed to report stopped to dora-daemon")?;
        match self
            .channel
            .receive()
            .wrap_err("failed to receive stopped reply from dora-daemon")?
        {
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
        self.channel
            .send(&ControlRequest::PrepareOutputMessage {
                output_id,
                metadata,
                data_len,
            })
            .wrap_err("failed to send PrepareOutputMessage request to dora-daemon")?;
        match self
            .channel
            .receive()
            .wrap_err("failed to receive PrepareOutputMessage reply from dora-daemon")?
        {
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
        self.channel
            .send(&ControlRequest::SendOutMessage { id: sample.id })
            .wrap_err("failed to send SendOutMessage request to dora-daemon")?;
        match self
            .channel
            .receive()
            .wrap_err("failed to receive SendOutMessage reply from dora-daemon")?
        {
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
        daemon_addr: SocketAddr,
        dataflow_id: DataflowId,
        node_id: &NodeId,
    ) -> eyre::Result<Self> {
        let mut event_stream =
            TcpStream::connect(daemon_addr).wrap_err("failed to connect to dora-daemon")?;
        event_stream
            .set_nodelay(true)
            .wrap_err("failed to set TCP_NODELAY")?;
        tcp_send(
            &mut event_stream,
            &ControlRequest::Subscribe {
                dataflow_id,
                node_id: node_id.clone(),
            },
        )
        .wrap_err("failed to send subscribe request to dora-daemon")?;
        match tcp_receive(&mut event_stream)
            .wrap_err("failed to receive subscribe reply from dora-daemon")?
        {
            dora_core::daemon_messages::ControlReply::Result(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to create subscription with dora-daemon")?,
            other => bail!("unexpected subscribe reply: {other:?}"),
        }

        let (tx, rx) = flume::bounded(1);
        std::thread::spawn(move || loop {
            let event: NodeEvent = match tcp_receive(&mut event_stream) {
                Ok(event) => event,
                Err(err) if err.kind() == ErrorKind::UnexpectedEof => break,
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
                let message = DropEvent { token };
                if let Err(err) = tcp_send(&mut event_stream, &message) {
                    tracing::warn!("failed to send drop token: {err}");
                    break;
                }
            }
        });

        Ok(EventStream { receiver: rx })
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

fn tcp_send<T: serde::Serialize>(connection: &mut TcpStream, request: &T) -> std::io::Result<()> {
    let serialized = serde_json::to_vec(request)?;

    let len_raw = (serialized.len() as u64).to_le_bytes();
    connection.write_all(&len_raw)?;
    connection.write_all(&serialized)?;
    Ok(())
}

fn tcp_receive<T>(connection: &mut TcpStream) -> std::io::Result<T>
where
    T: for<'a> serde::Deserialize<'a>,
{
    let reply_len = {
        let mut raw = [0; 8];
        connection.read_exact(&mut raw)?;
        u64::from_le_bytes(raw) as usize
    };
    let mut reply_raw = vec![0; reply_len];
    connection.read_exact(&mut reply_raw)?;

    let reply = serde_json::from_slice(&reply_raw)?;

    Ok(reply)
}
