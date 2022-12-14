use std::{
    io::{ErrorKind, Read, Write},
    marker::PhantomData,
    net::{Ipv4Addr, SocketAddr, TcpStream},
    time::Duration,
};

use dora_core::{
    config::{DataId, NodeId},
    daemon_messages::{ControlRequest, DataflowId, NodeEvent},
};
use dora_message::Metadata;
use eyre::{bail, eyre, Context};
use shared_memory::{Shmem, ShmemConf};

pub struct DaemonConnection {
    pub control_channel: ControlChannel,
    pub event_stream: EventStream,
}

impl DaemonConnection {
    pub fn init(dataflow_id: DataflowId, node_id: &NodeId, daemon_port: u16) -> eyre::Result<Self> {
        let daemon_addr = (Ipv4Addr::new(127, 0, 0, 1), daemon_port).into();
        let control_stream = init_control_stream(daemon_addr, dataflow_id, &node_id)
            .wrap_err("failed to init control stream")?;

        let event_stream = init_event_stream(daemon_addr, dataflow_id, &node_id)
            .wrap_err("failed to init event stream")?;

        Ok(Self {
            control_channel: ControlChannel(control_stream),
            event_stream,
        })
    }
}

pub struct ControlChannel(TcpStream);

impl ControlChannel {
    pub fn report_stop(&mut self) -> eyre::Result<()> {
        tcp_send(&mut self.0, &ControlRequest::Stopped)
            .wrap_err("failed to send subscribe request to dora-daemon")?;
        match tcp_receive(&mut self.0)
            .wrap_err("failed to receive subscribe reply from dora-daemon")?
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
        tcp_send(
            &mut self.0,
            &ControlRequest::PrepareOutputMessage {
                output_id,
                metadata,
                data_len,
            },
        )
        .wrap_err("failed to send PrepareOutputMessage request to dora-daemon")?;
        match tcp_receive(&mut self.0)
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
        tcp_send(
            &mut self.0,
            &ControlRequest::SendOutMessage { id: sample.id },
        )
        .wrap_err("failed to send SendOutMessage request to dora-daemon")?;
        match tcp_receive(&mut self.0)
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
    pub fn recv(&mut self) -> Option<Event> {
        let (node_event, ack) = match self.receiver.recv() {
            Ok(d) => d,
            Err(flume::RecvError::Disconnected) => return None,
        };
        let event = match node_event {
            NodeEvent::Stop => Event::Stop,
            NodeEvent::InputClosed { id } => Event::InputClosed { id },
            NodeEvent::Input { id, metadata, data } => {
                let mapped = data
                    .map(|d| unsafe { MappedInputData::map(&d.shared_memory_id) })
                    .transpose();
                match mapped {
                    Ok(mapped) => Event::Input {
                        id,
                        metadata,
                        data: mapped.map(|data| Data { data, _ack: ack }),
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

fn init_event_stream(
    daemon_addr: SocketAddr,
    dataflow_id: DataflowId,
    node_id: &NodeId,
) -> eyre::Result<EventStream> {
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

        let (ack_tx, ack_rx) = std::sync::mpsc::channel();
        match tx.send((event, ack_tx)) {
            Ok(()) => {}
            Err(_) => {
                // receiving end of channel was closed
                break;
            }
        }

        match ack_rx.recv_timeout(Duration::from_secs(30)) {
            Ok(()) => panic!("Node API should not send anything on ACK channel"),
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                tracing::warn!("timeout while waiting for input ACK");
            }
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {} // expected result
        }
    });

    Ok(EventStream { receiver: rx })
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
    _ack: std::sync::mpsc::Sender<()>,
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
    _data: PhantomData<&'a [u8]>,
}

impl MappedInputData<'_> {
    unsafe fn map(shared_memory_id: &str) -> eyre::Result<Self> {
        let memory = ShmemConf::new()
            .os_id(shared_memory_id)
            .open()
            .wrap_err("failed to map shared memory input")?;
        Ok(MappedInputData {
            memory,
            _data: PhantomData,
        })
    }
}

impl std::ops::Deref for MappedInputData<'_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        unsafe { self.memory.as_slice() }
    }
}

fn init_control_stream(
    daemon_addr: SocketAddr,
    dataflow_id: DataflowId,
    node_id: &NodeId,
) -> eyre::Result<TcpStream> {
    let mut control_stream =
        TcpStream::connect(daemon_addr).wrap_err("failed to connect to dora-daemon")?;
    control_stream
        .set_nodelay(true)
        .wrap_err("failed to set TCP_NODELAY")?;
    tcp_send(
        &mut control_stream,
        &ControlRequest::Register {
            dataflow_id,
            node_id: node_id.clone(),
        },
    )
    .wrap_err("failed to send register request to dora-daemon")?;
    match tcp_receive(&mut control_stream)
        .wrap_err("failed to receive register reply from dora-daemon")?
    {
        dora_core::daemon_messages::ControlReply::Result(result) => result
            .map_err(|e| eyre!(e))
            .wrap_err("failed to register node with dora-daemon")?,
        other => bail!("unexpected register reply: {other:?}"),
    }
    Ok(control_stream)
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
