use std::{
    io::{ErrorKind, Read, Write},
    net::{Ipv4Addr, SocketAddr, TcpStream},
};

use dora_core::{
    config::{DataId, NodeId},
    daemon_messages::{ControlRequest, DataflowId, NodeEvent},
};
use eyre::{bail, eyre, Context};

pub type EventStream = flume::Receiver<NodeEvent>;

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
        len: usize,
    ) -> eyre::Result<MessageSample> {
        tcp_send(
            &mut self.0,
            &ControlRequest::PrepareOutputMessage { output_id, len },
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
        let event = match tcp_receive(&mut event_stream) {
            Ok(event) => event,
            Err(err) if err.kind() == ErrorKind::UnexpectedEof => break,
            Err(err) => {
                let err = eyre!(err).wrap_err("failed to receive incoming event");
                tracing::warn!("{err:?}");
                continue;
            }
        };
        match tx.send(event) {
            Ok(()) => {}
            Err(_) => {
                // receiving end of channel was closed
                break;
            }
        }
    });

    Ok(rx)
}

fn init_control_stream(
    daemon_addr: SocketAddr,
    dataflow_id: DataflowId,
    node_id: &NodeId,
) -> eyre::Result<TcpStream> {
    let mut control_stream =
        TcpStream::connect(daemon_addr).wrap_err("failed to connect to dora-daemon")?;
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
