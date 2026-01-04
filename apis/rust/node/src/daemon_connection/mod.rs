use crate::daemon_connection::interactive::InteractiveEvents;
use communication_layer_request_reply::{
    EncodedTransport, Transport,
    encoding::BincodeEncoding,
    transport::{FramedTransport, ShmemTransport},
};
use dora_core::{config::NodeId, uhlc::Timestamp};
use dora_message::{
    DataflowId,
    daemon_to_node::DaemonReply,
    node_to_daemon::{DaemonRequest, NodeRegisterRequest, Timestamped},
};
use eyre::{Context, ContextCompat, bail, eyre};
pub use node_integration_testing::IntegrationTestingEvents;
use shared_memory_extended::ShmemConf;
use shared_memory_server::ShmemChannel;
#[cfg(unix)]
use std::os::unix::net::UnixStream;
use std::{
    net::{SocketAddr, TcpStream},
    time::Duration,
};
use tokio::sync::oneshot;

mod interactive;
pub(crate) mod node_integration_testing;

mod json_to_arrow;

pub enum BytesTransport {
    Shmem(Box<ShmemTransport>),
    Tcp(FramedTransport<TcpStream>),
    #[cfg(unix)]
    UnixDomain(FramedTransport<UnixStream>),
}

impl Transport<[u8], Vec<u8>> for BytesTransport {
    fn send(&mut self, item: &[u8]) -> std::io::Result<()> {
        match self {
            Self::Shmem(channel) => channel.send(item),
            Self::Tcp(stream) => stream.send(item),
            #[cfg(unix)]
            Self::UnixDomain(stream) => stream.send(item),
        }
    }

    fn receive(&mut self) -> std::io::Result<Option<Vec<u8>>> {
        match self {
            Self::Shmem(channel) => channel.receive(),
            Self::Tcp(stream) => stream.receive(),
            #[cfg(unix)]
            Self::UnixDomain(stream) => stream.receive(),
        }
    }
}

pub enum DaemonChannel {
    Bytes(
        EncodedTransport<BytesTransport, BincodeEncoding, Timestamped<DaemonRequest>, DaemonReply>,
    ),
    Interactive(InteractiveEvents),
    IntegrationTestChannel(
        tokio::sync::mpsc::Sender<(
            Timestamped<DaemonRequest>,
            tokio::sync::oneshot::Sender<DaemonReply>,
        )>,
    ),
}

impl DaemonChannel {
    #[tracing::instrument(level = "trace")]
    pub fn new_tcp(socket_addr: SocketAddr) -> eyre::Result<Self> {
        let stream = TcpStream::connect(socket_addr).wrap_err("failed to open TCP connection")?;
        stream.set_nodelay(true).context("failed to set nodelay")?;
        Ok(Self::new_bytes(BytesTransport::Tcp(stream.into())))
    }

    #[tracing::instrument(level = "trace")]
    pub unsafe fn new_shmem(daemon_control_region_id: &str) -> eyre::Result<Self> {
        let daemon_events_region = ShmemConf::new()
            .os_id(daemon_control_region_id)
            .open()
            .wrap_err("failed to connect to dora-daemon")?;
        Ok(Self::new_bytes(BytesTransport::Shmem(unsafe {
            Box::new(ShmemTransport::new(
                ShmemChannel::new_client(daemon_events_region)
                    .wrap_err("failed to create ShmemChannel")?,
                Some(Duration::from_secs(5)),
            ))
        })))
    }

    #[cfg(unix)]
    #[tracing::instrument(level = "trace")]
    pub fn new_unix_socket(path: &std::path::PathBuf) -> eyre::Result<Self> {
        let stream = UnixStream::connect(path).wrap_err("failed to open Unix socket")?;
        Ok(Self::new_bytes(BytesTransport::UnixDomain(stream.into())))
    }

    fn new_bytes(transport: BytesTransport) -> Self {
        DaemonChannel::Bytes(EncodedTransport::new(transport, BincodeEncoding))
    }

    pub fn register(
        &mut self,
        dataflow_id: DataflowId,
        node_id: NodeId,
        timestamp: Timestamp,
    ) -> eyre::Result<()> {
        let msg = Timestamped {
            inner: DaemonRequest::Register(NodeRegisterRequest::new(dataflow_id, node_id)),
            timestamp,
        };
        let reply = self
            .request(&msg)
            .wrap_err("failed to send register request to dora-daemon")?;

        match reply {
            DaemonReply::Result(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to register node with dora-daemon")?,
            other => bail!("unexpected register reply: {other:?}"),
        }
        Ok(())
    }

    pub fn request(&mut self, request: &Timestamped<DaemonRequest>) -> eyre::Result<DaemonReply> {
        match self {
            DaemonChannel::Bytes(bytes) => {
                bytes
                    .send(request)
                    .wrap_err("failed to send request to dora-daemon")?;
                if request.inner.should_ignore_response()
                    && !matches!(bytes.inner_mut(), BytesTransport::Shmem(_))
                {
                    return Ok(DaemonReply::Empty);
                }
                let reply = bytes
                    .receive()
                    .wrap_err("failed to receive reply from dora-daemon")?
                    .wrap_err("server disconnected unexpectedly")?;
                Ok(reply)
            }
            DaemonChannel::Interactive(events) => events.request(request),
            DaemonChannel::IntegrationTestChannel(channel) => {
                let (reply_tx, reply) = oneshot::channel();
                channel
                    .blocking_send((request.clone(), reply_tx))
                    .expect("failed to send request to IntegrationTestChannel");
                reply
                    .blocking_recv()
                    .context("failed to receive oneshot reply")
            }
        }
    }
}
