use crate::daemon_connection::interactive::InteractiveEvents;
use dora_core::{config::NodeId, uhlc::Timestamp};
use dora_message::{
    DataflowId,
    daemon_to_node::DaemonReply,
    node_to_daemon::{DaemonRequest, NodeRegisterRequest, Timestamped},
};
use eyre::{Context, bail, eyre};
pub use node_integration_testing::IntegrationTestingEvents;
use std::net::{SocketAddr, TcpStream};
use tokio::sync::oneshot;

mod interactive;
pub(crate) mod node_integration_testing;
mod tcp;

mod json_to_arrow;

pub enum DaemonChannel {
    Tcp(TcpStream),
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
        Ok(DaemonChannel::Tcp(stream))
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
            DaemonChannel::Tcp(stream) => tcp::request(stream, request),
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
