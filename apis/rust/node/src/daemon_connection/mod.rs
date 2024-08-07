use dora_core::{config::NodeId, uhlc::Timestamp};
use dora_message::{
    daemon_to_node::DaemonReply,
    node_to_daemon::{DaemonRequest, NodeRegisterRequest, Timestamped},
    DataflowId,
};
use eyre::{bail, eyre, Context};
use shared_memory_server::{ShmemClient, ShmemConf};
#[cfg(unix)]
use std::os::unix::net::UnixStream;
use std::{
    net::{SocketAddr, TcpStream},
    time::Duration,
};

mod tcp;
#[cfg(unix)]
mod unix_domain;

pub enum DaemonChannel {
    Shmem(ShmemClient<Timestamped<DaemonRequest>, DaemonReply>),
    Tcp(TcpStream),
    #[cfg(unix)]
    UnixDomain(UnixStream),
}

impl DaemonChannel {
    #[tracing::instrument(level = "trace")]
    pub fn new_tcp(socket_addr: SocketAddr) -> eyre::Result<Self> {
        let stream = TcpStream::connect(socket_addr).wrap_err("failed to open TCP connection")?;
        stream.set_nodelay(true).context("failed to set nodelay")?;
        Ok(DaemonChannel::Tcp(stream))
    }

    #[tracing::instrument(level = "trace")]
    pub unsafe fn new_shmem(daemon_control_region_id: &str) -> eyre::Result<Self> {
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

    #[cfg(unix)]
    #[tracing::instrument(level = "trace")]
    pub fn new_unix_socket(path: &std::path::PathBuf) -> eyre::Result<Self> {
        let stream = UnixStream::connect(path).wrap_err("failed to open Unix socket")?;
        Ok(DaemonChannel::UnixDomain(stream))
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
            DaemonChannel::Shmem(client) => client.request(request),
            DaemonChannel::Tcp(stream) => tcp::request(stream, request),
            #[cfg(unix)]
            DaemonChannel::UnixDomain(stream) => unix_domain::request(stream, request),
        }
    }
}
