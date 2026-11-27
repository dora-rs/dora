use std::{
    net::{Shutdown, SocketAddr, TcpStream},
    sync::Arc,
};

use dora_core::{config::NodeId, uhlc::HLC};
use dora_message::{
    DataflowId,
    daemon_to_node::DaemonReply,
    node_to_daemon::{DaemonRequest, Timestamped},
};

use crate::daemon_connection::DaemonChannel;
use futures::channel::oneshot;

/// Interrupt the blocking read if the event stream and its waiters go away.
struct CloseOnDrop(TcpStream);

impl Drop for CloseOnDrop {
    fn drop(&mut self) {
        let _ = self.0.shutdown(Shutdown::Both);
    }
}

pub(super) async fn wait(
    addr: SocketAddr,
    dataflow_id: DataflowId,
    node_id: NodeId,
    clock: Arc<HLC>,
) {
    let (connected, socket_guard) = oneshot::channel();
    let (done, reply) = oneshot::channel();
    let listen = move || -> eyre::Result<DaemonReply> {
        let socket = TcpStream::connect(addr)?;
        if connected.send(CloseOnDrop(socket.try_clone()?)).is_err() {
            return Ok(DaemonReply::Empty);
        }
        let mut channel = DaemonChannel::Tcp(socket);
        channel.register(dataflow_id, node_id, clock.new_timestamp())?;
        channel.request(&Timestamped {
            inner: DaemonRequest::WaitForManualStop,
            timestamp: clock.new_timestamp(),
        })
    };
    let thread = std::thread::Builder::new()
        .name("dora-manual-stop".into())
        .spawn(move || {
            let _ = done.send(listen());
        });
    match thread {
        Ok(_) => {
            let _guard = socket_guard.await;
            match reply.await {
                Ok(Ok(DaemonReply::Result(Ok(())))) => return,
                Ok(Ok(DaemonReply::Result(Err(err)))) => {
                    tracing::debug!(%err, "manual stop wait rejected")
                }
                Ok(Err(err)) => tracing::debug!(%err, "manual stop connection unavailable"),
                _ => tracing::debug!("manual stop connection ended without a stop"),
            }
        }
        Err(err) => tracing::debug!(%err, "could not start manual stop listener"),
    }
    // Older daemons reject the new request. The ordinary Stop event remains usable.
    std::future::pending().await
}
