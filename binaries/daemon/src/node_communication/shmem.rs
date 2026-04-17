use std::sync::{Arc, atomic::AtomicU64};

use super::{Connection, Listener};
use crate::Event;
use dora_core::uhlc::HLC;
use dora_message::{
    common::Timestamped, daemon_to_node::DaemonReply, node_to_daemon::DaemonRequest,
};
use eyre::eyre;
use shared_memory_server::ShmemServer;
use tokio::sync::{mpsc, oneshot};

#[tracing::instrument(skip(server, daemon_tx, clock, last_activity), level = "trace")]
pub async fn listener_loop(
    mut server: ShmemServer<Timestamped<DaemonRequest>, DaemonReply>,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    clock: Arc<HLC>,
    last_activity: Arc<AtomicU64>,
    shutdown: tokio::sync::watch::Receiver<bool>,
) {
    let (tx, rx) = flume::bounded(0);
    tokio::task::spawn_blocking(move || {
        while let Ok(operation) = rx.recv() {
            match operation {
                Operation::Receive(sender) => {
                    if sender.send(server.listen()).is_err() {
                        break;
                    }
                }
                Operation::Send {
                    message,
                    result_sender,
                } => {
                    let result = server.send_reply(&message);
                    if result_sender.send(result).is_err() {
                        break;
                    }
                }
            }
        }
    });
    let connection = ShmemConnection(tx);
    // `changed()` requires `&mut self`
    let mut shutdown = shutdown;
    tokio::select! {
        _ = Listener::run(connection, daemon_tx, clock, last_activity) => {}
        _ = shutdown.changed() => {
            tracing::trace!("shmem listener shutting down");
        }
    }
    // Dropping `ShmemConnection(tx)` causes `rx.recv()` in the spawn_blocking
    // thread to return Err, which exits the thread. Note: if ShmemServer::listen()
    // is currently blocked waiting for a client message, the thread won't exit
    // until listen() returns. This is a bounded leak (only at dataflow shutdown).
}

#[allow(clippy::large_enum_variant)]
enum Operation {
    Receive(oneshot::Sender<eyre::Result<Option<Timestamped<DaemonRequest>>>>),
    Send {
        message: DaemonReply,
        result_sender: oneshot::Sender<eyre::Result<()>>,
    },
}

struct ShmemConnection(flume::Sender<Operation>);

impl Connection for ShmemConnection {
    async fn receive_message(&mut self) -> eyre::Result<Option<Timestamped<DaemonRequest>>> {
        let (tx, rx) = oneshot::channel();
        self.0
            .send_async(Operation::Receive(tx))
            .await
            .map_err(|_| eyre!("failed send receive request to ShmemServer"))?;
        rx.await
            .map_err(|_| eyre!("failed to receive from ShmemServer"))
            .and_then(|r| r)
    }

    async fn send_reply(&mut self, reply: DaemonReply) -> eyre::Result<()> {
        let (tx, rx) = oneshot::channel();
        self.0
            .send_async(Operation::Send {
                message: reply,
                result_sender: tx,
            })
            .await
            .map_err(|_| eyre!("failed send send request to ShmemServer"))?;
        rx.await
            .map_err(|_| eyre!("failed to receive from ShmemServer"))
            .and_then(|r| r)
    }
}
