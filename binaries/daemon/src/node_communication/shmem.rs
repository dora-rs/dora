use std::{collections::BTreeMap, sync::Arc};

use super::{Connection, Listener};
use crate::Event;
use dora_core::{
    config::DataId,
    daemon_messages::{DaemonReply, DaemonRequest, Timestamped},
    message::uhlc::HLC,
};
use eyre::eyre;
use shared_memory_server::ShmemServer;
use tokio::sync::{mpsc, oneshot};

#[tracing::instrument(skip(server, daemon_tx, clock), level = "trace")]
pub async fn listener_loop(
    mut server: ShmemServer<Timestamped<DaemonRequest>, DaemonReply>,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    queue_sizes: BTreeMap<DataId, usize>,
    clock: Arc<HLC>,
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
    Listener::run(connection, daemon_tx, queue_sizes, clock).await
}

enum Operation {
    Receive(oneshot::Sender<eyre::Result<Option<Timestamped<DaemonRequest>>>>),
    Send {
        message: DaemonReply,
        result_sender: oneshot::Sender<eyre::Result<()>>,
    },
}

struct ShmemConnection(flume::Sender<Operation>);

#[async_trait::async_trait]
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
