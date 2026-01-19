use std::{collections::BTreeMap, sync::Arc};

use super::{Connection, Listener};
use crate::Event;
use communication_layer_request_reply::{
    Transport, encoding::BincodeEncoding, transport::ShmemTransport,
};
use dora_core::{config::DataId, uhlc::HLC};
use dora_message::{
    common::Timestamped, daemon_to_node::DaemonReply, node_to_daemon::DaemonRequest,
};
use eyre::eyre;
use shared_memory_server::ShmemChannel;
use tokio::sync::{mpsc, oneshot};

#[tracing::instrument(skip(server, daemon_tx, clock), level = "trace")]
pub async fn listener_loop(
    server: ShmemChannel,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    queue_sizes: BTreeMap<DataId, usize>,
    clock: Arc<HLC>,
) {
    let (tx, rx) = flume::bounded(0);
    let mut server = ShmemTransport::new(server, None).with_encoding(BincodeEncoding);
    tokio::task::spawn_blocking(move || {
        while let Ok(operation) = rx.recv() {
            match operation {
                Operation::Receive(sender) => {
                    if sender.send(server.receive().map_err(Into::into)).is_err() {
                        break;
                    }
                }
                Operation::Send {
                    message,
                    result_sender,
                } => {
                    if result_sender
                        .send(server.send(&message).map_err(Into::into))
                        .is_err()
                    {
                        break;
                    }
                }
            }
        }
    });
    let connection = ShmemConnection(tx);
    Listener::run(connection, daemon_tx, clock).await
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
