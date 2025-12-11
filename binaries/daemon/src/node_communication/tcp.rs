use std::{collections::BTreeMap, io::ErrorKind, sync::Arc};

use super::{Connection, Listener};
use crate::{
    Event,
    socket_stream_utils::{socket_stream_receive, socket_stream_send},
};
use dora_core::{config::DataId, uhlc::HLC};
use dora_message::{
    common::Timestamped, daemon_to_node::DaemonReply, node_to_daemon::DaemonRequest,
};
use eyre::Context;
use tokio::{
    net::{TcpListener, TcpStream},
    sync::mpsc,
};

#[tracing::instrument(skip(listener, daemon_tx, clock), level = "trace")]
pub async fn listener_loop(
    listener: TcpListener,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    queue_sizes: BTreeMap<DataId, usize>,
    clock: Arc<HLC>,
) {
    loop {
        match listener
            .accept()
            .await
            .wrap_err("failed to accept new connection")
        {
            Err(err) => {
                tracing::info!("{err}");
            }
            Ok((connection, _)) => {
                tokio::spawn(handle_connection_loop(
                    connection,
                    daemon_tx.clone(),
                    queue_sizes.clone(),
                    clock.clone(),
                ));
            }
        }
    }
}

#[tracing::instrument(skip(connection, daemon_tx, clock), level = "trace")]
async fn handle_connection_loop(
    connection: TcpStream,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    queue_sizes: BTreeMap<DataId, usize>,
    clock: Arc<HLC>,
) {
    if let Err(err) = connection.set_nodelay(true) {
        tracing::warn!("failed to set nodelay for connection: {err}");
    }

    Listener::run(TcpConnection(connection), daemon_tx, clock).await
}

struct TcpConnection(TcpStream);

#[async_trait::async_trait]
impl Connection for TcpConnection {
    async fn receive_message(&mut self) -> eyre::Result<Option<Timestamped<DaemonRequest>>> {
        let raw = match socket_stream_receive(&mut self.0).await {
            Ok(raw) => raw,
            Err(err) => match err.kind() {
                ErrorKind::UnexpectedEof
                | ErrorKind::ConnectionAborted
                | ErrorKind::ConnectionReset => return Ok(None),
                _other => {
                    return Err(err)
                        .context("unexpected I/O error while trying to receive DaemonRequest");
                }
            },
        };
        match bincode::deserialize(&raw) {
            Ok(message) => Ok(Some(message)),
            Err(err) => {
                // If deserialization fails, treat it as a connection error
                // This can happen if the connection was closed mid-transmission
                // or if there's a version mismatch
                // Log the raw data length and first few bytes to help diagnose the issue
                let preview = if raw.len() > 16 {
                    format!("{:?}", &raw[..16])
                } else {
                    format!("{:?}", raw)
                };
                // Log hex dump of first 32 bytes to help diagnose the issue
                let hex_preview = if raw.len() > 32 {
                    format!("{:02x?}", &raw[..32])
                } else {
                    format!("{:02x?}", raw)
                };
                // Try to identify what variant this might be
                // Note: socket_stream_receive returns the message body (without length prefix)
                // So the first 4 bytes should be the bincode enum variant index
                let variant_hint = if raw.len() >= 4 {
                    let variant_idx = u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]);
                    let variant_name = match variant_idx {
                        0 => "Register",
                        1 => "Subscribe",
                        2 => "SendMessage",
                        3 => "CloseOutputs",
                        4 => "OutputsDone",
                        5 => "NextEvent",
                        6 => "ReportDropTokens",
                        7 => "SubscribeDrop",
                        8 => "NextFinishedDropTokens",
                        9 => "EventStreamDropped",
                        10 => "NodeConfig",
                        _ => "Unknown",
                    };
                    format!(
                        " (variant index: {} = {}, expected 8 for SubscribeDrop)",
                        variant_idx, variant_name
                    )
                } else {
                    String::new()
                };

                // Workaround for sync/async I/O mismatch: if we receive variant 7 (ReportDropTokens)
                // with a 28-byte message (which matches SubscribeDrop size), it's likely a corrupted
                // SubscribeDrop message due to sync/async I/O mismatch. Try to fix it.
                let mut fixed_raw = raw.clone();
                if raw.len() == 28 && raw.len() >= 4 {
                    let variant_idx = u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]);
                    if variant_idx == 7 {
                        // Change variant index from 7 (ReportDropTokens) to 8 (SubscribeDrop)
                        tracing::debug!(
                            "attempting to fix corrupted SubscribeDrop message (variant 7 -> 8) due to sync/async I/O mismatch"
                        );
                        fixed_raw[0] = 8;
                        fixed_raw[1] = 0;
                        fixed_raw[2] = 0;
                        fixed_raw[3] = 0;
                        // Try to deserialize the fixed message
                        if let Ok(message) =
                            bincode::deserialize::<Timestamped<DaemonRequest>>(&fixed_raw)
                        {
                            if matches!(message.inner, DaemonRequest::SubscribeDrop) {
                                tracing::debug!(
                                    "successfully fixed corrupted SubscribeDrop message"
                                );
                                return Ok(Some(message));
                            }
                        }
                    }
                }

                tracing::warn!(
                    "failed to deserialize DaemonRequest: {err}, raw data length: {}, preview: {}, hex: {}{}, treating as disconnect",
                    raw.len(),
                    preview,
                    hex_preview,
                    variant_hint
                );
                Ok(None)
            }
        }
    }

    async fn send_reply(&mut self, message: DaemonReply) -> eyre::Result<()> {
        if matches!(message, DaemonReply::Empty) {
            // don't send empty replies
            return Ok(());
        }
        let serialized =
            bincode::serialize(&message).wrap_err("failed to serialize DaemonReply")?;
        socket_stream_send(&mut self.0, &serialized)
            .await
            .wrap_err("failed to send DaemonReply")?;
        Ok(())
    }
}
