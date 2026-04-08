use dora_core::{
    config::NodeId,
    uhlc::{self},
};
use dora_message::{
    common::Timestamped,
    daemon_to_node::{NodeEvent, NodeEventOrUnknown},
    node_to_daemon::DropToken,
};
use eyre::{Context, eyre};
use flume::RecvTimeoutError;
use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use crate::daemon_connection::DaemonChannel;

pub fn init(
    node_id: NodeId,
    tx: tokio::sync::mpsc::UnboundedSender<EventItem>,
    channel: DaemonChannel,
    clock: Arc<uhlc::HLC>,
) -> eyre::Result<EventStreamThreadHandle> {
    let node_id_cloned = node_id.clone();
    let join_handle = std::thread::spawn(|| event_stream_loop(node_id_cloned, tx, channel, clock));
    Ok(EventStreamThreadHandle::new(join_handle))
}

#[derive(Debug)]
#[allow(clippy::large_enum_variant)]
pub enum EventItem {
    NodeEvent {
        event: NodeEvent,
        ack_channel: flume::Sender<()>,
    },
    FatalError(eyre::Report),
    TimeoutError(eyre::Report),
}

pub struct EventStreamThreadHandle {
    handle: flume::Receiver<std::thread::Result<()>>,
}

impl EventStreamThreadHandle {
    fn new(join_handle: std::thread::JoinHandle<()>) -> Self {
        let (tx, rx) = flume::bounded(1);
        std::thread::spawn(move || {
            let _ = tx.send(join_handle.join());
        });
        Self { handle: rx }
    }
}

impl Drop for EventStreamThreadHandle {
    fn drop(&mut self) {
        if self.handle.is_empty() {
            tracing::trace!("waiting for event stream thread");
        }

        // TODO: The event stream duration has been shorten due to
        // Python Reference Counting not working properly and deleting the node
        // before deleting event creating a race condition.
        //
        // In the future, we hope to fix this issue so that
        // the event stream can be properly waited for every time.
        match self.handle.recv_timeout(Duration::from_secs(1)) {
            Ok(Ok(())) => {
                tracing::trace!("event stream thread finished");
            }
            Ok(Err(_)) => {
                tracing::error!("event stream thread panicked");
            }
            Err(RecvTimeoutError::Timeout) => {
                tracing::warn!("timeout while waiting for event stream thread");
            }
            Err(RecvTimeoutError::Disconnected) => {
                tracing::warn!("event stream thread result channel closed unexpectedly");
            }
        }
    }
}

#[tracing::instrument(skip(tx, channel, clock))]
fn event_stream_loop(
    node_id: NodeId,
    tx: tokio::sync::mpsc::UnboundedSender<EventItem>,
    mut channel: DaemonChannel,
    clock: Arc<uhlc::HLC>,
) {
    let mut tx = Some(tx);
    let mut close_tx = false;
    let mut pending_drop_tokens: Vec<(DropToken, flume::Receiver<()>, Instant, u64)> = Vec::new();
    let mut drop_tokens = Vec::new();

    let result = 'outer: loop {
        if let Err(err) = handle_pending_drop_tokens(&mut pending_drop_tokens, &mut drop_tokens) {
            break 'outer Err(err);
        }

        let events = match channel.next_event(std::mem::take(&mut drop_tokens)) {
            Ok(Some(events)) => {
                if events.is_empty() {
                    tracing::trace!("event stream closed for node `{node_id}`");
                    break Ok(());
                } else {
                    events
                }
            }
            Ok(None) => {
                // Keepalive — server had no events yet, silently retry
                continue;
            }
            Err(err) => {
                let err = err.wrap_err("failed to receive incoming event");
                break 'outer Err(err);
            }
        };
        for Timestamped { inner, timestamp } in events {
            if let Err(err) = clock.update_with_timestamp(&timestamp) {
                tracing::warn!("failed to update HLC: {err}");
            }
            let NodeEventOrUnknown::Known(inner) = inner else {
                tracing::info!("received unknown event from daemon -> skipping it");
                continue;
            };

            let drop_token = match inner.as_ref() {
                NodeEvent::Input {
                    data: Some(data), ..
                } => data.drop_token(),
                NodeEvent::AllInputsClosed => {
                    close_tx = true;
                    None
                }
                _ => None,
            };

            if let Some(tx) = tx.as_ref() {
                let (drop_tx, drop_rx) = flume::bounded(0);
                match tx.send(EventItem::NodeEvent {
                    event: *inner,
                    ack_channel: drop_tx,
                }) {
                    Ok(()) => {}
                    Err(send_error) => {
                        let event = send_error.0;
                        tracing::warn!(
                            "event channel was closed already, could not forward `{event:?}`"
                        );

                        break 'outer Ok(());
                    }
                }

                if let Some(token) = drop_token {
                    pending_drop_tokens.push((token, drop_rx, Instant::now(), 1));
                }
            } else {
                tracing::warn!("dropping event because event `tx` was already closed: `{inner:?}`");
            }

            if close_tx {
                tx = None;
            };
        }
    };
    if let Err(err) = result {
        if let Some(tx) = tx.as_ref() {
            if let Err(send_error) = tx.send(EventItem::FatalError(err)) {
                let err = match send_error.0 {
                    EventItem::FatalError(err) => err,
                    _ => unreachable!(),
                };
                tracing::error!("failed to report fatal EventStream error: {err:?}");
            }
        } else {
            tracing::error!("received error event after `tx` was closed: {err:?}");
        }
    }

    if let Err(err) = report_remaining_drop_tokens(channel, drop_tokens, pending_drop_tokens)
        .context("failed to report remaining drop tokens")
    {
        tracing::warn!("{err:?}");
    }
}

fn handle_pending_drop_tokens(
    pending_drop_tokens: &mut Vec<(DropToken, flume::Receiver<()>, Instant, u64)>,
    drop_tokens: &mut Vec<DropToken>,
) -> eyre::Result<()> {
    let mut still_pending = Vec::new();
    for (token, rx, since, warn) in pending_drop_tokens.drain(..) {
        match rx.try_recv() {
            Ok(()) => return Err(eyre!("Node API should not send anything on ACK channel")),
            Err(flume::TryRecvError::Disconnected) => {
                // the event was dropped -> add the drop token to the list
                drop_tokens.push(token);
            }
            Err(flume::TryRecvError::Empty) => {
                let duration = Duration::from_secs(30 * warn);
                if since.elapsed() > duration {
                    tracing::warn!("timeout: token {token:?} was not dropped after {duration:?}");
                }
                still_pending.push((token, rx, since, warn + 1));
            }
        }
    }
    *pending_drop_tokens = still_pending;
    Ok(())
}

fn report_remaining_drop_tokens(
    mut channel: DaemonChannel,
    mut drop_tokens: Vec<DropToken>,
    mut pending_drop_tokens: Vec<(DropToken, flume::Receiver<()>, Instant, u64)>,
) -> eyre::Result<()> {
    while !(pending_drop_tokens.is_empty() && drop_tokens.is_empty()) {
        if !drop_tokens.is_empty() {
            channel.report_drop_tokens_rpc(std::mem::take(&mut drop_tokens))?;
        }

        let mut still_pending = Vec::new();
        for (token, rx, since, _) in pending_drop_tokens.drain(..) {
            match rx.recv_timeout(Duration::from_millis(100)) {
                Ok(()) => return Err(eyre!("Node API should not send anything on ACK channel")),
                Err(flume::RecvTimeoutError::Disconnected) => {
                    // the event was dropped -> add the drop token to the list
                    drop_tokens.push(token);
                }
                Err(flume::RecvTimeoutError::Timeout) => {
                    let duration = Duration::from_secs(1);
                    if since.elapsed() > duration {
                        tracing::warn!(
                            "timeout: node finished, but token {token:?} was still not \
                            dropped after {duration:?} -> ignoring it"
                        );
                    } else {
                        still_pending.push((token, rx, since, 0));
                    }
                }
            }
        }
        pending_drop_tokens = still_pending;
        if !pending_drop_tokens.is_empty() {
            tracing::trace!("waiting for drop for {} events", pending_drop_tokens.len());
        }
    }

    Ok(())
}
