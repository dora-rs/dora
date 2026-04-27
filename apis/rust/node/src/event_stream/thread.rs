use dora_core::{config::NodeId, uhlc};
use dora_message::{
    daemon_to_node::{DaemonReply, NodeEvent},
    node_to_daemon::{DaemonRequest, Timestamped},
};
use eyre::eyre;
use flume::RecvTimeoutError;
use std::{sync::Arc, time::Duration};
use tokio::sync::mpsc;

use crate::daemon_connection::DaemonChannel;

pub fn init(
    node_id: NodeId,
    tx: mpsc::Sender<EventItem>,
    channel: DaemonChannel,
    clock: Arc<uhlc::HLC>,
) -> eyre::Result<EventStreamThreadHandle> {
    let node_id_cloned = node_id.clone();
    let join_handle = std::thread::spawn(|| event_stream_loop(node_id_cloned, tx, channel, clock));
    Ok(EventStreamThreadHandle::new(node_id, join_handle))
}

#[derive(Debug)]
#[allow(clippy::large_enum_variant)]
pub enum EventItem {
    NodeEvent {
        event: NodeEvent,
    },
    /// Zenoh-received input carrying the raw `ZBytes` payload.
    ///
    /// Unlike `NodeEvent::Input` which wraps data in `DataMessage::Vec`
    /// (requiring a copy for SHM payloads), this variant holds the
    /// original zenoh buffer so the Arrow conversion can use
    /// `Buffer::from_custom_allocation` for zero-copy
    /// (dora-rs/adora#132).
    ZenohInput {
        id: dora_core::config::DataId,
        metadata: std::sync::Arc<dora_message::metadata::Metadata>,
        payload: zenoh::bytes::ZBytes,
    },
    FatalError(eyre::Report),
    TimeoutError(eyre::Report),
}

pub struct EventStreamThreadHandle {
    _node_id: NodeId,
    handle: flume::Receiver<std::thread::Result<()>>,
}

impl EventStreamThreadHandle {
    fn new(node_id: NodeId, join_handle: std::thread::JoinHandle<()>) -> Self {
        let (tx, rx) = flume::bounded(1);
        std::thread::spawn(move || {
            let _ = tx.send(join_handle.join());
        });
        Self {
            _node_id: node_id,
            handle: rx,
        }
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
    tx: mpsc::Sender<EventItem>,
    mut channel: DaemonChannel,
    clock: Arc<uhlc::HLC>,
) {
    let mut tx = Some(tx);
    let mut close_tx = false;

    let result = 'outer: loop {
        let daemon_request = Timestamped {
            inner: DaemonRequest::NextEvent,
            timestamp: clock.new_timestamp(),
        };
        let events = match channel.request(&daemon_request) {
            Ok(DaemonReply::NextEvents(events)) => {
                if events.is_empty() {
                    tracing::trace!("event stream closed for node `{node_id}`");
                    break Ok(());
                } else {
                    events
                }
            }
            Ok(DaemonReply::Result(Err(err))) => {
                let err = eyre!(err).wrap_err("error in incoming event");
                tracing::error!("{err:?}");
                // Back off to avoid spinning on persistent daemon errors
                std::thread::sleep(Duration::from_millis(100));
                continue;
            }

            Ok(other) => {
                let err = eyre!("unexpected control reply: {other:?}");
                tracing::warn!("{err:?}");
                std::thread::sleep(Duration::from_millis(100));
                continue;
            }
            Err(err) => {
                // Channel error means the daemon connection is broken.
                // Break instead of retrying a dead connection.
                break Err(err.wrap_err("daemon channel broken"));
            }
        };
        for Timestamped { inner, timestamp } in events {
            if let Err(err) = clock.update_with_timestamp(&timestamp) {
                tracing::warn!("failed to update HLC: {err}");
            }
            if matches!(inner, NodeEvent::AllInputsClosed) {
                close_tx = true;
            }

            if let Some(tx) = tx.as_ref() {
                // `blocking_send` is used because this function runs on a
                // dedicated `std::thread` (not a tokio worker). Using
                // `tokio::sync::mpsc` here — instead of `flume` — avoids the
                // AB-BA deadlock between flume 0.10's spinlock and pyo3's
                // GIL-acquiring waker (upstream dora-rs/dora#1603).
                match tx.blocking_send(EventItem::NodeEvent { event: inner }) {
                    Ok(()) => {}
                    Err(send_error) => {
                        let event = send_error.0;
                        tracing::trace!(
                            "event channel was closed already, could not forward `{event:?}`"
                        );

                        break 'outer Ok(());
                    }
                }
            } else {
                tracing::warn!("dropping event because event `tx` was already closed: `{inner:?}`");
            }

            if close_tx {
                tx = None;
            };
        }
    };
    if let Err(err) = result
        && let Some(tx) = tx.as_ref()
        && let Err(mpsc::error::SendError(item)) = tx.blocking_send(EventItem::FatalError(err))
    {
        let err = match item {
            EventItem::FatalError(err) => err,
            _ => unreachable!(),
        };
        tracing::error!("failed to report fatal EventStream error: {err:?}");
    }
}
