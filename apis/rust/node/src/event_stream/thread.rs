use dora_core::{config::NodeId, uhlc};
use dora_message::{
    daemon_to_node::{DaemonReply, NodeEvent},
    node_to_daemon::{DaemonRequest, Timestamped},
};
use eyre::eyre;
use flume::RecvTimeoutError;
use std::{sync::Arc, time::Duration};
use tokio::sync::{OwnedSemaphorePermit, Semaphore, mpsc};

use crate::daemon_connection::DaemonChannel;

const REPLAY_NODE_EVENT_BYTE_LIMIT: usize = 4 * dora_message::MAX_MESSAGE_BYTES;

pub fn init(
    node_id: NodeId,
    tx: mpsc::Sender<EventItem>,
    channel: DaemonChannel,
    clock: Arc<uhlc::HLC>,
    replay_byte_budget: Option<Arc<Semaphore>>,
) -> eyre::Result<EventStreamThreadHandle> {
    let node_id_cloned = node_id.clone();
    let join_handle = std::thread::spawn(|| {
        event_stream_loop(node_id_cloned, tx, channel, clock, replay_byte_budget)
    });
    Ok(EventStreamThreadHandle::new(node_id, join_handle))
}

#[derive(Debug)]
#[allow(clippy::large_enum_variant)]
pub enum EventItem {
    NodeEvent {
        event: NodeEvent,
        _byte_permit: Option<OwnedSemaphorePermit>,
    },
    /// Zenoh-received input carrying the already-decoded Arrow array.
    ///
    /// The decode happens in the subscriber callback (in zenoh *receipt* order)
    /// rather than lazily downstream, because the per-input persistent
    /// `StreamDecoder` of the schema-once path must be fed in order — the
    /// scheduler reorders/drops events. For SHM payloads the decoded array still
    /// aliases the zenoh buffer (the decoder wraps it via
    /// `Buffer::from_custom_allocation`), so this stays zero-copy
    /// (dora-rs/adora#132).
    ZenohInput {
        id: dora_core::config::DataId,
        metadata: std::sync::Arc<dora_message::metadata::Metadata>,
        data: arrow::array::ArrayData,
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
    replay_byte_budget: Option<Arc<Semaphore>>,
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
        for event in events {
            if let Err(err) = clock.update_with_timestamp(&event.timestamp) {
                tracing::warn!("failed to update HLC: {err}");
            }
            if matches!(&event.inner, NodeEvent::AllInputsClosed) {
                close_tx = true;
            }

            // Out-of-band: an extension's bookkeeping, not a dataflow input.
            // Consume it so user code never has to match on an event it did
            // not ask for; the extension drains the queue on its own schedule.
            if let NodeEvent::ExtensionDropped { namespace, key } = &event.inner {
                crate::event_stream::extensions::push_dropped(namespace.clone(), key.clone());
                continue;
            }

            if let Some(tx) = tx.as_ref() {
                // `blocking_send` is used because this function runs on a
                // dedicated `std::thread` (not a tokio worker). Using
                // `tokio::sync::mpsc` here — instead of `flume` — avoids the
                // AB-BA deadlock between flume 0.10's spinlock and pyo3's
                // GIL-acquiring waker (upstream dora-rs/dora#1603).
                let Some(item) = node_event_item(event, replay_byte_budget.as_ref(), &node_id)
                else {
                    continue;
                };
                match tx.blocking_send(item) {
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
                tracing::warn!(
                    "dropping event because event `tx` was already closed: `{:?}`",
                    event.inner
                );
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

fn node_event_item(
    event: Timestamped<NodeEvent>,
    replay_byte_budget: Option<&Arc<Semaphore>>,
    node_id: &NodeId,
) -> Option<EventItem> {
    let byte_permit = if matches!(&event.inner, NodeEvent::Input { .. }) {
        let Some(budget) = replay_byte_budget else {
            return Some(EventItem::NodeEvent {
                event: event.inner,
                _byte_permit: None,
            });
        };
        let wire_size = match dora_message::serialized_size(&event) {
            Ok(size) => size,
            Err(error) => {
                tracing::error!(node = %node_id, "cannot size verified replay input: {error}");
                return None;
            }
        };
        let Ok(wire_size) = u32::try_from(wire_size) else {
            tracing::error!(node = %node_id, wire_size, "verified replay input is too large");
            return None;
        };
        match budget.clone().try_acquire_many_owned(wire_size) {
            Ok(permit) => Some(permit),
            Err(_) => {
                tracing::error!(
                    node = %node_id,
                    wire_size,
                    available_bytes = budget.available_permits(),
                    "verified replay input byte limit reached; dropping input"
                );
                return None;
            }
        }
    } else {
        None
    };

    Some(EventItem::NodeEvent {
        event: event.inner,
        _byte_permit: byte_permit,
    })
}

pub(super) fn replay_byte_budget(enabled: bool) -> Option<Arc<Semaphore>> {
    enabled.then(|| Arc::new(Semaphore::new(REPLAY_NODE_EVENT_BYTE_LIMIT)))
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use dora_core::{config::NodeId, uhlc::HLC};
    use dora_message::{common::Timestamped, daemon_to_node::NodeEvent, metadata::Metadata};
    use tokio::sync::Semaphore;

    use super::node_event_item;

    fn input_event() -> Timestamped<NodeEvent> {
        let clock = HLC::default();
        Timestamped {
            inner: NodeEvent::Input {
                id: "cam".into(),
                metadata: Arc::new(Metadata::new(clock.new_timestamp())),
                data: None,
            },
            timestamp: clock.new_timestamp(),
        }
    }

    #[test]
    fn verified_input_holds_its_exact_wire_size_until_drop() {
        let event = input_event();
        let wire_size = dora_message::serialized_size(&event).unwrap();
        let budget = Arc::new(Semaphore::new(wire_size));

        let item = node_event_item(event, Some(&budget), &NodeId::from("sink".to_owned()))
            .expect("input fits the byte budget");
        assert_eq!(budget.available_permits(), 0);

        drop(item);
        assert_eq!(budget.available_permits(), wire_size);
    }

    #[test]
    fn verified_input_is_rejected_when_its_exact_wire_size_does_not_fit() {
        let event = input_event();
        let wire_size = dora_message::serialized_size(&event).unwrap();
        let budget = Arc::new(Semaphore::new(wire_size - 1));

        assert!(node_event_item(event, Some(&budget), &NodeId::from("sink".to_owned())).is_none());
        assert_eq!(budget.available_permits(), wire_size - 1);
    }

    #[test]
    fn control_event_does_not_use_the_replay_byte_budget() {
        let clock = HLC::default();
        let event = Timestamped {
            inner: NodeEvent::Stop,
            timestamp: clock.new_timestamp(),
        };
        let budget = Arc::new(Semaphore::new(0));

        assert!(node_event_item(event, Some(&budget), &NodeId::from("sink".to_owned())).is_some());
        assert_eq!(budget.available_permits(), 0);
    }
}
