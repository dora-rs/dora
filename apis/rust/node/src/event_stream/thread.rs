use dora_core::{config::NodeId, uhlc};
use dora_message::{
    daemon_to_node::{DaemonReply, NodeEvent},
    node_to_daemon::{DaemonRequest, Timestamped},
};
use eyre::eyre;
use flume::RecvTimeoutError;
use std::{sync::Arc, time::Duration};

use crate::daemon_connection::DaemonChannel;

pub fn init(
    node_id: NodeId,
    tx: flume::Sender<EventItem>,
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
        ack_channel: flume::Sender<()>,
    },
    FatalError(eyre::Report),
    TimeoutError(eyre::Report),
}

pub struct EventStreamThreadHandle {
    node_id: NodeId,
    handle: flume::Receiver<std::thread::Result<()>>,
}

impl EventStreamThreadHandle {
    fn new(node_id: NodeId, join_handle: std::thread::JoinHandle<()>) -> Self {
        let (tx, rx) = flume::bounded(1);
        std::thread::spawn(move || {
            let _ = tx.send(join_handle.join());
        });
        Self {
            node_id,
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
    tx: flume::Sender<EventItem>,
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
                continue;
            }

            Ok(other) => {
                let err = eyre!("unexpected control reply: {other:?}");
                tracing::warn!("{err:?}");
                continue;
            }
            Err(err) => {
                let err = err.wrap_err("failed to receive incoming event");
                tracing::warn!("{err:?}");
                continue;
            }
        };
        for Timestamped { inner, timestamp } in events {
            if let Err(err) = clock.update_with_timestamp(&timestamp) {
                tracing::warn!("failed to update HLC: {err}");
            }
            if matches!(&inner, NodeEvent::AllInputsClosed) {
                close_tx = true;
            }

            if let Some(tx) = tx.as_ref() {
                let (drop_tx, _drop_rx) = flume::bounded(0);
                match tx.send(EventItem::NodeEvent {
                    event: inner,
                    ack_channel: drop_tx,
                }) {
                    Ok(()) => {}
                    Err(send_error) => {
                        let event = send_error.into_inner();
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
    if let Err(err) = result {
        if let Some(tx) = tx.as_ref() {
            if let Err(flume::SendError(item)) = tx.send(EventItem::FatalError(err)) {
                let err = match item {
                    EventItem::FatalError(err) => err,
                    _ => unreachable!(),
                };
                tracing::error!("failed to report fatal EventStream error: {err:?}");
            }
        } else {
            tracing::error!("received error event after `tx` was closed: {err:?}");
        }
    }
}
