use dora_core::{config::NodeId, uhlc};
use dora_message::{
    daemon_to_node::{DaemonReply, NodeEvent, NodeEventOrUnknown},
    id::DataId,
    metadata::Metadata,
    node_to_daemon::{DaemonRequest, Timestamped},
};
use eyre::eyre;
use flume::RecvTimeoutError;
use std::{sync::Arc, time::Duration};
use zenoh::shm::ZShm;

use crate::daemon_connection::DaemonChannel;

pub fn init(
    node_id: NodeId,
    tx: tokio::sync::mpsc::UnboundedSender<EventItem>,
    channel: DaemonChannel,
    clock: Arc<uhlc::HLC>,
) -> eyre::Result<EventStreamThreadHandle> {
    let node_id_cloned = node_id.clone();
    let join_handle =
        std::thread::spawn(move || event_stream_loop(node_id_cloned, tx, channel, clock));
    Ok(EventStreamThreadHandle::new(join_handle))
}

#[derive(Debug)]
#[allow(clippy::large_enum_variant)]
pub enum EventItem {
    NodeEvent {
        event: NodeEvent,
    },
    /// Zero-copy input delivered via zenoh SHM. The ZShm buffer references
    /// shared memory directly without copying the payload.
    ZenohShmInput {
        id: DataId,
        metadata: Metadata,
        shm: ZShm,
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

            let inner = *inner;

            if matches!(&inner, NodeEvent::AllInputsClosed) {
                close_tx = true;
            }

            if let Some(tx) = tx.as_ref() {
                match tx.send(EventItem::NodeEvent { event: inner }) {
                    Ok(()) => {}
                    Err(send_error) => {
                        let event = send_error.0;
                        tracing::warn!(
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
}
