use dora_core::{
    config::NodeId,
    uhlc::{self, Timestamp},
};
use dora_message::{
    common::{DataMessage, DropTokenState, DropTokenStatus},
    daemon_to_node::{DaemonReply, NodeEvent},
    node_to_daemon::{DaemonRequest, DropToken, Timestamped},
};
use eyre::{eyre, Context};
use flume::RecvTimeoutError;
use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use crate::daemon_connection::DaemonChannel;

use super::{event::SharedMemoryData, Event, MappedInputData, RawData};

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
pub enum EventItem {
    NodeEvent { event: super::Event },
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
    #[tracing::instrument(skip(self), fields(node_id = %self.node_id))]
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
    let mut pending_drop_tokens: Vec<(DropToken, flume::Receiver<()>, Instant, u64)> = Vec::new();
    let mut drop_tokens = Vec::new();

    let result = 'outer: loop {
        if let Err(err) = handle_pending_drop_tokens(&mut pending_drop_tokens, &mut drop_tokens) {
            break 'outer Err(err);
        }

        let daemon_request = Timestamped {
            inner: DaemonRequest::NextEvent {
                drop_tokens: std::mem::take(&mut drop_tokens),
            },
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
            Ok(other) => {
                let err = eyre!("unexpected control reply: {other:?}");
                tracing::warn!("{err:?}");
                continue;
            }
            Err(err) => {
                let err = eyre!(err).wrap_err("failed to receive incoming event");
                tracing::warn!("{err:?}");
                continue;
            }
        };
        for Timestamped { inner, timestamp } in events {
            if let Err(err) = clock.update_with_timestamp(&timestamp) {
                tracing::warn!("failed to update HLC: {err}");
            }

            let event = match inner {
                NodeEvent::Stop => Event::Stop,
                NodeEvent::Reload { operator_id } => Event::Reload { operator_id },
                NodeEvent::InputClosed { id } => Event::InputClosed { id },
                NodeEvent::Input { id, metadata, data } => {
                    let data = match data {
                        None => Ok(None),
                        Some(DataMessage::Vec(v)) => Ok(Some(RawData::Vec(v))),
                        Some(DataMessage::SharedMemory {
                            shared_memory_id,
                            len,
                            drop_token,
                        }) => unsafe {
                            let (drop_tx, drop_rx) = flume::bounded(0);
                            let data = MappedInputData::map(&shared_memory_id, len).map(|data| {
                                Some(RawData::SharedMemory(SharedMemoryData {
                                    data,
                                    _drop: drop_tx,
                                }))
                            });
                            drop_tokens.push(DropTokenStatus {
                                token: drop_token,
                                state: DropTokenState::Mapped,
                            });
                            pending_drop_tokens.push((drop_token, drop_rx, Instant::now(), 1));
                            data
                        },
                    };
                    let data = data.and_then(|data| {
                        let raw_data = data.unwrap_or(RawData::Empty);
                        raw_data
                            .into_arrow_array(&metadata.type_info)
                            .map(arrow::array::make_array)
                    });
                    match data {
                        Ok(data) => Event::Input {
                            id,
                            metadata,
                            data: data.into(),
                        },
                        Err(err) => Event::Error(format!("{err:?}")),
                    }
                }
                NodeEvent::AllInputsClosed => {
                    // close the event stream
                    tx = None;
                    // skip this internal event
                    continue;
                }
            };

            if let Some(tx) = tx.as_ref() {
                match tx.send(EventItem::NodeEvent { event }) {
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
                tracing::warn!("dropping event because event `tx` was already closed: `{event:?}`");
            }
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

    if let Err(err) = report_remaining_drop_tokens(
        channel,
        drop_tokens,
        pending_drop_tokens,
        clock.new_timestamp(),
    )
    .context("failed to report remaining drop tokens")
    {
        tracing::warn!("{err:?}");
    }
}

fn handle_pending_drop_tokens(
    pending_drop_tokens: &mut Vec<(DropToken, flume::Receiver<()>, Instant, u64)>,
    drop_tokens: &mut Vec<DropTokenStatus>,
) -> eyre::Result<()> {
    let mut still_pending = Vec::new();
    for (token, rx, since, warn) in pending_drop_tokens.drain(..) {
        match rx.try_recv() {
            Ok(()) => return Err(eyre!("Node API should not send anything on ACK channel")),
            Err(flume::TryRecvError::Disconnected) => {
                // the event was dropped -> add the drop token to the list
                drop_tokens.push(DropTokenStatus {
                    token,
                    state: DropTokenState::Dropped,
                });
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
    mut drop_tokens: Vec<DropTokenStatus>,
    mut pending_drop_tokens: Vec<(DropToken, flume::Receiver<()>, Instant, u64)>,
    timestamp: Timestamp,
) -> eyre::Result<()> {
    while !(pending_drop_tokens.is_empty() && drop_tokens.is_empty()) {
        report_drop_tokens(&mut drop_tokens, &mut channel, timestamp)?;

        let mut still_pending = Vec::new();
        for (token, rx, since, _) in pending_drop_tokens.drain(..) {
            match rx.recv_timeout(Duration::from_millis(100)) {
                Ok(()) => return Err(eyre!("Node API should not send anything on ACK channel")),
                Err(flume::RecvTimeoutError::Disconnected) => {
                    // the event was dropped -> add the drop token to the list
                    drop_tokens.push(DropTokenStatus {
                        token,
                        state: DropTokenState::Dropped,
                    });
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

fn report_drop_tokens(
    drop_tokens: &mut Vec<DropTokenStatus>,
    channel: &mut DaemonChannel,
    timestamp: Timestamp,
) -> Result<(), eyre::ErrReport> {
    if drop_tokens.is_empty() {
        return Ok(());
    }
    let daemon_request = Timestamped {
        inner: DaemonRequest::ReportDropTokens {
            drop_tokens: std::mem::take(drop_tokens),
        },
        timestamp,
    };
    match channel.request(&daemon_request)? {
        DaemonReply::Empty => Ok(()),
        other => Err(eyre!("unexpected ReportDropTokens reply: {other:?}")),
    }
}
