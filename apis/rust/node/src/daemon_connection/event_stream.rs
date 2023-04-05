use dora_core::{
    config::NodeId,
    daemon_messages::{self, DaemonReply, DaemonRequest, DataflowId, DropToken, NodeEvent},
};
use eyre::{eyre, Context};
use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use crate::{event::Data, Event, MappedInputData};

use super::{DaemonChannel, EventStreamThreadHandle};

pub struct EventStream {
    node_id: NodeId,
    receiver: flume::Receiver<EventItem>,
    _thread_handle: Arc<EventStreamThreadHandle>,
}

impl EventStream {
    pub(crate) fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        mut channel: DaemonChannel,
    ) -> eyre::Result<(
        Self,
        Arc<EventStreamThreadHandle>,
        flume::Receiver<DropToken>,
    )> {
        channel.register(dataflow_id, node_id.clone())?;

        channel
            .request(&DaemonRequest::Subscribe)
            .map_err(|e| eyre!(e))
            .wrap_err("failed to create subscription with dora-daemon")?;

        let (tx, rx) = flume::bounded(0);
        let node_id_cloned = node_id.clone();
        let (finished_drop_tokens, finished_drop_tokens_rx) = flume::unbounded();

        let join_handle = std::thread::spawn(|| {
            event_stream_loop(node_id_cloned, tx, channel, finished_drop_tokens)
        });
        let thread_handle = EventStreamThreadHandle::new(join_handle);

        Ok((
            EventStream {
                node_id: node_id.clone(),
                receiver: rx,
                _thread_handle: thread_handle.clone(),
            },
            thread_handle,
            finished_drop_tokens_rx,
        ))
    }

    pub fn recv(&mut self) -> Option<Event> {
        let event = self.receiver.recv();
        self.recv_common(event)
    }

    pub async fn recv_async(&mut self) -> Option<Event> {
        let event = self.receiver.recv_async().await;
        self.recv_common(event)
    }

    #[tracing::instrument(skip(self), fields(%self.node_id))]
    fn recv_common(&mut self, event: Result<EventItem, flume::RecvError>) -> Option<Event> {
        let event = match event {
            Ok(event) => event,
            Err(flume::RecvError::Disconnected) => {
                tracing::trace!("event channel disconnected");
                return None;
            }
        };
        let event = match event {
            EventItem::NodeEvent { event, ack_channel } => match event {
                NodeEvent::Stop => Event::Stop,
                NodeEvent::Reload { operator_id } => Event::Reload { operator_id },
                NodeEvent::InputClosed { id } => Event::InputClosed { id },
                NodeEvent::Input { id, metadata, data } => {
                    let data = match data {
                        None => Ok(None),
                        Some(daemon_messages::Data::Vec(v)) => Ok(Some(Data::Vec(v))),
                        Some(daemon_messages::Data::SharedMemory {
                            shared_memory_id,
                            len,
                            drop_token: _, // handled in `event_stream_loop`
                        }) => unsafe {
                            MappedInputData::map(&shared_memory_id, len).map(|data| {
                                Some(Data::SharedMemory {
                                    data,
                                    _drop: ack_channel,
                                })
                            })
                        },
                    };
                    match data {
                        Ok(data) => Event::Input { id, metadata, data },
                        Err(err) => Event::Error(format!("{err:?}")),
                    }
                }
                NodeEvent::AllInputsClosed => {
                    let err = eyre!(
                        "received `AllInputsClosed` event, which should be handled by background task"
                    );
                    tracing::error!("{err:?}");
                    Event::Error(err.wrap_err("internal error").to_string())
                }
                NodeEvent::OutputDropped { .. } => {
                    let err = eyre!(
                        "received OutputDrop event, which should be handled by background task"
                    );
                    tracing::error!("{err:?}");
                    Event::Error(err.wrap_err("internal error").to_string())
                }
            },
            EventItem::FatalError(err) => {
                Event::Error(format!("fatal event stream error: {err:?}"))
            }
        };

        Some(event)
    }
}

#[tracing::instrument(skip(tx, channel, finished_drop_tokens))]
fn event_stream_loop(
    node_id: NodeId,
    tx: flume::Sender<EventItem>,
    mut channel: DaemonChannel,
    finished_drop_tokens: flume::Sender<DropToken>,
) {
    let mut tx = Some(tx);
    let mut pending_drop_tokens: Vec<(DropToken, flume::Receiver<()>, Instant, u64)> = Vec::new();
    let mut drop_tokens = Vec::new();

    let result = 'outer: loop {
        if let Err(err) = handle_pending_drop_tokens(&mut pending_drop_tokens, &mut drop_tokens) {
            break 'outer Err(err);
        }

        let daemon_request = DaemonRequest::NextEvent {
            drop_tokens: std::mem::take(&mut drop_tokens),
        };
        let events = match channel.request(&daemon_request) {
            Ok(DaemonReply::NextEvents(events)) if events.is_empty() => {
                tracing::debug!("Event stream closed for node ID `{node_id}`");
                break Ok(());
            }
            Ok(DaemonReply::NextEvents(events)) => events,
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
        for event in events {
            let drop_token = match &event {
                NodeEvent::Input {
                    data: Some(data), ..
                } => data.drop_token(),
                NodeEvent::AllInputsClosed => {
                    // close the event stream
                    tx = None;
                    // skip this internal event
                    continue;
                }
                NodeEvent::OutputDropped { drop_token } => {
                    if let Err(flume::SendError(token)) = finished_drop_tokens.send(*drop_token) {
                        tracing::error!("failed to report drop_token `{token:?}` to dora node");
                    }
                    // skip this internal event
                    continue;
                }
                _ => None,
            };

            if let Some(tx) = tx.as_ref() {
                let (drop_tx, drop_rx) = flume::bounded(0);
                match tx.send(EventItem::NodeEvent {
                    event,
                    ack_channel: drop_tx,
                }) {
                    Ok(()) => {}
                    Err(send_error) => {
                        let event = send_error.into_inner();
                        tracing::debug!(
                            "event channel was closed already, could no forward `{event:?}`"
                        );
                        if finished_drop_tokens.is_disconnected() {
                            // both the event stream and the dora node were dropped
                            // -> break from the `event_stream_loop`
                            break 'outer Ok(());
                        }
                    }
                }

                if let Some(token) = drop_token {
                    pending_drop_tokens.push((token, drop_rx, Instant::now(), 1));
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
        report_drop_tokens(&mut drop_tokens, &mut channel)?;

        let mut still_pending = Vec::new();
        for (token, rx, since, _) in pending_drop_tokens.drain(..) {
            match rx.recv_timeout(Duration::from_millis(100)) {
                Ok(()) => return Err(eyre!("Node API should not send anything on ACK channel")),
                Err(flume::RecvTimeoutError::Disconnected) => {
                    // the event was dropped -> add the drop token to the list
                    drop_tokens.push(token);
                }
                Err(flume::RecvTimeoutError::Timeout) => {
                    let duration = Duration::from_secs(30);
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
        tracing::debug!("waiting for drop for {} events", still_pending.len());
        pending_drop_tokens = still_pending;
    }

    Ok(())
}

fn report_drop_tokens(
    drop_tokens: &mut Vec<DropToken>,
    channel: &mut DaemonChannel,
) -> Result<(), eyre::ErrReport> {
    if drop_tokens.is_empty() {
        return Ok(());
    }
    let daemon_request = DaemonRequest::ReportDropTokens {
        drop_tokens: std::mem::take(drop_tokens),
    };
    let reply = channel.request(&daemon_request)?;
    match reply {
        dora_core::daemon_messages::DaemonReply::Empty => Ok(()),
        other => Err(eyre!("unexpected ReportDropTokens reply: {other:?}")),
    }
}

#[derive(Debug)]
enum EventItem {
    NodeEvent {
        event: NodeEvent,
        ack_channel: flume::Sender<()>,
    },
    FatalError(eyre::Report),
}
