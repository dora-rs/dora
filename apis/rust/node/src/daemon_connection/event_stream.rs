use dora_core::{
    config::NodeId,
    daemon_messages::{DaemonReply, DaemonRequest, DataflowId, NodeEvent},
};
use eyre::{eyre, Context};
use std::{sync::Arc, time::Duration};

use crate::{event::Data, Event, MappedInputData};

use super::{DaemonChannel, EventStreamThreadHandle};

pub struct EventStream {
    receiver: flume::Receiver<EventItem>,
    _thread_handle: Arc<EventStreamThreadHandle>,
}

impl EventStream {
    pub(crate) fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        mut channel: DaemonChannel,
    ) -> eyre::Result<(Self, Arc<EventStreamThreadHandle>)> {
        channel.register(dataflow_id, node_id.clone())?;

        channel
            .request(&DaemonRequest::Subscribe)
            .map_err(|e| eyre!(e))
            .wrap_err("failed to create subscription with dora-daemon")?;

        let (tx, rx) = flume::bounded(0);
        let mut drop_tokens = Vec::new();
        let node_id = node_id.clone();
        let join_handle = std::thread::spawn(move || {
            let result = 'outer: loop {
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
                        NodeEvent::Stop
                        | NodeEvent::InputClosed { .. }
                        | NodeEvent::Input { data: None, .. } => None,
                    };

                    let (drop_tx, drop_rx) = std::sync::mpsc::channel();
                    match tx.send(EventItem::NodeEvent {
                        event,
                        ack_channel: drop_tx,
                    }) {
                        Ok(()) => {}
                        Err(_) => {
                            // receiving end of channel was closed
                            break 'outer Ok(());
                        }
                    }

                    let timeout = Duration::from_secs(30);
                    match drop_rx.recv_timeout(timeout) {
                        Ok(()) => {
                            break 'outer Err(eyre!(
                                "Node API should not send anything on ACK channel"
                            ))
                        }
                        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                            tracing::warn!("timeout: event was not dropped after {timeout:?}");
                        }
                        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {} // expected result
                    }

                    if let Some(token) = drop_token {
                        drop_tokens.push(token);
                    }
                }
            };
            if let Err(err) = result {
                if let Err(flume::SendError(item)) = tx.send(EventItem::FatalError(err)) {
                    let err = match item {
                        EventItem::FatalError(err) => err,
                        _ => unreachable!(),
                    };
                    tracing::error!("failed to report fatal EventStream error: {err:?}");
                }
            }
        });

        let thread_handle = EventStreamThreadHandle::new(join_handle);

        Ok((
            EventStream {
                receiver: rx,
                _thread_handle: thread_handle.clone(),
            },
            thread_handle,
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

    fn recv_common(&mut self, event: Result<EventItem, flume::RecvError>) -> Option<Event> {
        let event = match event {
            Ok(event) => event,
            Err(flume::RecvError::Disconnected) => {
                tracing::info!("event channel disconnected");
                return None;
            }
        };
        let event = match event {
            EventItem::NodeEvent { event, ack_channel } => match event {
                NodeEvent::Stop => Event::Stop,
                NodeEvent::InputClosed { id } => Event::InputClosed { id },
                NodeEvent::Input { id, metadata, data } => {
                    let data = data
                        .map(|data| match data {
                            dora_core::daemon_messages::InputData::Vec(d) => Ok(Data::Vec(d)),
                            dora_core::daemon_messages::InputData::SharedMemory(d) => unsafe {
                                MappedInputData::map(&d.shared_memory_id, d.len).map(|data| {
                                    Data::SharedMemory {
                                        data,
                                        _drop: ack_channel,
                                    }
                                })
                            },
                        })
                        .transpose();
                    match data {
                        Ok(data) => Event::Input { id, metadata, data },
                        Err(err) => Event::Error(format!("{err:?}")),
                    }
                }
            },
            EventItem::FatalError(err) => {
                Event::Error(format!("fatal event stream error: {err:?}"))
            }
        };

        Some(event)
    }
}

enum EventItem {
    NodeEvent {
        event: NodeEvent,
        ack_channel: std::sync::mpsc::Sender<()>,
    },
    FatalError(eyre::Report),
}
