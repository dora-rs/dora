use crate::{DaemonNodeEvent, Event};
use dora_core::{
    daemon_messages::{self, DropEvent},
    shared_memory::ShmemServer,
};
use eyre::Context;
use tokio::sync::{mpsc, oneshot};

#[tracing::instrument(skip(channel, events_tx))]
pub fn listener_loop(mut channel: ShmemServer, events_tx: mpsc::Sender<Event>) {
    let mut id = None;
    let mut events = None;
    loop {
        // receive the next message
        let message = match channel.listen().wrap_err("failed to receive node message") {
            Ok(Some(m)) => m,
            Ok(None) => {
                tracing::info!("control channel disconnected: {id:?}");
                break;
            } // disconnected
            Err(err) => {
                tracing::warn!("{err:?}");
                continue;
            }
        };

        // handle the message and translate it to a NodeEvent
        let node_event = match message {
            daemon_messages::ControlRequest::Register {
                dataflow_id,
                node_id,
            } => {
                id = Some((dataflow_id, node_id));

                let reply = daemon_messages::ControlReply::Result(Ok(()));

                match channel.send_reply(&reply) {
                    Ok(()) => continue, // don't trigger an event for register calls
                    Err(err) => {
                        tracing::warn!("{err:?}");
                        break; // close connection
                    }
                }
            }
            daemon_messages::ControlRequest::Stopped => DaemonNodeEvent::Stopped,
            daemon_messages::ControlRequest::PrepareOutputMessage {
                output_id,
                metadata,
                data_len,
            } => DaemonNodeEvent::PrepareOutputMessage {
                output_id,
                metadata,
                data_len,
            },
            daemon_messages::ControlRequest::SendOutMessage { id } => {
                DaemonNodeEvent::SendOutMessage { id }
            }
            daemon_messages::ControlRequest::Subscribe {
                dataflow_id,
                node_id,
            } => {
                let (tx, rx) = flume::bounded(10);

                id = Some((dataflow_id, node_id));
                events = Some(rx);

                DaemonNodeEvent::Subscribe { event_sender: tx }
            }
            daemon_messages::ControlRequest::NextEvent { drop_tokens } => {
                let drop_event = Event::Drop(DropEvent {
                    tokens: drop_tokens,
                });
                if events_tx.blocking_send(drop_event).is_err() {
                    break;
                }

                let Some(events) = events.as_mut() else {
                    tracing::warn!(
                        "Ignoring event request because no subscribe \
                        message was sent yet"
                    );
                    continue;
                };

                let event = match events.recv() {
                    Ok(event) => event,
                    Err(_) => break,
                };

                if let Err(err) = channel.send_reply(&event).wrap_err("failed to send reply") {
                    tracing::error!("{err:?}");
                    break;
                }

                continue; // don't trigger an event (apart from the drop event sent above)
            }
        };

        let (dataflow_id, node_id) = match &id {
            Some(id) => id.clone(),
            None => {
                tracing::warn!(
                    "Ignoring node event because no register \
                    message was sent yet: {node_event:?}"
                );
                continue;
            }
        };

        // send NodeEvent to daemon main loop
        let (reply_tx, reply) = oneshot::channel();
        let event = Event::Node {
            dataflow_id,
            node_id,
            event: node_event,
            reply_sender: reply_tx,
        };
        let Ok(()) = events_tx.blocking_send(event) else {
            break;
        };

        // wait for reply and send it out
        let Ok(reply) = reply.blocking_recv() else {
            break; // main loop exited
        };
        if let Err(err) = channel.send_reply(&reply).wrap_err("failed to send reply") {
            tracing::error!("{err:?}");
            break;
        }
    }
}
