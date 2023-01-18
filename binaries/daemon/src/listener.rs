use crate::{shared_mem_handler, DaemonNodeEvent, Event};
use dora_core::{
    config::NodeId,
    daemon_messages::{DaemonReply, DaemonRequest, DataflowId, DropEvent, NodeEvent},
    shared_memory::ShmemServer,
};
use eyre::{eyre, Context};
use tokio::sync::{mpsc, oneshot};

#[tracing::instrument(skip(server, daemon_tx, shmem_handler_tx))]
pub fn listener_loop(
    mut server: ShmemServer<DaemonRequest, DaemonReply>,
    daemon_tx: mpsc::Sender<Event>,
    shmem_handler_tx: flume::Sender<shared_mem_handler::NodeEvent>,
) {
    // receive the first message
    let message = match server
        .listen()
        .wrap_err("failed to receive register message")
    {
        Ok(Some(m)) => m,
        Ok(None) => {
            tracing::info!("channel disconnected before register message");
            return;
        } // disconnected
        Err(err) => {
            tracing::info!("{err:?}");
            return;
        }
    };

    match message {
        DaemonRequest::Register {
            dataflow_id,
            node_id,
        } => {
            let reply = DaemonReply::Result(Ok(()));
            match server
                .send_reply(&reply)
                .wrap_err("failed to send register reply")
            {
                Ok(()) => {
                    let mut listener = Listener {
                        dataflow_id,
                        node_id,
                        server,
                        daemon_tx,
                        shmem_handler_tx,
                        subscribed_events: None,
                    };
                    match listener.run().wrap_err("listener failed") {
                        Ok(()) => {}
                        Err(err) => tracing::error!("{err:?}"),
                    }
                }
                Err(err) => {
                    tracing::warn!("{err:?}");
                }
            }
        }
        _ => {
            let reply = DaemonReply::Result(Err("must send register message first".into()));
            if let Err(err) = server.send_reply(&reply).wrap_err("failed to send  reply") {
                tracing::warn!("{err:?}");
            }
        }
    }
}

struct Listener {
    dataflow_id: DataflowId,
    node_id: NodeId,
    server: ShmemServer<DaemonRequest, DaemonReply>,
    daemon_tx: mpsc::Sender<Event>,
    shmem_handler_tx: flume::Sender<shared_mem_handler::NodeEvent>,
    subscribed_events: Option<flume::Receiver<NodeEvent>>,
}

impl Listener {
    fn run(&mut self) -> eyre::Result<()> {
        loop {
            // receive the next message
            let message = match self
                .server
                .listen()
                .wrap_err("failed to receive DaemonRequest")
            {
                Ok(Some(m)) => m,
                Ok(None) => {
                    tracing::info!(
                        "channel disconnected: {}/{}",
                        self.dataflow_id,
                        self.node_id
                    );
                    break;
                } // disconnected
                Err(err) => {
                    tracing::warn!("{err:?}");
                    continue;
                }
            };
            self.handle_message(message)?;
        }
        Ok(())
    }

    fn handle_message(&mut self, message: DaemonRequest) -> eyre::Result<()> {
        match message {
            DaemonRequest::Register { .. } => {
                let reply = DaemonReply::Result(Err("unexpected register message".into()));
                self.send_reply(&reply)?;
            }
            DaemonRequest::Stopped => self.process_daemon_event(DaemonNodeEvent::Stopped)?,
            DaemonRequest::PrepareOutputMessage {
                output_id,
                metadata,
                data_len,
            } => {
                let (reply_sender, reply) = oneshot::channel();
                let event = shared_mem_handler::NodeEvent::PrepareOutputMessage {
                    dataflow_id: self.dataflow_id,
                    node_id: self.node_id.clone(),
                    output_id,
                    metadata,
                    data_len,
                    reply_sender,
                };
                self.send_shared_memory_event(event)?;
                self.send_reply(
                    &reply
                        .blocking_recv()
                        .wrap_err("failed to receive prepare output reply")?,
                )?;
            }
            DaemonRequest::SendOutMessage { id } => {
                let (reply_sender, reply) = oneshot::channel();
                let event = shared_mem_handler::NodeEvent::SendOutMessage { id, reply_sender };
                self.send_shared_memory_event(event)?;
                self.send_reply(
                    &reply
                        .blocking_recv()
                        .wrap_err("failed to receive send output reply")?,
                )?;
            }
            DaemonRequest::Subscribe => {
                let (tx, rx) = flume::bounded(10);
                self.process_daemon_event(DaemonNodeEvent::Subscribe { event_sender: tx })?;
                self.subscribed_events = Some(rx);
            }
            DaemonRequest::NextEvent { drop_tokens } => {
                let drop_event = shared_mem_handler::NodeEvent::Drop(DropEvent {
                    tokens: drop_tokens,
                });
                self.send_shared_memory_event(drop_event)?;

                let reply = match self.subscribed_events.as_mut() {
                    Some(events) => match events.recv() {
                        Ok(event) => DaemonReply::NodeEvent(event),
                        Err(flume::RecvError::Disconnected) => DaemonReply::Closed,
                    },
                    None => {
                        DaemonReply::Result(Err("Ignoring event request because no subscribe \
                            message was sent yet"
                            .into()))
                    }
                };

                self.send_reply(&reply)?;
            }
        }
        Ok(())
    }

    fn process_daemon_event(&mut self, event: DaemonNodeEvent) -> eyre::Result<()> {
        // send NodeEvent to daemon main loop
        let (reply_tx, reply) = oneshot::channel();
        let event = Event::Node {
            dataflow_id: self.dataflow_id.clone(),
            node_id: self.node_id.clone(),
            event,
            reply_sender: reply_tx,
        };
        self.daemon_tx
            .blocking_send(event)
            .map_err(|_| eyre!("failed to send event to daemon"))?;
        let reply = reply
            .blocking_recv()
            .map_err(|_| eyre!("failed to receive reply from daemon"))?;
        self.send_reply(&reply)?;
        Ok(())
    }

    fn send_reply(&mut self, reply: &DaemonReply) -> eyre::Result<()> {
        self.server
            .send_reply(&reply)
            .wrap_err("failed to send reply to node")
    }

    fn send_shared_memory_event(&self, event: shared_mem_handler::NodeEvent) -> eyre::Result<()> {
        self.shmem_handler_tx
            .send(event)
            .map_err(|_| eyre!("failed to send event to shared_mem_handler"))
    }
}
