use std::collections::VecDeque;

use crate::{shared_mem_handler, DaemonNodeEvent, Event};
use dora_core::{
    config::NodeId,
    daemon_messages::{DaemonReply, DaemonRequest, DataflowId, DropEvent, NodeEvent},
};
use eyre::{eyre, Context};
use shared_memory_server::ShmemServer;
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
                        max_queue_len: 10, // TODO: make this configurable
                        queue: VecDeque::new(),
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
    max_queue_len: usize,
    queue: VecDeque<NodeEvent>,
}

impl Listener {
    fn run(&mut self) -> eyre::Result<()> {
        loop {
            // receive the next node message
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

            // handle incoming events
            self.handle_events()?;

            self.handle_message(message)?;
        }
        Ok(())
    }

    fn handle_events(&mut self) -> eyre::Result<()> {
        if let Some(events) = &mut self.subscribed_events {
            while let Ok(event) = events.try_recv() {
                self.queue.push_back(event);
            }

            // drop oldest input events to maintain max queue length queue
            let input_event_count = self
                .queue
                .iter()
                .filter(|e| matches!(e, NodeEvent::Input { .. }))
                .count();
            let drop_n = input_event_count.saturating_sub(self.max_queue_len);
            self.drop_oldest_inputs(drop_n)?;
        }
        Ok(())
    }

    fn drop_oldest_inputs(&mut self, number: usize) -> Result<(), eyre::ErrReport> {
        let mut drop_tokens = Vec::new();
        for i in 0..number {
            // find index of oldest input event
            let index = self
                .queue
                .iter()
                .position(|e| matches!(e, NodeEvent::Input { .. }))
                .expect(&format!("no input event found in drop iteration {i}"));

            // remove that event
            if let Some(event) = self.queue.remove(index) {
                if let NodeEvent::Input {
                    data: Some(data), ..
                } = event
                {
                    drop_tokens.push(data.drop_token);
                }
            }
        }
        self.report_drop_tokens(drop_tokens)?;
        Ok(())
    }

    fn handle_message(&mut self, message: DaemonRequest) -> eyre::Result<()> {
        match message {
            DaemonRequest::Register { .. } => {
                let reply = DaemonReply::Result(Err("unexpected register message".into()));
                self.send_reply(&reply)?;
            }
            DaemonRequest::Stopped => self.process_daemon_event(DaemonNodeEvent::Stopped)?,
            DaemonRequest::CloseOutputs(outputs) => {
                self.process_daemon_event(DaemonNodeEvent::CloseOutputs(outputs))?
            }
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
                let reply = reply
                    .blocking_recv()
                    .wrap_err("failed to receive prepare output reply")?;
                // tracing::debug!("prepare latency: {:?}", start.elapsed()?);
                self.send_reply(&reply)?;
            }
            DaemonRequest::SendPreparedMessage { id } => {
                let (reply_sender, reply) = oneshot::channel();
                let event = shared_mem_handler::NodeEvent::SendPreparedMessage { id, reply_sender };
                self.send_shared_memory_event(event)?;
                self.send_reply(
                    &reply
                        .blocking_recv()
                        .wrap_err("failed to receive send output reply")?,
                )?;
            }
            DaemonRequest::SendEmptyMessage {
                output_id,
                metadata,
            } => {
                // let elapsed = metadata.timestamp().get_time().to_system_time().elapsed()?;
                // tracing::debug!("listener SendEmptyMessage: {elapsed:?}");
                let event = crate::Event::ShmemHandler(crate::ShmemHandlerEvent::SendOut {
                    dataflow_id: self.dataflow_id,
                    node_id: self.node_id.clone(),
                    output_id,
                    metadata,
                    data: None,
                });
                let result = self
                    .send_daemon_event(event)
                    .map_err(|_| "failed to receive send_empty_message reply".to_owned());
                self.send_reply(&DaemonReply::Result(result))?;
            }
            DaemonRequest::Subscribe => {
                let (tx, rx) = flume::bounded(100);
                self.process_daemon_event(DaemonNodeEvent::Subscribe { event_sender: tx })?;
                self.subscribed_events = Some(rx);
            }
            DaemonRequest::NextEvent { drop_tokens } => {
                self.report_drop_tokens(drop_tokens)?;

                // try to take the latest queued event first
                let queued_event = self.queue.pop_front().map(DaemonReply::NodeEvent);
                let reply = queued_event.unwrap_or_else(|| {
                    match self.subscribed_events.as_mut() {
                        // wait for next event
                        Some(events) => match events.recv() {
                            Ok(event) => DaemonReply::NodeEvent(event),
                            Err(flume::RecvError::Disconnected) => DaemonReply::Closed,
                        },
                        None => {
                            DaemonReply::Result(Err("Ignoring event request because no subscribe \
                                message was sent yet"
                                .into()))
                        }
                    }
                });

                self.send_reply(&reply)?;
            }
        }
        Ok(())
    }

    fn report_drop_tokens(
        &mut self,
        drop_tokens: Vec<dora_core::daemon_messages::DropToken>,
    ) -> eyre::Result<()> {
        if !drop_tokens.is_empty() {
            let drop_event = shared_mem_handler::NodeEvent::Drop(DropEvent {
                tokens: drop_tokens,
            });
            self.send_shared_memory_event(drop_event)?;
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

    fn send_daemon_event(&self, event: crate::Event) -> eyre::Result<()> {
        self.daemon_tx
            .blocking_send(event)
            .map_err(|_| eyre!("failed to send event to daemon"))
    }
}
