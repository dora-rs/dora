use crate::{shared_mem_handler, DaemonNodeEvent, Event};
use dora_core::{
    config::NodeId,
    daemon_messages::{
        DaemonCommunication, DaemonCommunicationConfig, DaemonReply, DaemonRequest, DataflowId,
        DropEvent, NodeEvent,
    },
};
use eyre::{eyre, Context};
use shared_memory_server::{ShmemConf, ShmemServer};
use std::{collections::VecDeque, net::Ipv4Addr};
use tokio::{
    net::TcpListener,
    sync::{mpsc, oneshot},
};

// TODO unify and avoid duplication;
pub mod shmem;
pub mod tcp;

pub async fn spawn_listener_loop(
    dataflow_id: &DataflowId,
    node_id: &NodeId,
    daemon_tx: &mpsc::Sender<Event>,
    shmem_handler_tx: &flume::Sender<shared_mem_handler::NodeEvent>,
    config: DaemonCommunicationConfig,
) -> eyre::Result<DaemonCommunication> {
    match config {
        DaemonCommunicationConfig::Tcp => {
            let localhost = Ipv4Addr::new(127, 0, 0, 1);
            let socket = match TcpListener::bind((localhost, 0)).await {
                Ok(socket) => socket,
                Err(err) => {
                    return Err(
                        eyre::Report::new(err).wrap_err("failed to create local TCP listener")
                    )
                }
            };
            let socket_addr = socket
                .local_addr()
                .wrap_err("failed to get local addr of socket")?;

            let event_loop_node_id = format!("{dataflow_id}/{node_id}");
            let daemon_tx = daemon_tx.clone();
            let shmem_handler_tx = shmem_handler_tx.clone();
            tokio::spawn(async move {
                tcp::listener_loop(socket, daemon_tx, shmem_handler_tx).await;
                tracing::debug!("event listener loop finished for `{event_loop_node_id}`");
            });

            Ok(DaemonCommunication::Tcp { socket_addr })
        }
        DaemonCommunicationConfig::Shmem => {
            let daemon_control_region = ShmemConf::new()
                .size(4096)
                .create()
                .wrap_err("failed to allocate daemon_control_region")?;
            let daemon_events_region = ShmemConf::new()
                .size(4096)
                .create()
                .wrap_err("failed to allocate daemon_events_region")?;
            let daemon_control_region_id = daemon_control_region.get_os_id().to_owned();
            let daemon_events_region_id = daemon_events_region.get_os_id().to_owned();

            {
                let server = unsafe { ShmemServer::new(daemon_control_region) }
                    .wrap_err("failed to create control server")?;
                let daemon_tx = daemon_tx.clone();
                let shmem_handler_tx = shmem_handler_tx.clone();
                tokio::spawn(shmem::listener_loop(server, daemon_tx, shmem_handler_tx));
            }

            {
                let server = unsafe { ShmemServer::new(daemon_events_region) }
                    .wrap_err("failed to create events server")?;
                let event_loop_node_id = format!("{dataflow_id}/{node_id}");
                let daemon_tx = daemon_tx.clone();
                let shmem_handler_tx = shmem_handler_tx.clone();
                tokio::task::spawn(async move {
                    shmem::listener_loop(server, daemon_tx, shmem_handler_tx).await;
                    tracing::debug!("event listener loop finished for `{event_loop_node_id}`");
                });
            }

            Ok(DaemonCommunication::Shmem {
                daemon_control_region_id,
                daemon_events_region_id,
            })
        }
    }
}

struct Listener<C> {
    dataflow_id: DataflowId,
    node_id: NodeId,
    daemon_tx: mpsc::Sender<Event>,
    shmem_handler_tx: flume::Sender<shared_mem_handler::NodeEvent>,
    subscribed_events: Option<flume::Receiver<NodeEvent>>,
    max_queue_len: usize,
    queue: VecDeque<NodeEvent>,
    connection: C,
}

impl<C> Listener<C>
where
    C: Connection,
{
    pub(crate) async fn run(
        mut connection: C,
        daemon_tx: mpsc::Sender<Event>,
        shmem_handler_tx: flume::Sender<shared_mem_handler::NodeEvent>,
    ) {
        // receive the first message
        let message = match connection
            .receive_message()
            .await
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
                match connection
                    .send_reply(reply)
                    .await
                    .wrap_err("failed to send register reply")
                {
                    Ok(()) => {
                        let mut listener = Listener {
                            dataflow_id,
                            node_id,
                            connection,
                            daemon_tx,
                            shmem_handler_tx,
                            subscribed_events: None,
                            max_queue_len: 10, // TODO: make this configurable
                            queue: VecDeque::new(),
                        };
                        match listener.run_inner().await.wrap_err("listener failed") {
                            Ok(()) => {}
                            Err(err) => tracing::error!("{err:?}"),
                        }
                    }
                    Err(err) => {
                        tracing::warn!("{err:?}");
                    }
                }
            }
            other => {
                tracing::warn!("expected register message, got `{other:?}`");
                let reply = DaemonReply::Result(Err("must send register message first".into()));
                if let Err(err) = connection
                    .send_reply(reply)
                    .await
                    .wrap_err("failed to send  reply")
                {
                    tracing::warn!("{err:?}");
                }
            }
        }
    }

    async fn run_inner(&mut self) -> eyre::Result<()> {
        loop {
            // receive the next node message
            let message = match self
                .connection
                .receive_message()
                .await
                .wrap_err("failed to receive DaemonRequest")
            {
                Ok(Some(m)) => m,
                Ok(None) => {
                    tracing::debug!(
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
            self.handle_events().await?;

            self.handle_message(message).await?;
        }
        Ok(())
    }

    async fn handle_events(&mut self) -> eyre::Result<()> {
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
            if drop_n > 0 {
                self.drop_oldest_inputs(drop_n).await?;
            }
        }
        Ok(())
    }

    async fn drop_oldest_inputs(&mut self, number: usize) -> Result<(), eyre::ErrReport> {
        tracing::debug!("dropping {number} inputs because event queue is too full");
        let mut drop_tokens = Vec::new();
        for i in 0..number {
            // find index of oldest input event
            let index = self
                .queue
                .iter()
                .position(|e| matches!(e, NodeEvent::Input { .. }))
                .unwrap_or_else(|| panic!("no input event found in drop iteration {i}"));

            // remove that event
            if let Some(NodeEvent::Input {
                data: Some(data), ..
            }) = self.queue.remove(index)
            {
                if let Some(drop_token) = data.drop_token() {
                    drop_tokens.push(drop_token);
                }
            }
        }
        self.report_drop_tokens(drop_tokens).await?;
        Ok(())
    }

    #[tracing::instrument(skip(self), fields(%self.dataflow_id, %self.node_id))]
    async fn handle_message(&mut self, message: DaemonRequest) -> eyre::Result<()> {
        match message {
            DaemonRequest::Register { .. } => {
                let reply = DaemonReply::Result(Err("unexpected register message".into()));
                self.send_reply(reply)
                    .await
                    .wrap_err("failed to send register reply")?;
            }
            DaemonRequest::Stopped => self.process_daemon_event(DaemonNodeEvent::Stopped).await?,
            DaemonRequest::CloseOutputs(outputs) => {
                self.process_daemon_event(DaemonNodeEvent::CloseOutputs(outputs))
                    .await?
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
                self.send_shared_memory_event(event).await?;
                let reply = reply
                    .await
                    .wrap_err("failed to receive prepare output reply")?;
                // tracing::debug!("prepare latency: {:?}", start.elapsed()?);
                self.send_reply(reply)
                    .await
                    .wrap_err("failed to send PrepareOutputMessage reply")?;
            }
            DaemonRequest::SendPreparedMessage { id } => {
                let (reply_sender, reply) = oneshot::channel();
                let event = shared_mem_handler::NodeEvent::SendPreparedMessage { id, reply_sender };
                self.send_shared_memory_event(event).await?;
                self.send_reply(
                    reply
                        .await
                        .wrap_err("failed to receive SendPreparedMessage reply")?,
                )
                .await?;
            }
            DaemonRequest::SendMessage {
                output_id,
                metadata,
                data,
            } => {
                // let elapsed = metadata.timestamp().get_time().to_system_time().elapsed()?;
                // tracing::debug!("listener SendEmptyMessage: {elapsed:?}");
                let event = crate::Event::ShmemHandler(crate::ShmemHandlerEvent::SendOut {
                    dataflow_id: self.dataflow_id,
                    node_id: self.node_id.clone(),
                    output_id,
                    metadata,
                    data: data.into(),
                });
                let result = self
                    .send_daemon_event(event)
                    .await
                    .map_err(|_| "failed to receive send_empty_message reply".to_owned());
                self.send_reply(DaemonReply::Result(result))
                    .await
                    .wrap_err("failed to send SendEmptyMessage reply")?;
            }
            DaemonRequest::Subscribe => {
                let (tx, rx) = flume::bounded(100);
                self.process_daemon_event(DaemonNodeEvent::Subscribe { event_sender: tx })
                    .await?;
                self.subscribed_events = Some(rx);
            }
            DaemonRequest::NextEvent { drop_tokens } => {
                self.report_drop_tokens(drop_tokens).await?;

                // try to take the latest queued event first
                let queued_event = self.queue.pop_front().map(DaemonReply::NodeEvent);
                let reply = match queued_event {
                    Some(reply) => reply,
                    None => {
                        match self.subscribed_events.as_mut() {
                            // wait for next event
                            Some(events) => match events.recv_async().await {
                                Ok(event) => DaemonReply::NodeEvent(event),
                                Err(flume::RecvError::Disconnected) => DaemonReply::Closed,
                            },
                            None => DaemonReply::Result(Err(
                                "Ignoring event request because no subscribe \
                                    message was sent yet"
                                    .into(),
                            )),
                        }
                    }
                };

                self.send_reply(reply)
                    .await
                    .wrap_err("failed to send NextEvent reply")?;
            }
        }
        Ok(())
    }

    async fn report_drop_tokens(
        &mut self,
        drop_tokens: Vec<dora_core::daemon_messages::DropToken>,
    ) -> eyre::Result<()> {
        if !drop_tokens.is_empty() {
            let drop_event = shared_mem_handler::NodeEvent::Drop(DropEvent {
                tokens: drop_tokens,
            });
            self.send_shared_memory_event(drop_event).await?;
        }
        Ok(())
    }

    async fn process_daemon_event(&mut self, event: DaemonNodeEvent) -> eyre::Result<()> {
        // send NodeEvent to daemon main loop
        let (reply_tx, reply) = oneshot::channel();
        let event = Event::Node {
            dataflow_id: self.dataflow_id,
            node_id: self.node_id.clone(),
            event,
            reply_sender: reply_tx,
        };
        self.daemon_tx
            .send(event)
            .await
            .map_err(|_| eyre!("failed to send event to daemon"))?;
        let reply = reply
            .await
            .map_err(|_| eyre!("failed to receive reply from daemon"))?;
        self.send_reply(reply).await?;
        Ok(())
    }

    async fn send_reply(&mut self, reply: DaemonReply) -> eyre::Result<()> {
        self.connection
            .send_reply(reply)
            .await
            .wrap_err_with(|| format!("failed to send reply to node `{}`", self.node_id))
    }

    async fn send_shared_memory_event(
        &self,
        event: shared_mem_handler::NodeEvent,
    ) -> eyre::Result<()> {
        self.shmem_handler_tx
            .send_async(event)
            .await
            .map_err(|_| eyre!("failed to send event to shared_mem_handler"))
    }

    async fn send_daemon_event(&self, event: crate::Event) -> eyre::Result<()> {
        self.daemon_tx
            .send(event)
            .await
            .map_err(|_| eyre!("failed to send event to daemon"))
    }
}

#[async_trait::async_trait]
trait Connection {
    async fn receive_message(&mut self) -> eyre::Result<Option<DaemonRequest>>;
    async fn send_reply(&mut self, message: DaemonReply) -> eyre::Result<()>;
}
