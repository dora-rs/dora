use crate::{DaemonNodeEvent, Event};
use dora_core::{
    config::{DataId, NodeId},
    daemon_messages::{
        DaemonCommunication, DaemonCommunicationConfig, DaemonReply, DaemonRequest, DataflowId,
        NodeEvent,
    },
};
use eyre::{eyre, Context};
use futures::{
    future::{self, Fuse},
    FutureExt,
};
use shared_memory_server::{ShmemConf, ShmemServer};
use std::{
    collections::{BTreeMap, VecDeque},
    mem,
    net::Ipv4Addr,
};
use tokio::{
    net::TcpListener,
    sync::{
        mpsc::{self, UnboundedReceiver},
        oneshot,
    },
};

// TODO unify and avoid duplication;
pub mod shmem;
pub mod tcp;

pub async fn spawn_listener_loop(
    dataflow_id: &DataflowId,
    node_id: &NodeId,
    daemon_tx: &mpsc::Sender<Event>,
    config: DaemonCommunicationConfig,
    queue_sizes: BTreeMap<DataId, usize>,
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
            tokio::spawn(async move {
                tcp::listener_loop(socket, daemon_tx, queue_sizes).await;
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
                let queue_sizes = queue_sizes.clone();
                tokio::spawn(shmem::listener_loop(server, daemon_tx, queue_sizes));
            }

            {
                let server = unsafe { ShmemServer::new(daemon_events_region) }
                    .wrap_err("failed to create events server")?;
                let event_loop_node_id = format!("{dataflow_id}/{node_id}");
                let daemon_tx = daemon_tx.clone();
                tokio::task::spawn(async move {
                    shmem::listener_loop(server, daemon_tx, queue_sizes).await;
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

struct Listener {
    dataflow_id: DataflowId,
    node_id: NodeId,
    daemon_tx: mpsc::Sender<Event>,
    subscribed_events: Option<UnboundedReceiver<NodeEvent>>,
    queue: VecDeque<Box<Option<NodeEvent>>>,
    queue_sizes: BTreeMap<DataId, usize>,
}

impl Listener {
    pub(crate) async fn run<C: Connection>(
        mut connection: C,
        daemon_tx: mpsc::Sender<Event>,
        queue_sizes: BTreeMap<DataId, usize>,
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
                            daemon_tx,
                            subscribed_events: None,
                            queue_sizes,
                            queue: VecDeque::new(),
                        };
                        match listener
                            .run_inner(connection)
                            .await
                            .wrap_err("listener failed")
                        {
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

    async fn run_inner<C: Connection>(&mut self, mut connection: C) -> eyre::Result<()> {
        loop {
            let mut next_message = connection.receive_message();
            let message = loop {
                let next_event = if let Some(events) = &mut self.subscribed_events {
                    Box::pin(events.recv()).fuse()
                } else {
                    Fuse::terminated()
                };
                let event = match future::select(next_event, next_message).await {
                    future::Either::Left((event, n)) => {
                        next_message = n;
                        event
                    }
                    future::Either::Right((message, _)) => break message,
                };
                if let Some(event) = event {
                    self.queue.push_back(Box::new(Some(event)));
                    self.handle_events().await?;
                }
            };

            // TODO: wait for event queue and connection simultaneously

            match message.wrap_err("failed to receive DaemonRequest") {
                Ok(Some(message)) => {
                    self.handle_message(message, &mut connection).await?;
                }
                Err(err) => {
                    tracing::warn!("{err:?}");
                }
                Ok(None) => {
                    tracing::debug!(
                        "channel disconnected: {}/{}",
                        self.dataflow_id,
                        self.node_id
                    );
                    break; // disconnected
                }
            }
        }
        Ok(())
    }

    async fn handle_events(&mut self) -> eyre::Result<()> {
        if let Some(events) = &mut self.subscribed_events {
            while let Ok(event) = events.try_recv() {
                self.queue.push_back(Box::new(Some(event)));
            }

            // drop oldest input events to maintain max queue length queue
            self.drop_oldest_inputs().await?;
        }
        Ok(())
    }

    #[tracing::instrument(skip(self), fields(%self.node_id))]
    async fn drop_oldest_inputs(&mut self) -> Result<(), eyre::ErrReport> {
        let mut queue_size_remaining = self.queue_sizes.clone();
        let mut dropped = 0;
        let mut drop_tokens = Vec::new();

        // iterate over queued events, newest first
        for event in self.queue.iter_mut().rev() {
            let Some(NodeEvent::Input { id, data, .. }) = event.as_mut() else {
                continue;
            };
            match queue_size_remaining.get_mut(id) {
                Some(0) => {
                    dropped += 1;
                    if let Some(drop_token) = data.as_ref().and_then(|d| d.drop_token()) {
                        drop_tokens.push(drop_token);
                    }
                    *event.as_mut() = None;
                }
                Some(size_remaining) => {
                    *size_remaining = size_remaining.saturating_sub(1);
                }
                None => {
                    tracing::warn!("no queue size known for received input `{id}`");
                }
            }
        }
        self.report_drop_tokens(drop_tokens).await?;

        if dropped > 0 {
            tracing::debug!("dropped {dropped} inputs because event queue was too full");
        }
        Ok(())
    }

    #[tracing::instrument(skip(self, connection), fields(%self.dataflow_id, %self.node_id))]
    async fn handle_message<C: Connection>(
        &mut self,
        message: DaemonRequest,
        connection: &mut C,
    ) -> eyre::Result<()> {
        match message {
            DaemonRequest::Register { .. } => {
                let reply = DaemonReply::Result(Err("unexpected register message".into()));
                self.send_reply(reply, connection)
                    .await
                    .wrap_err("failed to send register reply")?;
            }
            DaemonRequest::Stopped => {
                let (reply_sender, reply) = oneshot::channel();
                self.process_daemon_event(
                    DaemonNodeEvent::Stopped { reply_sender },
                    Some(reply),
                    connection,
                )
                .await?
            }
            DaemonRequest::CloseOutputs(outputs) => {
                let (reply_sender, reply) = oneshot::channel();
                self.process_daemon_event(
                    DaemonNodeEvent::CloseOutputs {
                        outputs,
                        reply_sender,
                    },
                    Some(reply),
                    connection,
                )
                .await?
            }
            DaemonRequest::SendMessage {
                output_id,
                metadata,
                data,
            } => {
                let event = crate::DaemonNodeEvent::SendOut {
                    output_id,
                    metadata,
                    data,
                };
                self.process_daemon_event(event, None, connection).await?;
            }
            DaemonRequest::Subscribe => {
                let (tx, rx) = mpsc::unbounded_channel();
                let (reply_sender, reply) = oneshot::channel();
                self.process_daemon_event(
                    DaemonNodeEvent::Subscribe {
                        event_sender: tx,
                        reply_sender,
                    },
                    Some(reply),
                    connection,
                )
                .await?;
                self.subscribed_events = Some(rx);
            }
            DaemonRequest::NextEvent { drop_tokens } => {
                self.report_drop_tokens(drop_tokens).await?;

                // try to take the queued events first
                let queued_events: Vec<_> = mem::take(&mut self.queue)
                    .into_iter()
                    .filter_map(|e| *e)
                    .collect();
                let reply = if queued_events.is_empty() {
                    match self.subscribed_events.as_mut() {
                        // wait for next event
                        Some(events) => match events.recv().await {
                            Some(event) => DaemonReply::NextEvents(vec![event]),
                            None => DaemonReply::NextEvents(vec![]),
                        },
                        None => {
                            DaemonReply::Result(Err("Ignoring event request because no subscribe \
                                message was sent yet"
                                .into()))
                        }
                    }
                } else {
                    DaemonReply::NextEvents(queued_events)
                };

                self.send_reply(reply, connection)
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
            let event = Event::Node {
                dataflow_id: self.dataflow_id,
                node_id: self.node_id.clone(),
                event: DaemonNodeEvent::ReportDrop {
                    tokens: drop_tokens,
                },
            };
            self.daemon_tx
                .send(event)
                .await
                .map_err(|_| eyre!("failed to report drop tokens to daemon"))?;
        }
        Ok(())
    }

    async fn process_daemon_event<C: Connection>(
        &mut self,
        event: DaemonNodeEvent,
        reply: Option<oneshot::Receiver<DaemonReply>>,
        connection: &mut C,
    ) -> eyre::Result<()> {
        // send NodeEvent to daemon main loop
        let event = Event::Node {
            dataflow_id: self.dataflow_id,
            node_id: self.node_id.clone(),
            event,
        };
        self.daemon_tx
            .send(event)
            .await
            .map_err(|_| eyre!("failed to send event to daemon"))?;
        let reply = if let Some(reply) = reply {
            reply
                .await
                .map_err(|_| eyre!("failed to receive reply from daemon"))?
        } else {
            DaemonReply::Empty
        };
        self.send_reply(reply, connection).await?;
        Ok(())
    }

    async fn send_reply<C: Connection>(
        &mut self,
        reply: DaemonReply,
        connection: &mut C,
    ) -> eyre::Result<()> {
        connection
            .send_reply(reply)
            .await
            .wrap_err_with(|| format!("failed to send reply to node `{}`", self.node_id))
    }
}

#[async_trait::async_trait]
trait Connection {
    async fn receive_message(&mut self) -> eyre::Result<Option<DaemonRequest>>;
    async fn send_reply(&mut self, message: DaemonReply) -> eyre::Result<()>;
}
