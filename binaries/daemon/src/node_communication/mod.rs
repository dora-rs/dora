use crate::{DaemonNodeEvent, Event};
use dora_core::{
    config::{DataId, LocalCommunicationConfig, NodeId},
    daemon_messages::{
        DaemonCommunication, DaemonReply, DaemonRequest, DataflowId, NodeDropEvent, NodeEvent,
        Timestamped,
    },
    message::uhlc,
};
use eyre::{eyre, Context};
use futures::{future, task, Future};
use shared_memory_server::{ShmemConf, ShmemServer};
use std::{
    collections::{BTreeMap, VecDeque},
    mem,
    net::Ipv4Addr,
    sync::Arc,
    task::Poll,
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
    daemon_tx: &mpsc::Sender<Timestamped<Event>>,
    config: LocalCommunicationConfig,
    queue_sizes: BTreeMap<DataId, usize>,
    clock: Arc<uhlc::HLC>,
) -> eyre::Result<DaemonCommunication> {
    match config {
        LocalCommunicationConfig::Tcp => {
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
                tcp::listener_loop(socket, daemon_tx, queue_sizes, clock).await;
                tracing::debug!("event listener loop finished for `{event_loop_node_id}`");
            });

            Ok(DaemonCommunication::Tcp { socket_addr })
        }
        LocalCommunicationConfig::Shmem => {
            let daemon_control_region = ShmemConf::new()
                .size(4096)
                .create()
                .wrap_err("failed to allocate daemon_control_region")?;
            let daemon_events_region = ShmemConf::new()
                .size(4096)
                .create()
                .wrap_err("failed to allocate daemon_events_region")?;
            let daemon_drop_region = ShmemConf::new()
                .size(4096)
                .create()
                .wrap_err("failed to allocate daemon_drop_region")?;
            let daemon_events_close_region = ShmemConf::new()
                .size(4096)
                .create()
                .wrap_err("failed to allocate daemon_drop_region")?;
            let daemon_control_region_id = daemon_control_region.get_os_id().to_owned();
            let daemon_events_region_id = daemon_events_region.get_os_id().to_owned();
            let daemon_drop_region_id = daemon_drop_region.get_os_id().to_owned();
            let daemon_events_close_region_id = daemon_events_close_region.get_os_id().to_owned();

            {
                let server = unsafe { ShmemServer::new(daemon_control_region) }
                    .wrap_err("failed to create control server")?;
                let daemon_tx = daemon_tx.clone();
                let queue_sizes = queue_sizes.clone();
                let clock = clock.clone();
                tokio::spawn(shmem::listener_loop(server, daemon_tx, queue_sizes, clock));
            }

            {
                let server = unsafe { ShmemServer::new(daemon_events_region) }
                    .wrap_err("failed to create events server")?;
                let event_loop_node_id = format!("{dataflow_id}/{node_id}");
                let daemon_tx = daemon_tx.clone();
                let queue_sizes = queue_sizes.clone();
                let clock = clock.clone();
                tokio::task::spawn(async move {
                    shmem::listener_loop(server, daemon_tx, queue_sizes, clock).await;
                    tracing::debug!("event listener loop finished for `{event_loop_node_id}`");
                });
            }

            {
                let server = unsafe { ShmemServer::new(daemon_drop_region) }
                    .wrap_err("failed to create drop server")?;
                let drop_loop_node_id = format!("{dataflow_id}/{node_id}");
                let daemon_tx = daemon_tx.clone();
                let queue_sizes = queue_sizes.clone();
                let clock = clock.clone();
                tokio::task::spawn(async move {
                    shmem::listener_loop(server, daemon_tx, queue_sizes, clock).await;
                    tracing::debug!("drop listener loop finished for `{drop_loop_node_id}`");
                });
            }

            {
                let server = unsafe { ShmemServer::new(daemon_events_close_region) }
                    .wrap_err("failed to create events close server")?;
                let drop_loop_node_id = format!("{dataflow_id}/{node_id}");
                let daemon_tx = daemon_tx.clone();
                let clock = clock.clone();
                tokio::task::spawn(async move {
                    shmem::listener_loop(server, daemon_tx, queue_sizes, clock).await;
                    tracing::debug!(
                        "events close listener loop finished for `{drop_loop_node_id}`"
                    );
                });
            }

            Ok(DaemonCommunication::Shmem {
                daemon_control_region_id,
                daemon_events_region_id,
                daemon_drop_region_id,
                daemon_events_close_region_id,
            })
        }
    }
}

struct Listener {
    dataflow_id: DataflowId,
    node_id: NodeId,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    subscribed_events: Option<UnboundedReceiver<Timestamped<NodeEvent>>>,
    subscribed_drop_events: Option<UnboundedReceiver<Timestamped<NodeDropEvent>>>,
    queue: VecDeque<Box<Option<Timestamped<NodeEvent>>>>,
    queue_sizes: BTreeMap<DataId, usize>,
    clock: Arc<uhlc::HLC>,
}

impl Listener {
    pub(crate) async fn run<C: Connection>(
        mut connection: C,
        daemon_tx: mpsc::Sender<Timestamped<Event>>,
        queue_sizes: BTreeMap<DataId, usize>,
        hlc: Arc<uhlc::HLC>,
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

        if let Err(err) = hlc.update_with_timestamp(&message.timestamp) {
            tracing::warn!("failed to update HLC: {err}");
        }

        match message.inner {
            DaemonRequest::Register {
                dataflow_id,
                node_id,
                dora_version: node_api_version,
            } => {
                let daemon_version = env!("CARGO_PKG_VERSION");
                let result = if node_api_version == daemon_version {
                    Ok(())
                } else {
                    Err(format!(
                        "version mismatch: node API v{node_api_version} is not compatible \
                        with daemon v{daemon_version}"
                    ))
                };
                let send_result = connection
                    .send_reply(DaemonReply::Result(result.clone()))
                    .await
                    .wrap_err("failed to send register reply");
                match (result, send_result) {
                    (Ok(()), Ok(())) => {
                        let mut listener = Listener {
                            dataflow_id,
                            node_id,
                            daemon_tx,
                            subscribed_events: None,
                            subscribed_drop_events: None,
                            queue_sizes,
                            queue: VecDeque::new(),
                            clock: hlc.clone(),
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
                    (Err(err), _) => {
                        tracing::warn!("failed to register node {dataflow_id}/{node_id}: {err}");
                    }
                    (Ok(()), Err(err)) => {
                        tracing::warn!(
                            "failed send register reply to node {dataflow_id}/{node_id}: {err:?}"
                        );
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
                let next_event = self.next_event();
                let event = match future::select(next_event, next_message).await {
                    future::Either::Left((event, n)) => {
                        next_message = n;
                        event
                    }
                    future::Either::Right((message, _)) => break message,
                };

                self.queue.push_back(Box::new(Some(event)));
                self.handle_events().await?;
            };

            match message.wrap_err("failed to receive DaemonRequest") {
                Ok(Some(message)) => {
                    if let Err(err) = self.handle_message(message, &mut connection).await {
                        tracing::warn!("{err:?}");
                    }
                }
                Err(err) => {
                    tracing::warn!("{err:?}");
                }
                Ok(None) => {
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

    #[tracing::instrument(skip(self), fields(%self.node_id), level = "trace")]
    async fn drop_oldest_inputs(&mut self) -> Result<(), eyre::ErrReport> {
        let mut queue_size_remaining = self.queue_sizes.clone();
        let mut dropped = 0;
        let mut drop_tokens = Vec::new();

        // iterate over queued events, newest first
        for event in self.queue.iter_mut().rev() {
            let Some(Timestamped {
                inner: NodeEvent::Input { id, data, .. },
                ..
            }) = event.as_mut()
            else {
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

    #[tracing::instrument(skip(self, connection), fields(%self.dataflow_id, %self.node_id), level = "trace")]
    async fn handle_message<C: Connection>(
        &mut self,
        message: Timestamped<DaemonRequest>,
        connection: &mut C,
    ) -> eyre::Result<()> {
        let timestamp = message.timestamp;
        if let Err(err) = self.clock.update_with_timestamp(&timestamp) {
            tracing::warn!("failed to update HLC: {err}");
        }
        match message.inner {
            DaemonRequest::Register { .. } => {
                let reply = DaemonReply::Result(Err("unexpected register message".into()));
                self.send_reply(reply, connection)
                    .await
                    .wrap_err("failed to send register reply")?;
            }
            DaemonRequest::OutputsDone => {
                let (reply_sender, reply) = oneshot::channel();
                self.process_daemon_event(
                    DaemonNodeEvent::OutputsDone { reply_sender },
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
            DaemonRequest::SubscribeDrop => {
                let (tx, rx) = mpsc::unbounded_channel();
                let (reply_sender, reply) = oneshot::channel();
                self.process_daemon_event(
                    DaemonNodeEvent::SubscribeDrop {
                        event_sender: tx,
                        reply_sender,
                    },
                    Some(reply),
                    connection,
                )
                .await?;
                self.subscribed_drop_events = Some(rx);
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

                self.send_reply(reply.clone(), connection)
                    .await
                    .wrap_err_with(|| format!("failed to send NextEvent reply: {reply:?}"))?;
            }
            DaemonRequest::ReportDropTokens { drop_tokens } => {
                self.report_drop_tokens(drop_tokens).await?;

                self.send_reply(DaemonReply::Empty, connection)
                    .await
                    .wrap_err("failed to send ReportDropTokens reply")?;
            }
            DaemonRequest::NextFinishedDropTokens => {
                let reply = match self.subscribed_drop_events.as_mut() {
                    // wait for next event
                    Some(events) => match events.recv().await {
                        Some(event) => DaemonReply::NextDropEvents(vec![event]),
                        None => DaemonReply::NextDropEvents(vec![]),
                    },
                    None => DaemonReply::Result(Err("Ignoring event request because no drop \
                        subscribe message was sent yet"
                        .into())),
                };

                self.send_reply(reply.clone(), connection)
                    .await
                    .wrap_err_with(|| {
                        format!("failed to send NextFinishedDropTokens reply: {reply:?}")
                    })?;
            }
            DaemonRequest::EventStreamDropped => {
                let (reply_sender, reply) = oneshot::channel();
                self.process_daemon_event(
                    DaemonNodeEvent::EventStreamDropped { reply_sender },
                    Some(reply),
                    connection,
                )
                .await?;
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
            let event = Timestamped {
                inner: event,
                timestamp: self.clock.new_timestamp(),
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
        let event = Timestamped {
            inner: event,
            timestamp: self.clock.new_timestamp(),
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

    /// Awaits the next subscribed event if any. Never resolves if the event channel is closed.
    ///
    /// This is similar to `self.subscribed_events.recv()`. The difference is that the future
    /// does not return `None` when the channel is closed and instead stays pending forever.
    /// This behavior can be useful when waiting for multiple event sources at once.
    fn next_event(&mut self) -> impl Future<Output = Timestamped<NodeEvent>> + Unpin + '_ {
        let poll = |cx: &mut task::Context<'_>| {
            if let Some(events) = &mut self.subscribed_events {
                match events.poll_recv(cx) {
                    Poll::Ready(Some(event)) => Poll::Ready(event),
                    Poll::Ready(None) | Poll::Pending => Poll::Pending,
                }
            } else {
                Poll::Pending
            }
        };
        future::poll_fn(poll)
    }
}

#[async_trait::async_trait]
trait Connection {
    async fn receive_message(&mut self) -> eyre::Result<Option<Timestamped<DaemonRequest>>>;
    async fn send_reply(&mut self, message: DaemonReply) -> eyre::Result<()>;
}
