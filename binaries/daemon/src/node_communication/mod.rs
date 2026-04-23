use crate::{DaemonNodeEvent, Event};
use dora_core::{
    config::{LocalCommunicationConfig, NodeId},
    topics::LOCALHOST,
    uhlc,
};
use dora_message::{
    DataflowId,
    common::Timestamped,
    daemon_to_node::{DaemonCommunication, DaemonReply, NodeEvent},
    node_to_daemon::DaemonRequest,
};
use eyre::{Context, eyre};
use futures::{Future, future, task};
use std::{
    collections::VecDeque,
    mem,
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
    task::Poll,
};
use tokio::{
    net::TcpListener,
    sync::{
        mpsc::{self, Receiver},
        oneshot,
    },
};

pub mod tcp;

pub fn current_millis() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

pub async fn spawn_listener_loop(
    dataflow_id: &DataflowId,
    node_id: &NodeId,
    daemon_tx: &mpsc::Sender<Timestamped<Event>>,
    config: LocalCommunicationConfig,
    clock: Arc<uhlc::HLC>,
    last_activity: Arc<AtomicU64>,
    shutdown: tokio::sync::watch::Receiver<bool>,
) -> eyre::Result<DaemonCommunication> {
    match config {
        LocalCommunicationConfig::Tcp => {
            let socket = match TcpListener::bind((LOCALHOST, 0)).await {
                Ok(socket) => socket,
                Err(err) => {
                    return Err(
                        eyre::Report::new(err).wrap_err("failed to create local TCP listener")
                    );
                }
            };
            let socket_addr = socket
                .local_addr()
                .wrap_err("failed to get local addr of socket")?;

            let event_loop_node_id = format!("{dataflow_id}/{node_id}");
            let daemon_tx = daemon_tx.clone();
            let shutdown = shutdown.clone();
            tokio::spawn(async move {
                tcp::listener_loop(socket, daemon_tx, clock, last_activity, shutdown).await;
                tracing::debug!("event listener loop finished for `{event_loop_node_id}`");
            });

            Ok(DaemonCommunication::Tcp { socket_addr })
        }
    }
}

struct Listener {
    dataflow_id: DataflowId,
    node_id: NodeId,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    subscribed_events: Option<Receiver<Timestamped<NodeEvent>>>,
    pending_counter: Option<Arc<AtomicU64>>,
    queue: VecDeque<Box<Option<Timestamped<NodeEvent>>>>,
    clock: Arc<uhlc::HLC>,
    last_activity: Arc<AtomicU64>,
}

impl Listener {
    pub(crate) async fn run<C: Connection>(
        mut connection: C,
        daemon_tx: mpsc::Sender<Timestamped<Event>>,
        hlc: Arc<uhlc::HLC>,
        last_activity: Arc<AtomicU64>,
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
            DaemonRequest::Register(register_request) => {
                let result = register_request.check_version();
                let send_result = connection
                    .send_reply(DaemonReply::Result(result.clone()))
                    .await
                    .wrap_err("failed to send register reply");
                let dataflow_id = register_request.dataflow_id;
                let node_id = register_request.node_id;
                match (result, send_result) {
                    (Ok(()), Ok(())) => {
                        let mut listener = Listener {
                            dataflow_id,
                            node_id,
                            daemon_tx,
                            subscribed_events: None,
                            pending_counter: None,
                            queue: VecDeque::new(),
                            clock: hlc.clone(),
                            last_activity,
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
                    .wrap_err("failed to send reply")
                {
                    tracing::warn!("{err:?}");
                }
            }
        }
    }

    async fn run_inner<C: Connection>(&mut self, mut connection: C) -> eyre::Result<()> {
        loop {
            let mut next_message = Box::pin(connection.receive_message());
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
                if let Some(counter) = &self.pending_counter {
                    counter.fetch_sub(1, Ordering::Relaxed);
                }
                self.queue.push_back(Box::new(Some(event)));
            }
        }
        Ok(())
    }

    #[tracing::instrument(skip(self, connection), fields(%self.dataflow_id, %self.node_id), level = "trace")]
    async fn handle_message<C: Connection>(
        &mut self,
        message: Timestamped<DaemonRequest>,
        connection: &mut C,
    ) -> eyre::Result<()> {
        self.last_activity
            .store(current_millis(), Ordering::Release);
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
            DaemonRequest::NodeConfig { .. } => {
                let reply = DaemonReply::Result(Err("unexpected node config message".into()));
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
                let (tx, rx) = mpsc::channel(crate::NODE_EVENT_CHANNEL_CAPACITY);
                let pending_counter = Arc::new(AtomicU64::new(0));
                let (reply_sender, reply) = oneshot::channel();
                self.process_daemon_event(
                    DaemonNodeEvent::Subscribe {
                        event_sender: tx,
                        pending_counter: pending_counter.clone(),
                        reply_sender,
                    },
                    Some(reply),
                    connection,
                )
                .await?;
                self.subscribed_events = Some(rx);
                self.pending_counter = Some(pending_counter);
            }
            DaemonRequest::NextEvent => {
                // try to take the queued events first
                let queued_events: Vec<_> = mem::take(&mut self.queue)
                    .into_iter()
                    .filter_map(|e| *e)
                    .collect();
                let reply = if queued_events.is_empty() {
                    match self.subscribed_events.as_mut() {
                        // wait for next event
                        Some(events) => match events.recv().await {
                            Some(event) => {
                                if let Some(counter) = &self.pending_counter {
                                    counter.fetch_sub(1, Ordering::Relaxed);
                                }
                                DaemonReply::NextEvents(vec![event])
                            }
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
                    Poll::Ready(Some(event)) => {
                        if let Some(counter) = &self.pending_counter {
                            counter.fetch_sub(1, Ordering::Relaxed);
                        }
                        Poll::Ready(event)
                    }
                    Poll::Ready(None) | Poll::Pending => Poll::Pending,
                }
            } else {
                Poll::Pending
            }
        };
        future::poll_fn(poll)
    }
}

trait Connection {
    fn receive_message(
        &mut self,
    ) -> impl Future<Output = eyre::Result<Option<Timestamped<DaemonRequest>>>> + Send;
    fn send_reply(&mut self, message: DaemonReply)
    -> impl Future<Output = eyre::Result<()>> + Send;
}
