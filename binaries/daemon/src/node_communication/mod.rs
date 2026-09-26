use crate::{
    CONTROL_EVENT_HEADROOM, DaemonNodeEvent, Event, FaultToleranceStats,
    NODE_EVENT_CHANNEL_CAPACITY,
    local_delivery::{DeferredDelivery, DrainSignal},
};
use dora_core::{
    config::{DataId, NodeId},
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
    collections::{BTreeSet, VecDeque},
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
    task::Poll,
    time::Duration,
};
use tokio::{
    net::TcpListener,
    sync::{
        mpsc::{self, Receiver},
        oneshot,
    },
};

pub mod tcp;

/// What a producer's listener needs to hold the producer on a backpressure
/// edge instead of letting the daemon drop (dora-rs/dora#3397).
#[derive(Debug)]
pub(crate) struct BackpressureConfig {
    /// Outputs of this node that feed a `queue_policy: backpressure` input
    /// (`output_routing::backpressured_outputs`).
    pub outputs: BTreeSet<DataId>,
    pub ft_stats: Arc<FaultToleranceStats>,
}

/// How long a held producer may wait for a receiver that frees no slot at
/// all before the event is dropped instead. A receiver that is merely slow
/// frees a slot per event it processes and never trips this; one that
/// stopped draining altogether — wedged, or blocked on its own producer in
/// a backpressure cycle — would otherwise hold the producer forever. The
/// drop is an `error!` and counts as a lost backpressure message.
const BACKPRESSURE_STALL_LIMIT: Duration = Duration::from_secs(60);

/// The wait is cut into ticks of this length so the held producer's
/// `last_activity` keeps advancing: it is silent because the daemon holds
/// it, not because it hung, and the health check must not kill it for that.
const BACKPRESSURE_STALL_TICK: Duration = Duration::from_secs(1);

pub fn current_millis() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

/// `backpressure` names this node's outputs with at least one
/// `queue_policy: backpressure` consumer; see [`Listener::backpressure`].
#[allow(clippy::too_many_arguments)]
pub async fn spawn_listener_loop(
    dataflow_id: &DataflowId,
    node_id: &NodeId,
    generation: Arc<AtomicU64>,
    daemon_tx: &mpsc::Sender<Timestamped<Event>>,
    clock: Arc<uhlc::HLC>,
    last_activity: Arc<AtomicU64>,
    shutdown: tokio::sync::watch::Receiver<bool>,
    node_shutdown: tokio::sync::watch::Receiver<bool>,
    backpressure: Arc<BackpressureConfig>,
) -> eyre::Result<DaemonCommunication> {
    let socket = match TcpListener::bind((LOCALHOST, 0)).await {
        Ok(socket) => socket,
        Err(err) => {
            return Err(eyre::Report::new(err).wrap_err("failed to create local TCP listener"));
        }
    };
    let socket_addr = socket
        .local_addr()
        .wrap_err("failed to get local addr of socket")?;

    let event_loop_node_id = format!("{dataflow_id}/{node_id}");
    let daemon_tx = daemon_tx.clone();
    let shutdown = shutdown.clone();
    tokio::spawn(async move {
        tcp::listener_loop(
            socket,
            generation,
            daemon_tx,
            clock,
            last_activity,
            backpressure,
            shutdown,
            node_shutdown,
        )
        .await;
        tracing::debug!("event listener loop finished for `{event_loop_node_id}`");
    });

    Ok(DaemonCommunication::Tcp { socket_addr })
}

/// What a `NextEvents` reply adds around its events: the reply's enum
/// discriminant and the vector's length prefix, two varints of at most ten
/// bytes each. Rounded up.
const NEXT_EVENTS_FRAME_OVERHEAD: usize = 32;

/// Upper bound on the encoded size of the events in one `NextEvents` reply.
///
/// A node that stalls and then asks for events used to get its *whole*
/// backlog in a single reply. Past `MAX_MESSAGE_BYTES` the frame sender
/// refuses to write it, and the events — already taken off the queue — were
/// lost with nothing but a warning. Sizes are the exact wire lengths
/// (`dora_message::serialized_size`), computed once when an event is queued,
/// so metadata parameters and variable-length control fields count too — a
/// per-event estimate that ignored them let a metadata-heavy backlog
/// overshoot the frame (PR #3429 review).
const NEXT_EVENTS_REPLY_BUDGET: usize =
    dora_message::MAX_MESSAGE_BYTES - NEXT_EVENTS_FRAME_OVERHEAD;

/// Bounds on what the listener holds for a node that is not asking for
/// events. Without them the listener kept draining the node's (bounded)
/// subscription channel into its own unbounded queue, so one consumer that
/// stopped reading could grow daemon memory without limit while its producers
/// kept sending. Once either bound is reached the listener stops pulling from
/// the channel; the channel then fills and the sender-side policy applies
/// (`send_output_to_local_receivers` drops data with a warning to keep
/// control-event headroom).
const LISTENER_QUEUE_MAX_EVENTS: usize = NODE_EVENT_CHANNEL_CAPACITY;
const LISTENER_QUEUE_MAX_BYTES: usize = 4 * dora_message::MAX_MESSAGE_BYTES;

/// Whether a listener queue of `events` events whose encodings total `bytes`
/// may take no more from the channel.
fn queue_saturated(events: usize, bytes: usize) -> bool {
    events >= LISTENER_QUEUE_MAX_EVENTS || bytes >= LISTENER_QUEUE_MAX_BYTES
}

/// A queued event with its exact wire length, measured once at enqueue.
struct SizedEvent {
    size: usize,
    event: Timestamped<NodeEvent>,
}

impl SizedEvent {
    fn new(event: Timestamped<NodeEvent>) -> Self {
        let size = dora_message::serialized_size(&event).unwrap_or_else(|err| {
            // Serializing these types cannot fail in practice; if it ever
            // does, the reply itself fails the same way, so any size will do.
            tracing::warn!("cannot size queued node event ({err}); using the size hint");
            event.inner.encode_size_hint()
        });
        Self { size, event }
    }

    fn describe(&self) -> String {
        match &self.event.inner {
            NodeEvent::Input { id, .. } => format!("input `{id}`"),
            other => format!("{other:?}"),
        }
    }
}

struct Listener {
    dataflow_id: DataflowId,
    node_id: NodeId,
    /// Incarnation of the spawn this listener was bound for; stamped on
    /// every `Event::Node` so the daemon can drop control events from a
    /// superseded process (dora-rs/dora#2927).
    generation: u64,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    subscribed_events: Option<Receiver<Timestamped<NodeEvent>>>,
    pending_counter: Option<Arc<AtomicU64>>,
    queue: VecDeque<SizedEvent>,
    /// Sum of the exact wire lengths over `queue`, kept incrementally.
    queued_bytes: usize,
    clock: Arc<uhlc::HLC>,
    last_activity: Arc<AtomicU64>,
    /// A `SendMessage` on one of `backpressure.outputs` asks the daemon loop
    /// for what it could not deliver (`DeferredDelivery`). Delivery starts
    /// at once on its own task (`spawn_deferred`), so it does not depend on
    /// the producer ever sending again; the node's next ordering-relevant
    /// request merely waits for that task (`flush_deferred`), and the reply
    /// to the request itself stays immediate.
    backpressure: Arc<BackpressureConfig>,
    pending_deferred: Option<tokio::task::JoinHandle<()>>,
    /// Notified whenever this listener takes an event out of
    /// `subscribed_events`; a producer held for this node's full channel
    /// waits on it, and gives up on it through it
    /// (`RunningDataflow::drain_signals`).
    drained: Arc<DrainSignal>,
}

impl Listener {
    pub(crate) async fn run<C: Connection>(
        mut connection: C,
        generation: Arc<AtomicU64>,
        daemon_tx: mpsc::Sender<Timestamped<Event>>,
        hlc: Arc<uhlc::HLC>,
        last_activity: Arc<AtomicU64>,
        backpressure: Arc<BackpressureConfig>,
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
                        // Snapshot the node's CURRENT incarnation at
                        // register time: the socket is shared across
                        // respawns, but each connection belongs to exactly
                        // the incarnation that was current when the process
                        // connected.
                        let connection_generation = generation.load(Ordering::Acquire);
                        let mut listener = Listener {
                            dataflow_id,
                            node_id,
                            generation: connection_generation,
                            daemon_tx,
                            subscribed_events: None,
                            pending_counter: None,
                            queue: VecDeque::new(),
                            queued_bytes: 0,
                            clock: hlc.clone(),
                            last_activity,
                            backpressure,
                            pending_deferred: None,
                            drained: Default::default(),
                        };
                        match listener
                            .run_inner(connection)
                            .await
                            .wrap_err("listener failed")
                        {
                            Ok(()) => {}
                            Err(err) => tracing::error!("{err:?}"),
                        }
                        // The subscribe channel's receiver goes with this
                        // listener: wake anyone waiting for room in it so
                        // they see it closed.
                        listener.drained.notify.notify_waiters();
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

                self.note_drained();
                self.enqueue(event);
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
                    // The node's last `SendMessage` may still have deliveries
                    // waiting for room; it is gone, but they need not be.
                    self.flush_deferred().await;
                    break; // disconnected
                }
            }
        }
        Ok(())
    }

    async fn handle_events(&mut self) -> eyre::Result<()> {
        while !self.queue_saturated() {
            let Some(events) = &mut self.subscribed_events else {
                break;
            };
            let Ok(event) = events.try_recv() else {
                break;
            };
            if let Some(counter) = &self.pending_counter {
                counter.fetch_sub(1, Ordering::Relaxed);
            }
            self.note_drained();
            self.enqueue(event);
        }
        Ok(())
    }

    fn queue_saturated(&self) -> bool {
        queue_saturated(self.queue.len(), self.queued_bytes)
    }

    fn enqueue(&mut self, event: Timestamped<NodeEvent>) {
        let sized = SizedEvent::new(event);
        self.queued_bytes = self.queued_bytes.saturating_add(sized.size);
        self.queue.push_back(sized);
    }

    fn pop_front(&mut self) -> Option<SizedEvent> {
        let sized = self.queue.pop_front()?;
        self.queued_bytes = self.queued_bytes.saturating_sub(sized.size);
        Some(sized)
    }

    /// Takes queued events from the front, in order, while their exact wire
    /// lengths stay within [`NEXT_EVENTS_REPLY_BUDGET`]; whatever does not
    /// fit stays queued for the next request. An event that alone exceeds the
    /// budget can never be framed — the node-side send caps a payload at the
    /// frame limit, but not payload plus metadata — so it is dropped with an
    /// error rather than left to wedge the queue forever.
    fn take_queued_events_within_budget(&mut self) -> Vec<Timestamped<NodeEvent>> {
        let mut batch = Vec::new();
        let mut batch_bytes = 0usize;
        while let Some(front) = self.queue.front() {
            if front.size > NEXT_EVENTS_REPLY_BUDGET {
                let Some(dropped) = self.pop_front() else {
                    break;
                };
                tracing::error!(
                    node = %self.node_id,
                    size = dropped.size,
                    "dropping {}: its encoding exceeds the {}-byte daemon frame limit and can \
                     never be delivered",
                    dropped.describe(),
                    dora_message::MAX_MESSAGE_BYTES,
                );
                continue;
            }
            if !batch.is_empty()
                && batch_bytes.saturating_add(front.size) > NEXT_EVENTS_REPLY_BUDGET
            {
                break;
            }
            let Some(sized) = self.pop_front() else {
                break;
            };
            batch_bytes = batch_bytes.saturating_add(sized.size);
            batch.push(sized.event);
        }
        batch
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
        // Settle the previous `SendMessage`'s deferred deliveries before
        // anything that must land after them, so a held producer cannot get
        // ahead of the event still waiting for room. Other requests need no
        // ordering against it and are not made to wait.
        if matches!(
            message.inner,
            DaemonRequest::SendMessage { .. }
                | DaemonRequest::OutputsDone
                | DaemonRequest::CloseOutputs(_)
        ) {
            self.flush_deferred().await;
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
                let (deferred_reply, deferred) = if self.backpressure.outputs.contains(&output_id) {
                    let (tx, rx) = oneshot::channel();
                    (Some(tx), Some(rx))
                } else {
                    (None, None)
                };
                let event = crate::DaemonNodeEvent::SendOut {
                    output_id,
                    metadata,
                    data,
                    deferred_reply,
                };
                self.forward_to_daemon(event).await?;
                self.pending_deferred = deferred.map(|deferred| self.spawn_deferred(deferred));
                self.send_reply(DaemonReply::Empty, connection).await?;
            }
            DaemonRequest::OutputSent {
                output_id,
                metadata,
            } => {
                let event = crate::DaemonNodeEvent::OutputSent {
                    output_id,
                    metadata,
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
                        drained: self.drained.clone(),
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
                // Take queued events first, bounded by encoded size so the
                // reply fits one frame (see `NEXT_EVENTS_REPLY_BUDGET`).
                let queued_events = self.take_queued_events_within_budget();
                let reply = if queued_events.is_empty() {
                    match self.subscribed_events.as_mut() {
                        // wait for next event
                        Some(events) => match events.recv().await {
                            Some(event) => {
                                if let Some(counter) = &self.pending_counter {
                                    counter.fetch_sub(1, Ordering::Relaxed);
                                }
                                self.note_drained();
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

                self.send_reply(reply, connection)
                    .await
                    .wrap_err("failed to send NextEvent reply")?;
            }
            DaemonRequest::EventStreamDropped => {
                // Deliberate: a held delivery that finds the channel closed
                // from here on is not a loss. The daemon loop marks the drain
                // signal the node subscribed with
                // (`RunningDataflow::mark_event_stream_dropped`); this request
                // arrives on the node's close connection, not that one.
                let (reply_sender, reply) = oneshot::channel();
                self.process_daemon_event(
                    DaemonNodeEvent::EventStreamDropped { reply_sender },
                    Some(reply),
                    connection,
                )
                .await?;
            }
            DaemonRequest::ExtensionStore {
                namespace,
                key,
                value,
            } => {
                let (reply_sender, reply) = oneshot::channel();
                self.process_daemon_event(
                    DaemonNodeEvent::ExtensionStore {
                        namespace,
                        key,
                        value,
                        reply_sender,
                    },
                    Some(reply),
                    connection,
                )
                .await?;
            }
            DaemonRequest::ExtensionLoad {
                namespace,
                key,
                remove,
            } => {
                let (reply_sender, reply) = oneshot::channel();
                self.process_daemon_event(
                    DaemonNodeEvent::ExtensionLoad {
                        namespace,
                        key,
                        remove,
                        reply_sender,
                    },
                    Some(reply),
                    connection,
                )
                .await?;
            }
            DaemonRequest::ExtensionDrop { namespace, key } => {
                let (reply_sender, reply) = oneshot::channel();
                self.process_daemon_event(
                    DaemonNodeEvent::ExtensionDrop {
                        namespace,
                        key,
                        reply_sender,
                    },
                    Some(reply),
                    connection,
                )
                .await?;
            }
            DaemonRequest::ExtensionRequest { namespace, payload } => {
                let (reply_sender, reply) = oneshot::channel();
                self.process_daemon_event(
                    DaemonNodeEvent::ExtensionRequest {
                        namespace,
                        payload,
                        reply_sender,
                    },
                    Some(reply),
                    connection,
                )
                .await?;
            }
            // `DaemonRequest` is `#[non_exhaustive]`: a node built against a newer
            // dora-node-api may send a request this daemon predates. Answer with an
            // explicit error so the node fails loudly instead of hanging on a reply
            // that never comes.
            other => {
                let reply = DaemonReply::Result(Err(format!(
                    "unsupported request from node (node is likely newer than this daemon): {other:?}"
                )));
                self.send_reply(reply, connection).await?;
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
        self.forward_to_daemon(event).await?;
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

    /// Sends a `NodeEvent` to the daemon main loop.
    async fn forward_to_daemon(&mut self, event: DaemonNodeEvent) -> eyre::Result<()> {
        let event = Event::Node {
            dataflow_id: self.dataflow_id,
            node_id: self.node_id.clone(),
            generation: self.generation,
            event,
        };
        let event = Timestamped {
            inner: event,
            timestamp: self.clock.new_timestamp(),
        };
        self.daemon_tx
            .send(event)
            .await
            .map_err(|_| eyre!("failed to send event to daemon"))
    }

    /// Starts delivering whatever the daemon loop hands back for a
    /// `SendMessage`, as soon as it does. Its own task, so a producer that
    /// emits once and then waits — for a reply from the very receiver it is
    /// held for, say — still gets its message through the moment the
    /// receiver makes room, and the stall limit runs against a wedged
    /// receiver whether or not the producer ever sends again.
    fn spawn_deferred(
        &self,
        deferred: oneshot::Receiver<Vec<DeferredDelivery>>,
    ) -> tokio::task::JoinHandle<()> {
        let last_activity = self.last_activity.clone();
        let ft_stats = self.backpressure.ft_stats.clone();
        tokio::spawn(async move {
            // A dropped sender means the daemon loop never routed the event
            // (dataflow gone, delivery failed): nothing to wait for.
            if let Ok(deferred) = deferred.await {
                for delivery in deferred {
                    deliver_when_room(delivery, &last_activity, &ft_stats).await;
                }
            }
        })
    }

    /// Waits for the previous `SendMessage`'s deferred deliveries, if any,
    /// so nothing this node sends next can land ahead of them.
    async fn flush_deferred(&mut self) {
        if let Some(pending) = self.pending_deferred.take() {
            // The task only panics on a bug in `deliver_when_room`; the
            // producer's next request must not be lost to that.
            let _ = pending.await;
        }
    }

    /// Wakes a producer held for this node's full channel
    /// (`DeferredDelivery::drained`). Only worth the waiter-list lock while
    /// one could be waiting, i.e. while the channel is at or below the
    /// headroom a delivery waits for.
    fn note_drained(&self) {
        let Some(events) = self.subscribed_events.as_ref() else {
            return;
        };
        let room = events.capacity();
        // Hold producers for this node again only once it has made the room
        // a held delivery waits for. Clearing on any single take would re-arm
        // the hold while the channel is still full, so a receiver that frees
        // a few slots per read (large payloads) would stall its producer for
        // another full stall limit after every read (dora-rs/dora#3601).
        if room >= CONTROL_EVENT_HEADROOM && self.drained.gave_up.load(Ordering::Relaxed) {
            self.drained.gave_up.store(false, Ordering::Release);
            tracing::info!(
                node = %self.node_id,
                "receiver is draining again: holding its backpressure producers again"
            );
        }
        if room <= CONTROL_EVENT_HEADROOM {
            self.drained.notify.notify_waiters();
        }
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
            // A saturated queue takes nothing more from the channel. The
            // caller recreates this future after every request, so polling
            // resumes as soon as a `NextEvent` reply has made room.
            if queue_saturated(self.queue.len(), self.queued_bytes) {
                return Poll::Pending;
            }
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

/// Delivers one event the daemon loop could not, once
/// `CONTROL_EVENT_HEADROOM` slots are free — the rule the daemon loop
/// applies. This runs on the producer's own task, so the wait stalls only
/// that producer and never the daemon loop, and it is a wait on the
/// receiver's drain signal rather than on a channel permit, so control
/// events keep finding room meanwhile. `last_activity` is the held
/// producer's, kept advancing so the health check does not read the hold as
/// a hang. A receiver whose event stream closed is gone with its node — the
/// same outcome as a `Closed` on the daemon loop's `try_send` — and the
/// event is counted as lost unless the node dropped its stream deliberately.
/// A receiver that frees nothing for `BACKPRESSURE_STALL_LIMIT` is not
/// waited for any longer: the event is dropped, loudly, and counted as lost,
/// and the receiver is marked `gave_up` so that its producers are not held
/// for it again until it drains.
async fn deliver_when_room(
    delivery: DeferredDelivery,
    last_activity: &AtomicU64,
    ft_stats: &FaultToleranceStats,
) {
    let DeferredDelivery {
        receiver,
        channel,
        pending,
        drained,
        mut event,
    } = delivery;
    let mut stalled = Duration::ZERO;
    loop {
        // Register before looking, so a drain between the look and the
        // wait is not lost.
        let notified = drained.notify.notified();
        tokio::pin!(notified);
        notified.as_mut().enable();
        if channel.capacity() >= CONTROL_EVENT_HEADROOM {
            match channel.try_send(event) {
                Ok(()) => {
                    if let Some(pending) = &pending {
                        pending.fetch_add(1, Ordering::Relaxed);
                    }
                    return;
                }
                Err(mpsc::error::TrySendError::Closed(_)) => break,
                Err(mpsc::error::TrySendError::Full(returned)) => event = returned,
            }
        }
        if channel.is_closed() {
            break;
        }
        // Every pass through here is the producer being held, not hanging —
        // whether the receiver is draining (but not yet up to the headroom)
        // or not — and the health check must see it that way.
        last_activity.store(current_millis(), Ordering::Release);
        match tokio::time::timeout(BACKPRESSURE_STALL_TICK, notified).await {
            Ok(()) => stalled = Duration::ZERO,
            Err(_elapsed) => {
                stalled += BACKPRESSURE_STALL_TICK;
                // Another delivery to this receiver already gave up on it
                // (dora-rs/dora#3601): don't hold this producer any longer.
                if drained.gave_up.load(Ordering::Acquire) {
                    tracing::warn!(
                        node = %receiver,
                        "receiver was given up on: dropping a held message its input \
                         (queue_policy: backpressure) was promised"
                    );
                    ft_stats.record_drop(1, true);
                    return;
                }
                if stalled >= BACKPRESSURE_STALL_LIMIT {
                    tracing::error!(
                        node = %receiver,
                        "receiver freed no room in {:?}: dropping a message its input \
                         (queue_policy: backpressure) was promised — it is wedged, or \
                         blocked on its own producer in a backpressure cycle",
                        BACKPRESSURE_STALL_LIMIT,
                    );
                    ft_stats.record_drop(1, true);
                    drained.gave_up.store(true, Ordering::Release);
                    return;
                }
            }
        }
    }
    if drained.stream_dropped.load(Ordering::Acquire) {
        tracing::debug!(
            receiver = %receiver,
            "receiver dropped its event stream before a deferred delivery could complete"
        );
    } else {
        // Crashed or restarting: a message its backpressure input was
        // promised is gone, like a mid-restart drop on the daemon loop.
        tracing::warn!(
            receiver = %receiver,
            "receiver's event stream closed before a deferred delivery could complete: \
             dropping a message its input (queue_policy: backpressure) was promised"
        );
        ft_stats.record_drop(1, true);
    }
}

trait Connection {
    fn receive_message(
        &mut self,
    ) -> impl Future<Output = eyre::Result<Option<Timestamped<DaemonRequest>>>> + Send;
    fn send_reply(&mut self, message: DaemonReply)
    -> impl Future<Output = eyre::Result<()>> + Send;
}

#[cfg(test)]
mod tests {
    use super::*;
    use aligned_vec::AVec;
    use dora_core::config::DataId;
    use dora_message::{
        common::DataMessage,
        metadata::{Metadata, Parameter},
    };
    use uuid::Uuid;

    const MIB: usize = 1024 * 1024;

    /// The frame `TcpConnection::send_reply` would write for `batch`.
    fn encoded_reply_len(batch: Vec<Timestamped<NodeEvent>>) -> usize {
        let reply = DaemonReply::NextEvents(batch);
        dora_message::encode_presized(&reply, reply.encode_size_hint())
            .expect("encode NextEvents reply")
            .len()
    }

    /// An input with no payload but `param_len` bytes of metadata — the part
    /// `encode_size_hint` does not count.
    fn metadata_heavy_input(clock: &uhlc::HLC, param_len: usize) -> Timestamped<NodeEvent> {
        let mut metadata = Metadata::new(clock.new_timestamp());
        metadata
            .parameters
            .insert("blob".to_string(), Parameter::String("x".repeat(param_len)));
        Timestamped {
            inner: NodeEvent::Input {
                id: DataId::from("in".to_string()),
                metadata: Arc::new(metadata),
                data: None,
            },
            timestamp: clock.new_timestamp(),
        }
    }

    fn listener() -> (Listener, mpsc::Sender<Timestamped<NodeEvent>>) {
        let (daemon_tx, _daemon_rx) = mpsc::channel(1);
        let (tx, rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        let listener = Listener {
            dataflow_id: Uuid::nil(),
            node_id: NodeId::from("sink".to_string()),
            generation: 0,
            daemon_tx,
            subscribed_events: Some(rx),
            pending_counter: None,
            queue: VecDeque::new(),
            queued_bytes: 0,
            clock: Arc::new(uhlc::HLC::default()),
            last_activity: Arc::new(AtomicU64::new(0)),
            backpressure: Arc::new(BackpressureConfig {
                outputs: Default::default(),
                ft_stats: Default::default(),
            }),
            pending_deferred: None,
            drained: Default::default(),
        };
        (listener, tx)
    }

    fn input(clock: &uhlc::HLC, payload_len: usize) -> Timestamped<NodeEvent> {
        let data = (payload_len > 0).then(|| {
            Arc::new(DataMessage::Vec(AVec::from_slice(
                128,
                &vec![0u8; payload_len],
            )))
        });
        Timestamped {
            inner: NodeEvent::Input {
                id: DataId::from("in".to_string()),
                metadata: Arc::new(Metadata::new(clock.new_timestamp())),
                data,
            },
            timestamp: clock.new_timestamp(),
        }
    }

    fn deferred(
        channel: &mpsc::Sender<Timestamped<NodeEvent>>,
        drained: &Arc<DrainSignal>,
        event: Timestamped<NodeEvent>,
    ) -> (DeferredDelivery, Arc<AtomicU64>) {
        let pending = Arc::new(AtomicU64::new(0));
        let delivery = DeferredDelivery {
            receiver: NodeId::from("sink".to_string()),
            channel: channel.clone(),
            pending: Some(pending.clone()),
            drained: drained.clone(),
            event,
        };
        (delivery, pending)
    }

    /// A deferred delivery lands only once the receiver's channel has the
    /// control-event headroom free again — the same rule the daemon loop's
    /// `try_send` path applies — and bumps the receiver's pending counter.
    /// While it waits it holds no permit, so a control event still gets
    /// through (a `reserve_many` wait would have been handed every freed
    /// slot first and starved them).
    #[tokio::test]
    async fn deferred_delivery_waits_for_control_headroom() {
        let (listener, _tx) = listener();
        let clock = listener.clock.clone();
        let drained = Arc::new(DrainSignal::default());
        let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        // One slot short of the headroom: the daemon loop would not deliver
        // here, and neither may the listener.
        for _ in 0..NODE_EVENT_CHANNEL_CAPACITY - (CONTROL_EVENT_HEADROOM - 1) {
            tx.try_send(input(&clock, 0)).unwrap();
        }
        let (delivery, pending) = deferred(&tx, &drained, metadata_heavy_input(&clock, 7));

        let complete = deliver_when_room(
            delivery,
            &listener.last_activity,
            &listener.backpressure.ft_stats,
        );
        let receiver_side = async {
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
            assert_eq!(pending.load(Ordering::Relaxed), 0, "still waiting");
            // Control events find room while the delivery waits.
            assert!(
                crate::send_with_timestamp(&tx, NodeEvent::Stop, &clock).unwrap(),
                "a control event must not be starved by a waiting delivery"
            );
            rx.recv().await.expect("a queued event");
            rx.recv().await.expect("a queued event");
            drained.notify.notify_waiters();
        };
        let started = std::time::Instant::now();
        tokio::join!(complete, receiver_side);
        assert!(
            started.elapsed() >= std::time::Duration::from_millis(100),
            "the delivery must wait for the receiver to make room"
        );
        assert_eq!(pending.load(Ordering::Relaxed), 1);

        // The deferred event is the newest one in the channel.
        let mut last = None;
        while let Ok(event) = rx.try_recv() {
            last = Some(event);
        }
        let last = last.expect("the deferred event landed");
        match &last.inner {
            NodeEvent::Input { metadata, .. } => {
                assert!(metadata.parameters.contains_key("blob"));
            }
            other => panic!("unexpected event {other:?}"),
        }
    }

    /// A deferred delivery lands as soon as the receiver makes room, without
    /// the producer sending anything else first: it runs on its own task from
    /// the moment the daemon loop hands it back, and `flush_deferred` only
    /// waits for it. A producer that emits once and then waits for that very
    /// receiver's reply would otherwise be deadlocked, with the stall limit
    /// never even started (PR #3590 review).
    #[tokio::test]
    async fn deferred_delivery_proceeds_without_a_further_request() {
        let (mut listener, _tx) = listener();
        let clock = listener.clock.clone();
        let drained = Arc::new(DrainSignal::default());
        let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        for _ in 0..NODE_EVENT_CHANNEL_CAPACITY {
            tx.try_send(input(&clock, 0)).unwrap();
        }
        let (delivery, pending) = deferred(&tx, &drained, metadata_heavy_input(&clock, 7));
        let (reply, deferred_rx) = oneshot::channel();
        listener.pending_deferred = Some(listener.spawn_deferred(deferred_rx));
        reply.send(vec![delivery]).unwrap();

        // The producer goes quiet. The receiver drains; nothing else happens.
        for _ in 0..CONTROL_EVENT_HEADROOM {
            rx.recv().await.expect("a queued event");
        }
        drained.notify.notify_waiters();
        tokio::time::timeout(std::time::Duration::from_secs(5), async {
            while pending.load(Ordering::Relaxed) == 0 {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("the deferred event must land without another request from the producer");

        // And the next ordering-relevant request finds nothing left to wait for.
        tokio::time::timeout(std::time::Duration::from_secs(1), listener.flush_deferred())
            .await
            .expect("flush must not block once the task is done");
        assert!(listener.pending_deferred.is_none());
    }

    /// A held producer looks idle to the health check — its listener is
    /// parked — so `deliver_when_room` keeps its `last_activity` moving for
    /// as long as it waits, including while the receiver drains without ever
    /// reaching the headroom (fan-in keeps refilling the channel): that path
    /// never hits the stall tick, and used to leave the stamp frozen until
    /// the watchdog killed a producer that was making progress (PR #3590
    /// review).
    #[tokio::test]
    async fn held_producer_stays_alive_to_the_health_check_while_the_receiver_drains() {
        let (listener, _tx) = listener();
        let clock = listener.clock.clone();
        let drained = Arc::new(DrainSignal::default());
        let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        for _ in 0..NODE_EVENT_CHANNEL_CAPACITY - (CONTROL_EVENT_HEADROOM - 1) {
            tx.try_send(input(&clock, 0)).unwrap();
        }
        let (delivery, pending) = deferred(&tx, &drained, input(&clock, 0));
        let last_activity = Arc::new(AtomicU64::new(0));

        let deliver = deliver_when_room(delivery, &last_activity, &listener.backpressure.ft_stats);
        // The receiver drains one event and is refilled by someone else, over
        // and over: notifications keep coming, room never reaches the
        // headroom, and no stall tick ever elapses.
        let churn = async {
            for _ in 0..6 {
                tokio::time::sleep(std::time::Duration::from_millis(50)).await;
                rx.recv().await.expect("a queued event");
                tx.try_send(input(&clock, 0)).unwrap();
                drained.notify.notify_waiters();
            }
            assert_eq!(pending.load(Ordering::Relaxed), 0, "still held");
            assert!(
                last_activity.load(Ordering::Relaxed) > 0,
                "the held producer's last_activity must advance while it waits"
            );
            // Free the headroom so the delivery completes and the test ends.
            for _ in 0..CONTROL_EVENT_HEADROOM {
                rx.recv().await.expect("a queued event");
            }
            drained.notify.notify_waiters();
        };
        tokio::time::timeout(std::time::Duration::from_secs(5), async {
            tokio::join!(deliver, churn)
        })
        .await
        .expect("the delivery completes once the headroom is free");
        assert_eq!(pending.load(Ordering::Relaxed), 1);
    }

    /// A receiver whose event stream is gone cannot be waited for: the
    /// delivery is abandoned, like a `Closed` on the daemon loop's `try_send`,
    /// and the producer is not held.
    #[tokio::test]
    async fn deferred_delivery_to_a_closed_receiver_is_abandoned() {
        let (listener, _tx) = listener();
        let clock = listener.clock.clone();
        let drained = Arc::new(DrainSignal::default());
        let (tx, rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        let (delivery, pending) = deferred(&tx, &drained, input(&clock, 0));
        drop(rx);
        tokio::time::timeout(
            std::time::Duration::from_secs(1),
            deliver_when_room(
                delivery,
                &listener.last_activity,
                &listener.backpressure.ft_stats,
            ),
        )
        .await
        .expect("must not wait on a closed receiver");
        assert_eq!(pending.load(Ordering::Relaxed), 0);
        assert_eq!(
            listener
                .backpressure
                .ft_stats
                .lost_backpressure_messages
                .load(Ordering::Relaxed),
            1,
            "a receiver that did not drop its stream deliberately lost a promised message"
        );
    }

    /// A node that dropped its event stream on purpose (it finished) did not
    /// lose anything it was promised.
    #[tokio::test]
    async fn deferred_delivery_to_a_deliberately_dropped_stream_is_not_a_loss() {
        let (listener, _tx) = listener();
        let clock = listener.clock.clone();
        let drained = Arc::new(DrainSignal::default());
        drained.stream_dropped.store(true, Ordering::Relaxed);
        let (tx, rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        let (delivery, _pending) = deferred(&tx, &drained, input(&clock, 0));
        drop(rx);
        deliver_when_room(
            delivery,
            &listener.last_activity,
            &listener.backpressure.ft_stats,
        )
        .await;
        assert_eq!(
            listener
                .backpressure
                .ft_stats
                .lost_backpressure_messages
                .load(Ordering::Relaxed),
            0
        );
    }

    /// A receiver that frees no room for the stall limit is given up on
    /// once: the held message is dropped and counted, the receiver is marked
    /// `gave_up` so later deliveries to it are not held for another stall
    /// limit each, and the mark goes away as soon as it takes an event
    /// again (dora-rs/dora#3601).
    #[tokio::test(start_paused = true)]
    async fn stall_limit_marks_the_receiver_until_it_drains() {
        let (mut listener, _tx) = listener();
        let clock = listener.clock.clone();
        // The receiver's listener shares its drain signal with the delivery.
        let (tx, rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
        for _ in 0..NODE_EVENT_CHANNEL_CAPACITY {
            tx.try_send(input(&clock, 0)).unwrap();
        }
        listener.subscribed_events = Some(rx);
        let (delivery, pending) = deferred(&tx, &listener.drained, input(&clock, 0));

        let started = tokio::time::Instant::now();
        deliver_when_room(
            delivery,
            &listener.last_activity,
            &listener.backpressure.ft_stats,
        )
        .await;
        assert!(started.elapsed() >= BACKPRESSURE_STALL_LIMIT);
        assert_eq!(pending.load(Ordering::Relaxed), 0);
        assert!(listener.drained.gave_up.load(Ordering::Relaxed));
        let lost = || {
            listener
                .backpressure
                .ft_stats
                .lost_backpressure_messages
                .load(Ordering::Relaxed)
        };
        assert_eq!(lost(), 1);

        // A delivery that was already waiting when the receiver was given up
        // on stops at its next tick instead of serving a stall limit of its own.
        let (delivery, _pending) = deferred(&tx, &listener.drained, input(&clock, 0));
        let started = tokio::time::Instant::now();
        deliver_when_room(
            delivery,
            &listener.last_activity,
            &listener.backpressure.ft_stats,
        )
        .await;
        assert!(started.elapsed() <= BACKPRESSURE_STALL_TICK);
        assert_eq!(lost(), 2);

        // One take frees one slot, which is not the room a held delivery
        // waits for: holding a producer now would stall it for another full
        // stall limit, so the mark stays.
        listener
            .subscribed_events
            .as_mut()
            .unwrap()
            .try_recv()
            .expect("a queued event");
        listener.note_drained();
        assert!(listener.drained.gave_up.load(Ordering::Relaxed));

        // The receiver drains up to the headroom: hold for it again.
        listener.handle_events().await.unwrap();
        assert!(!listener.drained.gave_up.load(Ordering::Relaxed));
    }

    /// A stalled receiver's backlog used to come back in ONE reply; past the
    /// frame limit the whole batch was lost. Replies are now cut at the
    /// budget and the remainder stays queued for the next request.
    #[test]
    fn next_events_reply_is_cut_at_the_frame_budget() {
        let (mut listener, _tx) = listener();
        let clock = listener.clock.clone();
        for _ in 0..5 {
            listener.enqueue(input(&clock, 20 * MIB));
        }
        // 3 x 20 MiB fit a 64 MiB frame; the 4th does not.
        let mut delivered = 0;
        for (expected, remaining) in [(3, 2), (2, 0)] {
            let batch = listener.take_queued_events_within_budget();
            assert_eq!(batch.len(), expected);
            assert_eq!(listener.queue.len(), remaining);
            delivered += batch.len();
            assert!(encoded_reply_len(batch) <= dora_message::MAX_MESSAGE_BYTES);
        }
        assert_eq!(delivered, 5, "nothing is dropped, only deferred");
        assert!(listener.take_queued_events_within_budget().is_empty());
        assert_eq!(listener.queued_bytes, 0, "byte accounting returns to zero");
    }

    #[test]
    fn small_events_still_batch_into_one_reply() {
        let (mut listener, _tx) = listener();
        let clock = listener.clock.clone();
        for _ in 0..500 {
            listener.enqueue(input(&clock, 0));
        }
        assert_eq!(listener.take_queued_events_within_budget().len(), 500);
    }

    /// PR #3429 review: 80 inputs with no payload and a 1 MiB metadata
    /// parameter each. The size hint saw 80 x 128 bytes and batched them all
    /// into an 84 MB frame that the transport refused, losing the backlog.
    /// Exact sizing must cut the batch so every reply fits, and lose nothing.
    #[test]
    fn a_metadata_heavy_backlog_is_cut_so_every_reply_fits_the_frame() {
        let (mut listener, _tx) = listener();
        let clock = listener.clock.clone();
        for _ in 0..80 {
            listener.enqueue(metadata_heavy_input(&clock, MIB));
        }
        let mut delivered = 0;
        let mut replies = 0;
        loop {
            let batch = listener.take_queued_events_within_budget();
            if batch.is_empty() {
                break;
            }
            delivered += batch.len();
            replies += 1;
            assert!(
                encoded_reply_len(batch) <= dora_message::MAX_MESSAGE_BYTES,
                "reply {replies} exceeds the frame limit"
            );
        }
        assert_eq!(delivered, 80, "nothing is dropped, only deferred");
        assert!(replies >= 2, "80 MiB of metadata cannot be one frame");
        assert_eq!(listener.queued_bytes, 0);
    }

    /// Payload and metadata are both counted: events that each fit the
    /// transport, together within the hint's view but not in reality, are
    /// still framed correctly.
    #[test]
    fn payload_plus_metadata_is_sized_exactly() {
        let (mut listener, _tx) = listener();
        let clock = listener.clock.clone();
        for _ in 0..4 {
            listener.enqueue(input(&clock, 20 * MIB));
            listener.enqueue(metadata_heavy_input(&clock, 12 * MIB));
        }
        let mut delivered = 0;
        loop {
            let batch = listener.take_queued_events_within_budget();
            if batch.is_empty() {
                break;
            }
            delivered += batch.len();
            assert!(encoded_reply_len(batch) <= dora_message::MAX_MESSAGE_BYTES);
        }
        assert_eq!(delivered, 8);
    }

    /// An event that alone exceeds the frame limit can never be delivered
    /// (the node-side cap covers the payload, not payload plus metadata). It
    /// is dropped with an error and the queue keeps flowing.
    #[test]
    fn an_undeliverable_event_is_dropped_and_the_queue_keeps_flowing() {
        let (mut listener, _tx) = listener();
        let clock = listener.clock.clone();
        listener.enqueue(metadata_heavy_input(
            &clock,
            dora_message::MAX_MESSAGE_BYTES,
        ));
        listener.enqueue(input(&clock, 0));
        let batch = listener.take_queued_events_within_budget();
        assert_eq!(batch.len(), 1);
        assert!(encoded_reply_len(batch) < MIB, "the small event went out");
        assert!(listener.queue.is_empty());
        assert_eq!(listener.queued_bytes, 0);
    }

    #[test]
    fn saturation_is_by_event_count_or_payload_bytes() {
        assert!(!queue_saturated(LISTENER_QUEUE_MAX_EVENTS - 1, 0));
        assert!(queue_saturated(LISTENER_QUEUE_MAX_EVENTS, 0));
        assert!(!queue_saturated(1, LISTENER_QUEUE_MAX_BYTES - 1));
        assert!(queue_saturated(1, LISTENER_QUEUE_MAX_BYTES));
    }

    /// The listener used to drain the bounded subscription channel into its
    /// unbounded queue regardless of whether the node was asking. It now
    /// stops at the queue bound, leaving the rest in the channel (where the
    /// sender-side policy applies), and resumes once a reply made room.
    #[tokio::test]
    async fn a_saturated_queue_leaves_events_in_the_channel() {
        let (mut listener, tx) = listener();
        let clock = listener.clock.clone();
        for _ in 0..LISTENER_QUEUE_MAX_EVENTS {
            tx.try_send(input(&clock, 0)).unwrap();
        }
        listener.handle_events().await.unwrap();
        assert_eq!(listener.queue.len(), LISTENER_QUEUE_MAX_EVENTS);

        for _ in 0..5 {
            tx.try_send(input(&clock, 0)).unwrap();
        }
        listener.handle_events().await.unwrap();
        assert_eq!(
            listener.queue.len(),
            LISTENER_QUEUE_MAX_EVENTS,
            "saturated: nothing more is pulled from the channel"
        );
        assert!(
            futures::poll!(listener.next_event()).is_pending(),
            "and the select arm does not pull either"
        );

        let batch = listener.take_queued_events_within_budget();
        assert_eq!(batch.len(), LISTENER_QUEUE_MAX_EVENTS);
        listener.handle_events().await.unwrap();
        assert_eq!(listener.queue.len(), 5, "room again: the channel drains");
    }
}
