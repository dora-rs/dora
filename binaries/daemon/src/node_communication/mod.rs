use crate::{
    DaemonNodeEvent, Event, NODE_EVENT_CHANNEL_CAPACITY, NodeEventReceiver, QueuedNodeEvent,
    node_event_channel,
};
use dora_core::{config::NodeId, topics::LOCALHOST, uhlc};
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
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
    task::Poll,
};
use tokio::{
    net::TcpListener,
    sync::{OwnedSemaphorePermit, Semaphore, mpsc, oneshot},
};

pub mod tcp;

pub(crate) const REPLAY_NODE_INGRESS_BYTE_LIMIT: usize = 4 * dora_message::MAX_MESSAGE_BYTES;

pub fn current_millis() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

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
    replay_ingress_budget: Option<Arc<Semaphore>>,
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
        let context = tcp::ListenerContext::new(
            generation,
            daemon_tx,
            clock,
            last_activity,
            replay_ingress_budget,
        );
        tcp::listener_loop(socket, context, shutdown, node_shutdown).await;
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

fn describe_event(event: &Timestamped<NodeEvent>) -> String {
    match &event.inner {
        NodeEvent::Input { id, .. } => format!("input `{id}`"),
        other => format!("{other:?}"),
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
    subscribed_events: Option<NodeEventReceiver>,
    pending_counter: Option<Arc<AtomicU64>>,
    queue: VecDeque<QueuedNodeEvent>,
    /// Sum of the exact wire lengths over `queue`, kept incrementally.
    queued_bytes: usize,
    clock: Arc<uhlc::HLC>,
    last_activity: Arc<AtomicU64>,
    replay_ingress_budget: Option<Arc<Semaphore>>,
}

impl Listener {
    pub(crate) async fn run<C: Connection>(
        mut connection: C,
        generation: Arc<AtomicU64>,
        daemon_tx: mpsc::Sender<Timestamped<Event>>,
        hlc: Arc<uhlc::HLC>,
        last_activity: Arc<AtomicU64>,
        replay_ingress_budget: Option<Arc<Semaphore>>,
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
                            replay_ingress_budget,
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

                self.enqueue_queued(event);
                self.handle_events().await?;
            };

            match message.wrap_err("failed to receive DaemonRequest") {
                Ok(Some(message)) => {
                    if let Err(err) = self.handle_message(message, &mut connection).await {
                        tracing::warn!("{err:?}");
                        if self.replay_ingress_budget.is_some() {
                            return Err(err);
                        }
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
        while !self.queue_saturated() {
            let Some(events) = &mut self.subscribed_events else {
                break;
            };
            let Ok(event) = events.try_recv_queued() else {
                break;
            };
            if let Some(counter) = &self.pending_counter {
                counter.fetch_sub(1, Ordering::Relaxed);
            }
            self.enqueue_queued(event);
        }
        Ok(())
    }

    fn queue_saturated(&self) -> bool {
        queue_saturated(self.queue.len(), self.queued_bytes)
    }

    #[cfg(test)]
    fn enqueue(&mut self, event: Timestamped<NodeEvent>) {
        self.enqueue_queued(QueuedNodeEvent::unbudgeted(event));
    }

    fn enqueue_queued(&mut self, mut event: QueuedNodeEvent) {
        let size = event.ensure_wire_size();
        self.queued_bytes = self.queued_bytes.saturating_add(size);
        self.queue.push_back(event);
    }

    fn pop_front(&mut self) -> Option<QueuedNodeEvent> {
        let sized = self.queue.pop_front()?;
        self.queued_bytes = self
            .queued_bytes
            .saturating_sub(sized.wire_size().expect("queued event was sized"));
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
            let front_size = front.wire_size().expect("queued event was sized");
            if front_size > NEXT_EVENTS_REPLY_BUDGET {
                let Some(dropped) = self.pop_front() else {
                    break;
                };
                tracing::error!(
                    node = %self.node_id,
                    size = dropped.wire_size().expect("queued event was sized"),
                    "dropping {}: its encoding exceeds the {}-byte daemon frame limit and can \
                     never be delivered",
                    describe_event(dropped.event()),
                    dora_message::MAX_MESSAGE_BYTES,
                );
                continue;
            }
            if !batch.is_empty()
                && batch_bytes.saturating_add(front_size) > NEXT_EVENTS_REPLY_BUDGET
            {
                break;
            }
            let Some(sized) = self.pop_front() else {
                break;
            };
            batch_bytes =
                batch_bytes.saturating_add(sized.wire_size().expect("queued event was sized"));
            batch.push(sized.into_event());
        }
        batch
    }

    #[tracing::instrument(skip(self, connection), fields(%self.dataflow_id, %self.node_id), level = "trace")]
    async fn handle_message<C: Connection>(
        &mut self,
        message: Timestamped<DaemonRequest>,
        connection: &mut C,
    ) -> eyre::Result<()> {
        let replay_ingress_permit = if matches!(&message.inner, DaemonRequest::SendMessage { .. }) {
            self.acquire_replay_ingress(&message)?
        } else {
            None
        };
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
                    replay_ingress_permit,
                };
                self.process_daemon_event(event, None, connection).await?;
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
                let (tx, rx) = node_event_channel(crate::NODE_EVENT_CHANNEL_CAPACITY);
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
                // Take queued events first, bounded by encoded size so the
                // reply fits one frame (see `NEXT_EVENTS_REPLY_BUDGET`).
                let queued_events = self.take_queued_events_within_budget();
                let reply = if queued_events.is_empty() {
                    match self.subscribed_events.as_mut() {
                        // wait for next event
                        Some(events) => match events.recv_queued().await {
                            Some(mut event) => {
                                if let Some(counter) = &self.pending_counter {
                                    counter.fetch_sub(1, Ordering::Relaxed);
                                }
                                event.ensure_wire_size();
                                DaemonReply::NextEvents(vec![event.into_event()])
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

    fn acquire_replay_ingress(
        &self,
        request: &Timestamped<DaemonRequest>,
    ) -> eyre::Result<Option<OwnedSemaphorePermit>> {
        let Some(budget) = &self.replay_ingress_budget else {
            return Ok(None);
        };
        let wire_size = dora_message::serialized_size(request)
            .wrap_err("failed to size verified replay SendMessage request")?;
        let wire_size = u32::try_from(wire_size)
            .wrap_err("verified replay SendMessage request exceeds byte admission range")?;
        budget
            .clone()
            .try_acquire_many_owned(wire_size)
            .map(Some)
            .map_err(|_| {
                eyre!(
                    "verified replay producer exceeded the {}-byte daemon ingress limit",
                    REPLAY_NODE_INGRESS_BYTE_LIMIT
                )
            })
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
    fn next_event(&mut self) -> impl Future<Output = QueuedNodeEvent> + Unpin + '_ {
        let poll = |cx: &mut task::Context<'_>| {
            // A saturated queue takes nothing more from the channel. The
            // caller recreates this future after every request, so polling
            // resumes as soon as a `NextEvent` reply has made room.
            if queue_saturated(self.queue.len(), self.queued_bytes) {
                return Poll::Pending;
            }
            if let Some(events) = &mut self.subscribed_events {
                match events.poll_recv_queued(cx) {
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

    fn listener() -> (Listener, crate::NodeEventSender) {
        let (daemon_tx, _daemon_rx) = mpsc::channel(1);
        let (tx, rx) = node_event_channel(NODE_EVENT_CHANNEL_CAPACITY);
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
            replay_ingress_budget: None,
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

    #[tokio::test]
    async fn replay_byte_permit_covers_subscription_and_listener_queues() {
        let (mut listener, tx) = listener();
        let clock = listener.clock.clone();
        let event = metadata_heavy_input(&clock, 4096);
        let size = dora_message::serialized_size(&event).unwrap();
        tx.enable_data_byte_limit(size);

        tx.try_send(event).unwrap();
        assert_eq!(tx.available_data_bytes(), Some(0));

        listener.handle_events().await.unwrap();
        assert_eq!(listener.queue.len(), 1);
        assert_eq!(tx.available_data_bytes(), Some(0));

        assert_eq!(listener.take_queued_events_within_budget().len(), 1);
        assert_eq!(tx.available_data_bytes(), Some(size));
    }
}
