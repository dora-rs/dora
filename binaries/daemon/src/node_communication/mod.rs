use crate::{DaemonNodeEvent, Event};
use dora_core::{
    config::{DataId, NodeId},
    topics::LOCALHOST,
    uhlc,
};
use dora_message::{
    DataflowId,
    common::{DropToken, Timestamped},
    daemon_to_node::{DaemonCommunication, DaemonReply, NodeDropEvent, NodeEvent},
    metadata::Metadata,
    node_to_daemon::{DataMessage, NodeControl, NodeRegisterRequest},
    tarpc,
};
use eyre::Context;
use std::{
    collections::BTreeMap,
    sync::{Arc, OnceLock},
    time::Duration,
};
use tokio::{
    net::TcpListener,
    sync::{
        Mutex,
        mpsc::{self, UnboundedReceiver},
        oneshot,
    },
};

pub mod tcp;

pub async fn spawn_listener_loop(
    dataflow_id: &DataflowId,
    node_id: &NodeId,
    daemon_tx: &mpsc::Sender<Timestamped<Event>>,
    _queue_sizes: BTreeMap<DataId, usize>,
    clock: Arc<uhlc::HLC>,
) -> eyre::Result<(DaemonCommunication, Option<tokio::task::AbortHandle>)> {
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
    let handle = tokio::spawn(async move {
        tcp::listener_loop(socket, daemon_tx, clock).await;
        tracing::debug!("event listener loop finished for `{event_loop_node_id}`");
    });
    let abort_handle = handle.abort_handle();

    Ok((DaemonCommunication::Tcp { socket_addr }, Some(abort_handle)))
}

/// Server-side keepalive timeout for long-polling RPCs.
/// Must be shorter than the client-side `long_context()` deadline so the
/// server replies before tarpc times out the call.
const KEEPALIVE_TIMEOUT: Duration = Duration::from_secs(300);

/// Registration info, set once via `register` and then read-only.
struct Registration {
    dataflow_id: DataflowId,
    node_id: NodeId,
}

/// tarpc-based server that implements `NodeControl`.
///
/// Fields are either thread-safe (`mpsc::Sender`, `Arc`), set-once (`OnceLock`),
/// or per-field `Mutex`. The per-field mutexes protect the event/drop receivers,
/// which are each used by exactly one RPC method.
#[derive(Clone)]
pub(crate) struct NodeControlServer {
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    clock: Arc<uhlc::HLC>,
    registration: Arc<OnceLock<Registration>>,
    subscribed_events: Arc<Mutex<Option<UnboundedReceiver<Timestamped<NodeEvent>>>>>,
    subscribed_drop_events: Arc<Mutex<Option<UnboundedReceiver<Timestamped<NodeDropEvent>>>>>,
}

impl NodeControlServer {
    pub(crate) fn new(daemon_tx: mpsc::Sender<Timestamped<Event>>, clock: Arc<uhlc::HLC>) -> Self {
        Self {
            daemon_tx,
            clock,
            registration: Arc::new(OnceLock::new()),
            subscribed_events: Arc::new(Mutex::new(None)),
            subscribed_drop_events: Arc::new(Mutex::new(None)),
        }
    }

    fn registration(&self) -> Result<&Registration, String> {
        self.registration
            .get()
            .ok_or_else(|| "node not registered".to_string())
    }

    /// Synchronize the daemon's HLC with the node's timestamp.
    fn sync_clock(&self, timestamp: &uhlc::Timestamp) {
        if let Err(err) = self.clock.update_with_timestamp(timestamp) {
            tracing::warn!("failed to update HLC: {err}");
        }
    }

    /// Send a fire-and-forget event to the daemon main loop (no reply expected).
    async fn send_daemon_event_no_reply(&self, event: DaemonNodeEvent) -> Result<(), String> {
        let reg = self.registration()?;
        let timestamped = Timestamped {
            inner: Event::Node {
                dataflow_id: reg.dataflow_id,
                node_id: reg.node_id.clone(),
                event,
            },
            timestamp: self.clock.new_timestamp(),
        };
        self.daemon_tx
            .send(timestamped)
            .await
            .map_err(|_| "failed to send event to daemon".to_string())
    }

    /// Send a DaemonNodeEvent that carries its own oneshot reply_sender, and wait for the reply.
    async fn send_daemon_event_with_reply(
        &self,
        event: DaemonNodeEvent,
        reply_rx: oneshot::Receiver<DaemonReply>,
    ) -> Result<DaemonReply, String> {
        self.send_daemon_event_no_reply(event).await?;
        reply_rx
            .await
            .map_err(|_| "failed to receive reply from daemon".to_string())
    }

    async fn report_drop_tokens_inner(&self, drop_tokens: Vec<DropToken>) -> Result<(), String> {
        if drop_tokens.is_empty() {
            return Ok(());
        }
        self.send_daemon_event_no_reply(DaemonNodeEvent::ReportDrop {
            tokens: drop_tokens,
        })
        .await
    }

    /// Extract the Result payload from a DaemonReply.
    fn extract_result(reply: DaemonReply) -> Result<(), String> {
        match reply {
            DaemonReply::Result(r) => r,
            DaemonReply::Empty => Ok(()),
            other => Err(format!("unexpected reply: {other:?}")),
        }
    }
}

impl NodeControl for NodeControlServer {
    async fn register(
        self,
        _context: tarpc::context::Context,
        timestamp: uhlc::Timestamp,
        request: NodeRegisterRequest,
    ) -> Result<(), String> {
        self.sync_clock(&timestamp);
        request.check_version()?;
        self.registration
            .set(Registration {
                dataflow_id: request.dataflow_id,
                node_id: request.node_id,
            })
            .map_err(|_| "node already registered".to_string())
    }

    async fn subscribe(
        self,
        _context: tarpc::context::Context,
        timestamp: uhlc::Timestamp,
    ) -> Result<(), String> {
        self.sync_clock(&timestamp);
        let (tx, rx) = mpsc::unbounded_channel();
        let (reply_sender, reply_rx) = oneshot::channel();
        let reply = self
            .send_daemon_event_with_reply(
                DaemonNodeEvent::Subscribe {
                    event_sender: tx,
                    reply_sender,
                },
                reply_rx,
            )
            .await?;
        *self.subscribed_events.lock().await = Some(rx);
        Self::extract_result(reply)
    }

    async fn subscribe_drop(
        self,
        _context: tarpc::context::Context,
        timestamp: uhlc::Timestamp,
    ) -> Result<(), String> {
        self.sync_clock(&timestamp);
        let (tx, rx) = mpsc::unbounded_channel();
        let (reply_sender, reply_rx) = oneshot::channel();
        let reply = self
            .send_daemon_event_with_reply(
                DaemonNodeEvent::SubscribeDrop {
                    event_sender: tx,
                    reply_sender,
                },
                reply_rx,
            )
            .await?;
        *self.subscribed_drop_events.lock().await = Some(rx);
        Self::extract_result(reply)
    }

    async fn next_event(
        self,
        _context: tarpc::context::Context,
        timestamp: uhlc::Timestamp,
        drop_tokens: Vec<DropToken>,
    ) -> Option<Vec<Timestamped<NodeEvent>>> {
        self.sync_clock(&timestamp);
        // Report drop tokens (no lock needed — uses only thread-safe fields)
        if let Err(err) = self.report_drop_tokens_inner(drop_tokens).await {
            tracing::warn!("failed to report drop tokens: {err}");
        }

        // Hold the lock across the await to prevent concurrent next_event calls
        // from racing on the receiver.
        let mut guard = self.subscribed_events.lock().await;
        match guard.as_mut() {
            Some(rx) => {
                // Drain any already-buffered events
                let mut events = Vec::new();
                while let Ok(event) = rx.try_recv() {
                    events.push(event);
                }
                if events.is_empty() {
                    // Nothing buffered — wait with a keepalive timeout
                    match tokio::time::timeout(KEEPALIVE_TIMEOUT, rx.recv()).await {
                        Ok(Some(event)) => Some(vec![event]),
                        Ok(None) => Some(vec![]), // channel closed
                        Err(_) => None,           // timeout → keepalive
                    }
                } else {
                    Some(events)
                }
            }
            None => {
                tracing::warn!("next_event called before subscribe — returning keepalive");
                None
            }
        }
    }

    async fn next_finished_drop_tokens(
        self,
        _context: tarpc::context::Context,
        timestamp: uhlc::Timestamp,
    ) -> Option<Vec<Timestamped<NodeDropEvent>>> {
        self.sync_clock(&timestamp);
        // Hold the lock across the await to prevent concurrent calls from
        // racing on the receiver.
        let mut guard = self.subscribed_drop_events.lock().await;
        match guard.as_mut() {
            Some(rx) => {
                match tokio::time::timeout(KEEPALIVE_TIMEOUT, rx.recv()).await {
                    Ok(Some(event)) => Some(vec![event]),
                    Ok(None) => Some(vec![]), // channel closed
                    Err(_) => None,           // timeout → keepalive
                }
            }
            None => {
                tracing::warn!(
                    "next_finished_drop_tokens called before subscribe_drop — returning keepalive"
                );
                None
            }
        }
    }

    async fn send_message(
        self,
        _context: tarpc::context::Context,
        timestamp: uhlc::Timestamp,
        output_id: DataId,
        metadata: Metadata,
        data: Option<DataMessage>,
    ) -> Result<(), String> {
        self.sync_clock(&timestamp);
        let event = DaemonNodeEvent::SendOut {
            output_id,
            metadata,
            data,
        };
        self.send_daemon_event_no_reply(event).await
    }

    async fn close_outputs(
        self,
        _context: tarpc::context::Context,
        timestamp: uhlc::Timestamp,
        outputs: Vec<DataId>,
    ) -> Result<(), String> {
        self.sync_clock(&timestamp);
        let (reply_sender, reply_rx) = oneshot::channel();
        let reply = self
            .send_daemon_event_with_reply(
                DaemonNodeEvent::CloseOutputs {
                    outputs,
                    reply_sender,
                },
                reply_rx,
            )
            .await?;
        Self::extract_result(reply)
    }

    async fn outputs_done(
        self,
        _context: tarpc::context::Context,
        timestamp: uhlc::Timestamp,
    ) -> Result<(), String> {
        self.sync_clock(&timestamp);
        let (reply_sender, reply_rx) = oneshot::channel();
        let reply = self
            .send_daemon_event_with_reply(DaemonNodeEvent::OutputsDone { reply_sender }, reply_rx)
            .await?;
        Self::extract_result(reply)
    }

    async fn report_drop_tokens(
        self,
        _context: tarpc::context::Context,
        timestamp: uhlc::Timestamp,
        drop_tokens: Vec<DropToken>,
    ) -> Result<(), String> {
        self.sync_clock(&timestamp);
        self.report_drop_tokens_inner(drop_tokens).await
    }

    async fn event_stream_dropped(
        self,
        _context: tarpc::context::Context,
        timestamp: uhlc::Timestamp,
    ) -> Result<(), String> {
        self.sync_clock(&timestamp);
        let (reply_sender, reply_rx) = oneshot::channel();
        let reply = self
            .send_daemon_event_with_reply(
                DaemonNodeEvent::EventStreamDropped { reply_sender },
                reply_rx,
            )
            .await?;
        Self::extract_result(reply)
    }

    async fn node_config(
        self,
        _context: tarpc::context::Context,
        timestamp: uhlc::Timestamp,
        _node_id: NodeId,
    ) -> Result<String, String> {
        self.sync_clock(&timestamp);
        Err("unexpected node config request on per-node channel".to_string())
    }
}
