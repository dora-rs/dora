use crate::{DaemonNodeEvent, Event};
use dora_core::{
    config::{DataId, NodeId},
    topics::LOCALHOST,
    uhlc,
};
use dora_message::{
    DataflowId,
    common::{DropToken, Timestamped},
    daemon_to_node::{DaemonCommunication, DaemonReply, NodeConfig, NodeDropEvent, NodeEvent},
    metadata::Metadata,
    node_to_daemon::{DataMessage, NodeControl, NodeRegisterRequest},
    tarpc,
};
use eyre::Context;
use std::{collections::BTreeMap, mem, sync::Arc};
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
    queue_sizes: BTreeMap<DataId, usize>,
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
        tcp::listener_loop(socket, daemon_tx, queue_sizes, clock).await;
        tracing::debug!("event listener loop finished for `{event_loop_node_id}`");
    });
    let abort_handle = handle.abort_handle();

    Ok((DaemonCommunication::Tcp { socket_addr }, Some(abort_handle)))
}

/// Shared mutable state for the tarpc NodeControl server.
///
/// Since tarpc clones the server struct for each request, we wrap the mutable
/// state in `Arc<Mutex<...>>`.
struct NodeControlState {
    dataflow_id: Option<DataflowId>,
    node_id: Option<NodeId>,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    subscribed_events: Option<UnboundedReceiver<Timestamped<NodeEvent>>>,
    subscribed_drop_events: Option<UnboundedReceiver<Timestamped<NodeDropEvent>>>,
    event_queue: Vec<Timestamped<NodeEvent>>,
    clock: Arc<uhlc::HLC>,
}

/// tarpc-based server that implements `NodeControl`.
#[derive(Clone)]
pub(crate) struct NodeControlServer {
    state: Arc<Mutex<NodeControlState>>,
}

impl NodeControlServer {
    pub(crate) fn new(daemon_tx: mpsc::Sender<Timestamped<Event>>, clock: Arc<uhlc::HLC>) -> Self {
        Self {
            state: Arc::new(Mutex::new(NodeControlState {
                dataflow_id: None,
                node_id: None,
                daemon_tx,
                subscribed_events: None,
                subscribed_drop_events: None,
                event_queue: Vec::new(),
                clock,
            })),
        }
    }
}


impl NodeControlServer {
    /// Send a fire-and-forget event to the daemon main loop (no reply expected).
    async fn send_daemon_event_no_reply(
        state: &mut NodeControlState,
        event: DaemonNodeEvent,
    ) -> Result<(), String> {
        let dataflow_id = state
            .dataflow_id
            .ok_or_else(|| "node not registered".to_string())?;
        let node_id = state
            .node_id
            .clone()
            .ok_or_else(|| "node not registered".to_string())?;

        let daemon_event = Event::Node {
            dataflow_id,
            node_id,
            event,
        };
        let timestamped = Timestamped {
            inner: daemon_event,
            timestamp: state.clock.new_timestamp(),
        };
        state
            .daemon_tx
            .send(timestamped)
            .await
            .map_err(|_| "failed to send event to daemon".to_string())
    }

    /// Send a DaemonNodeEvent that carries its own oneshot reply_sender, and wait for the reply.
    async fn send_daemon_event_with_reply(
        state: &mut NodeControlState,
        event: DaemonNodeEvent,
        reply_rx: oneshot::Receiver<DaemonReply>,
    ) -> Result<DaemonReply, String> {
        let dataflow_id = state
            .dataflow_id
            .ok_or_else(|| "node not registered".to_string())?;
        let node_id = state
            .node_id
            .clone()
            .ok_or_else(|| "node not registered".to_string())?;

        let daemon_event = Event::Node {
            dataflow_id,
            node_id,
            event,
        };
        let timestamped = Timestamped {
            inner: daemon_event,
            timestamp: state.clock.new_timestamp(),
        };
        state
            .daemon_tx
            .send(timestamped)
            .await
            .map_err(|_| "failed to send event to daemon".to_string())?;

        reply_rx
            .await
            .map_err(|_| "failed to receive reply from daemon".to_string())
    }

    async fn report_drop_tokens_inner(
        state: &mut NodeControlState,
        drop_tokens: Vec<DropToken>,
    ) -> Result<(), String> {
        if drop_tokens.is_empty() {
            return Ok(());
        }
        let dataflow_id = state
            .dataflow_id
            .ok_or_else(|| "node not registered".to_string())?;
        let node_id = state
            .node_id
            .clone()
            .ok_or_else(|| "node not registered".to_string())?;

        let event = Event::Node {
            dataflow_id,
            node_id,
            event: DaemonNodeEvent::ReportDrop {
                tokens: drop_tokens,
            },
        };
        let timestamped = Timestamped {
            inner: event,
            timestamp: state.clock.new_timestamp(),
        };
        state
            .daemon_tx
            .send(timestamped)
            .await
            .map_err(|_| "failed to report drop tokens to daemon".to_string())
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
        request: NodeRegisterRequest,
    ) -> Result<(), String> {
        request.check_version()?;

        let mut state = self.state.lock().await;
        state.dataflow_id = Some(request.dataflow_id);
        state.node_id = Some(request.node_id);
        Ok(())
    }

    async fn subscribe(self, _context: tarpc::context::Context) -> Result<(), String> {
        let mut state = self.state.lock().await;
        let (tx, rx) = mpsc::unbounded_channel();
        let (reply_sender, reply_rx) = oneshot::channel();

        let reply = Self::send_daemon_event_with_reply(
            &mut state,
            DaemonNodeEvent::Subscribe {
                event_sender: tx,
                reply_sender,
            },
            reply_rx,
        )
        .await?;
        state.subscribed_events = Some(rx);
        Self::extract_result(reply)
    }

    async fn subscribe_drop(self, _context: tarpc::context::Context) -> Result<(), String> {
        let mut state = self.state.lock().await;
        let (tx, rx) = mpsc::unbounded_channel();
        let (reply_sender, reply_rx) = oneshot::channel();

        let reply = Self::send_daemon_event_with_reply(
            &mut state,
            DaemonNodeEvent::SubscribeDrop {
                event_sender: tx,
                reply_sender,
            },
            reply_rx,
        )
        .await?;
        state.subscribed_drop_events = Some(rx);
        Self::extract_result(reply)
    }

    async fn next_event(
        self,
        _context: tarpc::context::Context,
        drop_tokens: Vec<DropToken>,
    ) -> Vec<Timestamped<NodeEvent>> {
        let mut state = self.state.lock().await;

        // Report drop tokens
        if let Err(err) = Self::report_drop_tokens_inner(&mut state, drop_tokens).await {
            tracing::warn!("failed to report drop tokens: {err}");
        }

        // Drain any queued events first
        // Also drain any ready events from the channel
        {
            let mut drained = Vec::new();
            if let Some(events) = &mut state.subscribed_events {
                while let Ok(event) = events.try_recv() {
                    drained.push(event);
                }
            }
            state.event_queue.extend(drained);
        }

        if !state.event_queue.is_empty() {
            return mem::take(&mut state.event_queue);
        }

        // Wait for the next event
        match state.subscribed_events.as_mut() {
            Some(events) => {
                // We need to drop the lock while waiting, otherwise other requests
                // would deadlock. Use a pattern where we take the receiver out,
                // drop the lock, wait, then put it back.
                // But since tarpc serializes calls through the server, we can
                // just hold the lock.
                match events.recv().await {
                    Some(event) => vec![event],
                    None => vec![],
                }
            }
            None => vec![],
        }
    }

    async fn next_finished_drop_tokens(
        self,
        _context: tarpc::context::Context,
    ) -> Vec<Timestamped<NodeDropEvent>> {
        let mut state = self.state.lock().await;

        match state.subscribed_drop_events.as_mut() {
            Some(events) => match events.recv().await {
                Some(event) => vec![event],
                None => vec![],
            },
            None => vec![],
        }
    }

    async fn send_message(
        self,
        _context: tarpc::context::Context,
        output_id: DataId,
        metadata: Metadata,
        data: Option<DataMessage>,
    ) {
        let mut state = self.state.lock().await;
        let event = DaemonNodeEvent::SendOut {
            output_id,
            metadata,
            data,
        };
        if let Err(err) = Self::send_daemon_event_no_reply(&mut state, event).await {
            tracing::warn!("failed to send message to daemon: {err}");
        }
    }

    async fn close_outputs(
        self,
        _context: tarpc::context::Context,
        outputs: Vec<DataId>,
    ) -> Result<(), String> {
        let mut state = self.state.lock().await;
        let (reply_sender, reply_rx) = oneshot::channel();
        let reply = Self::send_daemon_event_with_reply(
            &mut state,
            DaemonNodeEvent::CloseOutputs {
                outputs,
                reply_sender,
            },
            reply_rx,
        )
        .await?;
        Self::extract_result(reply)
    }

    async fn outputs_done(self, _context: tarpc::context::Context) -> Result<(), String> {
        let mut state = self.state.lock().await;
        let (reply_sender, reply_rx) = oneshot::channel();
        let reply = Self::send_daemon_event_with_reply(
            &mut state,
            DaemonNodeEvent::OutputsDone { reply_sender },
            reply_rx,
        )
        .await?;
        Self::extract_result(reply)
    }

    async fn report_drop_tokens(
        self,
        _context: tarpc::context::Context,
        drop_tokens: Vec<DropToken>,
    ) {
        let mut state = self.state.lock().await;
        if let Err(err) = Self::report_drop_tokens_inner(&mut state, drop_tokens).await {
            tracing::warn!("failed to report drop tokens: {err}");
        }
    }

    async fn event_stream_dropped(self, _context: tarpc::context::Context) -> Result<(), String> {
        let mut state = self.state.lock().await;
        let (reply_sender, reply_rx) = oneshot::channel();
        let reply = Self::send_daemon_event_with_reply(
            &mut state,
            DaemonNodeEvent::EventStreamDropped { reply_sender },
            reply_rx,
        )
        .await?;
        Self::extract_result(reply)
    }

    async fn node_config(
        self,
        _context: tarpc::context::Context,
        _node_id: NodeId,
    ) -> Result<NodeConfig, String> {
        Err("unexpected node config request on per-node channel".to_string())
    }
}
