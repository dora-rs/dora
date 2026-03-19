use std::{
    collections::{BTreeSet, HashMap, HashSet},
    sync::Arc,
};

use dora_core::{config::NodeId, uhlc::HLC};
use dora_message::{
    DataflowId,
    common::{DaemonId, Timestamped},
    daemon_to_coordinator::{CoordinatorNotifyClient, LogMessage},
    daemon_to_node::DaemonReply,
    tarpc,
};
use eyre::bail;
use tokio::sync::{mpsc, oneshot};

use crate::{CascadingErrorCauses, Event, log::DataflowLogger};

pub struct PendingNodes {
    dataflow_id: DataflowId,
    #[allow(dead_code)]
    daemon_id: DaemonId,

    /// The local nodes that are still waiting to start.
    local_nodes: HashSet<NodeId>,
    /// Whether there are external nodes for this dataflow.
    external_nodes: bool,

    /// Used to synchronize node starts.
    ///
    /// Subscribe requests block the node until all other nodes are ready too.
    waiting_subscribers: HashMap<NodeId, oneshot::Sender<DaemonReply>>,
    /// List of nodes that finished before connecting to the dora daemon.
    ///
    /// If this list is non-empty, we should not start the dataflow at all. Instead,
    /// we report an error to the other nodes.
    exited_before_subscribe: Vec<NodeId>,

    /// Whether the local init result was already reported to the coordinator.
    reported_init_to_coordinator: bool,

    /// Channel to send fatal errors back to the daemon event loop.
    events_tx: mpsc::Sender<Timestamped<Event>>,
    /// Clock for timestamping events sent to the event loop.
    clock: Arc<HLC>,
}

impl PendingNodes {
    pub fn new(
        dataflow_id: DataflowId,
        daemon_id: DaemonId,
        events_tx: mpsc::Sender<Timestamped<Event>>,
        clock: Arc<HLC>,
    ) -> Self {
        Self {
            dataflow_id,
            daemon_id,
            local_nodes: HashSet::new(),
            external_nodes: false,
            waiting_subscribers: HashMap::new(),
            exited_before_subscribe: Default::default(),
            reported_init_to_coordinator: false,
            events_tx,
            clock,
        }
    }

    pub fn insert(&mut self, node_id: NodeId) {
        self.local_nodes.insert(node_id);
    }

    pub fn set_external_nodes(&mut self, value: bool) {
        self.external_nodes = value;
    }

    pub fn local_nodes_pending(&self) -> bool {
        !self.local_nodes.is_empty()
    }

    pub fn local_nodes_snapshot(&self) -> Vec<NodeId> {
        self.local_nodes.iter().cloned().collect()
    }

    pub async fn handle_node_subscription(
        &mut self,
        node_id: NodeId,
        reply_sender: oneshot::Sender<DaemonReply>,
        coordinator_client: &Option<CoordinatorNotifyClient>,
        clock: &HLC,
        cascading_errors: &mut CascadingErrorCauses,
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<DataflowStatus> {
        self.waiting_subscribers
            .insert(node_id.clone(), reply_sender);
        self.local_nodes.remove(&node_id);

        self.update_dataflow_status(coordinator_client, clock, cascading_errors, logger)
            .await
    }

    pub async fn handle_node_stop(
        &mut self,
        node_id: &NodeId,
        coordinator_client: &Option<CoordinatorNotifyClient>,
        clock: &HLC,
        cascading_errors: &mut CascadingErrorCauses,
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<()> {
        if self.local_nodes.remove(node_id) {
            logger
                .log(
                    dora_message::daemon_to_coordinator::LogLevel::Warn,
                    Some(node_id.clone()),
                    Some("daemon::pending".into()),
                    "node exited before initializing dora connection",
                )
                .await;
            self.exited_before_subscribe.push(node_id.clone());
            self.update_dataflow_status(coordinator_client, clock, cascading_errors, logger)
                .await?;
        }
        Ok(())
    }

    /// Synchronous part of [`handle_node_stop`] that only mutates local state
    /// without doing any async I/O. Use [`check_and_answer_subscribers`] +
    /// a manual RPC call afterwards to complete the reporting.
    ///
    /// Returns `true` if the node was pending and was removed, indicating
    /// that the caller should log the event via the logger.
    pub fn handle_node_stop_sync(
        &mut self,
        node_id: &NodeId,
        cascading_errors: &mut CascadingErrorCauses,
    ) -> bool {
        if self.local_nodes.remove(node_id) {
            self.exited_before_subscribe.push(node_id.clone());
            // Check and answer locally without RPC — caller must handle
            // reporting to coordinator separately.
            if self.local_nodes.is_empty() && !self.external_nodes {
                self.answer_subscribe_requests(Vec::new(), cascading_errors);
            }
            true
        } else {
            false
        }
    }

    /// Check if all local nodes are ready and, if so, answer waiting
    /// subscribers. Returns `true` if the caller should report readiness
    /// to the coordinator (i.e., `external_nodes` is true, all local nodes
    /// are ready, and we haven't reported yet).
    pub fn check_and_answer_subscribers(
        &mut self,
        cascading_errors: &mut CascadingErrorCauses,
    ) -> bool {
        if !self.local_nodes.is_empty() {
            return false;
        }
        if self.external_nodes {
            if !self.reported_init_to_coordinator {
                self.reported_init_to_coordinator = true;
                true
            } else {
                false
            }
        } else {
            self.answer_subscribe_requests(Vec::new(), cascading_errors);
            false
        }
    }

    /// Access the list of nodes that exited before subscribing.
    pub fn exited_before_subscribe(&self) -> &[NodeId] {
        &self.exited_before_subscribe
    }

    pub async fn handle_dataflow_stop(
        &mut self,
        coordinator_client: &Option<CoordinatorNotifyClient>,
        clock: &HLC,
        cascading_errors: &mut CascadingErrorCauses,
        dynamic_nodes: &BTreeSet<NodeId>,
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<Vec<LogMessage>> {
        // remove all local dynamic nodes that are not yet started
        for node_id in dynamic_nodes {
            if self.local_nodes.remove(node_id) {
                self.update_dataflow_status(coordinator_client, clock, cascading_errors, logger)
                    .await?;
            }
        }

        Ok(Vec::new())
    }

    pub async fn handle_external_all_nodes_ready(
        &mut self,
        exited_before_subscribe: Vec<NodeId>,
        cascading_errors: &mut CascadingErrorCauses,
    ) -> eyre::Result<()> {
        if !self.local_nodes.is_empty() {
            bail!("received external `all_nodes_ready` event before local nodes were ready");
        }

        self.answer_subscribe_requests(exited_before_subscribe, cascading_errors);

        Ok(())
    }

    async fn update_dataflow_status(
        &mut self,
        coordinator_client: &Option<CoordinatorNotifyClient>,
        clock: &HLC,
        cascading_errors: &mut CascadingErrorCauses,
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<DataflowStatus> {
        if self.local_nodes.is_empty() {
            if self.external_nodes {
                if !self.reported_init_to_coordinator {
                    self.report_nodes_ready(coordinator_client, clock, logger)
                        .await?;
                    self.reported_init_to_coordinator = true;
                }
                Ok(DataflowStatus::Pending)
            } else {
                self.answer_subscribe_requests(Vec::new(), cascading_errors);
                Ok(DataflowStatus::AllNodesReady)
            }
        } else {
            Ok(DataflowStatus::Pending)
        }
    }

    fn answer_subscribe_requests(
        &mut self,
        exited_before_subscribe_external: Vec<NodeId>,
        cascading_errors: &mut CascadingErrorCauses,
    ) {
        let node_exited_before_subscribe = match self.exited_before_subscribe.as_slice() {
            [first, ..] => Some(first),
            [] => match exited_before_subscribe_external.as_slice() {
                [first, ..] => Some(first),
                [] => None,
            },
        };

        let result = match &node_exited_before_subscribe {
            Some(causing_node) => Err(format!(
                "Node {causing_node} exited before initializing dora. For \
                more information, run `dora logs {} {causing_node}`.",
                self.dataflow_id
            )),
            None => Ok(()),
        };

        // answer all subscribe requests
        let subscribe_replies = std::mem::take(&mut self.waiting_subscribers);
        for (node_id, reply_sender) in subscribe_replies.into_iter() {
            if let Some(causing_node) = node_exited_before_subscribe {
                cascading_errors.report_cascading_error(causing_node.clone(), node_id.clone());
            }
            let _ = reply_sender.send(DaemonReply::Result(result.clone()));
        }
    }

    async fn report_nodes_ready(
        &self,
        coordinator_client: &Option<CoordinatorNotifyClient>,
        _clock: &HLC,
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<()> {
        let Some(client) = coordinator_client else {
            bail!("no coordinator client to send AllNodesReady");
        };

        logger
            .log(
                dora_message::daemon_to_coordinator::LogLevel::Info,
                None,
                Some("daemon".into()),
                format!(
                    "all local nodes are ready (exit before subscribe: {:?}), waiting for remote nodes",
                    self.exited_before_subscribe
                ),
            )
            .await;

        let client = client.clone();
        let dataflow_id = self.dataflow_id;
        let exited = self.exited_before_subscribe.clone();
        let events_tx = self.events_tx.clone();
        let clock = self.clock.clone();
        tokio::spawn(async move {
            if let Err(err) = client
                .all_nodes_ready(tarpc::context::current(), dataflow_id, exited)
                .await
            {
                let _ = events_tx
                    .send(Timestamped {
                        inner: Event::DaemonError(
                            eyre::eyre!(err).wrap_err("failed to send AllNodesReady RPC"),
                        ),
                        timestamp: clock.new_timestamp(),
                    })
                    .await;
            }
        });

        Ok(())
    }
}

pub enum DataflowStatus {
    AllNodesReady,
    Pending,
}
