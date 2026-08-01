use std::collections::{BTreeSet, HashMap, HashSet};

use dora_core::{
    config::NodeId,
    uhlc::{HLC, Timestamp},
};
use dora_message::{
    DataflowId,
    common::DaemonId,
    daemon_to_coordinator::{CoordinatorRequest, DaemonEvent, LogLevel, Timestamped},
    daemon_to_node::DaemonReply,
};
use eyre::{Context, bail};
use tokio::sync::oneshot;

use crate::{CascadingErrorCauses, coordinator::CoordinatorSender, log::DataflowLogger};

pub struct PendingNodes {
    dataflow_id: DataflowId,
    daemon_id: DaemonId,

    /// The local nodes that are still waiting to start.
    local_nodes: HashSet<NodeId>,
    /// Whether there are external nodes for this dataflow.
    external_nodes: bool,

    /// Used to synchronize node starts.
    ///
    /// Subscribe requests block the node until all other nodes are ready too.
    waiting_subscribers: HashMap<NodeId, oneshot::Sender<DaemonReply>>,
    /// Nodes added to this dataflow at runtime (`dora node add`) rather
    /// than declared in its descriptor.
    ///
    /// They still wait on the barrier — a node added while the dataflow
    /// is coming up should not start producing before its consumers are
    /// listening — but they are not *part* of the startup cohort:
    /// their failures never gate the cohort, and the cohort's failures
    /// are never reported as theirs (dora-rs/dora#2917).
    runtime_added: HashSet<NodeId>,
    /// List of nodes that finished before connecting to the dora daemon.
    ///
    /// If this list is non-empty, we should not start the dataflow at all. Instead,
    /// we report an error to the other nodes.
    ///
    /// Only ever holds startup-cohort nodes: runtime additions are kept
    /// out of `local_nodes`, so `handle_node_stop` cannot record them
    /// here. That scoping matters because this list is never cleared —
    /// were a runtime addition able to land in it, one crashed node
    /// would poison every later `dora node add` for the life of the
    /// dataflow (dora-rs/dora#2917).
    exited_before_subscribe: Vec<NodeId>,

    /// Whether the local init result was already reported to the coordinator.
    reported_init_to_coordinator: bool,
}

impl PendingNodes {
    pub fn new(dataflow_id: DataflowId, daemon_id: DaemonId) -> Self {
        Self {
            dataflow_id,
            daemon_id,
            local_nodes: HashSet::new(),
            external_nodes: false,
            waiting_subscribers: HashMap::new(),
            runtime_added: HashSet::new(),
            exited_before_subscribe: Default::default(),
            reported_init_to_coordinator: false,
        }
    }

    pub fn insert(&mut self, node_id: NodeId) {
        self.local_nodes.insert(node_id);
    }

    /// Register a node added at runtime (`dora node add`).
    ///
    /// Deliberately NOT `local_nodes`: a runtime addition must not gate
    /// the startup barrier (a node that never subscribes would stall it
    /// forever) and must not be able to fail it (dora-rs/dora#2917).
    pub fn insert_runtime_added(&mut self, node_id: NodeId) {
        self.runtime_added.insert(node_id);
    }

    pub fn set_external_nodes(&mut self, value: bool) {
        self.external_nodes = value;
    }

    pub fn local_nodes_pending(&self) -> bool {
        !self.local_nodes.is_empty()
    }

    pub async fn handle_node_subscription(
        &mut self,
        node_id: NodeId,
        reply_sender: oneshot::Sender<DaemonReply>,
        coordinator_sender: &mut Option<CoordinatorSender>,
        clock: &HLC,
        cascading_errors: &mut CascadingErrorCauses,
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<DataflowStatus> {
        self.waiting_subscribers
            .insert(node_id.clone(), reply_sender);
        self.local_nodes.remove(&node_id);

        self.update_dataflow_status(coordinator_sender, clock, cascading_errors, logger)
            .await
    }

    pub async fn handle_node_stop(
        &mut self,
        node_id: &NodeId,
        coordinator_sender: &mut Option<CoordinatorSender>,
        clock: &HLC,
        cascading_errors: &mut CascadingErrorCauses,
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<()> {
        if self.local_nodes.remove(node_id) {
            logger
                .log(
                    LogLevel::Warn,
                    Some(node_id.clone()),
                    Some("daemon::pending".into()),
                    "node exited before initializing dora connection",
                )
                .await;
            self.exited_before_subscribe.push(node_id.clone());
            self.update_dataflow_status(coordinator_sender, clock, cascading_errors, logger)
                .await?;
        }
        Ok(())
    }

    pub async fn handle_dataflow_stop(
        &mut self,
        coordinator_sender: &mut Option<CoordinatorSender>,
        clock: &HLC,
        cascading_errors: &mut CascadingErrorCauses,
        dynamic_nodes: &BTreeSet<NodeId>,
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<()> {
        // remove all local dynamic nodes that are not yet started
        for node_id in dynamic_nodes {
            if self.local_nodes.remove(node_id) {
                self.update_dataflow_status(coordinator_sender, clock, cascading_errors, logger)
                    .await?;
            }
        }

        Ok(())
    }

    pub async fn handle_external_all_nodes_ready(
        &mut self,
        exited_before_subscribe: Vec<NodeId>,
        cascading_errors: &mut CascadingErrorCauses,
    ) -> eyre::Result<()> {
        if !self.local_nodes.is_empty() {
            tracing::warn!(
                remaining = ?self.local_nodes,
                "received external `all_nodes_ready` before local nodes were ready; \
                 proceeding anyway (coordinator may be ahead of this daemon)"
            );
        }

        self.answer_subscribe_requests(exited_before_subscribe, cascading_errors)
            .await;

        Ok(())
    }

    async fn update_dataflow_status(
        &mut self,
        coordinator_sender: &mut Option<CoordinatorSender>,
        clock: &HLC,
        cascading_errors: &mut CascadingErrorCauses,
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<DataflowStatus> {
        if self.local_nodes.is_empty() {
            if self.external_nodes {
                if !self.reported_init_to_coordinator {
                    self.report_nodes_ready(coordinator_sender, clock.new_timestamp(), logger)
                        .await?;
                    self.reported_init_to_coordinator = true;
                }
                Ok(DataflowStatus::Pending)
            } else {
                self.answer_subscribe_requests(Vec::new(), cascading_errors)
                    .await;
                Ok(DataflowStatus::AllNodesReady)
            }
        } else {
            Ok(DataflowStatus::Pending)
        }
    }

    async fn answer_subscribe_requests(
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
                more information, run `dora logs {} --node {causing_node}`.",
                self.dataflow_id
            )),
            None => Ok(()),
        };

        // answer all subscribe requests
        let subscribe_replies = std::mem::take(&mut self.waiting_subscribers);
        for (node_id, reply_sender) in subscribe_replies.into_iter() {
            // A startup failure is a property of the startup cohort, not
            // of the dataflow forever after. Reporting it to a node that
            // was added at runtime blames it for something it has no
            // relationship to, and (because the node then fails its own
            // `Node()` init) takes down a healthy node
            // (dora-rs/dora#2917).
            let scoped_result = if self.runtime_added.contains(&node_id) {
                Ok(())
            } else {
                result.clone()
            };
            if scoped_result.is_err()
                && let Some(causing_node) = node_exited_before_subscribe
            {
                cascading_errors.report_cascading_error(causing_node.clone(), node_id.clone());
            }
            let _ = reply_sender.send(DaemonReply::Result(scoped_result));
        }
    }

    async fn report_nodes_ready(
        &self,
        coordinator_sender: &mut Option<CoordinatorSender>,
        timestamp: Timestamp,
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<()> {
        let Some(sender) = coordinator_sender else {
            bail!("no coordinator sender to send AllNodesReady");
        };

        logger
            .log(
                LogLevel::Info,
                None,
                Some("daemon".into()),
                format!(
                "all local nodes are ready (exit before subscribe: {:?}), waiting for remote nodes",
                self.exited_before_subscribe
            ),
            )
            .await;

        let msg = serde_json::to_vec(&Timestamped {
            inner: CoordinatorRequest::Event {
                daemon_id: self.daemon_id.clone(),
                event: DaemonEvent::AllNodesReady {
                    dataflow_id: self.dataflow_id,
                    exited_before_subscribe: self.exited_before_subscribe.clone(),
                },
            },
            timestamp,
        })?;
        sender
            .send_event(&msg)
            .await
            .wrap_err("failed to send AllNodesReady message to dora-coordinator")?;
        Ok(())
    }
}

pub enum DataflowStatus {
    AllNodesReady,
    Pending,
}
