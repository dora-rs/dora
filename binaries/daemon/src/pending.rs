use std::collections::{HashMap, HashSet};

use dora_core::{
    config::NodeId,
    coordinator_messages::{CoordinatorRequest, DaemonEvent},
    daemon_messages::{DaemonReply, DataflowId, Timestamped},
    message::uhlc::{Timestamp, HLC},
};
use eyre::{bail, Context};
use tokio::{net::TcpStream, sync::oneshot};

use crate::{tcp_utils::tcp_send, CascadingErrorCauses};

pub struct PendingNodes {
    dataflow_id: DataflowId,
    machine_id: String,

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
}

impl PendingNodes {
    pub fn new(dataflow_id: DataflowId, machine_id: String) -> Self {
        Self {
            dataflow_id,
            machine_id,
            local_nodes: HashSet::new(),
            external_nodes: false,
            waiting_subscribers: HashMap::new(),
            exited_before_subscribe: Default::default(),
            reported_init_to_coordinator: false,
        }
    }

    pub fn insert(&mut self, node_id: NodeId) {
        self.local_nodes.insert(node_id);
    }

    pub fn set_external_nodes(&mut self, value: bool) {
        self.external_nodes = value;
    }

    pub async fn handle_node_subscription(
        &mut self,
        node_id: NodeId,
        reply_sender: oneshot::Sender<DaemonReply>,
        coordinator_connection: &mut Option<TcpStream>,
        clock: &HLC,
        cascading_errors: &mut CascadingErrorCauses,
    ) -> eyre::Result<DataflowStatus> {
        self.waiting_subscribers
            .insert(node_id.clone(), reply_sender);
        self.local_nodes.remove(&node_id);

        self.update_dataflow_status(coordinator_connection, clock, cascading_errors)
            .await
    }

    pub async fn handle_node_stop(
        &mut self,
        node_id: &NodeId,
        coordinator_connection: &mut Option<TcpStream>,
        clock: &HLC,
        cascading_errors: &mut CascadingErrorCauses,
    ) -> eyre::Result<()> {
        if self.local_nodes.remove(node_id) {
            tracing::warn!("node `{node_id}` exited before initializing dora connection");
            self.exited_before_subscribe.push(node_id.clone());
            self.update_dataflow_status(coordinator_connection, clock, cascading_errors)
                .await?;
        }
        Ok(())
    }

    pub async fn handle_external_all_nodes_ready(
        &mut self,
        exited_before_subscribe: Vec<NodeId>,
        cascading_errors: &mut CascadingErrorCauses,
    ) -> eyre::Result<()> {
        if !self.local_nodes.is_empty() {
            bail!("received external `all_nodes_ready` event before local nodes were ready");
        }

        self.answer_subscribe_requests(exited_before_subscribe, cascading_errors)
            .await;

        Ok(())
    }

    async fn update_dataflow_status(
        &mut self,
        coordinator_connection: &mut Option<TcpStream>,
        clock: &HLC,
        cascading_errors: &mut CascadingErrorCauses,
    ) -> eyre::Result<DataflowStatus> {
        if self.local_nodes.is_empty() {
            if self.external_nodes {
                if !self.reported_init_to_coordinator {
                    self.report_nodes_ready(coordinator_connection, clock.new_timestamp())
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
                "Some nodes exited before subscribing to dora: {:?}\n\n\
                This is typically happens when an initialization error occurs
                in the node or operator. To check the output of the causing
                node, run `dora logs {} {causing_node}`.",
                self.exited_before_subscribe, self.dataflow_id
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
        coordinator_connection: &mut Option<TcpStream>,
        timestamp: Timestamp,
    ) -> eyre::Result<()> {
        let Some(connection) = coordinator_connection else {
            bail!("no coordinator connection to send AllNodesReady");
        };

        tracing::info!(
            "all local nodes are ready (exit before subscribe: {:?}), waiting for remote nodes",
            self.exited_before_subscribe
        );

        let msg = serde_json::to_vec(&Timestamped {
            inner: CoordinatorRequest::Event {
                machine_id: self.machine_id.clone(),
                event: DaemonEvent::AllNodesReady {
                    dataflow_id: self.dataflow_id,
                    exited_before_subscribe: self.exited_before_subscribe.clone(),
                },
            },
            timestamp,
        })?;
        tcp_send(connection, &msg)
            .await
            .wrap_err("failed to send AllNodesReady message to dora-coordinator")?;
        Ok(())
    }
}

pub enum DataflowStatus {
    AllNodesReady,
    Pending,
}
