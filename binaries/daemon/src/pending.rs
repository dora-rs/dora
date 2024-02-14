use std::collections::{HashMap, HashSet};

use dora_core::{
    config::NodeId,
    coordinator_messages::{CoordinatorRequest, DaemonEvent},
    daemon_messages::{DaemonReply, DataflowId, Timestamped},
    message::uhlc::{Timestamp, HLC},
};
use eyre::{bail, Context};
use tokio::{net::TcpStream, sync::oneshot};

use crate::tcp_utils::tcp_send;

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
    exited_before_subscribe: HashSet<NodeId>,

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
            exited_before_subscribe: HashSet::new(),
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
    ) -> eyre::Result<DataflowStatus> {
        self.waiting_subscribers
            .insert(node_id.clone(), reply_sender);
        self.local_nodes.remove(&node_id);

        self.update_dataflow_status(coordinator_connection, clock)
            .await
    }

    pub async fn handle_node_stop(
        &mut self,
        node_id: &NodeId,
        coordinator_connection: &mut Option<TcpStream>,
        clock: &HLC,
    ) -> eyre::Result<()> {
        if self.local_nodes.remove(node_id) {
            tracing::warn!("node `{node_id}` exited before initializing dora connection");
            self.exited_before_subscribe.insert(node_id.clone());
            self.update_dataflow_status(coordinator_connection, clock)
                .await?;
        }
        Ok(())
    }

    pub async fn handle_external_all_nodes_ready(&mut self, success: bool) -> eyre::Result<()> {
        if !self.local_nodes.is_empty() {
            bail!("received external `all_nodes_ready` event before local nodes were ready");
        }
        let external_error = if success {
            None
        } else {
            Some("some nodes failed to initalize on remote machines".to_string())
        };
        self.answer_subscribe_requests(external_error).await;

        Ok(())
    }

    async fn update_dataflow_status(
        &mut self,
        coordinator_connection: &mut Option<TcpStream>,
        clock: &HLC,
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
                self.answer_subscribe_requests(None).await;
                Ok(DataflowStatus::AllNodesReady)
            }
        } else {
            Ok(DataflowStatus::Pending)
        }
    }

    async fn answer_subscribe_requests(&mut self, external_error: Option<String>) {
        let result = if self.exited_before_subscribe.is_empty() {
            match external_error {
                Some(err) => Err(err),
                None => Ok(()),
            }
        } else {
            let node_id_message = if self.exited_before_subscribe.len() == 1 {
                self.exited_before_subscribe
                    .iter()
                    .next()
                    .map(|node_id| node_id.to_string())
                    .unwrap_or("<node_id>".to_string())
            } else {
                "<node_id>".to_string()
            };
            Err(format!(
                "Some nodes exited before subscribing to dora: {:?}\n\n\
                This is typically happens when an initialization error occurs
                in the node or operator. To check the output of the failed
                nodes, run `dora logs {} {node_id_message}`.",
                self.exited_before_subscribe, self.dataflow_id
            ))
        };
        // answer all subscribe requests
        let subscribe_replies = std::mem::take(&mut self.waiting_subscribers);
        for reply_sender in subscribe_replies.into_values() {
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

        let success = self.exited_before_subscribe.is_empty();
        tracing::info!("all local nodes are ready (success = {success}), waiting for remote nodes");

        let msg = serde_json::to_vec(&Timestamped {
            inner: CoordinatorRequest::Event {
                machine_id: self.machine_id.clone(),
                event: DaemonEvent::AllNodesReady {
                    dataflow_id: self.dataflow_id,
                    success,
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
