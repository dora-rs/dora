use std::collections::{HashMap, HashSet};

use dora_core::{config::NodeId, daemon_messages::DaemonReply};
use tokio::sync::oneshot;

#[derive(Default)]
pub struct PendingNodes {
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
}

impl PendingNodes {
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
    ) -> eyre::Result<DataflowStatus> {
        self.waiting_subscribers
            .insert(node_id.clone(), reply_sender);

        self.local_nodes.remove(&node_id);
        if self.local_nodes.is_empty() {
            if self.external_nodes {
                Ok(DataflowStatus::LocalNodesReady { success: true })
            } else {
                self.answer_subscribe_requests(None).await;
                Ok(DataflowStatus::AllNodesReady)
            }
        } else {
            Ok(DataflowStatus::LocalNodesPending)
        }
    }

    pub async fn handle_node_stop(&mut self, node_id: &NodeId) -> DataflowStatus {
        self.exited_before_subscribe.insert(node_id.clone());

        self.local_nodes.remove(node_id);
        if self.local_nodes.is_empty() {
            if self.external_nodes {
                DataflowStatus::LocalNodesReady { success: false }
            } else {
                self.answer_subscribe_requests(None).await;
                DataflowStatus::AllNodesReady
            }
        } else {
            // continue waiting for other nodes
            DataflowStatus::LocalNodesPending
        }
    }

    pub async fn handle_external_all_nodes_ready(&mut self, success: bool) {
        let external_error = if success {
            None
        } else {
            Some("some nodes failed to initalize on remote machines".to_string())
        };
        self.answer_subscribe_requests(external_error).await
    }

    async fn answer_subscribe_requests(&mut self, external_error: Option<String>) {
        let result = if self.exited_before_subscribe.is_empty() {
            match external_error {
                Some(err) => Err(err),
                None => Ok(()),
            }
        } else {
            Err(format!(
                "Nodes failed before subscribing: {:?}",
                self.exited_before_subscribe
            ))
        };
        // answer all subscribe requests
        let subscribe_replies = std::mem::take(&mut self.waiting_subscribers);
        for reply_sender in subscribe_replies.into_values() {
            let _ = reply_sender.send(DaemonReply::Result(result.clone()));
        }
    }
}

pub enum DataflowStatus {
    AllNodesReady,
    LocalNodesPending,
    LocalNodesReady { success: bool },
}
