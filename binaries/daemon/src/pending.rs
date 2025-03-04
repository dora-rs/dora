use std::collections::{BTreeSet, HashMap, HashSet};

use dora_core::{
    config::NodeId,
    uhlc::{Timestamp, HLC},
};
use dora_message::{
    common::DaemonId,
    daemon_to_coordinator::{CoordinatorRequest, DaemonEvent, LogLevel, LogMessage, Timestamped},
    daemon_to_node::DaemonReply,
    DataflowId,
};
use eyre::{bail, Context};
use tokio::{net::TcpStream, sync::oneshot};

use crate::{log::DataflowLogger, socket_stream_utils::socket_stream_send, CascadingErrorCauses};

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
    /// List of nodes that finished before connecting to the dora daemon.
    ///
    /// If this list is non-empty, we should not start the dataflow at all. Instead,
    /// we report an error to the other nodes.
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
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<DataflowStatus> {
        self.waiting_subscribers
            .insert(node_id.clone(), reply_sender);
        self.local_nodes.remove(&node_id);

        self.update_dataflow_status(coordinator_connection, clock, cascading_errors, logger)
            .await
    }

    pub async fn handle_node_stop(
        &mut self,
        node_id: &NodeId,
        coordinator_connection: &mut Option<TcpStream>,
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
            self.update_dataflow_status(coordinator_connection, clock, cascading_errors, logger)
                .await?;
        }
        Ok(())
    }

    pub async fn handle_dataflow_stop(
        &mut self,
        coordinator_connection: &mut Option<TcpStream>,
        clock: &HLC,
        cascading_errors: &mut CascadingErrorCauses,
        dynamic_nodes: &BTreeSet<NodeId>,
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<Vec<LogMessage>> {
        // remove all local dynamic nodes that are not yet started
        for node_id in dynamic_nodes {
            if self.local_nodes.remove(node_id) {
                self.update_dataflow_status(
                    coordinator_connection,
                    clock,
                    cascading_errors,
                    logger,
                )
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

        self.answer_subscribe_requests(exited_before_subscribe, cascading_errors)
            .await;

        Ok(())
    }

    async fn update_dataflow_status(
        &mut self,
        coordinator_connection: &mut Option<TcpStream>,
        clock: &HLC,
        cascading_errors: &mut CascadingErrorCauses,
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<DataflowStatus> {
        if self.local_nodes.is_empty() {
            if self.external_nodes {
                if !self.reported_init_to_coordinator {
                    self.report_nodes_ready(coordinator_connection, clock.new_timestamp(), logger)
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
        coordinator_connection: &mut Option<TcpStream>,
        timestamp: Timestamp,
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<()> {
        let Some(connection) = coordinator_connection else {
            bail!("no coordinator connection to send AllNodesReady");
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
        socket_stream_send(connection, &msg)
            .await
            .wrap_err("failed to send AllNodesReady message to dora-coordinator")?;
        Ok(())
    }
}

pub enum DataflowStatus {
    AllNodesReady,
    Pending,
}
