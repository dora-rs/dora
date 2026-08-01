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
    /// Every node this barrier was built for: the local, non-dynamic
    /// nodes declared in the descriptor the dataflow started from.
    ///
    /// `local_nodes` drains as nodes subscribe, so membership has to be
    /// remembered separately. A startup failure is a property of THIS
    /// set: nodes outside it (runtime `dora node add`s, and dynamic
    /// nodes that connect whenever they like) neither gate the barrier
    /// nor inherit its failures (dora-rs/dora#2917). They still wait on
    /// it, so a node arriving mid-startup does not begin producing
    /// before its consumers are listening.
    ///
    /// Caveat, pre-existing and not addressed here: on a dataflow with
    /// external nodes, `update_dataflow_status` parks waiters and
    /// returns `Pending` without answering, leaving
    /// `handle_external_all_nodes_ready` — a one-shot event at dataflow
    /// start — as the only path that ever answers them. A node that
    /// subscribes after that has fired waits forever.
    cohort: HashSet<NodeId>,
    /// List of nodes that finished before connecting to the dora daemon.
    ///
    /// If this list is non-empty, we should not start the dataflow at all. Instead,
    /// we report an error to the other nodes.
    ///
    /// Holds cohort members only — `handle_node_stop` records a node
    /// here only if it was still in `local_nodes`. Nothing else clears
    /// this list, so anything that leaks in poisons the dataflow for as
    /// long as it runs; `remove_node` exists to keep a removed node's
    /// id from doing exactly that (dora-rs/dora#2917).
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
            cohort: HashSet::new(),
            exited_before_subscribe: Default::default(),
            reported_init_to_coordinator: false,
        }
    }

    /// Enrol a node in the startup cohort. Called once per local,
    /// non-dynamic descriptor node as the dataflow is spawned.
    pub fn insert(&mut self, node_id: NodeId) {
        self.local_nodes.insert(node_id.clone());
        self.cohort.insert(node_id);
    }

    /// Forget a node that has been removed from the dataflow
    /// (`dora node remove`).
    ///
    /// Without this a removed node keeps gating the barrier until it
    /// exits, and its id can still reach `exited_before_subscribe` — a
    /// list nothing clears — so a `remove` + re-`add` of a node that
    /// had not yet subscribed would poison every later subscribe for
    /// the life of the dataflow, naming an id that is alive again by
    /// then (dora-rs/dora#2917).
    pub fn remove_node(&mut self, node_id: &NodeId) {
        self.local_nodes.remove(node_id);
        self.cohort.remove(node_id);
        self.waiting_subscribers.remove(node_id);
        self.exited_before_subscribe.retain(|id| id != node_id);
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
            // A startup failure belongs to the startup cohort, not to
            // the dataflow forever after. Reporting it to a node that
            // was never in that cohort — a runtime `dora node add`, or a
            // dynamic node connecting on its own schedule — blames it
            // for something it has no relationship to, and (because the
            // node then fails its own `Node()` init) takes down a
            // healthy node (dora-rs/dora#2917).
            let scoped_result = if self.cohort.contains(&node_id) {
                result.clone()
            } else {
                Ok(())
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

#[cfg(test)]
mod tests {
    use super::*;

    fn pending() -> PendingNodes {
        PendingNodes::new(uuid::Uuid::nil(), DaemonId::new(None))
    }

    fn node(id: &str) -> NodeId {
        NodeId::from(id.to_string())
    }

    /// Park a subscribe request and hand back the receiving end.
    fn park(p: &mut PendingNodes, id: &NodeId) -> oneshot::Receiver<DaemonReply> {
        let (tx, rx) = oneshot::channel();
        p.waiting_subscribers.insert(id.clone(), tx);
        rx
    }

    fn reply_of(rx: &mut oneshot::Receiver<DaemonReply>) -> Result<(), String> {
        match rx.try_recv().expect("subscribe was never answered") {
            DaemonReply::Result(result) => result,
            other => panic!("unexpected reply: {other:?}"),
        }
    }

    /// The startup cohort's whole point: if one member dies before
    /// subscribing, the others are told the dataflow cannot start.
    #[tokio::test]
    async fn cohort_members_receive_the_startup_failure() {
        let (dead, alive) = (node("dead"), node("alive"));
        let mut p = pending();
        p.insert(dead.clone());
        p.insert(alive.clone());
        let mut rx = park(&mut p, &alive);
        p.exited_before_subscribe.push(dead.clone());

        p.answer_subscribe_requests(Vec::new(), &mut CascadingErrorCauses::default())
            .await;

        let err = reply_of(&mut rx).expect_err("cohort member should inherit the failure");
        assert!(
            err.contains("dead"),
            "error should name the dead node: {err}"
        );
    }

    /// dora-rs/dora#2917: a node that was never in the cohort — a
    /// runtime `dora node add` — must not be blamed for it.
    #[tokio::test]
    async fn non_cohort_nodes_are_not_blamed_for_startup_failures() {
        let (dead, added) = (node("dead"), node("added-at-runtime"));
        let mut p = pending();
        p.insert(dead.clone());
        let mut rx = park(&mut p, &added);
        p.exited_before_subscribe.push(dead.clone());

        p.answer_subscribe_requests(Vec::new(), &mut CascadingErrorCauses::default())
            .await;

        assert_eq!(
            reply_of(&mut rx),
            Ok(()),
            "a node outside the startup cohort must not inherit its failure"
        );
    }

    /// The same exemption has to hold for dynamic nodes, which connect
    /// on their own schedule and are never enrolled via `insert`.
    #[tokio::test]
    async fn dynamic_nodes_are_not_blamed_for_startup_failures() {
        let (dead, dynamic) = (node("dead"), node("dynamic"));
        let mut p = pending();
        p.insert(dead.clone());
        let mut rx = park(&mut p, &dynamic);
        p.exited_before_subscribe.push(dead.clone());

        p.answer_subscribe_requests(Vec::new(), &mut CascadingErrorCauses::default())
            .await;

        assert_eq!(reply_of(&mut rx), Ok(()));
    }

    /// Only cohort members are recorded as causes, so a non-cohort node
    /// is not marked as a cascading victim either.
    #[tokio::test]
    async fn cascading_errors_are_recorded_for_cohort_members_only() {
        let (dead, member, added) = (node("dead"), node("member"), node("added"));
        let mut p = pending();
        p.insert(dead.clone());
        p.insert(member.clone());
        let _member_rx = park(&mut p, &member);
        let _added_rx = park(&mut p, &added);
        p.exited_before_subscribe.push(dead.clone());

        let mut causes = CascadingErrorCauses::default();
        p.answer_subscribe_requests(Vec::new(), &mut causes).await;

        assert_eq!(causes.error_caused_by(&member), Some(&dead));
        assert_eq!(causes.error_caused_by(&added), None);
    }

    /// A runtime addition must not gate the barrier. Before #2917 it
    /// joined `local_nodes`, so one that never subscribed stalled
    /// startup for everyone.
    #[test]
    fn runtime_additions_do_not_gate_the_barrier() {
        let mut p = pending();
        p.insert(node("cohort-member"));
        assert!(p.local_nodes_pending());

        // A runtime addition is simply never enrolled.
        p.local_nodes.remove(&node("cohort-member"));
        assert!(
            !p.local_nodes_pending(),
            "only cohort members may hold the barrier open"
        );
    }

    /// `remove_node` has to scrub every trace, or a removed id keeps
    /// gating startup and can still poison later subscribes.
    #[test]
    fn remove_node_scrubs_barrier_state() {
        let removed = node("removed");
        let mut p = pending();
        p.insert(removed.clone());
        let _rx = park(&mut p, &removed);
        p.exited_before_subscribe.push(removed.clone());

        p.remove_node(&removed);

        assert!(!p.local_nodes_pending(), "removed node still gates startup");
        assert!(!p.cohort.contains(&removed));
        assert!(!p.waiting_subscribers.contains_key(&removed));
        assert!(
            p.exited_before_subscribe.is_empty(),
            "a removed id left in `exited_before_subscribe` poisons every later subscribe"
        );
    }

    /// The regression in full: a cohort member dies before subscribing,
    /// the operator removes it, then adds a replacement under the same
    /// id. The replacement must come up clean.
    #[tokio::test]
    async fn remove_then_readd_does_not_inherit_the_dead_incarnation() {
        let id = node("swapped");
        let mut p = pending();
        p.insert(id.clone());
        // Died before subscribing.
        p.exited_before_subscribe.push(id.clone());
        // Operator removes it, then adds a replacement (runtime
        // additions are not enrolled).
        p.remove_node(&id);
        let mut rx = park(&mut p, &id);

        p.answer_subscribe_requests(Vec::new(), &mut CascadingErrorCauses::default())
            .await;

        assert_eq!(
            reply_of(&mut rx),
            Ok(()),
            "the replacement inherited its dead predecessor's failure"
        );
    }
}
