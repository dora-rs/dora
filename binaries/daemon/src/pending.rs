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
    cohort: HashSet<NodeId>,
    /// List of nodes that finished before connecting to the dora daemon.
    ///
    /// If this list is non-empty, we should not start the dataflow at all. Instead,
    /// we report an error to the other nodes.
    ///
    /// Holds cohort members only — `handle_node_stop` records a node
    /// here only if it was still in `local_nodes`. Nothing else clears
    /// this list, so anything that leaks in poisons the dataflow for as
    /// long as it runs; `handle_node_removal` exists to keep a removed node’s
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
    /// (`dora node remove`), then re-evaluate the barrier.
    ///
    /// Scrubbing is needed because a removed node otherwise keeps
    /// gating the barrier until it exits, and its id can still reach
    /// `exited_before_subscribe` — a list nothing clears — so a
    /// `remove` + re-`add` of a node that had not yet subscribed would
    /// poison every later subscribe for the life of the dataflow,
    /// naming an id that is alive again by then (dora-rs/dora#2917).
    ///
    /// Re-evaluating is just as necessary: removing a node that had not
    /// yet subscribed can be the thing that completes the cohort. The
    /// removed process's own exit cannot release the barrier later —
    /// its id is gone from `local_nodes` by then, so `handle_node_stop`
    /// does nothing — so if this did not drive the same transition as a
    /// subscription, the last pending member's removal would strand
    /// every parked subscriber and the dataflow would never start.
    pub async fn handle_node_removal(
        &mut self,
        node_id: &NodeId,
        coordinator_sender: &mut Option<CoordinatorSender>,
        clock: &HLC,
        cascading_errors: &mut CascadingErrorCauses,
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<DataflowStatus> {
        self.local_nodes.remove(node_id);
        self.cohort.remove(node_id);
        self.waiting_subscribers.remove(node_id);
        self.exited_before_subscribe.retain(|id| id != node_id);

        self.update_dataflow_status(coordinator_sender, clock, cascading_errors, logger)
            .await
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

    /// Re-evaluate the barrier after something changed the pending set.
    ///
    /// Known limitation, pre-existing and tracked in dora-rs/dora#2938:
    /// the `external_nodes` branch parks waiters and returns `Pending`
    /// without answering them, leaving `handle_external_all_nodes_ready`
    /// — driven by a one-shot coordinator broadcast at dataflow start —
    /// as the only path that ever does. Anything subscribing after that
    /// has fired waits forever on a multi-daemon dataflow.
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
    use crate::log::DaemonLogger;

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

    /// dora-rs/dora#2917: a node that was never in the cohort must not
    /// be blamed for its failure. Covers both spellings at once —
    /// a runtime `dora node add` and a descriptor-declared dynamic node
    /// are the same case here, since neither is enrolled via `insert`
    /// and `PendingNodes` has no notion of dynamic-ness.
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

    /// A `DataflowLogger` for tests. `LogDestination::Tracing` needs no
    /// coordinator connection or channel.
    fn test_logger() -> DaemonLogger {
        let daemon_id = DaemonId::new(None);
        crate::log::Logger {
            destination: crate::log::LogDestination::Tracing,
            daemon_id: daemon_id.clone(),
            clock: std::sync::Arc::new(HLC::default()),
        }
        .for_daemon(daemon_id)
    }

    /// Removal has to scrub every trace, or a removed id keeps gating
    /// startup and can still poison later subscribes.
    #[tokio::test]
    async fn removal_scrubs_barrier_state() {
        let removed = node("removed");
        let mut p = pending();
        p.insert(removed.clone());
        let mut rx = park(&mut p, &removed);
        p.exited_before_subscribe.push(removed.clone());

        let mut daemon_logger = test_logger();
        p.handle_node_removal(
            &removed,
            &mut None,
            &HLC::default(),
            &mut CascadingErrorCauses::default(),
            &mut daemon_logger.for_dataflow(uuid::Uuid::nil()),
        )
        .await
        .expect("removal should succeed");

        assert!(!p.local_nodes_pending(), "removed node still gates startup");
        assert!(!p.cohort.contains(&removed));
        assert!(
            p.exited_before_subscribe.is_empty(),
            "a removed id left in `exited_before_subscribe` poisons every later subscribe"
        );
        // Asserting on the receiver, not on `waiting_subscribers`:
        // `answer_subscribe_requests` drains that map unconditionally, so
        // a `contains_key` check passes even when the scrub is deleted.
        // A dropped sender is what actually distinguishes "the removed
        // node's parked request was discarded" from "it was answered".
        assert!(
            matches!(rx.try_recv(), Err(oneshot::error::TryRecvError::Closed)),
            "a removed node's parked subscribe should be dropped, not answered"
        );
    }

    /// Removing the LAST pending cohort member must complete the
    /// barrier, exactly as that member subscribing would have.
    ///
    /// The removed process's own exit cannot do it later — its id is
    /// gone from `local_nodes`, so `handle_node_stop` is a no-op — so if
    /// removal does not drive the transition, every already-parked
    /// subscriber is stranded and `dataflow.start()` is never called.
    #[tokio::test]
    async fn removing_the_last_pending_member_releases_parked_subscribers() {
        let (waiting, never_subscribed) = (node("waiting"), node("never-subscribed"));
        let mut p = pending();
        p.insert(waiting.clone());
        p.insert(never_subscribed.clone());

        // One cohort member subscribed and is parked; the other never
        // will, and is removed by the operator.
        let mut rx = park(&mut p, &waiting);
        p.local_nodes.remove(&waiting);
        assert!(p.local_nodes_pending(), "the other member still gates");

        let mut daemon_logger = test_logger();
        let status = p
            .handle_node_removal(
                &never_subscribed,
                &mut None,
                &HLC::default(),
                &mut CascadingErrorCauses::default(),
                &mut daemon_logger.for_dataflow(uuid::Uuid::nil()),
            )
            .await
            .expect("removal should succeed");

        assert!(
            matches!(status, DataflowStatus::AllNodesReady),
            "removing the last pending member must report the cohort ready"
        );
        assert_eq!(
            reply_of(&mut rx),
            Ok(()),
            "the parked subscriber was stranded by the removal"
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
        let mut daemon_logger = test_logger();
        p.handle_node_removal(
            &id,
            &mut None,
            &HLC::default(),
            &mut CascadingErrorCauses::default(),
            &mut daemon_logger.for_dataflow(uuid::Uuid::nil()),
        )
        .await
        .expect("removal should succeed");
        let mut rx = park(&mut p, &id);

        p.answer_subscribe_requests(Vec::new(), &mut CascadingErrorCauses::default())
            .await;

        assert_eq!(
            reply_of(&mut rx),
            Ok(()),
            "the replacement inherited its dead predecessor's failure"
        );
    }
    /// `handle_node_stop` is the only production writer of
    /// `exited_before_subscribe`, and the whole fix rests on it
    /// recording cohort members only. Every other test pushes into that
    /// vector by hand, so without this the invariant is unverified —
    /// making the guard unconditional would restore #2917 through the
    /// other half of the mechanism with all tests still green.
    #[tokio::test]
    async fn only_cohort_members_are_recorded_as_exited_before_subscribe() {
        let (member, outsider) = (node("member"), node("outsider"));
        let mut p = pending();
        p.insert(member.clone());
        let mut daemon_logger = test_logger();

        // A node that was never enrolled exits before subscribing.
        p.handle_node_stop(
            &outsider,
            &mut None,
            &HLC::default(),
            &mut CascadingErrorCauses::default(),
            &mut daemon_logger.for_dataflow(uuid::Uuid::nil()),
        )
        .await
        .expect("stop should succeed");
        assert!(
            p.exited_before_subscribe.is_empty(),
            "a node outside the cohort must not be able to fail the barrier"
        );

        // A cohort member doing the same IS recorded.
        p.handle_node_stop(
            &member,
            &mut None,
            &HLC::default(),
            &mut CascadingErrorCauses::default(),
            &mut daemon_logger.for_dataflow(uuid::Uuid::nil()),
        )
        .await
        .expect("stop should succeed");
        assert_eq!(p.exited_before_subscribe, vec![member]);
    }

    /// End to end through the production writer: a cohort member dies
    /// before subscribing, and a non-cohort node parked at that moment
    /// is still answered `Ok`.
    #[tokio::test]
    async fn cohort_death_recorded_by_handle_node_stop_spares_non_cohort_waiters() {
        let (dead, outsider) = (node("dead"), node("outsider"));
        let mut p = pending();
        p.insert(dead.clone());
        let mut rx = park(&mut p, &outsider);
        let mut daemon_logger = test_logger();

        p.handle_node_stop(
            &dead,
            &mut None,
            &HLC::default(),
            &mut CascadingErrorCauses::default(),
            &mut daemon_logger.for_dataflow(uuid::Uuid::nil()),
        )
        .await
        .expect("stop should succeed");

        assert_eq!(
            reply_of(&mut rx),
            Ok(()),
            "the parked non-cohort node inherited a cohort member's death"
        );
    }

    /// With external (multi-daemon) nodes the barrier is resolved by the
    /// coordinator, so a local removal that empties `local_nodes`
    /// reports readiness upward instead of answering waiters, and does
    /// so at most once.
    #[tokio::test]
    async fn external_dataflows_report_ready_upward_at_most_once() {
        let member = node("member");
        let mut p = pending();
        p.insert(member.clone());
        p.set_external_nodes(true);
        let mut daemon_logger = test_logger();

        // No coordinator sender wired up, so reporting fails loudly
        // rather than silently claiming the cohort is ready.
        let status = p
            .handle_node_removal(
                &member,
                &mut None,
                &HLC::default(),
                &mut CascadingErrorCauses::default(),
                &mut daemon_logger.for_dataflow(uuid::Uuid::nil()),
            )
            .await;
        assert!(
            status.is_err(),
            "reporting readiness with no coordinator sender must surface an error"
        );
        assert!(
            !p.reported_init_to_coordinator,
            "a failed report must not latch, or the retry is lost"
        );
    }

    /// The external `exited_before_subscribe` list the coordinator sends
    /// is scoped the same way as the local one.
    #[tokio::test]
    async fn external_failures_are_scoped_to_the_cohort_too() {
        let (remote_dead, member, outsider) = (node("remote"), node("member"), node("outsider"));
        let mut p = pending();
        p.insert(member.clone());
        let mut member_rx = park(&mut p, &member);
        let mut outsider_rx = park(&mut p, &outsider);

        p.handle_external_all_nodes_ready(
            vec![remote_dead.clone()],
            &mut CascadingErrorCauses::default(),
        )
        .await
        .expect("external ready should succeed");

        let err = reply_of(&mut member_rx).expect_err("cohort member should inherit");
        assert!(err.contains("remote"), "should name the remote node: {err}");
        assert_eq!(
            reply_of(&mut outsider_rx),
            Ok(()),
            "a non-cohort node must not inherit a remote startup failure either"
        );
    }
}
