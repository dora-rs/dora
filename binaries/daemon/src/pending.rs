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

    /// The cross-daemon barrier's release, if it has happened.
    ///
    /// `None` until the coordinator broadcasts; then `Some(list)` carrying
    /// its `exited_before_subscribe`. The LIST matters, not just the fact:
    /// a barrier can release having failed, and a node subscribing
    /// afterwards must get the same answer as one parked at release time —
    /// including "this dataflow died during startup".
    ///
    /// The coordinator broadcasts `AllNodesReady` exactly once per
    /// dataflow, when its last pending daemon reports in. Without a latch,
    /// only the subscribers parked at that instant are ever answered:
    /// anything subscribing later — a `dora node add`, a dynamic node
    /// connecting on its own schedule, or a node being restarted — parks
    /// in `waiting_subscribers` behind an event that will not fire again,
    /// and its `init_from_env()` never returns (dora-rs/dora#2938).
    ///
    /// Single-daemon dataflows never hit this: `external_nodes` stays
    /// false and every subscribe is answered inline, which is why the
    /// local path was the one everyone exercised.
    external_ready: Option<Vec<NodeId>>,
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
            external_ready: None,
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

    /// Whether `node_id` is still gating the startup barrier: enrolled in
    /// the cohort and not yet subscribed. Used by `ReplaceNode` to reject
    /// swapping a node whose original incarnation the barrier still waits
    /// on (dora-rs/dora#2927).
    pub fn is_pending(&self, node_id: &NodeId) -> bool {
        self.local_nodes.contains(node_id)
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

        // Latch before answering: the coordinator fires this once, so
        // everything that subscribes from here on has to be answered
        // inline rather than parked (#2938). Record the list, not just a
        // flag — a later subscriber has to inherit a failed startup too.
        self.external_ready = Some(exited_before_subscribe.clone());

        self.answer_subscribe_requests(exited_before_subscribe, cascading_errors)
            .await;

        Ok(())
    }

    /// Re-evaluate the barrier after something changed the pending set.
    ///
    /// The `external_nodes` branch parks waiters until the coordinator's
    /// one-shot broadcast releases the barrier. That broadcast fires once
    /// per dataflow, so the release is latched in `external_ready` and
    /// every later subscribe is answered inline from it — otherwise a
    /// `dora node add`, a dynamic node, or a restarting node would wait
    /// forever on a multi-daemon dataflow (dora-rs/dora#2938).
    async fn update_dataflow_status(
        &mut self,
        coordinator_sender: &mut Option<CoordinatorSender>,
        clock: &HLC,
        cascading_errors: &mut CascadingErrorCauses,
        logger: &mut DataflowLogger<'_>,
    ) -> eyre::Result<DataflowStatus> {
        if self.local_nodes.is_empty() {
            if self.external_nodes {
                // Report upward FIRST, unconditionally. The coordinator can
                // release the barrier before this daemon's local nodes are
                // ready (see the warn in `handle_external_all_nodes_ready`),
                // and returning early from the latch below without reporting
                // would leave this daemon in the coordinator's
                // `pending_daemons` forever.
                if !self.reported_init_to_coordinator {
                    self.report_nodes_ready(coordinator_sender, clock.new_timestamp(), logger)
                        .await?;
                    self.reported_init_to_coordinator = true;
                }
                if let Some(exited_before_subscribe) = self.external_ready.clone() {
                    // The barrier is already down; a late subscriber must
                    // not wait for an event that has been and gone.
                    let barrier_succeeded = exited_before_subscribe.is_empty();
                    self.answer_subscribe_requests(exited_before_subscribe, cascading_errors)
                        .await;
                    // Only report ready if the barrier actually succeeded.
                    // `AllNodesReady` is what makes the daemon call
                    // `dataflow.start()`, and the release path deliberately
                    // does not start a dataflow whose barrier reported a
                    // node that died before subscribing. Saying "ready"
                    // here would let a late subscriber start a dataflow the
                    // coordinator already declared failed.
                    return Ok(if barrier_succeeded {
                        DataflowStatus::AllNodesReady
                    } else {
                        DataflowStatus::Pending
                    });
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

#[derive(Debug)]
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

    /// `ReplaceNode`'s startup gate (dora-rs/dora#2927): a node is
    /// "pending" from cohort enrollment until its subscription is
    /// handled, and never for ids the barrier does not know.
    #[tokio::test]
    async fn is_pending_tracks_barrier_membership() {
        let id = node("swapme");
        let mut p = pending();
        assert!(!p.is_pending(&id), "unknown ids are never pending");

        p.insert(id.clone());
        assert!(p.is_pending(&id), "enrolled node must gate the barrier");

        let mut logger = test_logger();
        let mut logger = logger.for_dataflow(uuid::Uuid::nil());
        let (tx, _rx) = oneshot::channel();
        p.handle_node_subscription(
            id.clone(),
            tx,
            &mut None,
            &HLC::default(),
            &mut CascadingErrorCauses::default(),
            &mut logger,
        )
        .await
        .expect("subscription handling");
        assert!(
            !p.is_pending(&id),
            "a subscribed node no longer gates the barrier"
        );
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

    // ---- dora-rs/dora#2938: on a multi-daemon dataflow the coordinator
    //      broadcasts `AllNodesReady` once; anything subscribing after
    //      that must not park behind an event that will not fire again ----

    /// Build a barrier in the multi-daemon shape: external nodes present,
    /// no local nodes left to wait for.
    fn external_barrier() -> PendingNodes {
        let mut p = pending();
        p.set_external_nodes(true);
        // Pretend the upward report already happened: with no coordinator
        // sender wired up it would error by design (see
        // `external_dataflows_report_ready_upward_at_most_once`), and the
        // state under test here is the one *after* this daemon reported —
        // waiting on the coordinator's broadcast.
        p.reported_init_to_coordinator = true;
        p
    }

    /// Release the barrier with the coordinator's `exited_before_subscribe`.
    async fn release(p: &mut PendingNodes, exited: Vec<NodeId>) {
        p.handle_external_all_nodes_ready(exited, &mut CascadingErrorCauses::default())
            .await
            .expect("release the barrier");
    }

    async fn settle(p: &mut PendingNodes) -> DataflowStatus {
        let mut causes = CascadingErrorCauses::default();
        let mut daemon_logger = test_logger();
        let mut logger = daemon_logger.for_dataflow(uuid::Uuid::nil());
        p.update_dataflow_status(&mut None, &HLC::default(), &mut causes, &mut logger)
            .await
            .expect("update_dataflow_status")
    }

    /// The reported hang: `dora node add`, a dynamic node connecting late,
    /// or a node being restarted all subscribe after the one-shot has
    /// fired. Before the latch this returned `Pending` and the reply was
    /// never sent, so `init_from_env()` never returned.
    #[tokio::test]
    async fn late_subscriber_is_answered_after_the_external_barrier_released() {
        let mut p = external_barrier();
        p.handle_external_all_nodes_ready(Vec::new(), &mut CascadingErrorCauses::default())
            .await
            .expect("release the barrier");

        // A node arrives afterwards — the `dora node add` case.
        let latecomer = node("added-at-runtime");
        let mut rx = park(&mut p, &latecomer);
        let status = settle(&mut p).await;

        assert!(
            matches!(status, DataflowStatus::AllNodesReady),
            "a barrier that has already been released must report ready, got {status:?}"
        );
        reply_of(&mut rx).expect("late subscriber must be answered, not parked forever");
    }

    /// The latch must not open the barrier early: before the coordinator
    /// says every daemon is ready, a subscriber still waits. Otherwise a
    /// node could start producing before its remote consumers exist.
    #[tokio::test]
    async fn subscriber_still_waits_until_the_external_barrier_releases() {
        let mut p = external_barrier();
        let early = node("early");
        let mut rx = park(&mut p, &early);

        let status = settle(&mut p).await;

        assert!(
            matches!(status, DataflowStatus::Pending),
            "the cross-daemon barrier must still hold before the coordinator \
             reports every daemon ready, got {status:?}"
        );
        assert!(
            rx.try_recv().is_err(),
            "subscriber must remain parked until the barrier releases"
        );

        // ...and is released normally.
        p.handle_external_all_nodes_ready(Vec::new(), &mut CascadingErrorCauses::default())
            .await
            .expect("release");
        reply_of(&mut rx).expect("released subscriber should be answered");
    }

    /// A single-daemon dataflow has no external barrier to latch, so its
    /// behavior is unchanged: every subscribe is answered inline.
    #[tokio::test]
    async fn local_only_dataflow_answers_inline_as_before() {
        let mut p = pending();
        let solo = node("solo");
        let mut rx = park(&mut p, &solo);

        let status = settle(&mut p).await;

        assert!(matches!(status, DataflowStatus::AllNodesReady));
        reply_of(&mut rx).expect("local-only subscribe is answered inline");
    }

    /// A barrier can release having FAILED. A node subscribing afterwards
    /// must inherit that, and must not report ready — `AllNodesReady` is
    /// what makes the daemon call `dataflow.start()`, and the release path
    /// deliberately refuses to start a dataflow whose barrier named a node
    /// that died before subscribing. Reporting ready here would let a late
    /// `dora node add` start a dataflow the coordinator declared dead.
    #[tokio::test]
    async fn late_subscriber_does_not_start_a_dataflow_whose_barrier_failed() {
        let member = node("member");
        let mut p = external_barrier();
        p.insert(member.clone()); // enrol in the cohort
        p.local_nodes.remove(&member); // ...and let it already be past
        release(&mut p, vec![node("died-on-another-daemon")]).await;

        let mut rx = park(&mut p, &member);
        let status = settle(&mut p).await;

        assert!(
            !matches!(status, DataflowStatus::AllNodesReady),
            "a barrier that released with a startup failure must not report \
             ready, or a late subscriber starts a dead dataflow; got {status:?}"
        );
        let err =
            reply_of(&mut rx).expect_err("a cohort member must inherit the remote startup failure");
        assert!(
            err.contains("died-on-another-daemon"),
            "the failure must name the node that died: {err}"
        );
    }

    /// The same failed barrier, but the late subscriber was never in the
    /// cohort — a runtime `dora node add`. It is not blamed (#2917), yet
    /// the dataflow still must not be started by its arrival.
    #[tokio::test]
    async fn non_cohort_latecomer_is_spared_but_still_does_not_start_it() {
        let mut p = external_barrier();
        release(&mut p, vec![node("died-on-another-daemon")]).await;

        let added = node("added-at-runtime");
        let mut rx = park(&mut p, &added);
        let status = settle(&mut p).await;

        reply_of(&mut rx).expect("a non-cohort node must not inherit the failure");
        assert!(
            !matches!(status, DataflowStatus::AllNodesReady),
            "adding a node must not start a dataflow whose barrier failed; got {status:?}"
        );
    }
}
