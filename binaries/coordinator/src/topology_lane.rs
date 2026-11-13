//! Running the node-lifecycle changes (`AddNode`, `RemoveNode`, `ReplaceNode`,
//! `AddMapping`, `RemoveMapping`) without waiting for the daemon inside the
//! coordinator's serial event loop.
//!
//! A change needs one round trip to a daemon, and a daemon can take a long
//! time to answer (`AddNode`/`ReplaceNode` prepare and spawn the node on the
//! daemon's own loop) or never answer. Awaiting that reply in the event loop
//! stopped everything else for up to `TCP_READ_TIMEOUT` (30 s): no heartbeats
//! to any daemon, and `last_heartbeat` of every daemon going stale, so the
//! watchdog could then disconnect daemons that were perfectly healthy.
//!
//! Now the loop only *dispatches* a change: it validates it against the current
//! state, queues the command to the daemon, and hands the wait to a spawned
//! task. The task reports back with [`Event::TopologyReply`], and the loop
//! commits the change to the coordinator's state when that event arrives.
//! This is the pattern `TopicUnsubscribe` uses (#3725), with the result fed
//! back because a change has state to commit.
//!
//! # Ordering and conflicts
//!
//! Changes to one dataflow run strictly one at a time (a *lane* per dataflow),
//! in the order they arrive; a change that arrives while another is in flight
//! waits in the lane and is validated only when it starts, against the state
//! the earlier change left behind. That is what the blocked loop used to
//! guarantee implicitly, so `add x` followed by `add x` still fails the second
//! one with "already exists" instead of racing it to the daemon. Changes to
//! different dataflows do not wait for each other.
//!
//! # Outcomes
//!
//! * The daemon confirms: the change is committed, then the CLI is answered.
//! * The daemon rejects: nothing is committed; the CLI gets the daemon's error.
//! * The dataflow is gone when the reply arrives: nothing is committed (there
//!   is no state left to commit to); the CLI gets an error that says so. A
//!   disconnect of the daemon itself does not get here: it commits a change
//!   the daemon already answered before it tears the dataflow down (#4161).
//! * No reply within `TCP_READ_TIMEOUT`, or the daemon's connection is gone:
//!   nothing is committed and the CLI gets an error saying the outcome is
//!   unknown. A reply that arrives after that is dropped by the WS layer (its
//!   pending entry is gone) or ignored here (its change is no longer in
//!   flight); it can never commit anything late.
//! * The CLI disconnects while its change is in flight: the change still
//!   commits, the answer is discarded.

use std::{
    collections::{BTreeSet, HashMap, VecDeque},
    sync::Arc,
    task::Poll,
};

use dora_message::{
    DataflowId,
    common::DaemonId,
    coordinator_to_cli::ControlRequestReply,
    coordinator_to_daemon::{DaemonCoordinatorEvent, Timestamped},
    descriptor::{Node, ResolvedNode},
    id::{DataId, NodeId},
};
use eyre::eyre;
use tokio::sync::mpsc;
use uuid::Uuid;

use crate::{
    Coordinator, Event,
    control_requests::{ReplySender, producer_daemons},
    ensure_add_mapping_applied, ensure_add_node_applied, ensure_remove_mapping_applied,
    ensure_remove_node_applied, ensure_replace_node_applied, resolve_single_node,
    state::{PendingDaemonReply, RunningDataflow},
};

/// How many changes may wait behind the one in flight, per dataflow. They are
/// operator commands; a queue this long means something is wrong, and
/// refusing is better than growing without bound.
const MAX_WAITING_PER_DATAFLOW: usize = 32;

/// A requested change, before it is validated.
pub(crate) enum TopologyRequest {
    AddNode {
        node: Node,
    },
    RemoveNode {
        node_id: NodeId,
        grace_duration: Option<std::time::Duration>,
    },
    ReplaceNode {
        node: Node,
        grace_duration: Option<std::time::Duration>,
    },
    AddMapping(MappingSpec),
    RemoveMapping(MappingSpec),
}

#[derive(Clone)]
pub(crate) struct MappingSpec {
    pub(crate) source_node: NodeId,
    pub(crate) source_output: DataId,
    pub(crate) target_node: NodeId,
    pub(crate) target_input: DataId,
}

impl MappingSpec {
    fn source(&self) -> String {
        format!("{}/{}", self.source_node, self.source_output)
    }

    fn target(&self) -> String {
        format!("{}/{}", self.target_node, self.target_input)
    }
}

/// What to do to the coordinator's state once the daemon has confirmed.
enum Commit {
    AddNode {
        node_id: NodeId,
        daemon_id: DaemonId,
        original_node: Node,
        resolved_node: ResolvedNode,
    },
    RemoveNode {
        node_id: NodeId,
    },
    ReplaceNode {
        node_id: NodeId,
        original_node: Node,
        resolved_node: ResolvedNode,
    },
    AddMapping(MappingSpec),
    RemoveMapping(MappingSpec),
}

impl Commit {
    fn describe(&self) -> String {
        match self {
            Commit::AddNode { node_id, .. } => format!("AddNode `{node_id}`"),
            Commit::RemoveNode { node_id } => format!("RemoveNode `{node_id}`"),
            Commit::ReplaceNode { node_id, .. } => format!("ReplaceNode `{node_id}`"),
            Commit::AddMapping(m) => format!("AddMapping `{}` -> `{}`", m.source(), m.target()),
            Commit::RemoveMapping(m) => {
                format!("RemoveMapping `{}` -> `{}`", m.source(), m.target())
            }
        }
    }
}

struct Queued {
    request: TopologyRequest,
    reply_sender: ReplySender,
}

/// The daemon's answer to the change in flight, shared by the waiter task and
/// the loop.
///
/// The loop takes the answer from here rather than from the `TopologyReply`
/// event, which only says it is ready. That event travels on its own stream,
/// so the daemon's disconnect can be handled first, and the disconnect may
/// take the dataflow out of `running_dataflows` (orphan reclaim). Because the
/// WS layer delivers the reply before it reports the disconnect, the loop can
/// still find the answer here and commit it beforehand (#4161).
enum Answer {
    /// Not taken from the WS layer yet (it may have arrived there).
    Waiting(PendingDaemonReply),
    /// Taken by the waiter.
    Ready(eyre::Result<Vec<u8>>),
    /// Taken by the loop.
    Taken,
}

type AnswerSlot = Arc<std::sync::Mutex<Answer>>;

fn lock_answer(slot: &AnswerSlot) -> std::sync::MutexGuard<'_, Answer> {
    slot.lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// The answer, if the daemon's reply has arrived (or failed), without waiting.
fn take_answer(slot: &AnswerSlot) -> Option<eyre::Result<Vec<u8>>> {
    let mut answer = lock_answer(slot);
    match std::mem::replace(&mut *answer, Answer::Taken) {
        Answer::Waiting(mut reply) => {
            let result = reply.try_reply();
            if result.is_none() {
                *answer = Answer::Waiting(reply);
            }
            result
        }
        Answer::Ready(result) => Some(result),
        Answer::Taken => None,
    }
}

/// Wait for the daemon's reply (for at most `TCP_READ_TIMEOUT`) and leave it
/// in `slot`. Returns early if the loop takes the reply first.
async fn wait_for_answer(slot: AnswerSlot) {
    let answered = tokio::time::timeout(
        dora_message::TCP_READ_TIMEOUT,
        std::future::poll_fn(|cx| {
            let mut answer = lock_answer(&slot);
            let Answer::Waiting(reply) = &mut *answer else {
                return Poll::Ready(());
            };
            let result = std::task::ready!(reply.poll_reply(cx));
            *answer = Answer::Ready(result);
            Poll::Ready(())
        }),
    )
    .await;
    if answered.is_err() {
        let timed_out = std::mem::replace(
            &mut *lock_answer(&slot),
            Answer::Ready(Err(eyre!("timeout waiting for daemon WS reply"))),
        );
        if let Answer::Waiting(reply) = timed_out {
            reply.forget().await;
        }
    }
}

/// The change currently waiting for its daemon.
struct InFlight {
    change_id: Uuid,
    daemon_id: DaemonId,
    /// The connection the command was queued on. If the daemon has since
    /// disconnected (or re-registered), its reply can no longer arrive.
    connection_id: Uuid,
    waiter: tokio::task::AbortHandle,
    answer: AnswerSlot,
    commit: Commit,
    reply_sender: ReplySender,
    /// `NodeStopped` reports for the node this change starts, held back until
    /// the change commits (see `park_node_stop`).
    parked_stops: Vec<ParkedStop>,
}

/// A `NodeStopped` report received while the change that starts its node was
/// still in flight.
struct ParkedStop {
    daemon_id: DaemonId,
    node_id: NodeId,
    clean_stop: bool,
}

#[derive(Default)]
struct Lane {
    in_flight: Option<InFlight>,
    waiting: VecDeque<Queued>,
}

/// The per-dataflow lanes, and the channel the spawned waiters report on.
pub(crate) struct TopologyLanes {
    events: mpsc::Sender<Event>,
    lanes: HashMap<DataflowId, Lane>,
}

impl TopologyLanes {
    pub(crate) fn new(events: mpsc::Sender<Event>) -> Self {
        Self {
            events,
            lanes: HashMap::new(),
        }
    }

    /// Lanes whose completions go nowhere. For state that is built by hand and
    /// never runs a change to completion.
    #[cfg(test)]
    pub(crate) fn detached() -> Self {
        Self::new(mpsc::channel(1).0)
    }

    fn has_in_flight(&self) -> bool {
        self.lanes.values().any(|lane| lane.in_flight.is_some())
    }

    /// A change is in flight or waiting on `dataflow_id`.
    pub(crate) fn is_busy(&self, dataflow_id: &DataflowId) -> bool {
        self.lanes
            .get(dataflow_id)
            .is_some_and(|lane| lane.in_flight.is_some() || !lane.waiting.is_empty())
    }

    /// Hold back a `NodeStopped` report for a node whose `AddNode`,
    /// `ReplaceNode` or `RemoveNode` is still in flight, returning whether it
    /// was held.
    ///
    /// The daemon's report travels on the event stream and the change's reply
    /// on the topology stream, so the report can overtake the reply. Handled
    /// right away it would be dropped (an added node is not yet known) or
    /// undone by the commit (which clears the node's stopped markers), and
    /// the node would show as running although it already exited.
    /// [`Coordinator::handle_topology_reply`] replays held reports after the
    /// commit: a removed node is then unknown and the report is ignored, as it
    /// was when the loop waited for the reply. The daemon drops the exit of a
    /// replaced incarnation, so a report during a replace is the new
    /// incarnation's. (A report about the old incarnation that was sent before
    /// the replace but is handled during it cannot be told apart: the report
    /// carries no incarnation.)
    pub(crate) fn park_node_stop(
        &mut self,
        dataflow_id: &DataflowId,
        daemon_id: &DaemonId,
        node_id: &NodeId,
        clean_stop: bool,
    ) -> bool {
        let Some(in_flight) = self
            .lanes
            .get_mut(dataflow_id)
            .and_then(|lane| lane.in_flight.as_mut())
        else {
            return false;
        };
        let starts_node = match &in_flight.commit {
            Commit::AddNode { node_id: id, .. }
            | Commit::ReplaceNode { node_id: id, .. }
            | Commit::RemoveNode { node_id: id } => id == node_id,
            Commit::AddMapping(_) | Commit::RemoveMapping(_) => false,
        };
        if !starts_node || &in_flight.daemon_id != daemon_id {
            return false;
        }
        in_flight.parked_stops.push(ParkedStop {
            daemon_id: daemon_id.clone(),
            node_id: node_id.clone(),
            clean_stop,
        });
        true
    }

    /// No change is in flight or waiting on any dataflow.
    #[cfg(test)]
    pub(crate) fn is_idle(&self) -> bool {
        self.lanes.is_empty()
    }
}

/// A command validated against the current state and queued to its daemon.
struct Dispatched {
    daemon_id: DaemonId,
    connection_id: Uuid,
    pending: PendingDaemonReply,
    commit: Commit,
}

impl Coordinator {
    /// Refuse an operation that snapshots the dataflow's descriptor while a
    /// node-lifecycle change is still on its way to the stored descriptor.
    /// A restart started now would re-create the dataflow without the change,
    /// which the CLI was told had been applied.
    pub(crate) fn ensure_no_topology_change(&self, dataflow_id: DataflowId) -> eyre::Result<()> {
        if self.topology.is_busy(&dataflow_id) {
            eyre::bail!(
                "a node change is still in progress on dataflow {dataflow_id}; \
                 retry the restart once it has completed"
            );
        }
        Ok(())
    }

    /// Queue a change on its dataflow's lane and start it if the lane is idle.
    /// The reply to `reply_sender` is sent when the change completes.
    pub(crate) async fn submit_topology_change(
        &mut self,
        dataflow_id: DataflowId,
        request: TopologyRequest,
        reply_sender: ReplySender,
    ) {
        let lane = self.topology.lanes.entry(dataflow_id).or_default();
        if lane.waiting.len() >= MAX_WAITING_PER_DATAFLOW {
            let _ = reply_sender.send(Err(eyre!(
                "too many topology changes are already waiting on dataflow {dataflow_id} \
                 ({MAX_WAITING_PER_DATAFLOW}); retry once they have completed"
            )));
            return;
        }
        lane.waiting.push_back(Queued {
            request,
            reply_sender,
        });
        self.pump_topology_lane(dataflow_id).await;
    }

    /// Start waiting changes until one is in flight or the lane is empty. A
    /// change that fails validation is answered at once and the next one tried.
    async fn pump_topology_lane(&mut self, dataflow_id: DataflowId) {
        loop {
            let Some(lane) = self.topology.lanes.get_mut(&dataflow_id) else {
                return;
            };
            if lane.in_flight.is_some() {
                return;
            }
            let Some(Queued {
                request,
                reply_sender,
            }) = lane.waiting.pop_front()
            else {
                self.topology.lanes.remove(&dataflow_id);
                return;
            };

            let dispatched = match self.dispatch_topology_change(dataflow_id, request).await {
                Ok(dispatched) => dispatched,
                Err(err) => {
                    let _ = reply_sender.send(Err(err));
                    continue;
                }
            };
            let Dispatched {
                daemon_id,
                connection_id,
                pending,
                commit,
            } = dispatched;

            let change_id = Uuid::new_v4();
            let events = self.topology.events.clone();
            let answer: AnswerSlot = Arc::new(std::sync::Mutex::new(Answer::Waiting(pending)));
            let waiter_answer = answer.clone();
            let waiter = tokio::spawn(async move {
                wait_for_answer(waiter_answer).await;
                // The loop is gone when this fails (coordinator shutting down).
                let _ = events
                    .send(Event::TopologyReply {
                        dataflow_id,
                        change_id,
                    })
                    .await;
            })
            .abort_handle();
            if let Some(lane) = self.topology.lanes.get_mut(&dataflow_id) {
                lane.in_flight = Some(InFlight {
                    change_id,
                    daemon_id,
                    connection_id,
                    waiter,
                    answer,
                    commit,
                    reply_sender,
                    parked_stops: Vec::new(),
                });
            }
            return;
        }
    }

    /// Validate `request` against the current state and queue its command to
    /// the daemon. Does not wait for the daemon.
    async fn dispatch_topology_change(
        &mut self,
        dataflow_id: DataflowId,
        request: TopologyRequest,
    ) -> eyre::Result<Dispatched> {
        let dataflow = self
            .running_dataflows
            .get(&dataflow_id)
            .ok_or_else(|| eyre!("no running dataflow with ID {dataflow_id}"))?;

        let (daemon_id, event, commit) = match request {
            TopologyRequest::AddNode { node } => {
                // Check the stored topology, not `node_to_daemon`: a dataflow
                // rebuilt after a coordinator restart or orphan reclaim maps
                // only the nodes its reconnected daemons still run, so an
                // exited node (or one on a daemon that has not reported yet)
                // would pass, and the push on commit would persist a duplicate
                // id that the next re-establish fails to resolve (#3759).
                if dataflow.descriptor.nodes.iter().any(|n| n.id == node.id) {
                    eyre::bail!(
                        "node '{}' already exists in dataflow {dataflow_id}",
                        node.id
                    );
                }
                // Keep the original Node so it can be pushed into the stored
                // descriptor once the daemon has confirmed (so `dora info`
                // reflects the new node).
                let original_node = node.clone();
                // See `resolve_single_node` for what the running descriptor
                // contributes (env carry-through #2919, single-operator
                // output prefixing #2877).
                let (node_id, resolved_node) = resolve_single_node(node, &dataflow.descriptor)?;
                // Honour the node's `deploy` placement among the dataflow's
                // daemons (#3672).
                let daemon_id = producer_daemons(
                    &resolved_node,
                    &dataflow.nodes,
                    &dataflow.node_to_daemon,
                    &dataflow.daemons,
                    &dataflow.exited_nodes,
                    &dataflow.departed_daemons,
                    &dataflow.persisted_placement,
                    &self.daemon_connections,
                )
                .and_then(|producer_daemons| {
                    self.daemon_connections.resolve_add_node_daemon(
                        &dataflow.daemons,
                        &producer_daemons,
                        resolved_node.deploy.as_ref(),
                    )
                })
                .map_err(|e| e.wrap_err(format!("cannot place node in dataflow {dataflow_id}")))?;
                let event = DaemonCoordinatorEvent::AddNode {
                    dataflow_id,
                    node: resolved_node.clone(),
                    uv: dataflow.uv,
                };
                (
                    daemon_id.clone(),
                    event,
                    Commit::AddNode {
                        node_id,
                        daemon_id,
                        original_node,
                        resolved_node,
                    },
                )
            }
            TopologyRequest::RemoveNode {
                node_id,
                grace_duration,
            } => {
                let daemon_id = dataflow
                    .node_to_daemon
                    .get(&node_id)
                    .cloned()
                    .ok_or_else(|| eyre!("node '{node_id}' not found in dataflow {dataflow_id}"))?;
                let event = DaemonCoordinatorEvent::RemoveNode {
                    dataflow_id,
                    node_id: node_id.clone(),
                    grace_duration,
                };
                (daemon_id, event, Commit::RemoveNode { node_id })
            }
            TopologyRequest::ReplaceNode {
                node,
                grace_duration,
            } => {
                // Route to the daemon that OWNS the id: a replace targets an
                // existing node, unlike AddNode's first-daemon placement.
                let original_node = node.clone();
                let daemon_id =
                    dataflow
                        .node_to_daemon
                        .get(&node.id)
                        .cloned()
                        .ok_or_else(|| {
                            eyre!(
                                "node '{}' not found in dataflow {dataflow_id}; \
                             use `dora node add` to add a new node",
                                node.id
                            )
                        })?;
                let (node_id, resolved_node) = resolve_single_node(node, &dataflow.descriptor)?;
                let event = DaemonCoordinatorEvent::ReplaceNode {
                    dataflow_id,
                    node: resolved_node.clone(),
                    // Ship the original YAML-shape node so the daemon can
                    // assign the descriptor entry wholesale, mirroring the
                    // `*existing = original_node` commit.
                    unresolved_node: original_node.clone(),
                    uv: dataflow.uv,
                    grace_duration,
                };
                (
                    daemon_id,
                    event,
                    Commit::ReplaceNode {
                        node_id,
                        original_node,
                        resolved_node,
                    },
                )
            }
            TopologyRequest::AddMapping(spec) => {
                let daemon_id = mapping_daemon(dataflow, &spec, dataflow_id)?;
                let event = DaemonCoordinatorEvent::AddMapping {
                    dataflow_id,
                    source_node: spec.source_node.clone(),
                    source_output: spec.source_output.clone(),
                    target_node: spec.target_node.clone(),
                    target_input: spec.target_input.clone(),
                };
                (daemon_id, event, Commit::AddMapping(spec))
            }
            TopologyRequest::RemoveMapping(spec) => {
                let daemon_id = mapping_daemon(dataflow, &spec, dataflow_id)?;
                let event = DaemonCoordinatorEvent::RemoveMapping {
                    dataflow_id,
                    source_node: spec.source_node.clone(),
                    source_output: spec.source_output.clone(),
                    target_node: spec.target_node.clone(),
                    target_input: spec.target_input.clone(),
                };
                (daemon_id, event, Commit::RemoveMapping(spec))
            }
        };

        let msg = serde_json::to_vec(&Timestamped {
            inner: event,
            timestamp: self.clock.new_timestamp(),
        })?;
        let conn = self
            .daemon_connections
            .get(&daemon_id)
            .ok_or_else(|| eyre!("no connection for daemon {daemon_id}"))?;
        let connection_id = conn.connection_id;
        let pending = conn
            .send_request(&msg)
            .await
            .map_err(|e| eyre!("daemon dispatch failed: {e}"))?;
        Ok(Dispatched {
            daemon_id,
            connection_id,
            pending,
            commit,
        })
    }

    /// The daemon answered, timed out, or its wait failed: finish the change
    /// that is in flight under `change_id`, answer the CLI, and start the next
    /// waiting change. A reply for a change that is no longer in flight (it
    /// was already failed because its daemon disconnected) is ignored.
    pub(crate) async fn handle_topology_reply(
        &mut self,
        dataflow_id: DataflowId,
        change_id: Uuid,
    ) -> eyre::Result<()> {
        let in_flight = self
            .topology
            .lanes
            .get_mut(&dataflow_id)
            .and_then(|lane| lane.in_flight.take_if(|f| f.change_id == change_id));
        let Some(in_flight) = in_flight else {
            tracing::debug!(%dataflow_id, %change_id, "ignoring reply to a topology change that is no longer in flight");
            return Ok(());
        };
        // The waiter reports only after leaving the answer in the slot.
        let result = take_answer(&in_flight.answer)
            .unwrap_or_else(|| Err(eyre!("the waiter reported without an answer")));
        self.finish_topology_change(dataflow_id, in_flight, result)
            .await
    }

    /// Commit the in-flight changes on `daemons` whose answer has already
    /// arrived, while their dataflow is still in `running_dataflows`.
    ///
    /// Called when those daemons disconnect, before the disconnect is applied
    /// to the dataflows: it can remove a dataflow whose last daemon left
    /// (orphan reclaim), and the change's `TopologyReply`, which may be handled
    /// after the disconnect, would then find no state to record a change the
    /// daemon had confirmed. The reclaim persists the dataflow's record, so the
    /// change must be in it by then (#4161).
    pub(crate) async fn commit_answered_topology_changes(&mut self, daemons: &BTreeSet<DaemonId>) {
        let mut answered = Vec::new();
        for (dataflow_id, lane) in &mut self.topology.lanes {
            let Some(in_flight) = lane.in_flight.as_ref() else {
                continue;
            };
            if !daemons.contains(&in_flight.daemon_id) {
                continue;
            }
            let Some(result) = take_answer(&in_flight.answer) else {
                continue;
            };
            if let Some(in_flight) = lane.in_flight.take() {
                in_flight.waiter.abort();
                answered.push((*dataflow_id, in_flight, result));
            }
        }
        for (dataflow_id, in_flight, result) in answered {
            // Logged, not returned: the caller must still apply the disconnect.
            if let Err(err) = self
                .finish_topology_change(dataflow_id, in_flight, result)
                .await
            {
                tracing::warn!(%dataflow_id, "failed to finish a topology change: {err:#}");
            }
        }
    }

    /// Finish a change taken out of its lane with the daemon's answer: commit
    /// it, answer the CLI, replay held node exits and start the next change.
    async fn finish_topology_change(
        &mut self,
        dataflow_id: DataflowId,
        in_flight: InFlight,
        result: eyre::Result<Vec<u8>>,
    ) -> eyre::Result<()> {
        let InFlight {
            commit,
            reply_sender,
            parked_stops,
            ..
        } = in_flight;
        let reply = match result {
            Ok(raw) => self.commit_topology_change(dataflow_id, commit, &raw),
            Err(e) => Err(eyre!(
                "daemon dispatch failed: {e}; the outcome of {} is unknown, \
                 check the dataflow's nodes before retrying",
                commit.describe()
            )),
        };
        // The CLI may have gone away while the change was in flight; the
        // change itself is committed regardless.
        let _ = reply_sender.send(reply);
        // After the commit, so they apply to the state the change produced.
        for stop in parked_stops {
            self.handle_daemon_node_stopped(
                stop.daemon_id,
                dataflow_id,
                stop.node_id,
                stop.clean_stop,
            )
            .await?;
        }
        self.pump_topology_lane(dataflow_id).await;
        Ok(())
    }

    /// Fail the in-flight changes whose daemon connection no longer exists:
    /// the reply cannot come any more, and waiting out the timeout would hold
    /// the whole dataflow's lane meanwhile. Called by the event loop after each
    /// event, so every way a daemon can leave is covered.
    pub(crate) async fn fail_topology_changes_of_gone_daemons(&mut self) {
        // The daemon's reply is resolved by the WS reader before the
        // disconnect is reported, but the waiter task may not have forwarded
        // it yet. Let it run, so a reply that did arrive is committed (below,
        // such a change is left to its `TopologyReply`) instead of being
        // reported as unknown.
        if self.topology.has_in_flight() {
            tokio::task::yield_now().await;
        }
        let gone: Vec<DataflowId> = self
            .topology
            .lanes
            .iter()
            .filter(|(_, lane)| {
                lane.in_flight.as_ref().is_some_and(|f| {
                    // A finished waiter has already queued its result.
                    !f.waiter.is_finished()
                        && self.daemon_connections.connection_id_of(&f.daemon_id)
                            != Some(f.connection_id)
                })
            })
            .map(|(id, _)| *id)
            .collect();
        for dataflow_id in gone {
            let Some(in_flight) = self
                .topology
                .lanes
                .get_mut(&dataflow_id)
                .and_then(|lane| lane.in_flight.take())
            else {
                continue;
            };
            in_flight.waiter.abort();
            let _ = in_flight.reply_sender.send(Err(eyre!(
                "daemon dispatch failed: daemon `{}` disconnected before it replied; \
                 the outcome of {} is unknown, check the dataflow's nodes before retrying",
                in_flight.daemon_id,
                in_flight.commit.describe()
            )));
            // Exits held back for the change are still news: apply them as
            // the loop would have had the change never been in flight.
            // Exits held back for the change are still news: apply them as
            // the loop would have had the change never been in flight.
            for stop in in_flight.parked_stops {
                self.handle_daemon_node_stopped(
                    stop.daemon_id,
                    dataflow_id,
                    stop.node_id,
                    stop.clean_stop,
                )
                .await
                .unwrap_or_else(|err| {
                    tracing::warn!(%dataflow_id, "failed to apply a held node exit: {err:#}");
                });
            }
            self.pump_topology_lane(dataflow_id).await;
        }
    }

    /// Apply a confirmed change to the coordinator's state and build the CLI's
    /// reply. Nothing is committed unless the daemon's reply is the success
    /// variant for exactly this change (#1682).
    fn commit_topology_change(
        &mut self,
        dataflow_id: DataflowId,
        commit: Commit,
        raw_reply: &[u8],
    ) -> eyre::Result<ControlRequestReply> {
        // Checked before looking at the dataflow, so a daemon's rejection is
        // reported as such even if the dataflow has ended since.
        match &commit {
            Commit::AddNode { node_id, .. } => ensure_add_node_applied(raw_reply, node_id)?,
            Commit::RemoveNode { node_id } => ensure_remove_node_applied(raw_reply, node_id)?,
            Commit::ReplaceNode { node_id, .. } => ensure_replace_node_applied(raw_reply, node_id)?,
            Commit::AddMapping(m) => {
                ensure_add_mapping_applied(raw_reply, &m.source(), &m.target())?
            }
            Commit::RemoveMapping(m) => {
                ensure_remove_mapping_applied(raw_reply, &m.source(), &m.target())?
            }
        }

        let described = commit.describe();
        let Some(dataflow) = self.running_dataflows.get_mut(&dataflow_id) else {
            eyre::bail!(
                "dataflow {dataflow_id} is no longer running; the daemon confirmed {described} \
                 but there is no coordinator state left to record it in"
            );
        };

        match commit {
            Commit::AddNode {
                node_id,
                daemon_id,
                original_node,
                resolved_node,
            } => {
                dataflow.node_to_daemon.insert(node_id.clone(), daemon_id);
                // Update the stored descriptor and resolved nodes so `dora
                // info` reflects the new node. Not twice: a dataflow
                // re-established while the change was in flight may already
                // list it (#3759).
                if !dataflow.descriptor.nodes.iter().any(|n| n.id == node_id) {
                    dataflow.descriptor.nodes.push(original_node);
                }
                dataflow.nodes.insert(node_id.clone(), resolved_node);
                // Clear any stale Stopped/Finalized state for this node id so
                // the new incarnation's metrics push isn't blocked by the
                // prior stop's `node_stopped_at` / `node_finalized` entries.
                dataflow.node_stopped_at.remove(&node_id);
                dataflow.node_finalized.remove(&node_id);
                dataflow.exited_nodes.remove(&node_id);
                dataflow.node_metrics.remove(&node_id);
                dataflow.persist_topology_change(&*self.store);
                Ok(ControlRequestReply::NodeAdded {
                    dataflow_id,
                    node_id,
                })
            }
            Commit::RemoveNode { node_id } => {
                // Clean up coordinator state (inverse of AddNode inserts).
                dataflow.node_to_daemon.remove(&node_id);
                dataflow.descriptor.nodes.retain(|n| n.id != node_id);
                dataflow.nodes.remove(&node_id);
                dataflow.persist_topology_change(&*self.store);
                Ok(ControlRequestReply::NodeRemoved {
                    dataflow_id,
                    node_id,
                })
            }
            Commit::ReplaceNode {
                node_id,
                original_node,
                resolved_node,
            } => {
                if let Some(existing) = dataflow
                    .descriptor
                    .nodes
                    .iter_mut()
                    .find(|n| n.id == node_id)
                {
                    *existing = original_node;
                } else {
                    dataflow.descriptor.nodes.push(original_node);
                }
                dataflow.nodes.insert(node_id.clone(), resolved_node);
                // Clear stale lifecycle markers so the new incarnation's
                // metrics are not suppressed (same set AddNode clears).
                dataflow.node_stopped_at.remove(&node_id);
                dataflow.node_finalized.remove(&node_id);
                dataflow.exited_nodes.remove(&node_id);
                dataflow.node_metrics.remove(&node_id);
                dataflow.persist_topology_change(&*self.store);
                Ok(ControlRequestReply::NodeReplaced {
                    dataflow_id,
                    node_id,
                })
            }
            Commit::AddMapping(m) => Ok(ControlRequestReply::MappingAdded {
                dataflow_id,
                source_node: m.source_node,
                source_output: m.source_output,
                target_node: m.target_node,
                target_input: m.target_input,
            }),
            Commit::RemoveMapping(m) => Ok(ControlRequestReply::MappingRemoved {
                dataflow_id,
                source_node: m.source_node,
                source_output: m.source_output,
                target_node: m.target_node,
                target_input: m.target_input,
            }),
        }
    }
}

/// The daemon that owns a mapping's target node.
fn mapping_daemon(
    dataflow: &RunningDataflow,
    spec: &MappingSpec,
    dataflow_id: DataflowId,
) -> eyre::Result<DaemonId> {
    dataflow
        .node_to_daemon
        .get(&spec.target_node)
        .cloned()
        .ok_or_else(|| {
            eyre!(
                "target node '{}' not found in dataflow {dataflow_id}",
                spec.target_node
            )
        })
}
