//! Live topology edits (`dora node add`/`replace`/`remove`, `dora mapping
//! add`/`remove`): checking that a daemon applied the edit it acknowledged.

use crate::state::RunningDataflow;
use dora_message::{
    daemon_to_coordinator::DaemonCoordinatorReply,
    descriptor::{Descriptor, Node, ResolvedNode},
    id::NodeId,
};
use eyre::eyre;
use std::time::{Duration, Instant};

/// Validate that the daemon's reply to `DaemonCoordinatorEvent::AddNode`
/// is a successful `AddNodeResult`. Returns `Err` for both an explicit
/// daemon failure and an unexpected reply variant; callers should forward
/// the error to the CLI as a `ControlRequestReply::Error` and NOT use `?`
/// to bubble out of the coordinator's main loop. Rescue of #1757,
/// addresses #1682.
pub(crate) fn ensure_add_node_applied(
    reply_raw: &[u8],
    node_id: &dora_core::config::NodeId,
) -> eyre::Result<()> {
    match serde_json::from_slice(reply_raw)? {
        DaemonCoordinatorReply::AddNodeResult(Ok(())) => Ok(()),
        DaemonCoordinatorReply::AddNodeResult(Err(err)) => {
            Err(eyre!("daemon failed to add node `{node_id}`: {err}"))
        }
        other => Err(eyre!(
            "unexpected daemon reply for AddNode on node `{node_id}`: {other:?}"
        )),
    }
}

/// Resolve a single dynamically-supplied node definition via a temporary
/// one-node descriptor, in the context of the dataflow it is joining.
/// Shared by the AddNode and ReplaceNode arms so the two commands can never
/// resolve differently.
///
/// `running_descriptor` is the dataflow's stored descriptor, which at call
/// time holds exactly the nodes already in the dataflow. Two things must come
/// from it rather than from the supplied node alone, and each was a silent
/// data-loss bug while they didn't:
///
/// - **`env`**: carried onto the temporary descriptor so the resolver's
///   dataflow-into-node merge gives the node the same environment its
///   statically declared peers got, including anything set via `dora start
///   --env`. Node keys still win on conflict (dora-rs/dora#2919).
/// - **Single-`operator:` output prefixing**: whole-descriptor resolution
///   rewrites an input referencing such a producer from the bare output name
///   to the operator-qualified one (`result` -> `op/result`). Resolving in
///   isolation left that producer invisible, so the node subscribed to a name
///   nobody publishes and silently received no data (dora-rs/dora#2877).
pub(crate) fn resolve_single_node(
    node: Node,
    running_descriptor: &Descriptor,
) -> eyre::Result<(NodeId, ResolvedNode)> {
    // Only `env` is carried over from the running dataflow — the rest stay at
    // `Descriptor::new`'s defaults, as before.
    let mut tmp_desc = Descriptor::new(vec![node]);
    tmp_desc.env = running_descriptor.env.clone();
    dora_core::descriptor::resolve_aliases_and_set_defaults_in_topology(
        &tmp_desc,
        &running_descriptor.nodes,
    )
    .map_err(|e| eyre!("failed to resolve node: {e}"))?
    .pop_first()
    .ok_or_else(|| eyre!("node descriptor resolved to empty map"))
}

/// Validate that the daemon's reply to `DaemonCoordinatorEvent::ReplaceNode`
/// is a successful `ReplaceNodeResult` — same specific-reply contract as
/// `ensure_add_node_applied` (#1682).
pub(crate) fn ensure_replace_node_applied(
    reply_raw: &[u8],
    node_id: &dora_core::config::NodeId,
) -> eyre::Result<()> {
    match serde_json::from_slice(reply_raw)? {
        DaemonCoordinatorReply::ReplaceNodeResult(Ok(())) => Ok(()),
        DaemonCoordinatorReply::ReplaceNodeResult(Err(err)) => {
            Err(eyre!("daemon failed to replace node `{node_id}`: {err}"))
        }
        other => Err(eyre!(
            "unexpected daemon reply for ReplaceNode on node `{node_id}`: {other:?}"
        )),
    }
}

/// How long a node's `Stopped` row stays visible in
/// `running_dataflows[df].node_metrics` after a `DaemonEvent::NodeStopped`
/// arrives before the coordinator drops it. Long enough for an operator
/// running `dora node list` to see the `Stopped` status; short enough that
/// the listing doesn't accumulate zombie rows over a long-lived dataflow.
pub(crate) const NODE_STOPPED_GRACE: Duration = Duration::from_secs(60);

/// Drop any `node_metrics` rows whose corresponding `node_stopped_at`
/// timestamp is older than `NODE_STOPPED_GRACE`. Called from the
/// `NodeMetrics` push handler and the heartbeat tick so cleanup runs
/// even when no live metrics flow.
pub(crate) fn expire_stopped_nodes(dataflow: &mut RunningDataflow) {
    let now = Instant::now();
    let expired: Vec<dora_core::config::NodeId> = dataflow
        .node_stopped_at
        .iter()
        .filter(|(_, t)| now.duration_since(**t) >= NODE_STOPPED_GRACE)
        .map(|(nid, _)| nid.clone())
        .collect();
    for nid in expired {
        dataflow.node_metrics.remove(&nid);
        dataflow.node_stopped_at.remove(&nid);
        // Clear the finalize marker too so a subsequent AddNode of the
        // same name (or any future metrics push for it) is not blocked
        // by the stale Stopped state.
        dataflow.node_finalized.remove(&nid);
    }
}

/// Validate that the daemon's reply to `DaemonCoordinatorEvent::RemoveNode`
/// is a successful `RemoveNodeResult`. Returns `Err` for both an explicit
/// daemon failure and an unexpected reply variant; callers should forward
/// the error to the CLI as a `ControlRequestReply::Error` and NOT use `?`
/// to bubble out of the coordinator's main loop. Parallel to
/// `ensure_add_node_applied` (#1873). Closes #1874.
pub(crate) fn ensure_remove_node_applied(
    reply_raw: &[u8],
    node_id: &dora_core::config::NodeId,
) -> eyre::Result<()> {
    match serde_json::from_slice(reply_raw)? {
        DaemonCoordinatorReply::RemoveNodeResult(Ok(())) => Ok(()),
        DaemonCoordinatorReply::RemoveNodeResult(Err(err)) => {
            Err(eyre!("daemon failed to remove node `{node_id}`: {err}"))
        }
        other => Err(eyre!(
            "unexpected daemon reply for RemoveNode on node `{node_id}`: {other:?}"
        )),
    }
}

/// Validate that the daemon's reply to `DaemonCoordinatorEvent::AddMapping`
/// is a successful `AddMappingResult`. Parallel to `ensure_add_node_applied`.
/// Before the daemon returned an explicit `AddMappingResult`, the coordinator's
/// `send_and_receive` for `AddMapping` timed out after 30s because the WS
/// layer dropped the daemon's `None` reply; closing that hole means we now
/// have a typed reply to check against — same #1682 class.
pub(crate) fn ensure_add_mapping_applied(
    reply_raw: &[u8],
    source: &str,
    target: &str,
) -> eyre::Result<()> {
    match serde_json::from_slice(reply_raw)? {
        DaemonCoordinatorReply::AddMappingResult(Ok(())) => Ok(()),
        DaemonCoordinatorReply::AddMappingResult(Err(err)) => Err(eyre!(
            "daemon failed to add mapping `{source}` -> `{target}`: {err}"
        )),
        other => Err(eyre!(
            "unexpected daemon reply for AddMapping `{source}` -> `{target}`: {other:?}"
        )),
    }
}

/// Validate that the daemon's reply to `DaemonCoordinatorEvent::RemoveMapping`
/// is a successful `RemoveMappingResult`. See `ensure_add_mapping_applied`.
pub(crate) fn ensure_remove_mapping_applied(
    reply_raw: &[u8],
    source: &str,
    target: &str,
) -> eyre::Result<()> {
    match serde_json::from_slice(reply_raw)? {
        DaemonCoordinatorReply::RemoveMappingResult(Ok(())) => Ok(()),
        DaemonCoordinatorReply::RemoveMappingResult(Err(err)) => Err(eyre!(
            "daemon failed to remove mapping `{source}` -> `{target}`: {err}"
        )),
        other => Err(eyre!(
            "unexpected daemon reply for RemoveMapping `{source}` -> `{target}`: {other:?}"
        )),
    }
}
