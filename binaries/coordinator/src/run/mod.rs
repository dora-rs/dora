use crate::DaemonConnections;

use dora_core::{descriptor::DescriptorExt, uhlc::HLC};
use dora_message::{
    BuildId, SessionId,
    common::DaemonId,
    coordinator_to_daemon::{DaemonCoordinatorEvent, SpawnDataflowNodes, Timestamped},
    daemon_to_coordinator::DaemonCoordinatorReply,
    descriptor::{Deploy, Descriptor, ResolvedNode},
    id::NodeId,
};
use eyre::{ContextCompat, WrapErr, bail, eyre};
use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
};
use uuid::{NoContext, Timestamp, Uuid};

#[allow(clippy::too_many_arguments)]
#[tracing::instrument(skip(daemon_connections, clock))]
pub(super) async fn spawn_dataflow(
    build_id: Option<BuildId>,
    session_id: SessionId,
    dataflow: Descriptor,
    local_working_dir: Option<PathBuf>,
    daemon_connections: &mut DaemonConnections,
    clock: &HLC,
    uv: bool,
    write_events_to: Option<PathBuf>,
) -> eyre::Result<SpawnedDataflow> {
    let nodes = dataflow.resolve_aliases_and_set_defaults()?;
    let uuid = Uuid::new_v7(Timestamp::now(NoContext));

    // Resolve each node to its target daemon, then group by daemon.
    let mut nodes_by_daemon: BTreeMap<DaemonId, Vec<&ResolvedNode>> = BTreeMap::new();
    for node in nodes.values() {
        let daemon_id = resolve_daemon(daemon_connections, node.deploy.as_ref())?;
        nodes_by_daemon.entry(daemon_id).or_default().push(node);
    }

    let mut daemons = BTreeSet::new();
    let mut node_to_daemon = BTreeMap::new();

    for (daemon_id, nodes_on_daemon) in &nodes_by_daemon {
        let spawn_nodes = nodes_on_daemon.iter().map(|n| n.id.clone()).collect();
        tracing::debug!(
            "Spawning dataflow `{uuid}` on daemon `{daemon_id}` (nodes: {spawn_nodes:?})"
        );

        let spawn_command = SpawnDataflowNodes {
            build_id,
            session_id,
            dataflow_id: uuid,
            local_working_dir: local_working_dir.clone(),
            nodes: nodes.clone(),
            dataflow_descriptor: dataflow.clone(),
            spawn_nodes,
            uv,
            write_events_to: write_events_to.clone(),
            artifact_base_url: None,
        };
        let message = serde_json::to_vec(&Timestamped {
            inner: DaemonCoordinatorEvent::Spawn(spawn_command),
            timestamp: clock.new_timestamp(),
        })?;

        let daemon_connection = daemon_connections
            .get_mut(daemon_id)
            .wrap_err_with(|| format!("no daemon connection for daemon `{daemon_id}`"))?;

        let spawn_result: eyre::Result<()> = async {
            let reply_raw = daemon_connection
                .send_and_receive(&message)
                .await
                .wrap_err("failed to send/receive spawn message")?;
            match serde_json::from_slice(&reply_raw)
                .wrap_err("failed to deserialize spawn reply from daemon")?
            {
                DaemonCoordinatorReply::TriggerSpawnResult(result) => result
                    .map_err(|e| eyre!(e))
                    .wrap_err("daemon returned an error")?,
                _ => bail!("unexpected reply"),
            }
            Ok(())
        }
        .await;

        if let Err(spawn_err) = spawn_result {
            // The failing daemon was never inserted into `daemons`, so rollback
            // only targets daemons that confirmed a successful spawn.
            debug_assert!(
                !daemons.contains(daemon_id),
                "failing daemon should not be in the spawned set"
            );
            // Rollback: stop dataflow on daemons that already started successfully.
            let rollback_errors =
                rollback_spawned_daemons(uuid, &daemons, daemon_connections, clock).await;
            if rollback_errors.is_empty() {
                return Err(spawn_err.wrap_err(format!(
                    "spawn failed on daemon `{daemon_id}`; \
                     rolled back {n} previously-started daemon(s)",
                    n = daemons.len()
                )));
            } else {
                let rollback_summary = rollback_errors
                    .iter()
                    .map(|(id, e)| format!("  {id}: {e}"))
                    .collect::<Vec<_>>()
                    .join("\n");
                return Err(spawn_err.wrap_err(format!(
                    "spawn failed on daemon `{daemon_id}`; \
                     rollback attempted on {n} previously-started daemon(s), \
                     {f} stop(s) failed:\n{rollback_summary}",
                    n = daemons.len(),
                    f = rollback_errors.len(),
                )));
            }
        }

        daemons.insert(daemon_id.clone());
        for node in nodes_on_daemon {
            node_to_daemon.insert(node.id.clone(), daemon_id.clone());
        }
    }

    tracing::info!("successfully triggered dataflow spawn `{uuid}`",);

    Ok(SpawnedDataflow {
        uuid,
        daemons,
        nodes,
        node_to_daemon,
    })
}

/// Resolve which daemon should run a node based on its deploy config.
/// Priority: machine > labels > unnamed.
pub(super) fn resolve_daemon(
    connections: &DaemonConnections,
    deploy: Option<&Deploy>,
) -> eyre::Result<DaemonId> {
    match deploy {
        Some(Deploy {
            machine: Some(machine),
            ..
        }) => connections
            .get_matching_daemon_id(machine)
            .cloned()
            .wrap_err_with(|| format!("no matching daemon for machine id `{machine}`")),
        Some(d) if !d.labels.is_empty() => connections
            .get_matching_daemon_by_labels(&d.labels)
            .cloned()
            .wrap_err_with(|| format!("no daemon matches labels {:?}", d.labels)),
        _ => connections
            .unnamed()
            .next()
            .cloned()
            .wrap_err("no unnamed daemon connections"),
    }
}

#[derive(Debug)]
pub struct SpawnedDataflow {
    pub uuid: Uuid,
    pub daemons: BTreeSet<DaemonId>,
    pub nodes: BTreeMap<NodeId, ResolvedNode>,
    pub node_to_daemon: BTreeMap<NodeId, DaemonId>,
}

/// Best-effort compensating rollback: send `StopDataflow` to each daemon that
/// already acknowledged a successful spawn. Returns a list of `(daemon_id, error)`
/// for any daemons where the stop itself failed.
///
/// # Preconditions
///
/// `spawned_daemons` must only contain daemon IDs that the coordinator confirmed
/// as successfully spawned during the current `spawn_dataflow` call.
///
/// # Design notes
///
/// - Uses `force: true` because nodes have not reached a ready state during a
///   partial spawn, so graceful shutdown hooks are unnecessary.
/// - Stops are issued sequentially because `&mut DaemonConnections` cannot
///   hand out multiple mutable references. Each call is individually bounded
///   by `TCP_READ_TIMEOUT`.
/// - Daemon-supplied error strings are truncated to prevent oversized messages
///   propagating to clients.
async fn rollback_spawned_daemons(
    dataflow_id: Uuid,
    spawned_daemons: &BTreeSet<DaemonId>,
    daemon_connections: &mut DaemonConnections,
    clock: &HLC,
) -> Vec<(DaemonId, String)> {
    if spawned_daemons.is_empty() {
        return Vec::new();
    }
    tracing::warn!(
        "rolling back dataflow `{dataflow_id}` on {} daemon(s) after partial spawn failure",
        spawned_daemons.len()
    );

    let stop_message = match serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::StopDataflow {
            dataflow_id,
            grace_duration: None,
            force: true,
        },
        timestamp: clock.new_timestamp(),
    }) {
        Ok(m) => m,
        Err(e) => {
            // Serialization failure is unlikely but fatal for rollback.
            return spawned_daemons
                .iter()
                .map(|id| (id.clone(), format!("failed to serialize stop message: {e}")))
                .collect();
        }
    };

    // NOTE: We cannot use `join_all` here because each `send_and_receive`
    // requires `&mut DaemonConnection`, and `DaemonConnections` cannot hand out
    // multiple mutable references concurrently. Each call is individually bounded
    // by `TCP_READ_TIMEOUT`, so worst-case wall time is N × timeout.
    let mut errors = Vec::new();
    for daemon_id in spawned_daemons {
        // The `?` stays inside this async block intentionally: one daemon's
        // failure must not abort rollback for the remaining daemons.
        let result: eyre::Result<()> = async {
            let conn = daemon_connections
                .get_mut(daemon_id)
                .wrap_err("no daemon connection")?;
            let reply_raw = conn
                .send_and_receive(&stop_message)
                .await
                .wrap_err("failed to send/receive stop message")?;
            match serde_json::from_slice(&reply_raw).wrap_err("failed to deserialize stop reply")? {
                DaemonCoordinatorReply::StopResult(r) => {
                    r.map_err(|e| eyre!(e)).wrap_err("daemon stop failed")?;
                }
                other => bail!("unexpected reply during rollback: {other:?}"),
            }
            Ok(())
        }
        .await;

        match result {
            Ok(()) => {
                tracing::info!(
                    "rollback stop succeeded for daemon `{daemon_id}` \
                     on dataflow `{dataflow_id}`"
                );
            }
            Err(e) => {
                tracing::error!(
                    "rollback stop failed for daemon `{daemon_id}` \
                     on dataflow `{dataflow_id}`: {e:#}"
                );
                let msg = format!("{e:#}");
                let msg = crate::state::truncate_str(&msg, MAX_ROLLBACK_ERROR_BYTES);
                errors.push((daemon_id.clone(), msg.to_owned()));
            }
        }
    }
    errors
}

/// Maximum bytes for a single daemon error string in rollback summaries.
const MAX_ROLLBACK_ERROR_BYTES: usize = 1024;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::DaemonConnection;
    use std::collections::HashMap;
    use std::sync::Arc;
    use tokio::sync::{Mutex, mpsc};

    /// Create a mock `DaemonConnection` + a background task that auto-replies.
    ///
    /// The mock extracts the WsRequest correlation `id` and the
    /// `DaemonCoordinatorEvent` variant name from each incoming message,
    /// passes the variant to `reply_fn`, and resolves the pending oneshot.
    ///
    /// Full deserialization of `Timestamped<DaemonCoordinatorEvent>` is
    /// intentionally skipped because uhlc timestamps use u128, which does
    /// not round-trip through `serde_json::Value`.
    fn mock_daemon(
        reply_fn: impl Fn(&str) -> DaemonCoordinatorReply + Send + 'static,
    ) -> DaemonConnection {
        let (tx, mut rx) = mpsc::channel::<String>(16);
        let pending: Arc<Mutex<HashMap<Uuid, tokio::sync::oneshot::Sender<String>>>> =
            Arc::new(Mutex::new(HashMap::new()));
        let pending_clone = pending.clone();

        tokio::spawn(async move {
            while let Some(msg) = rx.recv().await {
                let v: serde_json::Value = serde_json::from_str(&msg).unwrap();
                let id: Uuid = v["id"].as_str().unwrap().parse().unwrap();

                // Extract variant name from `params.inner`. The params field
                // is embedded as raw JSON (not a string). For externally-tagged
                // enum variants like `StopDataflow { .. }`, serde produces
                // `{"StopDataflow": {...}}`. For newtype variants like
                // `Spawn(...)`, serde produces `{"Spawn": {...}}`.
                let inner = &v["params"]["inner"];
                let variant = if let Some(obj) = inner.as_object() {
                    obj.keys().next().cloned().unwrap_or_default()
                } else if let Some(s) = inner.as_str() {
                    s.to_owned()
                } else {
                    String::new()
                };

                let reply = reply_fn(&variant);
                let reply_json = serde_json::to_string(&reply).unwrap();

                let sender = pending_clone.lock().await.remove(&id);
                if let Some(tx) = sender {
                    let _ = tx.send(reply_json);
                }
            }
        });

        DaemonConnection::new(tx, pending, BTreeMap::new())
    }

    #[tokio::test]
    async fn rollback_sends_stop_to_spawned_daemons() {
        let clock = HLC::default();
        let dataflow_id = Uuid::new_v4();

        let stop_received = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let stop_flag = stop_received.clone();

        let conn = mock_daemon(move |variant| {
            assert_eq!(
                variant, "StopDataflow",
                "expected StopDataflow, got {variant}"
            );
            stop_flag.store(true, std::sync::atomic::Ordering::SeqCst);
            DaemonCoordinatorReply::StopResult(Ok(()))
        });

        let daemon_id = DaemonId::new(Some("rollback-test".to_string()));
        let mut connections = DaemonConnections::default();
        connections.add(daemon_id.clone(), conn);

        let spawned = BTreeSet::from([daemon_id]);
        let errors =
            rollback_spawned_daemons(dataflow_id, &spawned, &mut connections, &clock).await;

        assert!(errors.is_empty(), "rollback should succeed: {errors:?}");
        assert!(
            stop_received.load(std::sync::atomic::Ordering::SeqCst),
            "daemon should have received StopDataflow"
        );
    }

    #[tokio::test]
    async fn rollback_reports_daemon_stop_failure() {
        let clock = HLC::default();
        let dataflow_id = Uuid::new_v4();

        let conn = mock_daemon(|variant| {
            assert_eq!(variant, "StopDataflow");
            DaemonCoordinatorReply::StopResult(Err("stop rejected".into()))
        });

        let daemon_id = DaemonId::new(Some("fail-stop".to_string()));
        let mut connections = DaemonConnections::default();
        connections.add(daemon_id.clone(), conn);

        let spawned = BTreeSet::from([daemon_id.clone()]);
        let errors =
            rollback_spawned_daemons(dataflow_id, &spawned, &mut connections, &clock).await;

        assert_eq!(errors.len(), 1);
        assert_eq!(errors[0].0, daemon_id);
        assert!(
            errors[0].1.contains("stop rejected"),
            "error should contain daemon message, got: {}",
            errors[0].1
        );
    }

    #[tokio::test]
    async fn rollback_empty_set_is_noop() {
        let clock = HLC::default();
        let mut connections = DaemonConnections::default();
        let errors =
            rollback_spawned_daemons(Uuid::new_v4(), &BTreeSet::new(), &mut connections, &clock)
                .await;
        assert!(errors.is_empty());
    }

    #[tokio::test]
    async fn spawn_failure_triggers_rollback_on_previous_daemons() {
        let clock = HLC::default();

        let first_stop_received = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let first_stop_flag = first_stop_received.clone();

        // First daemon: spawn succeeds, stop (rollback) succeeds.
        let first_conn = mock_daemon(move |variant| match variant {
            "Spawn" => DaemonCoordinatorReply::TriggerSpawnResult(Ok(())),
            "StopDataflow" => {
                first_stop_flag.store(true, std::sync::atomic::Ordering::SeqCst);
                DaemonCoordinatorReply::StopResult(Ok(()))
            }
            other => panic!("unexpected variant on first daemon: {other}"),
        });

        // Second daemon: spawn fails.
        let second_conn = mock_daemon(|variant| match variant {
            "Spawn" => {
                DaemonCoordinatorReply::TriggerSpawnResult(Err("node binary missing".into()))
            }
            other => panic!("unexpected variant on second daemon: {other}"),
        });

        // Use machine IDs that sort deterministically: "aaa" < "bbb" in BTreeMap.
        let first_id = DaemonId::new(Some("aaa".to_string()));

        let mut connections = DaemonConnections::default();
        connections.add(first_id.clone(), first_conn);
        connections.add(DaemonId::new(Some("bbb".to_string())), second_conn);

        // Minimal dataflow with two nodes deployed to different machines.
        let dataflow: Descriptor = serde_json::from_str(
            r#"{
                "nodes": [
                    {
                        "id": "node-a",
                        "path": "/tmp/dummy-a",
                        "_unstable_deploy": { "machine": "aaa" },
                        "outputs": ["out"]
                    },
                    {
                        "id": "node-b",
                        "path": "/tmp/dummy-b",
                        "_unstable_deploy": { "machine": "bbb" },
                        "inputs": { "in": "node-a/out" }
                    }
                ]
            }"#,
        )
        .unwrap();

        let result = spawn_dataflow(
            None,
            SessionId::generate(),
            dataflow,
            None,
            &mut connections,
            &clock,
            false,
            None,
        )
        .await;

        assert!(result.is_err(), "spawn should fail");
        let err_msg = format!("{:#}", result.unwrap_err());
        assert!(
            err_msg.contains("node binary missing"),
            "error should contain root cause, got: {err_msg}"
        );
        assert!(
            first_stop_received.load(std::sync::atomic::Ordering::SeqCst),
            "first daemon should have received StopDataflow for rollback"
        );
    }
}
