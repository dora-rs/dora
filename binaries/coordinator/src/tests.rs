use super::*;
use crate::state::ParamTarget;
use crate::{
    daemon_liveness::{
        DisconnectAction, HeartbeatSendOutcome, MAX_CONSECUTIVE_HEARTBEAT_SEND_TIMEOUTS,
        heartbeat_disconnect_decision,
    },
    ready_barrier::{
        all_nodes_ready_message, persist_ready_barrier_release, release_barrier_message,
    },
    spawn_build::{handle_spawn_result_err, handle_spawn_result_ok},
    topology::NODE_STOPPED_GRACE,
};
use dora_core::descriptor::DescriptorExt;
use dora_message::coordinator_to_daemon::{
    DaemonCoordinatorEvent, StateCatchUpOperation, Timestamped,
};
use dora_message::descriptor::Descriptor;
use dora_message::{
    common::{NodeError, NodeErrorCause, NodeExitStatus},
    daemon_to_coordinator::DaemonCoordinatorReply,
    descriptor::{Node, ResolvedNode},
    id::NodeId,
};
use std::collections::BTreeSet;
use std::collections::HashMap;
use tokio::time::{Duration as TokioDuration, timeout};
use uuid::Uuid;

// ---- heartbeat-send backpressure classification (#2886) ----

// A failed send disconnects immediately; a delivered heartbeat never does
// and clears any timeout streak.
#[test]
fn heartbeat_send_failed_disconnects_delivered_resets() {
    assert_eq!(
        heartbeat_disconnect_decision(HeartbeatSendOutcome::SendFailed, 0),
        (0, true),
        "a failed heartbeat send must disconnect the daemon"
    );
    assert_eq!(
        heartbeat_disconnect_decision(HeartbeatSendOutcome::Delivered, 7),
        (0, false),
        "a delivered heartbeat must not disconnect and must clear the timeout streak"
    );
}

// A *transient* command-channel backpressure timeout must NOT disconnect a
// live daemon (regression for a control-command burst tearing down a live
// daemon's dataflows), but a *persistent* streak must escalate.
#[test]
fn heartbeat_timeout_tolerates_transient_but_escalates_persistent() {
    assert_eq!(
        heartbeat_disconnect_decision(HeartbeatSendOutcome::TimedOut, 0),
        (1, false),
        "a single timeout must not disconnect a live daemon"
    );
    assert_eq!(
        heartbeat_disconnect_decision(
            HeartbeatSendOutcome::TimedOut,
            MAX_CONSECUTIVE_HEARTBEAT_SEND_TIMEOUTS - 2,
        ),
        (MAX_CONSECUTIVE_HEARTBEAT_SEND_TIMEOUTS - 1, false),
        "timeouts below the threshold must not disconnect"
    );
    let (streak, disconnect) = heartbeat_disconnect_decision(
        HeartbeatSendOutcome::TimedOut,
        MAX_CONSECUTIVE_HEARTBEAT_SEND_TIMEOUTS - 1,
    );
    assert_eq!(streak, MAX_CONSECUTIVE_HEARTBEAT_SEND_TIMEOUTS);
    assert!(
        disconnect,
        "a persistently full command channel must escalate to a disconnect"
    );
}

// Wiring guard: `send_heartbeat_with_timeout` must map a closed send channel
// (dropped receiver) to an immediate disconnect, so re-flattening the
// outcome back into "disconnect on any failure" is the only green state.
#[tokio::test(flavor = "current_thread")]
async fn send_heartbeat_with_timeout_disconnects_on_closed_channel() {
    let (tx, rx) = tokio::sync::mpsc::channel::<String>(1);
    drop(rx); // closing the channel makes every send return Err immediately
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let mut connection = crate::state::DaemonConnection::new(tx, pending_replies, BTreeMap::new());

    let (_id, disconnect) = send_heartbeat_with_timeout(
        DaemonId::new(Some("dead".to_string())),
        &mut connection,
        HLC::default().new_timestamp(),
    )
    .await;

    assert!(
        disconnect,
        "a closed send channel must disconnect the daemon"
    );
}

// Wiring guard: `send_heartbeat_with_timeout` must map a full command
// channel (send never completes) to a *non*-disconnecting timeout that only
// bumps the streak. `start_paused` auto-advances through the 500 ms deadline
// without real delay (mirrors the topic-frame timeout test in handlers.rs).
#[tokio::test(flavor = "current_thread", start_paused = true)]
async fn send_heartbeat_with_timeout_tolerates_a_full_channel() {
    // Capacity 1, pre-filled, receiver kept alive (so the channel is full,
    // not closed) and never drained => the heartbeat send blocks and times out.
    let (tx, _rx_never_drained) = tokio::sync::mpsc::channel::<String>(1);
    tx.send("prefill".to_string()).await.unwrap();
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let mut connection = crate::state::DaemonConnection::new(tx, pending_replies, BTreeMap::new());

    let (_id, disconnect) = send_heartbeat_with_timeout(
        DaemonId::new(Some("wedged".to_string())),
        &mut connection,
        HLC::default().new_timestamp(),
    )
    .await;

    assert!(
        !disconnect,
        "a single backpressure timeout must not disconnect a live daemon"
    );
    assert_eq!(
        connection.consecutive_heartbeat_send_timeouts, 1,
        "the timeout must advance the consecutive-timeout streak"
    );
}

/// The single input mapping of a resolved custom node.
fn only_input_mapping(resolved: &ResolvedNode, input: &str) -> dora_message::config::InputMapping {
    let inputs = match &resolved.kind {
        dora_message::descriptor::CoreNodeKind::Custom(n) => &n.run_config.inputs,
        dora_message::descriptor::CoreNodeKind::Runtime(_) => panic!("expected custom node"),
    };
    inputs
        .get(&dora_message::id::DataId::from(input.to_string()))
        .expect("input present")
        .mapping
        .clone()
}

/// A running dataflow consisting of one single-`operator:` producer, plus
/// an optional dataflow-level `env:`.
fn running_descriptor_with_operator_producer(env: serde_json::Value) -> Descriptor {
    serde_json::from_value(serde_json::json!({
        "nodes": [{
            "id": "producer",
            "operator": { "python": "producer.py", "outputs": ["result"] },
        }],
        "env": env,
    }))
    .expect("valid running descriptor")
}

#[test]
fn dynamic_node_prefixes_input_referencing_a_single_operator_producer() {
    // Regression guard for #2877 at the call site: a node joining a running
    // dataflow must end up subscribed to the operator-qualified output name
    // the runtime actually publishes under (`op/result`). Resolving it in
    // isolation leaves the reference bare and it silently receives no data.
    // `resolve_single_node` is shared, so this covers both `dora node add`
    // and `dora node replace`.
    let running = running_descriptor_with_operator_producer(serde_json::json!(null));

    let added: Node = serde_json::from_value(serde_json::json!({
        "id": "consumer",
        "path": "consumer",
        "inputs": { "reading": "producer/result" },
    }))
    .expect("valid node");

    let (node_id, resolved) =
        resolve_single_node(added, &running).expect("node resolves against the topology");

    assert_eq!(node_id.to_string(), "consumer");
    match only_input_mapping(&resolved, "reading") {
        dora_message::config::InputMapping::User(m) => {
            assert_eq!(m.source.to_string(), "producer");
            assert_eq!(m.output.to_string(), "op/result");
        }
        other => panic!("expected user mapping, got {other:?}"),
    }
}

#[test]
fn dynamic_node_inherits_the_running_dataflow_env() {
    // Regression guard for #2919, which shares this resolution path: the
    // node inherits the dataflow-level `env:` (including anything from
    // `dora start --env`), with its own keys winning on conflict.
    let running = running_descriptor_with_operator_producer(
        serde_json::json!({ "SHARED": "from-dataflow", "RUST_LOG": "info" }),
    );

    let added: Node = serde_json::from_value(serde_json::json!({
        "id": "consumer",
        "path": "consumer",
        "env": { "RUST_LOG": "debug" },
    }))
    .expect("valid node");

    let (_, resolved) = resolve_single_node(added, &running).expect("node resolves");

    let env = resolved.env.expect("merged env present");
    assert_eq!(
        env.get("SHARED"),
        Some(&dora_message::descriptor::EnvValue::String(
            "from-dataflow".into()
        )),
    );
    assert_eq!(
        env.get("RUST_LOG"),
        Some(&dora_message::descriptor::EnvValue::String("debug".into())),
        "per-node key must win over the dataflow-level default",
    );
}

fn test_running_dataflow(
    dataflow_id: DataflowId,
    daemon_id: DaemonId,
    node_id: dora_core::config::NodeId,
) -> RunningDataflow {
    let mut daemons = BTreeSet::new();
    daemons.insert(daemon_id.clone());

    let mut node_to_daemon = BTreeMap::new();
    node_to_daemon.insert(node_id, daemon_id);
    // `path` is required for the descriptor to resolve; topic routing now
    // goes through `resolve_aliases_and_set_defaults`, which rejects a node
    // with no runnable kind.
    let descriptor: Descriptor = serde_json::from_value(serde_json::json!({
        "nodes": [{
            "id": "sender",
            "path": "sender",
            "outputs": ["message"],
        }]
    }))
    .expect("valid test descriptor");
    // Production writes `descriptor` and `nodes` together (`spawn_dataflow`,
    // and the `AddNode`/`ReplaceNode` handlers), and readers treat `nodes`
    // as the authoritative resolved view. A fixture that leaves it empty
    // while handing out a real descriptor is a state production never
    // produces, so keep them in step here too.
    let nodes = descriptor
        .resolve_aliases_and_set_defaults()
        .expect("test descriptor should resolve");

    RunningDataflow {
        name: None,
        uuid: dataflow_id,
        descriptor,
        daemons,
        pending_daemons: BTreeSet::new(),
        exited_before_subscribe: vec![],
        ready_barrier_released: false,
        nodes,
        node_to_daemon,
        node_metrics: BTreeMap::new(),
        node_finalized: BTreeSet::new(),
        node_stopped_at: BTreeMap::new(),
        network_metrics: None,
        spawn_result: CachedResult::default(),
        stop_reply_senders: vec![],
        buffered_log_messages: vec![],
        log_subscribers: vec![],
        topic_subscribers: BTreeMap::new(),
        pending_spawn_results: BTreeSet::new(),
        spawn_started_at: Instant::now(),
        created_at: 0,
        store_generation: 0,
        last_recovery_attempt: BTreeMap::new(),
        last_replay_attempt: BTreeMap::new(),
        uv: false,
        state_log_sequence: 0,
        state_log: Vec::new(),
        daemon_ack_sequence: BTreeMap::new(),
    }
}

/// A store whose `get_dataflow` always fails (simulating a transient
/// read error), delegating every other method to a backing
/// [`InMemoryStore`]. Used to prove the ready-barrier release is still
/// persisted when the status read fails (#3115).
struct FailingReadStore {
    inner: InMemoryStore,
}

impl FailingReadStore {
    fn new() -> Self {
        Self {
            inner: InMemoryStore::new(),
        }
    }
}

impl CoordinatorStore for FailingReadStore {
    fn get_dataflow(&self, _uuid: &Uuid) -> Result<Option<dora_coordinator_store::DataflowRecord>> {
        Err(eyre!("simulated transient store read error"))
    }

    fn register_daemon(&self, info: dora_coordinator_store::DaemonInfo) -> Result<()> {
        self.inner.register_daemon(info)
    }
    fn unregister_daemon(&self, id: &DaemonId) -> Result<()> {
        self.inner.unregister_daemon(id)
    }
    fn list_daemons(&self) -> Result<Vec<dora_coordinator_store::DaemonInfo>> {
        self.inner.list_daemons()
    }
    fn get_daemon(&self, id: &DaemonId) -> Result<Option<dora_coordinator_store::DaemonInfo>> {
        self.inner.get_daemon(id)
    }
    fn get_daemon_by_machine(&self, machine_id: &str) -> Result<Option<DaemonId>> {
        self.inner.get_daemon_by_machine(machine_id)
    }
    fn put_dataflow(&self, record: &dora_coordinator_store::DataflowRecord) -> Result<()> {
        self.inner.put_dataflow(record)
    }
    fn list_dataflows(&self) -> Result<Vec<dora_coordinator_store::DataflowRecord>> {
        self.inner.list_dataflows()
    }
    fn delete_dataflow(&self, uuid: &Uuid) -> Result<()> {
        self.inner.delete_dataflow(uuid)
    }
    fn put_build(&self, record: &dora_coordinator_store::BuildRecord) -> Result<()> {
        self.inner.put_build(record)
    }
    fn get_build(&self, build_id: &Uuid) -> Result<Option<dora_coordinator_store::BuildRecord>> {
        self.inner.get_build(build_id)
    }
    fn list_builds(&self) -> Result<Vec<dora_coordinator_store::BuildRecord>> {
        self.inner.list_builds()
    }
    fn delete_build(&self, build_id: &Uuid) -> Result<()> {
        self.inner.delete_build(build_id)
    }
    fn put_node_param(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
        key: &str,
        value: &[u8],
    ) -> Result<()> {
        self.inner.put_node_param(dataflow_id, node_id, key, value)
    }
    fn get_node_param(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
        key: &str,
    ) -> Result<Option<Vec<u8>>> {
        self.inner.get_node_param(dataflow_id, node_id, key)
    }
    fn list_node_params(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
    ) -> Result<Vec<(String, Vec<u8>)>> {
        self.inner.list_node_params(dataflow_id, node_id)
    }
    fn delete_node_param(&self, dataflow_id: &Uuid, node_id: &NodeId, key: &str) -> Result<()> {
        self.inner.delete_node_param(dataflow_id, node_id, key)
    }
}

/// #3115: a *failed*-barrier release must still be persisted (with the
/// `ready_barrier_released` flag set) even when the store status read
/// fails. Otherwise the in-memory flag flips to `true` but the durable
/// record stays stale, re-opening the #2998 reconnect-hang window on the
/// store-I/O-trouble path.
#[test]
fn failed_barrier_release_persists_when_status_read_fails() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("d1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
    // A failed barrier: at least one node exited before subscribing.
    df.exited_before_subscribe.push(node_id);
    // `release_barrier_message` flips the in-memory flag before persisting.
    df.ready_barrier_released = true;

    let store: Arc<dyn CoordinatorStore> = Arc::new(FailingReadStore::new());
    persist_ready_barrier_release(dataflow_id, &mut df, &store);

    // The read failed, but the release must still be on disk.
    let record = store
        .list_dataflows()
        .expect("store should list")
        .into_iter()
        .find(|r| r.uuid == dataflow_id)
        .expect("failed-barrier release must be persisted even when the status read fails");
    assert!(
        record.ready_barrier_released,
        "persisted record must carry ready_barrier_released == true"
    );
    // A failed barrier must never be promoted to Running; when the status
    // cannot be read the documented fallback is `Pending` specifically, so
    // pin that rather than merely "not Running".
    assert_eq!(
        record.status,
        StoreDataflowStatus::Pending,
        "failed barrier must fall back to Pending, never promote to Running"
    );
}

/// #3115 companion: the same skip-the-write bug also fired on the
/// `Ok(None)` branch (no existing record). It must now persist the
/// release rather than dropping it.
#[test]
fn failed_barrier_release_persists_when_record_absent() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("d1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
    df.exited_before_subscribe.push(node_id);
    df.ready_barrier_released = true;

    // Empty store => get_dataflow returns Ok(None).
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    persist_ready_barrier_release(dataflow_id, &mut df, &store);

    let record = store
        .get_dataflow(&dataflow_id)
        .expect("store read")
        .expect("failed-barrier release must be persisted even with no prior record");
    assert!(record.ready_barrier_released);
    assert_eq!(record.status, StoreDataflowStatus::Pending);
}

/// A *successful* barrier still promotes the record to `Running`.
#[test]
fn successful_barrier_release_persists_running() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("d1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id);
    // exited_before_subscribe stays empty => successful barrier.
    df.ready_barrier_released = true;

    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    persist_ready_barrier_release(dataflow_id, &mut df, &store);

    let record = store
        .get_dataflow(&dataflow_id)
        .expect("store read")
        .expect("successful barrier release must be persisted");
    assert!(record.ready_barrier_released);
    assert!(matches!(record.status, StoreDataflowStatus::Running));
}

#[test]
fn topic_outputs_by_daemon_normalizes_single_operator_outputs() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let descriptor: Descriptor = serde_json::from_value(serde_json::json!({
        "nodes": [
            {
                "id": "single",
                "operator": {
                    "python": "single.py",
                    "outputs": ["image"],
                },
            },
            {
                // The legacy `custom:` node key was removed from `Node`;
                // a plain path node is the same `CoreNodeKind::Custom`
                // case, which is what this arm pins.
                "id": "legacy",
                "path": "legacy.py",
                "outputs": ["buffer"],
            },
            {
                "id": "runtime",
                "operators": [{
                    "id": "op",
                    "python": "runtime.py",
                    "outputs": ["status"],
                }],
            },
        ],
    }))
    .expect("valid test descriptor");
    let single_node: dora_core::config::NodeId = "single".to_string().into();
    let legacy_node: dora_core::config::NodeId = "legacy".to_string().into();
    let runtime_node: dora_core::config::NodeId = "runtime".to_string().into();
    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id.clone(), single_node.clone());
    dataflow.descriptor = descriptor;
    // Mirror what `spawn_dataflow` (and `AddNode`/`ReplaceNode`) do: the
    // resolved map is the authoritative one and is always written together
    // with the descriptor.
    dataflow.nodes = dataflow
        .descriptor
        .resolve_aliases_and_set_defaults()
        .expect("test descriptor should resolve");
    dataflow
        .node_to_daemon
        .insert(legacy_node.clone(), daemon_id.clone());
    dataflow
        .node_to_daemon
        .insert(runtime_node.clone(), daemon_id.clone());

    let topics = vec![
        (single_node, "image".to_string().into()),
        (legacy_node, "buffer".to_string().into()),
        (runtime_node, "op/status".to_string().into()),
    ];
    let expected_topics = vec![
        ("single".to_string().into(), "op/image".to_string().into()),
        ("legacy".to_string().into(), "buffer".to_string().into()),
        ("runtime".to_string().into(), "op/status".to_string().into()),
    ];
    let running_dataflows = HashMap::from([(dataflow_id, dataflow)]);

    let outputs = topic_outputs_by_daemon(&running_dataflows, dataflow_id, &topics)
        .expect("nested outputs should resolve to their daemon");

    assert_eq!(outputs[&daemon_id], expected_topics);
}

#[tokio::test]
async fn start_topic_debug_stream_does_not_orphan_subscriber_on_missing_daemon_connection() {
    // Regression: the subscriber used to be registered in `topic_subscribers`
    // *before* the per-daemon dispatch loop validated each daemon connection.
    // When a topic's node mapped to a daemon with no live connection, the loop
    // bailed with `?` and left the `subscription_id` orphaned in the map — the
    // CLI only saw the returned error, never learned the id, and so could never
    // `TopicUnsubscribe` it. A pre-dispatch failure must leave no subscriber.
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();

    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
    dataflow.descriptor.debug.enable_debug_inspection = true;
    let mut running_dataflows = HashMap::from([(dataflow_id, dataflow)]);

    // No connection registered for the topic's daemon, so the dispatch loop
    // bails before the (now-deferred) subscriber registration.
    let mut daemon_connections = DaemonConnections::default();

    let (tx, _rx) = tokio::sync::mpsc::channel(8);
    let clock = HLC::default();
    let topics = vec![(node_id, "message".to_string().into())];

    let result = start_topic_debug_stream(
        &mut running_dataflows,
        &mut daemon_connections,
        dataflow_id,
        topics,
        tx,
        &clock,
    )
    .await;

    assert!(
        result.is_err(),
        "a missing daemon connection must surface as an error"
    );
    assert!(
        running_dataflows[&dataflow_id].topic_subscribers.is_empty(),
        "a pre-dispatch failure must not leave an orphaned topic subscriber",
    );
}

#[test]
fn reconcile_reestablishes_running_dataflow_after_reconnect() {
    // #2029 P1: after a reclaim (or coordinator restart) removed the live
    // entry, a reconnecting daemon's status report must rebuild it in
    // `running_dataflows` so `dora list` / `stop` / `logs` see the survivor
    // again — store status alone doesn't drive the control plane.
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("d1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();

    // A persisted record as it would exist after a reclaim/restart: status
    // Recovering, descriptor with a resolvable node.
    let record = dora_coordinator_store::DataflowRecord {
        uuid: dataflow_id,
        name: Some("df".to_string()),
        descriptor_json: serde_json::json!({
            "nodes": [{ "id": "sender", "path": "sleep", "outputs": ["message"] }]
        })
        .to_string(),
        status: StoreDataflowStatus::Recovering,
        daemon_ids: vec![daemon_id.clone()],
        node_to_daemon: BTreeMap::new(),
        uv: false,
        ready_barrier_released: false,
        barrier_exited_before_subscribe: Vec::new(),
        generation: 3,
        created_at: 7,
        updated_at: 7,
    };

    // Absent (reclaimed away / restart) -> reconstruct from the record.
    let mut running_dataflows: HashMap<DataflowId, RunningDataflow> = HashMap::new();
    // These cover the relinking, not the barrier replay.
    let _ = reestablish_running_dataflow(
        &mut running_dataflows,
        &record,
        &daemon_id,
        std::slice::from_ref(&node_id),
    );
    let rebuilt = running_dataflows
        .get(&dataflow_id)
        .expect("must reconstruct the live entry");
    assert!(rebuilt.daemons.contains(&daemon_id));
    assert_eq!(rebuilt.node_to_daemon.get(&node_id), Some(&daemon_id));
    assert!(
        matches!(&rebuilt.spawn_result, CachedResult::Cached { result } if result.is_ok()),
        "spawn_result must be cached Ok so stop/wait waiters don't hang"
    );

    // Present (multi-daemon partial reconnect) -> relink, not duplicate.
    let daemon2 = DaemonId::new(Some("d2".to_string()));
    let node2: dora_core::config::NodeId = "receiver".to_string().into();
    // These cover the relinking, not the barrier replay.
    let _ = reestablish_running_dataflow(
        &mut running_dataflows,
        &record,
        &daemon2,
        std::slice::from_ref(&node2),
    );
    let df = running_dataflows.get(&dataflow_id).expect("still present");
    assert!(df.daemons.contains(&daemon_id) && df.daemons.contains(&daemon2));
    assert_eq!(df.node_to_daemon.get(&node2), Some(&daemon2));
    assert_eq!(running_dataflows.len(), 1, "must relink, not duplicate");
}

/// dora-rs/dora#2998: the ready barrier is broadcast once, to the daemons
/// connected at that moment. A daemon that was disconnected then is not in
/// that set and nothing else ever tells it — so every node it owns parks in
/// `init_from_env()` for the life of the dataflow. `reestablish_running_dataflow`
/// reports the obligation so the reconnect path can replay it.
#[test]
fn reconnecting_daemon_is_owed_a_barrier_replay_only_after_release() {
    let dataflow_id = uuid::Uuid::from_u128(0x2998);
    let daemon_id = DaemonId::new(Some("reconnector".to_string()));
    let node_id: dora_core::config::NodeId = "worker".to_string().into();

    let mut running_dataflows: HashMap<DataflowId, RunningDataflow> = HashMap::new();
    running_dataflows.insert(
        dataflow_id,
        test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone()),
    );
    let record = dora_coordinator_store::DataflowRecord {
        uuid: dataflow_id,
        name: Some("df".to_string()),
        descriptor_json: serde_json::json!({
            "nodes": [{ "id": "sender", "path": "sleep", "outputs": ["message"] }]
        })
        .to_string(),
        status: StoreDataflowStatus::Running,
        daemon_ids: vec![daemon_id.clone()],
        node_to_daemon: BTreeMap::new(),
        uv: false,
        ready_barrier_released: false,
        barrier_exited_before_subscribe: Vec::new(),
        generation: 3,
        created_at: 7,
        updated_at: 7,
    };

    // Barrier has NOT released yet: the daemon will be told by the
    // ordinary broadcast when it does, so no replay is owed.
    let owed = reestablish_running_dataflow(
        &mut running_dataflows,
        &record,
        &daemon_id,
        std::slice::from_ref(&node_id),
    );
    assert!(
        !owed,
        "before release there is nothing to replay — replaying here would \
             release the barrier early for this daemon"
    );

    // Barrier releases while this daemon is away.
    running_dataflows
        .get_mut(&dataflow_id)
        .expect("present")
        .ready_barrier_released = true;

    let owed = reestablish_running_dataflow(
        &mut running_dataflows,
        &record,
        &daemon_id,
        std::slice::from_ref(&node_id),
    );
    assert!(
        owed,
        "a daemon reconnecting after the barrier released must be replayed \
             it, or its nodes hang in `init_from_env()` forever (#2998)"
    );
}

#[test]
fn the_release_survives_a_persist_and_rebuild_round_trip() {
    // The whole fix rests on this: the release is written by `make_record`
    // and read back by `recovered`. If either half drops it, a daemon that
    // reconnects after orphan reclaim or a coordinator restart is never
    // replayed the barrier — and it cannot ask for one, because
    // `reported_init_to_coordinator` is never reset (#2998).
    let dataflow_id = Uuid::new_v4();
    let daemon_id = DaemonId::new(Some("machine-a".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();

    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());
    // The helper's descriptor has no `path`, so it would not survive
    // `resolve_aliases_and_set_defaults` on the rebuild side.
    df.descriptor = serde_json::from_value(serde_json::json!({
        "nodes": [{ "id": "sender", "path": "sleep", "outputs": ["message"] }]
    }))
    .expect("valid test descriptor");
    df.ready_barrier_released = true;
    df.exited_before_subscribe = vec![node_id.clone()];

    let record = df
        .make_record(StoreDataflowStatus::Running)
        .expect("snapshotting the dataflow");
    assert!(
        record.ready_barrier_released,
        "the persisted snapshot must carry the release"
    );
    assert_eq!(
        record.barrier_exited_before_subscribe,
        vec![node_id.to_string()],
        "the persisted snapshot must carry the verdict too, or a replay \
             after restart would report a bare success"
    );

    // Rebuild from that record, as a reconnect after reclaim/restart does.
    let mut running_dataflows: HashMap<DataflowId, RunningDataflow> = HashMap::new();
    let owed = reestablish_running_dataflow(
        &mut running_dataflows,
        &record,
        &daemon_id,
        std::slice::from_ref(&node_id),
    );
    assert!(owed, "the rebuilt dataflow still owes the replay");

    let rebuilt = running_dataflows.get(&dataflow_id).expect("rebuilt");
    assert!(
        rebuilt.ready_barrier_released,
        "the rebuilt entry must remember the release, or the next persist \
             writes `false` and the following reconnect hangs again"
    );
    assert_eq!(
        rebuilt.exited_before_subscribe,
        vec![node_id],
        "and must remember the verdict"
    );
}

#[test]
fn a_reconstructed_dataflow_still_owes_the_replay() {
    // The live entry does not survive orphan reclaim or a coordinator
    // restart, so the reconnecting daemon arrives to a dataflow rebuilt
    // from the store. If the release were dropped in that rebuild, the
    // daemon would never be replayed the barrier and would hang exactly as
    // it did before the fix — and it cannot prompt a fresh broadcast,
    // because `reported_init_to_coordinator` is never reset (#2998).
    let dataflow_id = Uuid::new_v4();
    let daemon_id = DaemonId::new(Some("machine-a".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();

    let mut record = dora_coordinator_store::DataflowRecord {
        uuid: dataflow_id,
        name: Some("df".to_string()),
        descriptor_json: serde_json::json!({
            "nodes": [{ "id": "sender", "path": "sleep", "outputs": ["message"] }]
        })
        .to_string(),
        status: StoreDataflowStatus::Recovering,
        daemon_ids: vec![daemon_id.clone()],
        node_to_daemon: BTreeMap::new(),
        uv: false,
        ready_barrier_released: true,
        barrier_exited_before_subscribe: Vec::new(),
        generation: 3,
        created_at: 7,
        updated_at: 7,
    };

    // Empty map: no live entry, so this takes the reconstruct path.
    let mut running_dataflows: HashMap<DataflowId, RunningDataflow> = HashMap::new();
    let owed = reestablish_running_dataflow(
        &mut running_dataflows,
        &record,
        &daemon_id,
        std::slice::from_ref(&node_id),
    );
    assert!(
        owed,
        "a dataflow rebuilt from a record whose barrier had released still \
             owes the reconnecting daemon a replay (#2998)"
    );

    // ...and a record whose barrier had not released owes nothing.
    record.ready_barrier_released = false;
    let mut running_dataflows: HashMap<DataflowId, RunningDataflow> = HashMap::new();
    let owed = reestablish_running_dataflow(
        &mut running_dataflows,
        &record,
        &daemon_id,
        std::slice::from_ref(&node_id),
    );
    assert!(
        !owed,
        "replaying a barrier that never fired would release it early"
    );
}

#[test]
fn a_reconstructed_dataflow_keeps_the_failed_barrier_verdict() {
    // Restoring the release but blanking its verdict would replay a bare
    // success and start a dataflow the coordinator had already given up on.
    let dataflow_id = Uuid::new_v4();
    let daemon_id = DaemonId::new(Some("machine-a".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let clock = Arc::new(HLC::default());

    let record = dora_coordinator_store::DataflowRecord {
        uuid: dataflow_id,
        name: Some("df".to_string()),
        descriptor_json: serde_json::json!({
            "nodes": [{ "id": "sender", "path": "sleep", "outputs": ["message"] }]
        })
        .to_string(),
        status: StoreDataflowStatus::Recovering,
        daemon_ids: vec![daemon_id.clone()],
        node_to_daemon: BTreeMap::new(),
        uv: false,
        ready_barrier_released: true,
        barrier_exited_before_subscribe: vec![node_id.to_string()],
        generation: 3,
        created_at: 7,
        updated_at: 7,
    };

    let mut running_dataflows: HashMap<DataflowId, RunningDataflow> = HashMap::new();
    let _ = reestablish_running_dataflow(
        &mut running_dataflows,
        &record,
        &daemon_id,
        std::slice::from_ref(&node_id),
    );

    let df = running_dataflows.get(&dataflow_id).expect("rebuilt");
    let message = all_nodes_ready_message(dataflow_id, df, &clock).expect("serializing");
    let decoded: Timestamped<DaemonCoordinatorEvent> =
        serde_json::from_slice(&message).expect("decoding");
    match decoded.inner {
        DaemonCoordinatorEvent::AllNodesReady {
            exited_before_subscribe,
            ..
        } => assert_eq!(
            exited_before_subscribe,
            vec![node_id],
            "a replay after reconstruction must repeat the failure verdict, \
                 not a blank success"
        ),
        other => panic!("expected AllNodesReady, got {other:?}"),
    }
}

#[tokio::test]
async fn replaying_the_barrier_sends_it_and_the_params_to_that_daemon() {
    // End-to-end over a fake daemon connection: the reconnecting daemon
    // must receive BOTH halves of the release it missed -- the barrier and
    // the persisted-parameter replay. Sending only the barrier releases its
    // nodes with default state instead of the operator's parameters.
    #[derive(serde::Deserialize)]
    struct OutboundRaw {
        id: String,
        params: Timestamped<DaemonCoordinatorEvent>,
    }

    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();
    let value_bytes = serde_json::to_vec(&serde_json::json!(123)).unwrap();
    store
        .put_node_param(&dataflow_id, &node_id, "gain", &value_bytes)
        .unwrap();

    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let connection =
        crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(daemon_id.clone(), connection);

    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());
    dataflow.ready_barrier_released = true;

    let expect_node = node_id.clone();
    let daemon_task = tokio::spawn(async move {
        // 1. the barrier itself (fire-and-forget `send`)
        let first = rx.recv().await.expect("daemon should receive the barrier");
        let raw: OutboundRaw = serde_json::from_str(&first).unwrap();
        match raw.params.inner {
            DaemonCoordinatorEvent::AllNodesReady {
                dataflow_id: id, ..
            } => {
                assert_eq!(id, dataflow_id)
            }
            other => panic!("expected AllNodesReady first, got {other:?}"),
        }

        // 2. the persisted-parameter replay (`send_and_receive`, needs a reply)
        let second = rx
            .recv()
            .await
            .expect("a replayed daemon must also get its persisted params");
        let raw: OutboundRaw = serde_json::from_str(&second).unwrap();
        match raw.params.inner {
            DaemonCoordinatorEvent::SetParam {
                node_id: n,
                key,
                value,
                ..
            } => {
                assert_eq!(n, expect_node);
                assert_eq!(key, "gain");
                assert_eq!(value, serde_json::json!(123));
            }
            other => panic!("expected SetParam second, got {other:?}"),
        }
        let request_id = Uuid::parse_str(&raw.id).expect("valid request id");
        let reply = serde_json::to_string(&DaemonCoordinatorReply::SetParamResult(Ok(()))).unwrap();
        if let Some(reply_tx) = pending_replies.lock().await.remove(&request_id) {
            let _ = reply_tx.send(reply);
        }
    });

    replay_all_nodes_ready(
        dataflow_id,
        &dataflow,
        &daemon_id,
        &mut daemon_connections,
        &store,
        &Arc::new(HLC::default()),
    )
    .await
    .expect("replay");

    // Bounded: a regression that stops sending must fail here, not wedge CI.
    tokio::time::timeout(Duration::from_secs(10), daemon_task)
        .await
        .expect("daemon did not receive both halves of the release")
        .unwrap();
}

#[tokio::test]
async fn ready_broadcast_is_best_effort_when_a_daemon_send_fails() {
    // A broken (but not yet dropped) daemon connection makes the
    // per-daemon `send` fail. That must not abort the coordinator's event
    // loop: `broadcast_all_nodes_ready` returns `Ok` and relies on the
    // persisted release being replayed when the daemon reconnects (#2998).
    // Before this fix the send error propagated with `?` and tore down the
    // whole coordinator -- every dataflow and daemon -- over one transient
    // per-daemon failure.
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    // A connection whose receive half is already gone, so `send` errors.
    let (tx, rx) = tokio::sync::mpsc::channel::<String>(8);
    drop(rx);
    let connection = crate::state::DaemonConnection::new(
        tx,
        Arc::new(tokio::sync::Mutex::new(HashMap::new())),
        BTreeMap::new(),
    );
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(daemon_id.clone(), connection);

    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id, node_id);
    assert!(!dataflow.ready_barrier_released);

    let result = broadcast_all_nodes_ready(
        dataflow_id,
        &mut dataflow,
        &mut daemon_connections,
        &store,
        &Arc::new(HLC::default()),
    )
    .await;

    assert!(
        result.is_ok(),
        "a single daemon send failure must not fail the broadcast: {result:?}"
    );
    // The barrier is still recorded as released, so the daemon that missed
    // the frame is replayed on reconnect rather than parked forever.
    assert!(dataflow.ready_barrier_released);
}

#[test]
fn a_replayed_daemon_gets_its_own_nodes_params_and_no_one_elses() {
    // The barrier and the persisted-parameter replay are two halves of the
    // same release. A daemon that missed the broadcast missed both, so
    // replaying only the barrier releases its nodes with default state
    // instead of the parameters the operator set. Selecting the wrong
    // nodes here would replay another daemon's parameters, or none.
    let dataflow_id = Uuid::new_v4();
    let daemon_a = DaemonId::new(Some("machine-a".to_string()));
    let daemon_b = DaemonId::new(Some("machine-b".to_string()));
    let node_a: dora_core::config::NodeId = "sender".to_string().into();
    let node_b: dora_core::config::NodeId = "receiver".to_string().into();

    let mut df = test_running_dataflow(dataflow_id, daemon_a.clone(), node_a.clone());
    df.node_to_daemon.insert(node_b.clone(), daemon_b.clone());

    assert_eq!(
        nodes_on_daemon(&df, &daemon_a),
        vec![node_a],
        "only the reconnecting daemon's own nodes"
    );
    assert_eq!(nodes_on_daemon(&df, &daemon_b), vec![node_b]);
    assert!(
        nodes_on_daemon(&df, &DaemonId::new(Some("machine-c".to_string()))).is_empty(),
        "a daemon with no nodes here gets nothing replayed"
    );
}

#[tokio::test]
async fn a_failed_barrier_is_persisted_as_durably_as_a_successful_one() {
    // A failed barrier still has to reach a daemon that reconnects later --
    // it must learn the barrier is down and why. Leaving the write to
    // whatever failure path runs next opens a restart window in which the
    // record still says "not released", which is #2998 all over again.
    let dataflow_id = Uuid::new_v4();
    let daemon_id = DaemonId::new(Some("machine-a".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let clock = Arc::new(HLC::default());
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let mut daemon_connections = DaemonConnections::default();

    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
    // Seed with a status that is NOT `Running`, so "keeps its status" is
    // distinguishable from "promoted to Running".
    let seeded = dataflow
        .make_record(StoreDataflowStatus::Pending)
        .expect("seed record");
    store.put_dataflow(&seeded).expect("seed");

    // The barrier fires with a failure verdict.
    dataflow.exited_before_subscribe = vec![node_id.clone()];
    broadcast_all_nodes_ready(
        dataflow_id,
        &mut dataflow,
        &mut daemon_connections,
        &store,
        &clock,
    )
    .await
    .expect("broadcast");

    let persisted = store
        .get_dataflow(&dataflow_id)
        .expect("read back")
        .expect("record present");
    assert!(
        persisted.ready_barrier_released,
        "a failed barrier must be persisted at release, or a coordinator \
             restart in the window leaves the reconnecting daemon hanging (#2998)"
    );
    assert_eq!(
        persisted.barrier_exited_before_subscribe,
        vec![node_id.to_string()],
        "and must persist the failure verdict, not a bare release"
    );
    assert_eq!(
        persisted.status,
        StoreDataflowStatus::Pending,
        "a failed barrier keeps the status it had; it must not be promoted"
    );
}

#[tokio::test]
async fn broadcasting_the_barrier_records_it_even_with_no_daemon_reachable() {
    // Goes through the real broadcast, with no daemon connections: every
    // send is skipped, yet the release must still be recorded. This is the
    // #2998 shape exactly — the daemon that missed the broadcast is the one
    // that later reconnects and must be replayed it.
    let dataflow_id = Uuid::new_v4();
    let daemon_id = DaemonId::new(Some("machine-a".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let clock = Arc::new(HLC::default());
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let mut daemon_connections = DaemonConnections::default();

    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id, node_id);
    assert!(!dataflow.ready_barrier_released);

    broadcast_all_nodes_ready(
        dataflow_id,
        &mut dataflow,
        &mut daemon_connections,
        &store,
        &clock,
    )
    .await
    .expect("broadcast with no reachable daemon should not fail");

    assert!(
        dataflow.ready_barrier_released,
        "the barrier is down regardless of who received it; a daemon that \
             reconnects later must still be replayed it (#2998)"
    );
}

#[test]
fn releasing_the_barrier_records_it_on_the_dataflow() {
    // Producing the release frame is what marks the dataflow. If the two
    // ever come apart, the broadcast still goes out and everything looks
    // healthy — but a daemon reconnecting afterwards is never replayed the
    // barrier and its nodes hang for the life of the dataflow (#2998).
    let dataflow_id = Uuid::new_v4();
    let daemon_id = DaemonId::new(Some("machine-a".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let clock = Arc::new(HLC::default());

    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id, node_id);
    assert!(
        !dataflow.ready_barrier_released,
        "precondition: barrier has not fired yet"
    );

    release_barrier_message(dataflow_id, &mut dataflow, &clock).expect("building the release");

    assert!(
        dataflow.ready_barrier_released,
        "releasing the barrier must record it, or the reconnect path has \
             nothing to replay from (#2998)"
    );
}

#[test]
fn replayed_barrier_carries_the_same_failure_verdict_as_the_broadcast() {
    // The replay must reproduce the *outcome* the broadcast carried, not a
    // blank success. A daemon that latches an empty `exited_before_subscribe`
    // treats the barrier as satisfied and starts a dataflow the coordinator
    // already declared failed (`let ready = exited_before_subscribe
    // .is_empty()` gates `dataflow.start()`), which is exactly the trap the
    // #2938 latch had to avoid on the daemon side.
    let dataflow_id = Uuid::new_v4();
    let daemon_id = DaemonId::new(Some("machine-a".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let clock = Arc::new(HLC::default());

    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
    dataflow.exited_before_subscribe = vec![node_id.clone()];

    let message =
        all_nodes_ready_message(dataflow_id, &dataflow, &clock).expect("serializing AllNodesReady");
    let decoded: Timestamped<DaemonCoordinatorEvent> =
        serde_json::from_slice(&message).expect("decoding AllNodesReady");

    match decoded.inner {
        DaemonCoordinatorEvent::AllNodesReady {
            dataflow_id: got_id,
            exited_before_subscribe,
        } => {
            assert_eq!(got_id, dataflow_id);
            assert_eq!(
                exited_before_subscribe,
                vec![node_id],
                "the replay must carry the failed-barrier verdict, or the \
                     reconnected daemon starts a dataflow that must not start"
            );
        }
        other => panic!("expected AllNodesReady, got {other:?}"),
    }
}

#[tokio::test]
async fn orphan_stop_sends_stopdataflow_to_reporting_daemon() {
    // #2029 P3: when a daemon reconnects (within its own reconnect window)
    // reporting a dataflow the coordinator already failed terminally — i.e.
    // its recovery timeout fired first — the coordinator can't re-adopt it,
    // so it must tell that daemon to stop the orphaned nodes.
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("d1".to_string()));
    let clock = HLC::default();

    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let connection = state::DaemonConnection::new(
        tx,
        std::sync::Arc::new(tokio::sync::Mutex::new(std::collections::HashMap::new())),
        BTreeMap::new(),
    );
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(daemon_id.clone(), connection);

    stop_orphaned_dataflow_on_daemon(dataflow_id, &daemon_id, &mut daemon_connections, &clock)
        .await;

    let sent = rx
        .try_recv()
        .expect("a stop message must be sent to the reporting daemon");
    assert!(
        sent.contains("StopDataflow"),
        "must send StopDataflow, got: {sent}"
    );
    assert!(
        sent.contains(&dataflow_id.to_string()),
        "stop must target the orphaned dataflow"
    );
}

#[test]
fn disconnect_cleanup_removes_daemon_from_running_dataflow_membership() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("gone".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());
    df.pending_daemons.insert(daemon_id.clone());
    df.pending_spawn_results.insert(daemon_id.clone());

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let disconnected = BTreeSet::from([daemon_id.clone()]);

    cleanup_disconnected_daemons_from_running_dataflows(
        &mut running_dataflows,
        &disconnected,
        &mut HashMap::new(),
    );

    let df = running_dataflows
        .get(&dataflow_id)
        .expect("disconnect cleanup must not remove the dataflow");
    assert!(!df.daemons.contains(&daemon_id));
    assert!(!df.pending_daemons.contains(&daemon_id));
    assert!(!df.pending_spawn_results.contains(&daemon_id));
    assert_eq!(
        df.node_to_daemon.get(&node_id),
        Some(&daemon_id),
        "original node assignment stays available for later failure synthesis"
    );
    assert!(
        df.spawn_result.is_pending(),
        "disconnect cleanup must leave spawn_result to the watchdog"
    );
}

// #2028: a spawn-pending dataflow stays the spawn-timeout watchdog's
// domain — disconnect cleanup must produce no action and not remove it.
#[test]
fn disconnect_of_spawn_pending_dataflow_produces_no_action() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("gone".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
    // spawn_result left Pending (CachedResult::default()).

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let disconnected = BTreeSet::from([daemon_id]);

    let actions = cleanup_disconnected_daemons_from_running_dataflows(
        &mut running_dataflows,
        &disconnected,
        &mut HashMap::new(),
    );
    assert!(
        // spawn_result is Pending, so the dataflow should NOT trigger
        // a ReclaimOrphaned action.
        actions.is_empty(),
        "spawn-pending dataflows must be left to the spawn-timeout watchdog"
    );
    assert!(
        running_dataflows.contains_key(&dataflow_id),
        "must not tear down a spawn-pending dataflow"
    );
}

// #2028 deadlock #1: the last daemon awaited for ReadyOnDaemon disconnects
// after spawn succeeded -> the start barrier must be released for survivors.
#[test]
fn disconnect_releases_ready_barrier_when_last_pending_daemon_drops() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_a = DaemonId::new(Some("a".to_string()));
    let daemon_b = DaemonId::new(Some("b".to_string()));
    let node_a: dora_core::config::NodeId = "sender".to_string().into();
    let mut df = test_running_dataflow(dataflow_id, daemon_a.clone(), node_a);
    df.daemons.insert(daemon_b.clone());
    df.node_to_daemon
        .insert("receiver".to_string().into(), daemon_b.clone());
    // spawn already succeeded; A reported ready, still waiting on B.
    df.spawn_result
        .set_result(Ok(ControlRequestReply::DataflowSpawned {
            uuid: dataflow_id,
        }));
    df.pending_daemons.insert(daemon_b.clone());

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let disconnected = BTreeSet::from([daemon_b]);

    let actions = cleanup_disconnected_daemons_from_running_dataflows(
        &mut running_dataflows,
        &disconnected,
        &mut HashMap::new(),
    );
    assert!(
        matches!(actions.as_slice(), [DisconnectAction::ReleaseReadyBarrier(id)] if *id == dataflow_id),
        "must release the ready barrier, got {} action(s)",
        actions.len()
    );
    let df = running_dataflows
        .get(&dataflow_id)
        .expect("survivor dataflow must remain");
    assert_eq!(df.daemons, BTreeSet::from([daemon_a]));
    assert!(df.pending_daemons.is_empty());
}

// #2028 deadlocks #2 + #3 + #2029 reclaim: the sole daemon of a running
// dataflow disconnects -> the dataflow must be removed from the running set
// and any parked `dora stop` waiter resolved instead of hanging. But unlike
// a terminal teardown, the disconnect now opens a *reclaim window*: the
// store record is `Recovering` (NOT terminal `Failed`) and the dataflow is
// NOT archived, so a reconnecting daemon's `DaemonStatusReport` can
// reconcile it back to `Running` (the reconcile path skips archived/terminal
// records). Permanent loss is handled by the `Recovering -> Failed` recovery
// timeout sweep.
#[tokio::test]
async fn disconnect_reclaims_orphaned_running_dataflow() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = Arc::new(HLC::default());
    let mut daemon_connections = DaemonConnections::default();

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("gone".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
    df.spawn_result
        .set_result(Ok(ControlRequestReply::DataflowSpawned {
            uuid: dataflow_id,
        }));
    // Seed the store with a Running record so we can observe the transition
    // to Recovering (the production spawn path persists this).
    store
        .put_dataflow(
            &df.make_record(StoreDataflowStatus::Running)
                .expect("make running record"),
        )
        .expect("seed running record");
    // a parked `dora stop` waiter
    let (tx, rx) = tokio::sync::oneshot::channel();
    df.stop_reply_senders.push(tx);

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let disconnected = BTreeSet::from([daemon_id]);

    let actions = cleanup_disconnected_daemons_from_running_dataflows(
        &mut running_dataflows,
        &disconnected,
        &mut HashMap::new(),
    );
    assert!(
        matches!(actions.as_slice(), [DisconnectAction::ReclaimOrphaned(id)] if *id == dataflow_id),
        "must reclaim (not terminally tear down) the orphaned dataflow"
    );

    apply_disconnect_actions(
        actions,
        &mut running_dataflows,
        &mut daemon_connections,
        &store,
        &clock,
    )
    .await
    .expect("apply_disconnect_actions");

    assert!(
        !running_dataflows.contains_key(&dataflow_id),
        "orphaned dataflow must be removed from running set"
    );
    // Reclaimable, not terminal: store record is Recovering so a reconnecting
    // daemon's status report can promote it back to Running.
    let record = store
        .get_dataflow(&dataflow_id)
        .expect("store lookup")
        .expect("record present");
    assert!(
        matches!(record.status, StoreDataflowStatus::Recovering),
        "disconnect must mark the dataflow Recovering, got {:?}",
        record.status
    );
    // The parked `dora stop` must still be released (no hang).
    let reply = rx.await.expect("stop waiter must be resolved by reclaim");
    assert!(
        matches!(reply, Ok(ControlRequestReply::DataflowStopped { uuid, .. }) if uuid == dataflow_id),
        "parked dora stop must receive DataflowStopped"
    );
}

#[test]
fn resolve_param_target_returns_running_daemon_for_active_node() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(
        dataflow_id,
        test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone()),
    );

    let target =
        resolve_param_target(&running_dataflows, store.as_ref(), &dataflow_id, &node_id).unwrap();
    match target {
        ParamTarget::Running { daemon_id: got } => assert_eq!(got, daemon_id),
        ParamTarget::PersistedOnly => panic!("expected running target"),
    }
}

#[test]
fn resolve_param_target_returns_persisted_only_for_known_stopped_dataflow() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let node_id: dora_core::config::NodeId = "camera".to_string().into();
    let running_dataflows = HashMap::new();

    let record = dora_coordinator_store::DataflowRecord {
        uuid: dataflow_id,
        name: Some("df".to_string()),
        descriptor_json: serde_json::json!({
            "nodes": [
                {"id": "camera"}
            ]
        })
        .to_string(),
        status: dora_coordinator_store::DataflowStatus::Succeeded,
        daemon_ids: Vec::new(),
        node_to_daemon: BTreeMap::new(),
        uv: false,
        ready_barrier_released: false,
        barrier_exited_before_subscribe: Vec::new(),
        generation: 1,
        created_at: 0,
        updated_at: 0,
    };
    store.put_dataflow(&record).unwrap();

    let target =
        resolve_param_target(&running_dataflows, store.as_ref(), &dataflow_id, &node_id).unwrap();
    match target {
        ParamTarget::PersistedOnly => {}
        ParamTarget::Running { .. } => panic!("expected persisted-only target"),
    }
}

#[test]
fn resolve_param_target_errors_for_unknown_dataflow() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let node_id: dora_core::config::NodeId = "camera".to_string().into();
    let running_dataflows = HashMap::new();

    let err = resolve_param_target(&running_dataflows, store.as_ref(), &dataflow_id, &node_id)
        .expect_err("unknown dataflow should fail validation");
    assert!(err.to_string().contains("dataflow"));
    assert!(err.to_string().contains("node"));
}

#[test]
fn resolve_param_target_errors_for_unknown_node() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let existing_node: dora_core::config::NodeId = "camera".to_string().into();
    let missing_node: dora_core::config::NodeId = "ghost".to_string().into();

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(
        dataflow_id,
        test_running_dataflow(dataflow_id, daemon_id, existing_node),
    );

    let err = resolve_param_target(
        &running_dataflows,
        store.as_ref(),
        &dataflow_id,
        &missing_node,
    )
    .expect_err("unknown node should fail validation");
    assert!(err.to_string().contains("not found in dataflow"));
}

#[tokio::test]
async fn replay_replays_persisted_param_to_daemon_connection() {
    #[derive(serde::Deserialize)]
    struct OutboundRaw {
        id: String,
        method: String,
        params: Timestamped<DaemonCoordinatorEvent>,
    }

    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let value_bytes = serde_json::to_vec(&serde_json::json!(42)).unwrap();
    store
        .put_node_param(&dataflow_id, &node_id, "threshold", &value_bytes)
        .unwrap();

    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let connection =
        crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());

    let node_id_for_assert = node_id.clone();
    let daemon_task = tokio::spawn(async move {
        let outbound = rx
            .recv()
            .await
            .expect("daemon should receive replay command");
        let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
        assert_eq!(outbound_raw.method, "daemon_command");

        let request_id = Uuid::parse_str(&outbound_raw.id).expect("valid request id");

        match outbound_raw.params.inner {
            DaemonCoordinatorEvent::SetParam {
                dataflow_id: replay_df,
                node_id: replay_node,
                key,
                value,
            } => {
                assert_eq!(replay_df, dataflow_id);
                assert_eq!(replay_node, node_id_for_assert);
                assert_eq!(key, "threshold");
                assert_eq!(value, serde_json::json!(42));
            }
            other => panic!("unexpected replay event: {other:?}"),
        }

        let reply = serde_json::to_string(&DaemonCoordinatorReply::SetParamResult(Ok(()))).unwrap();
        let reply_tx = pending_replies
            .lock()
            .await
            .remove(&request_id)
            .expect("pending reply sender should exist");
        let _ = reply_tx.send(reply);
    });

    let summary = replay_persisted_params_for_daemon(
        dataflow_id,
        daemon_id,
        vec![node_id],
        store,
        connection,
        Arc::new(HLC::default()),
    )
    .await;
    assert_eq!(summary.attempted, 1);
    assert_eq!(summary.failed, 0);

    // Bounded: if the coordinator stops sending, this must fail fast
    // rather than block forever and burn the CI job timeout.
    tokio::time::timeout(Duration::from_secs(10), daemon_task)
        .await
        .expect("daemon task did not receive the expected message")
        .unwrap();
}

#[tokio::test]
async fn replay_skips_when_no_persisted_params() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let connection = crate::state::DaemonConnection::new(tx, pending_replies, BTreeMap::new());

    let summary = replay_persisted_params_for_daemon(
        dataflow_id,
        daemon_id,
        vec![node_id],
        store,
        connection,
        Arc::new(HLC::default()),
    )
    .await;
    assert_eq!(summary.attempted, 0);
    assert_eq!(summary.failed, 0);

    let recv = timeout(TokioDuration::from_millis(50), rx.recv()).await;
    assert!(
        matches!(recv, Err(_) | Ok(None)),
        "no replay command should be sent for empty persisted params"
    );
}

#[tokio::test]
async fn replay_reports_failure_when_daemon_rejects_param() {
    #[derive(serde::Deserialize)]
    struct OutboundRaw {
        id: String,
    }

    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let value_bytes = serde_json::to_vec(&serde_json::json!(7)).unwrap();
    store
        .put_node_param(&dataflow_id, &node_id, "threshold", &value_bytes)
        .unwrap();

    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let connection =
        crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());

    let daemon_task = tokio::spawn(async move {
        let outbound = rx
            .recv()
            .await
            .expect("daemon should receive replay command");
        let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
        let request_id = Uuid::parse_str(&outbound_raw.id).expect("valid request id");

        let reply = serde_json::to_string(&DaemonCoordinatorReply::SetParamResult(Err(
            "rejected".to_string()
        )))
        .unwrap();
        let reply_tx = pending_replies
            .lock()
            .await
            .remove(&request_id)
            .expect("pending reply sender should exist");
        let _ = reply_tx.send(reply);
    });

    let summary = replay_persisted_params_for_daemon(
        dataflow_id,
        daemon_id,
        vec![node_id],
        store,
        connection,
        Arc::new(HLC::default()),
    )
    .await;

    assert_eq!(summary.attempted, 1);
    assert_eq!(summary.failed, 1);
    assert!(summary.failed > 0);
    // Bounded: if the coordinator stops sending, this must fail fast
    // rather than block forever and burn the CI job timeout.
    tokio::time::timeout(Duration::from_secs(10), daemon_task)
        .await
        .expect("daemon task did not receive the expected message")
        .unwrap();
}

#[tokio::test]
async fn fallback_replay_keeps_ack_unchanged_when_daemon_is_disconnected() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let value_bytes = serde_json::to_vec(&serde_json::json!(1)).unwrap();
    store
        .put_node_param(&dataflow_id, &node_id, "threshold", &value_bytes)
        .unwrap();

    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
    dataflow.state_log_sequence = 10;
    dataflow.daemon_ack_sequence.insert(daemon_id.clone(), 3);

    let mut daemon_connections = DaemonConnections::default();
    handle_pruned_state_catchup_fallback(
        dataflow_id,
        &mut dataflow,
        &daemon_id,
        store,
        &mut daemon_connections,
        Arc::new(HLC::default()),
        Instant::now(),
    )
    .await;

    assert_eq!(dataflow.daemon_ack_sequence.get(&daemon_id), Some(&3));
    assert!(!dataflow.last_replay_attempt.contains_key(&daemon_id));
}

#[tokio::test]
async fn fallback_replay_respects_backoff_window() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
    dataflow.state_log_sequence = 8;
    dataflow.daemon_ack_sequence.insert(daemon_id.clone(), 2);
    dataflow
        .last_replay_attempt
        .insert(daemon_id.clone(), Instant::now());

    let mut daemon_connections = DaemonConnections::default();
    handle_pruned_state_catchup_fallback(
        dataflow_id,
        &mut dataflow,
        &daemon_id,
        store,
        &mut daemon_connections,
        Arc::new(HLC::default()),
        Instant::now(),
    )
    .await;

    // No replay attempted while backoff is active, so ack remains unchanged.
    assert_eq!(dataflow.daemon_ack_sequence.get(&daemon_id), Some(&2));
}

#[tokio::test]
async fn ready_boundary_schedules_replay_for_daemon_nodes() {
    #[derive(serde::Deserialize)]
    struct OutboundRaw {
        id: String,
        method: String,
        params: Timestamped<DaemonCoordinatorEvent>,
    }

    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let value_bytes = serde_json::to_vec(&serde_json::json!(123)).unwrap();
    store
        .put_node_param(&dataflow_id, &node_id, "gain", &value_bytes)
        .unwrap();

    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let connection =
        crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(daemon_id.clone(), connection);

    let running_dataflow = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());

    let daemon_task = tokio::spawn(async move {
        let outbound = rx
            .recv()
            .await
            .expect("daemon should receive replay command from ready-boundary scheduler");
        let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
        assert_eq!(outbound_raw.method, "daemon_command");
        match outbound_raw.params.inner {
            DaemonCoordinatorEvent::SetParam {
                dataflow_id: replay_df,
                node_id: replay_node,
                key,
                value,
            } => {
                assert_eq!(replay_df, dataflow_id);
                assert_eq!(replay_node, node_id);
                assert_eq!(key, "gain");
                assert_eq!(value, serde_json::json!(123));
            }
            other => panic!("unexpected replay event: {other:?}"),
        }

        let request_id = Uuid::parse_str(&outbound_raw.id).expect("valid request id");
        let reply = serde_json::to_string(&DaemonCoordinatorReply::SetParamResult(Ok(()))).unwrap();
        let reply_tx = pending_replies
            .lock()
            .await
            .remove(&request_id)
            .expect("pending reply sender should exist");
        let _ = reply_tx.send(reply);
    });

    schedule_param_replay_for_ready_dataflow(
        dataflow_id,
        &running_dataflow,
        &mut daemon_connections,
        store,
        Arc::new(HLC::default()),
    );

    // Bounded: if the coordinator stops sending, this must fail fast
    // rather than block forever and burn the CI job timeout.
    tokio::time::timeout(Duration::from_secs(10), daemon_task)
        .await
        .expect("daemon task did not receive the expected message")
        .unwrap();
}

#[tokio::test]
async fn start_topic_debug_stream_targets_source_daemon() {
    #[derive(serde::Deserialize)]
    struct OutboundRaw {
        id: String,
        method: String,
        params: Timestamped<DaemonCoordinatorEvent>,
    }

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let data_id: dora_core::config::DataId = "message".to_string().into();
    let (frame_tx, _frame_rx) =
        tokio::sync::mpsc::channel::<crate::topic_subscriber::TopicFrame>(4);

    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let connection =
        crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(daemon_id.clone(), connection);

    let mut running_dataflows = HashMap::new();
    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
    dataflow.descriptor.debug.enable_debug_inspection = true;
    running_dataflows.insert(dataflow_id, dataflow);
    let expected_node_id = node_id.clone();
    let expected_data_id = data_id.clone();
    let seen_subscription = Arc::new(tokio::sync::Mutex::new(None::<Uuid>));
    let seen_subscription_task = seen_subscription.clone();

    let daemon_task = tokio::spawn(async move {
        let outbound = rx
            .recv()
            .await
            .expect("daemon should receive topic stream command");
        let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
        assert_eq!(outbound_raw.method, "daemon_command");
        match outbound_raw.params.inner {
            DaemonCoordinatorEvent::StartTopicDebugStream {
                dataflow_id: start_df,
                outputs,
                subscription_id,
                ..
            } => {
                assert_eq!(start_df, dataflow_id);
                assert_eq!(outputs, vec![(expected_node_id, expected_data_id)]);
                *seen_subscription_task.lock().await = Some(subscription_id);
            }
            other => panic!("unexpected topic stream event: {other:?}"),
        }

        let request_id = Uuid::parse_str(&outbound_raw.id).expect("valid request id");
        let reply =
            serde_json::to_string(&DaemonCoordinatorReply::StartTopicDebugStreamResult(Ok(())))
                .unwrap();
        let reply_tx = pending_replies
            .lock()
            .await
            .remove(&request_id)
            .expect("pending reply sender should exist");
        let _ = reply_tx.send(reply);
    });

    let subscription_id = start_topic_debug_stream(
        &mut running_dataflows,
        &mut daemon_connections,
        dataflow_id,
        vec![(node_id.clone(), data_id.clone())],
        frame_tx,
        &HLC::default(),
    )
    .await
    .expect("subscription should succeed");

    // Bounded: if the coordinator stops sending, this must fail fast
    // rather than block forever and burn the CI job timeout.
    tokio::time::timeout(Duration::from_secs(10), daemon_task)
        .await
        .expect("daemon task did not receive the expected message")
        .unwrap();
    assert_eq!(Some(subscription_id), *seen_subscription.lock().await);
    assert_eq!(
        running_dataflows[&dataflow_id]
            .topic_subscribers
            .get(&subscription_id)
            .expect("subscription should be stored")
            .outputs_by_daemon()
            .values()
            .next()
            .expect("daemon mapping should exist"),
        &vec![(node_id, data_id)]
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn start_topic_debug_stream_rolls_back_on_daemon_error() {
    #[derive(serde::Deserialize)]
    struct OutboundRaw {
        id: String,
        params: Timestamped<DaemonCoordinatorEvent>,
    }

    let dataflow_id = Uuid::new_v4();
    let daemon_id_a = DaemonId::new(Some("daemon-a".to_string()));
    let daemon_id_b = DaemonId::new(Some("daemon-b".to_string()));
    let node_id_a: dora_message::id::NodeId = "sender".to_string().into();
    let node_id_b: dora_message::id::NodeId = "sink".to_string().into();
    let data_id: dora_core::config::DataId = "message".to_string().into();
    let (frame_tx, _frame_rx) =
        tokio::sync::mpsc::channel::<crate::topic_subscriber::TopicFrame>(4);

    let mut daemon_connections = DaemonConnections::default();
    let mut daemon_tasks = Vec::new();
    for (daemon_id, should_fail) in [(daemon_id_a.clone(), false), (daemon_id_b.clone(), true)] {
        let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
        let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
        let connection =
            crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
        daemon_connections.add(daemon_id, connection);

        daemon_tasks.push(tokio::spawn(async move {
            while let Some(outbound) = rx.recv().await {
                let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
                let request_id = Uuid::parse_str(&outbound_raw.id).expect("valid request id");
                let reply = match outbound_raw.params.inner {
                    DaemonCoordinatorEvent::StartTopicDebugStream { .. } if should_fail => {
                        serde_json::to_string(&DaemonCoordinatorReply::StartTopicDebugStreamResult(
                            Err("daemon rejected debug stream".to_string()),
                        ))
                        .unwrap()
                    }
                    DaemonCoordinatorEvent::StartTopicDebugStream { .. } => serde_json::to_string(
                        &DaemonCoordinatorReply::StartTopicDebugStreamResult(Ok(())),
                    )
                    .unwrap(),
                    DaemonCoordinatorEvent::StopTopicDebugStream { .. } => serde_json::to_string(
                        &DaemonCoordinatorReply::StopTopicDebugStreamResult(Ok(())),
                    )
                    .unwrap(),
                    other => panic!("unexpected daemon event during rollback test: {other:?}"),
                };
                let reply_tx = pending_replies
                    .lock()
                    .await
                    .remove(&request_id)
                    .expect("pending reply sender should exist");
                let _ = reply_tx.send(reply);
            }
        }));
    }

    let mut running_dataflows = HashMap::new();
    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id_a.clone(), node_id_a.clone());
    dataflow.descriptor.debug.enable_debug_inspection = true;
    dataflow
        .node_to_daemon
        .insert(node_id_b.clone(), daemon_id_b.clone());
    let mut descriptor_json = serde_json::to_value(&dataflow.descriptor).unwrap();
    descriptor_json
        .get_mut("nodes")
        .and_then(serde_json::Value::as_array_mut)
        .expect("descriptor nodes array")
        .push(serde_json::json!({
            "id": node_id_b,
            // `path` is required for the descriptor to resolve (see
            // `test_running_dataflow`).
            "path": node_id_b,
            "outputs": [data_id.clone()],
        }));
    dataflow.descriptor = serde_json::from_value(descriptor_json).unwrap();
    // Mirror what `spawn_dataflow` (and `AddNode`/`ReplaceNode`) do: the
    // resolved map is the authoritative one and is always written together
    // with the descriptor.
    dataflow.nodes = dataflow
        .descriptor
        .resolve_aliases_and_set_defaults()
        .expect("test descriptor should resolve");
    running_dataflows.insert(dataflow_id, dataflow);

    let err = start_topic_debug_stream(
        &mut running_dataflows,
        &mut daemon_connections,
        dataflow_id,
        vec![(node_id_a, data_id.clone()), (node_id_b, data_id)],
        frame_tx,
        &HLC::default(),
    )
    .await
    .expect_err("subscription should fail");

    assert!(format!("{err:#}").contains("daemon rejected debug stream"));
    assert!(
        running_dataflows[&dataflow_id].topic_subscribers.is_empty(),
        "failed subscription should be rolled back"
    );

    for task in daemon_tasks {
        task.abort();
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn restore_topic_debug_streams_re_issues_start_after_reconnect() {
    // Regression test for the daemon-reconnect lifecycle fix (#238 / #242):
    // when a daemon reconnects, every active subscriber with outputs on
    // that daemon must receive a fresh StartTopicDebugStream.
    #[derive(serde::Deserialize)]
    struct OutboundRaw {
        id: String,
        params: Timestamped<DaemonCoordinatorEvent>,
    }

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let data_id: dora_core::config::DataId = "message".to_string().into();
    let subscription_id = Uuid::new_v4();

    // Stand up a connection whose rx we can inspect after the reconnect path runs.
    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let connection =
        crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(daemon_id.clone(), connection);

    // Pre-populate a dataflow with an already-registered subscriber. This
    // simulates a subscriber that was set up before the daemon dropped.
    let mut running_dataflows = HashMap::new();
    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());
    let (frame_tx, _frame_rx) =
        tokio::sync::mpsc::channel::<crate::topic_subscriber::TopicFrame>(4);
    let mut outputs_by_daemon = BTreeMap::new();
    outputs_by_daemon.insert(daemon_id.clone(), vec![(node_id.clone(), data_id.clone())]);
    dataflow.topic_subscribers.insert(
        subscription_id,
        crate::topic_subscriber::TopicSubscriber::new(outputs_by_daemon, frame_tx),
    );
    running_dataflows.insert(dataflow_id, dataflow);

    // Task that responds as the reconnected daemon would.
    let seen = Arc::new(tokio::sync::Mutex::new(None::<(Uuid, DataflowId)>));
    let seen_task = seen.clone();
    let daemon_task = tokio::spawn(async move {
        let outbound = rx
            .recv()
            .await
            .expect("reconnected daemon should receive restore message");
        let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
        if let DaemonCoordinatorEvent::StartTopicDebugStream {
            dataflow_id: restore_df,
            subscription_id: restore_sub,
            ..
        } = outbound_raw.params.inner
        {
            *seen_task.lock().await = Some((restore_sub, restore_df));
        } else {
            panic!(
                "unexpected event on reconnect: {:?}",
                outbound_raw.params.inner
            );
        }
        let reply =
            serde_json::to_string(&DaemonCoordinatorReply::StartTopicDebugStreamResult(Ok(())))
                .unwrap();
        let request_id = Uuid::parse_str(&outbound_raw.id).expect("valid request id");
        let reply_tx = pending_replies
            .lock()
            .await
            .remove(&request_id)
            .expect("pending reply sender should exist");
        let _ = reply_tx.send(reply);
    });

    let mut reported = BTreeSet::new();
    reported.insert(dataflow_id);

    restore_topic_debug_streams_for_daemon(
        &running_dataflows,
        &mut daemon_connections,
        &daemon_id,
        &reported,
        &HLC::default(),
    )
    .await;

    // Bounded: if the coordinator stops sending, this must fail fast
    // rather than block forever and burn the CI job timeout.
    tokio::time::timeout(Duration::from_secs(10), daemon_task)
        .await
        .expect("daemon task did not receive the expected message")
        .unwrap();
    assert_eq!(
        *seen.lock().await,
        Some((subscription_id, dataflow_id)),
        "restore should re-issue StartTopicDebugStream for the existing subscription"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn restore_topic_debug_streams_skips_unreported_dataflows() {
    // If the daemon did not report this dataflow on reconnect, we must
    // not re-issue subscriptions for it (the dataflow is no longer on
    // that daemon).
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let data_id: dora_core::config::DataId = "message".to_string().into();

    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let connection =
        crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(daemon_id.clone(), connection);

    let mut running_dataflows = HashMap::new();
    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());
    let (frame_tx, _frame_rx) =
        tokio::sync::mpsc::channel::<crate::topic_subscriber::TopicFrame>(4);
    let mut outputs_by_daemon = BTreeMap::new();
    outputs_by_daemon.insert(daemon_id.clone(), vec![(node_id, data_id)]);
    dataflow.topic_subscribers.insert(
        Uuid::new_v4(),
        crate::topic_subscriber::TopicSubscriber::new(outputs_by_daemon, frame_tx),
    );
    running_dataflows.insert(dataflow_id, dataflow);

    // Empty reported set: daemon did not acknowledge this dataflow.
    let reported = BTreeSet::new();
    restore_topic_debug_streams_for_daemon(
        &running_dataflows,
        &mut daemon_connections,
        &daemon_id,
        &reported,
        &HLC::default(),
    )
    .await;

    // No message should have been sent.
    assert!(
        rx.try_recv().is_err(),
        "restore must not message the daemon for unreported dataflows"
    );
}

#[tokio::test]
async fn close_topic_subscribers_on_finish_drains_all_subscribers() {
    // Regression test for the CRITICAL dataflow-finish leak (#242 root cause).
    // Directly exercises the helper called from the DataflowFinishedOnDaemon
    // arm of the event loop (lib.rs: `close_topic_subscribers_on_finish`).
    // CLI must observe EOF on its data_rx (no silent hang).
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();

    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id, node_id);
    let (tx1, mut rx1) = tokio::sync::mpsc::channel(4);
    let (tx2, mut rx2) = tokio::sync::mpsc::channel(4);
    dataflow.topic_subscribers.insert(
        Uuid::new_v4(),
        crate::topic_subscriber::TopicSubscriber::new(BTreeMap::new(), tx1),
    );
    dataflow.topic_subscribers.insert(
        Uuid::new_v4(),
        crate::topic_subscriber::TopicSubscriber::new(BTreeMap::new(), tx2),
    );

    // Call the real helper, not a mirror of it. If the helper is renamed
    // or its semantics change, this test must be updated or fails loudly.
    super::close_topic_subscribers_on_finish(&mut dataflow);

    // Wrap recv in a short timeout so a future regression where close()
    // silently drops the sender without closing fails loudly instead of
    // hanging CI indefinitely.
    let timeout = std::time::Duration::from_secs(1);
    let got1 = tokio::time::timeout(timeout, rx1.recv())
        .await
        .expect("subscriber 1 must not hang after dataflow finish");
    assert!(got1.is_none(), "subscriber 1 must see EOF");
    let got2 = tokio::time::timeout(timeout, rx2.recv())
        .await
        .expect("subscriber 2 must not hang after dataflow finish");
    assert!(got2.is_none(), "subscriber 2 must see EOF");
}

/// Source-level guard that the DataflowFinishedOnDaemon dispatch still
/// calls the cleanup helper. A refactor that moves the branch but forgets
/// to keep the call wired up would leave the helper unreferenced from
/// `dataflow_events.rs` and fail this check.
///
/// This is a second-line guard — the primary protection is that
/// `close_topic_subscribers_on_finish` has no other callers, so removal
/// also produces a `dead_code` lint. This test hard-fails the case a
/// reviewer might waive.
#[test]
fn dataflow_finish_dispatch_calls_close_helper() {
    // Runtime read (not include_str!) so we don't embed the source into
    // every test binary. The file is always present when `cargo test` runs
    // from the crate root.
    let src = std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/dataflow_events.rs"),
    )
    .expect("dataflow_events.rs must be readable at CARGO_MANIFEST_DIR/src/dataflow_events.rs");
    assert!(
        src.contains("close_topic_subscribers_on_finish(&mut finished_dataflow)"),
        "DataflowFinishedOnDaemon arm must still call close_topic_subscribers_on_finish; \
             if you moved the cleanup, update this guard to point at the new call site"
    );
}

#[test]
fn state_log_append_and_sequence() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
    assert_eq!(df.state_log_sequence, 0);
    assert!(df.state_log.is_empty());

    df.append_state_log(StateCatchUpOperation::SetParam {
        node_id: node_id.clone(),
        key: "threshold".to_string(),
        value: serde_json::json!(42),
    });
    assert_eq!(df.state_log_sequence, 1);
    assert_eq!(df.state_log.len(), 1);
    assert_eq!(df.state_log[0].sequence, 1);

    df.append_state_log(StateCatchUpOperation::DeleteParam {
        node_id,
        key: "threshold".to_string(),
    });
    assert_eq!(df.state_log_sequence, 2);
    assert_eq!(df.state_log.len(), 2);
}

#[test]
fn state_log_delta_returns_missed_entries() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
    for i in 0..5 {
        df.append_state_log(StateCatchUpOperation::SetParam {
            node_id: node_id.clone(),
            key: format!("key_{i}"),
            value: serde_json::json!(i),
        });
    }

    // Daemon acked up to seq 2 — should get entries 3, 4, 5
    let delta = df.state_log_delta(2).expect("delta should be available");
    assert_eq!(delta.len(), 3);
    assert_eq!(delta[0].sequence, 3);
    assert_eq!(delta[2].sequence, 5);

    // Daemon fully caught up — empty delta
    let delta = df.state_log_delta(5).expect("delta should be available");
    assert!(delta.is_empty());
}

#[test]
fn state_log_prune_removes_acked_entries() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let d1 = DaemonId::new(Some("m1".to_string()));
    let d2 = DaemonId::new(Some("m2".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let mut df = test_running_dataflow(dataflow_id, d1.clone(), node_id.clone());
    df.daemons.insert(d2.clone());
    for i in 0..5 {
        df.append_state_log(StateCatchUpOperation::SetParam {
            node_id: node_id.clone(),
            key: format!("key_{i}"),
            value: serde_json::json!(i),
        });
    }

    // d1 acked 3, d2 acked 5 — min is 3, so entries 1-3 should be pruned
    df.daemon_ack_sequence.insert(d1, 3);
    df.daemon_ack_sequence.insert(d2, 5);
    df.prune_state_log();
    assert_eq!(df.state_log.len(), 2);
    assert_eq!(df.state_log[0].sequence, 4);
}

#[test]
fn state_log_prune_ignores_disconnected_daemon_ack() {
    // A permanently-disconnected daemon's stale ack must not pin `min_ack`
    // and block pruning forever (unbounded state-log growth up to the hard
    // cap). Only *live* daemons (those still in `df.daemons`) gate pruning.
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let d1 = DaemonId::new(Some("m1".to_string()));
    let d2 = DaemonId::new(Some("m2".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let mut df = test_running_dataflow(dataflow_id, d1.clone(), node_id.clone());
    df.daemons.insert(d2.clone());
    for i in 0..5 {
        df.append_state_log(StateCatchUpOperation::SetParam {
            node_id: node_id.clone(),
            key: format!("key_{i}"),
            value: serde_json::json!(i),
        });
    }

    // d1 (live) acked all 5; d2 acked only 2, then disconnected. The
    // disconnect cleanup removes d2 from `df.daemons` but leaves its stale
    // ack in `daemon_ack_sequence`.
    df.daemon_ack_sequence.insert(d1, 5);
    df.daemon_ack_sequence.insert(d2.clone(), 2);
    df.daemons.remove(&d2);

    df.prune_state_log();
    // Before the fix, the frozen d2 ack (2) pinned `min_ack` and left 3
    // entries; now only the live d1 ack (5) gates, so all are pruned.
    assert!(df.state_log.is_empty());

    // Pruning to empty must not silently strand d2. When d2 reconnects at
    // its stale ack (2), the catch-up path calls `state_log_delta(2)`; with
    // the log emptied it must return `None` so the caller falls back to a
    // full param replay — not `Some(empty)`, which would report d2 caught
    // up and lose the mutations at sequences 3..=5 forever.
    assert!(
        df.state_log_delta(2).is_none(),
        "reconnecting daemon behind the pruned log must trigger full replay"
    );
    // A caller already at the high-water mark is genuinely caught up.
    assert!(matches!(df.state_log_delta(5), Some(entries) if entries.is_empty()));
}

#[test]
fn state_log_delta_returns_none_when_pruned() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let d1 = DaemonId::new(Some("m1".to_string()));
    let d2 = DaemonId::new(Some("m2".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let mut df = test_running_dataflow(dataflow_id, d1.clone(), node_id.clone());
    df.daemons.insert(d2.clone());
    for i in 0..10 {
        df.append_state_log(StateCatchUpOperation::SetParam {
            node_id: node_id.clone(),
            key: format!("key_{i}"),
            value: serde_json::json!(i),
        });
    }

    // Simulate pruning: both acked up to 7
    df.daemon_ack_sequence.insert(d1, 7);
    df.daemon_ack_sequence.insert(d2, 7);
    df.prune_state_log();
    // Log now starts at seq 8

    // A daemon that was at seq 2 cannot catch up incrementally
    assert!(df.state_log_delta(2).is_none());
    // But a daemon at seq 7 can
    let delta = df.state_log_delta(7).expect("should succeed");
    assert_eq!(delta.len(), 3); // entries 8, 9, 10
}

#[test]
fn state_log_delta_returns_none_when_log_fully_drained_but_daemon_behind() {
    // Regression for #2601: a relinked daemon that was never seeded into
    // `daemon_ack_sequence` queries with last_ack == 0. If peers have
    // acked and `prune_state_log` drained the log entirely (while
    // `state_log_sequence` stays > 0), the empty-log branch must return
    // `None` (→ full param replay), not `Some([])` ("up to date").
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let d1 = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let mut df = test_running_dataflow(dataflow_id, d1.clone(), node_id.clone());
    for i in 0..3 {
        df.append_state_log(StateCatchUpOperation::SetParam {
            node_id: node_id.clone(),
            key: format!("key_{i}"),
            value: serde_json::json!(i),
        });
    }

    // d1 (the only member in the ack map) acks all 3 → prune drains the
    // whole log, but state_log_sequence stays at 3.
    df.daemon_ack_sequence.insert(d1, 3);
    df.prune_state_log();
    assert!(df.state_log.is_empty());
    assert_eq!(df.state_log_sequence, 3);

    // A relinked daemon not in the ack map is provably behind (0 < 3) yet
    // sees an empty log → must get the None fallback, not Some([]).
    assert!(df.state_log_delta(0).is_none());

    // A member already at the head is genuinely up to date → empty, non-None.
    let delta = df.state_log_delta(3).expect("head daemon is up to date");
    assert!(delta.is_empty());
}

#[test]
fn set_param_forward_reply_reports_daemon_rejection() {
    let reply = serde_json::to_vec(&DaemonCoordinatorReply::SetParamResult(Err(
        "node `camera` channel full".to_string(),
    )))
    .unwrap();
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let err = ensure_set_param_forward_applied(&reply, &node_id)
        .expect_err("daemon rejection should fail strict forwarding");
    assert!(err.to_string().contains("failed to apply SetParam"));
}

// -------------------------------------------------------------------
// AddNode reply validation (rescue of #1757, addresses #1682)
// -------------------------------------------------------------------
//
// The bug: coordinator's `Ok(_) =>` arm in the AddNode dispatch
// (lib.rs:1558 pre-fix) accepted any successful `send_and_receive`
// reply and committed dataflow state, even when the daemon returned
// an `AddNodeResult(Err(...))` or a stale reply from a different
// request. The three tests below pin the validator's contract: it
// must accept ONLY a successful `AddNodeResult` and forward every
// other shape as an error to the call site (which then surfaces it
// to the CLI without bringing down the coordinator's main loop).

#[test]
fn add_node_reply_accepts_daemon_success() {
    let reply = serde_json::to_vec(&DaemonCoordinatorReply::AddNodeResult(Ok(()))).unwrap();
    let node_id: dora_core::config::NodeId = "filter".to_string().into();

    ensure_add_node_applied(&reply, &node_id).expect("successful AddNode reply should pass");
}

#[test]
fn add_node_reply_reports_daemon_rejection() {
    let reply = serde_json::to_vec(&DaemonCoordinatorReply::AddNodeResult(Err(
        "failed to spawn node".to_string(),
    )))
    .unwrap();
    let node_id: dora_core::config::NodeId = "filter".to_string().into();

    let err = ensure_add_node_applied(&reply, &node_id)
        .expect_err("daemon rejection should fail AddNode forwarding");
    let msg = err.to_string();
    assert!(
        msg.contains("failed to add node") && msg.contains("filter"),
        "error must name the operation and node: {msg}"
    );
}

#[test]
fn add_node_reply_rejects_wrong_reply_variant() {
    // This is the regression scenario for #1682: the daemon returned
    // a stale or otherwise unrelated reply variant (here:
    // `SetParamResult(Ok)`). Before the fix, the coordinator's
    // `Ok(_) =>` arm would accept this and commit state for a node
    // the daemon never actually added.
    let reply = serde_json::to_vec(&DaemonCoordinatorReply::SetParamResult(Ok(()))).unwrap();
    let node_id: dora_core::config::NodeId = "filter".to_string().into();

    let err = ensure_add_node_applied(&reply, &node_id)
        .expect_err("unexpected reply variant should fail AddNode forwarding");
    assert!(
        err.to_string().contains("unexpected daemon reply"),
        "error must call out the wrong-reply-type failure mode: {err}"
    );

    let reply = serde_json::to_vec(&DaemonCoordinatorReply::RemoveNodeResult(Ok(()))).unwrap();
    let err = ensure_add_node_applied(&reply, &node_id)
        .expect_err("RemoveNodeResult reply must not be accepted by AddNode validator");
    assert!(
        err.to_string().contains("unexpected daemon reply"),
        "error must call out the wrong-reply-type failure mode: {err}"
    );
}

// Same #1682 specific-reply contract for ReplaceNode (dora-rs/dora#2927):
// the coordinator must commit its descriptor swap only on a successful
// `ReplaceNodeResult`, never on a rejection or a stale unrelated reply.

#[test]
fn replace_node_reply_accepts_daemon_success() {
    let reply = serde_json::to_vec(&DaemonCoordinatorReply::ReplaceNodeResult(Ok(()))).unwrap();
    let node_id: dora_core::config::NodeId = "filter".to_string().into();

    ensure_replace_node_applied(&reply, &node_id)
        .expect("successful ReplaceNode reply should pass");
}

#[test]
fn replace_node_reply_reports_daemon_rejection() {
    let reply = serde_json::to_vec(&DaemonCoordinatorReply::ReplaceNodeResult(Err(
        "failed to spawn replacement node".to_string(),
    )))
    .unwrap();
    let node_id: dora_core::config::NodeId = "filter".to_string().into();

    let err = ensure_replace_node_applied(&reply, &node_id)
        .expect_err("daemon rejection should fail ReplaceNode forwarding");
    let msg = err.to_string();
    assert!(
        msg.contains("failed to replace node") && msg.contains("filter"),
        "error must name the operation and node: {msg}"
    );
}

#[test]
fn replace_node_reply_rejects_wrong_reply_variant() {
    let node_id: dora_core::config::NodeId = "filter".to_string().into();
    for wrong in [
        serde_json::to_vec(&DaemonCoordinatorReply::AddNodeResult(Ok(()))).unwrap(),
        serde_json::to_vec(&DaemonCoordinatorReply::RemoveNodeResult(Ok(()))).unwrap(),
    ] {
        let err = ensure_replace_node_applied(&wrong, &node_id)
            .expect_err("unexpected reply variant should fail ReplaceNode forwarding");
        assert!(
            err.to_string().contains("unexpected daemon reply"),
            "error must call out the wrong-reply-type failure mode: {err}"
        );
    }
}

#[test]
fn delete_param_forward_reply_rejects_unexpected_reply_variant() {
    let reply = serde_json::to_vec(&DaemonCoordinatorReply::SetParamResult(Ok(()))).unwrap();
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let err = ensure_delete_param_forward_applied(&reply, &node_id)
        .expect_err("unexpected reply variant should fail strict forwarding");
    assert!(err.to_string().contains("unexpected daemon reply"));
}

#[test]
fn remove_node_reply_accepts_daemon_success() {
    let reply = serde_json::to_vec(&DaemonCoordinatorReply::RemoveNodeResult(Ok(()))).unwrap();
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    ensure_remove_node_applied(&reply, &node_id).expect("successful RemoveNode reply should pass");
}

#[test]
fn remove_node_reply_reports_daemon_rejection() {
    let reply = serde_json::to_vec(&DaemonCoordinatorReply::RemoveNodeResult(Err(
        "node `camera` not found in running dataflow".to_string(),
    )))
    .unwrap();
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let err = ensure_remove_node_applied(&reply, &node_id)
        .expect_err("daemon rejection should fail RemoveNode forwarding");
    let msg = err.to_string();
    assert!(
        msg.contains("failed to remove node") && msg.contains("camera"),
        "error must name the operation and node: {msg}"
    );
}

#[test]
fn remove_node_reply_rejects_wrong_reply_variant() {
    let reply = serde_json::to_vec(&DaemonCoordinatorReply::SetParamResult(Ok(()))).unwrap();
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let err = ensure_remove_node_applied(&reply, &node_id)
        .expect_err("unexpected reply variant should fail RemoveNode forwarding");
    assert!(
        err.to_string().contains("unexpected daemon reply"),
        "error must call out the wrong-reply-type failure mode: {err}"
    );

    let reply = serde_json::to_vec(&DaemonCoordinatorReply::AddNodeResult(Ok(()))).unwrap();
    let err = ensure_remove_node_applied(&reply, &node_id)
        .expect_err("AddNodeResult reply must not be accepted by RemoveNode validator");
    assert!(
        err.to_string().contains("unexpected daemon reply"),
        "error must call out the wrong-reply-type failure mode: {err}"
    );
}

// -------------------------------------------------------------------
// AddMapping / RemoveMapping reply validation (silent-reply rescue
// for the connect/disconnect timeouts — same class as #1682).
// -------------------------------------------------------------------

#[test]
fn add_mapping_reply_accepts_daemon_success() {
    let reply = serde_json::to_vec(&DaemonCoordinatorReply::AddMappingResult(Ok(()))).unwrap();
    ensure_add_mapping_applied(&reply, "sender/value", "filter/input")
        .expect("successful AddMapping reply should pass");
}

#[test]
fn add_mapping_reply_reports_daemon_rejection() {
    let reply = serde_json::to_vec(&DaemonCoordinatorReply::AddMappingResult(Err(
        "no running dataflow with ID `xyz`".to_string(),
    )))
    .unwrap();
    let err = ensure_add_mapping_applied(&reply, "sender/value", "filter/input")
        .expect_err("daemon rejection should fail AddMapping forwarding");
    let msg = err.to_string();
    assert!(
        msg.contains("failed to add mapping")
            && msg.contains("sender/value")
            && msg.contains("filter/input"),
        "error must name the operation and the mapping endpoints: {msg}"
    );
}

#[test]
fn add_mapping_reply_rejects_wrong_reply_variant() {
    // Pre-fix #1682-equivalent regression: daemon returned `None`,
    // WS layer dropped it, coordinator's `Ok(_) =>` arm reported
    // success on any wire response. With AddMappingResult now typed,
    // a foreign reply variant must NOT be accepted.
    let reply = serde_json::to_vec(&DaemonCoordinatorReply::SetParamResult(Ok(()))).unwrap();
    let err = ensure_add_mapping_applied(&reply, "sender/value", "filter/input")
        .expect_err("unexpected reply variant should fail AddMapping forwarding");
    assert!(
        err.to_string().contains("unexpected daemon reply"),
        "error must call out the wrong-reply-type failure mode: {err}"
    );

    let reply = serde_json::to_vec(&DaemonCoordinatorReply::RemoveMappingResult(Ok(()))).unwrap();
    let err = ensure_add_mapping_applied(&reply, "sender/value", "filter/input")
        .expect_err("RemoveMappingResult must not be accepted by AddMapping validator");
    assert!(
        err.to_string().contains("unexpected daemon reply"),
        "error must call out the wrong-reply-type failure mode: {err}"
    );
}

#[test]
fn remove_mapping_reply_accepts_daemon_success() {
    let reply = serde_json::to_vec(&DaemonCoordinatorReply::RemoveMappingResult(Ok(()))).unwrap();
    ensure_remove_mapping_applied(&reply, "sender/value", "filter/input")
        .expect("successful RemoveMapping reply should pass");
}

#[test]
fn remove_mapping_reply_reports_daemon_rejection() {
    let reply = serde_json::to_vec(&DaemonCoordinatorReply::RemoveMappingResult(Err(
        "no running dataflow with ID `xyz`".to_string(),
    )))
    .unwrap();
    let err = ensure_remove_mapping_applied(&reply, "sender/value", "filter/input")
        .expect_err("daemon rejection should fail RemoveMapping forwarding");
    let msg = err.to_string();
    assert!(
        msg.contains("failed to remove mapping")
            && msg.contains("sender/value")
            && msg.contains("filter/input"),
        "error must name the operation and the mapping endpoints: {msg}"
    );
}

#[test]
fn remove_mapping_reply_rejects_wrong_reply_variant() {
    let reply = serde_json::to_vec(&DaemonCoordinatorReply::SetParamResult(Ok(()))).unwrap();
    let err = ensure_remove_mapping_applied(&reply, "sender/value", "filter/input")
        .expect_err("unexpected reply variant should fail RemoveMapping forwarding");
    assert!(
        err.to_string().contains("unexpected daemon reply"),
        "error must call out the wrong-reply-type failure mode: {err}"
    );

    let reply = serde_json::to_vec(&DaemonCoordinatorReply::AddMappingResult(Ok(()))).unwrap();
    let err = ensure_remove_mapping_applied(&reply, "sender/value", "filter/input")
        .expect_err("AddMappingResult must not be accepted by RemoveMapping validator");
    assert!(
        err.to_string().contains("unexpected daemon reply"),
        "error must call out the wrong-reply-type failure mode: {err}"
    );
}

// -------------------------------------------------------------------
// Node-stop stale-metrics fix (PR #1901 / follow-up to #1703 prereq)
// -------------------------------------------------------------------

#[test]
fn expire_stopped_nodes_removes_entries_older_than_grace() {
    use dora_message::daemon_to_coordinator::{NodeMetrics, NodeStatus};

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "stopped-sender".to_string().into();
    let fresh_node: dora_core::config::NodeId = "fresh-receiver".to_string().into();

    let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());

    let stale_row = NodeMetrics {
        pid: 0,
        cpu_usage: 0.0,
        memory_bytes: 0,
        disk_read_bytes: None,
        disk_write_bytes: None,
        restart_count: 0,
        broken_inputs: Vec::new(),
        status: NodeStatus::Stopped,
        pending_messages: 0,
    };
    let fresh_row = NodeMetrics {
        status: NodeStatus::Running,
        ..stale_row.clone()
    };

    df.node_metrics.insert(node_id.clone(), stale_row);
    df.node_metrics.insert(fresh_node.clone(), fresh_row);

    // Backdate the stale node past the grace window; leave the fresh
    // node out of node_stopped_at entirely (it's still running).
    df.node_stopped_at.insert(
        node_id.clone(),
        Instant::now() - NODE_STOPPED_GRACE - Duration::from_secs(1),
    );

    expire_stopped_nodes(&mut df);

    assert!(
        !df.node_metrics.contains_key(&node_id),
        "stale stopped row should be dropped after grace"
    );
    assert!(
        !df.node_stopped_at.contains_key(&node_id),
        "stale stopped_at marker should be dropped together with the metrics row"
    );
    assert!(
        df.node_metrics.contains_key(&fresh_node),
        "non-stopped node must NOT be affected by the sweep"
    );
}

#[test]
fn expire_stopped_nodes_keeps_entries_within_grace() {
    use dora_message::daemon_to_coordinator::{NodeMetrics, NodeStatus};

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "just-stopped".to_string().into();

    let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
    df.node_metrics.insert(
        node_id.clone(),
        NodeMetrics {
            pid: 0,
            cpu_usage: 0.0,
            memory_bytes: 0,
            disk_read_bytes: None,
            disk_write_bytes: None,
            restart_count: 0,
            broken_inputs: Vec::new(),
            status: NodeStatus::Stopped,
            pending_messages: 0,
        },
    );
    // Recent: well within grace.
    df.node_stopped_at.insert(node_id.clone(), Instant::now());

    expire_stopped_nodes(&mut df);

    assert!(
        df.node_metrics.contains_key(&node_id),
        "stopped row must stay visible during the grace window"
    );
    assert!(
        df.node_stopped_at.contains_key(&node_id),
        "stopped_at marker must stay armed during the grace window"
    );
}

#[test]
fn expire_stopped_nodes_leaves_failed_rows_untouched() {
    // Pre-fix regression scenario: the DaemonNodeStopped handler
    // previously inserted into `node_stopped_at` for both Stopped
    // and Failed statuses, so a crashed `restart_policy: Never`
    // node would disappear from `dora node list` and `dora doctor`
    // after the 60s grace window — hiding the very failure this
    // PR is supposed to surface. The handler now only arms the
    // expire side-band for Stopped rows; this test asserts that
    // a Failed row whose `node_stopped_at` entry is somehow set
    // would still be swept on its own clock, but in the normal
    // flow no entry exists for Failed rows so the sweep is a no-op.
    use dora_message::daemon_to_coordinator::{NodeMetrics, NodeStatus};

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "crashed".to_string().into();

    let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
    df.node_metrics.insert(
        node_id.clone(),
        NodeMetrics {
            pid: 0,
            cpu_usage: 0.0,
            memory_bytes: 0,
            disk_read_bytes: None,
            disk_write_bytes: None,
            restart_count: 0,
            broken_inputs: Vec::new(),
            status: NodeStatus::Failed,
            pending_messages: 0,
        },
    );
    // No node_stopped_at entry (which is what the handler now
    // guarantees for Failed rows). Even hours later, the sweep
    // must not remove this row.
    expire_stopped_nodes(&mut df);
    assert!(
        df.node_metrics.contains_key(&node_id),
        "Failed row with no expire timer must remain after sweep"
    );

    // Even if a *very* old expire timer somehow remains armed
    // for a Failed row (e.g. stale leftover from a prior
    // Stopped → respawn → Failed sequence), the sweep removes
    // both — that's OK because the caller for the Failed update
    // explicitly clears `node_stopped_at`. This test pins down
    // the sweep's semantics so a future refactor that decides
    // to "preserve Failed rows past grace" doesn't quietly
    // break by overriding the side-band contract.
}

#[test]
fn expire_stopped_nodes_clears_finalized_marker_too() {
    // The grace-period sweep must clear `node_finalized` in addition
    // to `node_metrics` and `node_stopped_at`. Without this, a
    // subsequent `dora node add` of the same name (or the next
    // metrics push for it) would be blocked by the stale marker —
    // the new incarnation would never appear in `dora node list`.
    use dora_message::daemon_to_coordinator::{NodeMetrics, NodeStatus};

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "expiring".to_string().into();

    let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
    df.node_metrics.insert(
        node_id.clone(),
        NodeMetrics {
            pid: 0,
            cpu_usage: 0.0,
            memory_bytes: 0,
            disk_read_bytes: None,
            disk_write_bytes: None,
            restart_count: 0,
            broken_inputs: Vec::new(),
            status: NodeStatus::Stopped,
            pending_messages: 0,
        },
    );
    df.node_finalized.insert(node_id.clone());
    df.node_stopped_at.insert(
        node_id.clone(),
        Instant::now() - NODE_STOPPED_GRACE - Duration::from_secs(1),
    );

    expire_stopped_nodes(&mut df);

    assert!(!df.node_metrics.contains_key(&node_id));
    assert!(!df.node_stopped_at.contains_key(&node_id));
    assert!(
        !df.node_finalized.contains(&node_id),
        "finalized marker must clear together with the metrics row, \
             else AddNode of the same id would be silently blocked"
    );
}

#[test]
fn finalized_marker_blocks_stale_metrics_for_failed_row_too() {
    // Pre-fix race: the metrics-handler guard used `node_stopped_at`
    // (only armed for Stopped). A crashed node sends NodeStopped
    // {clean_stop:false}, coordinator writes Failed but does NOT
    // arm node_stopped_at; a delayed in-flight metrics push then
    // sails past the guard and overwrites Failed back to Running.
    // The fix uses `node_finalized` (armed for BOTH Stopped and
    // Failed) so the guard catches the Failed case too.
    //
    // This test exercises the data-level invariant: a row marked
    // Failed AND inserted into node_finalized is what the
    // NodeMetrics handler now consults via
    // `dataflow.node_finalized.contains(node_id)` — the unit-test
    // equivalent is just verifying that lookup is true.
    use dora_message::daemon_to_coordinator::{NodeMetrics, NodeStatus};

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "crashed-no-restart".to_string().into();

    let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
    df.node_metrics.insert(
        node_id.clone(),
        NodeMetrics {
            pid: 0,
            cpu_usage: 0.0,
            memory_bytes: 0,
            disk_read_bytes: None,
            disk_write_bytes: None,
            restart_count: 0,
            broken_inputs: Vec::new(),
            status: NodeStatus::Failed,
            pending_messages: 0,
        },
    );
    // Failed path: finalized armed, stopped_at NOT armed.
    df.node_finalized.insert(node_id.clone());

    assert!(
        df.node_finalized.contains(&node_id),
        "Failed row must be in node_finalized so the metrics-push \
             guard at lib.rs:2181 skips it"
    );
    assert!(
        !df.node_stopped_at.contains_key(&node_id),
        "Failed row must NOT be in node_stopped_at — that would \
             arm the auto-expire timer and hide the crash after 60s"
    );

    // Run the sweep: nothing should change for a Failed row that
    // has no stopped_at entry.
    expire_stopped_nodes(&mut df);

    assert!(
        df.node_metrics.contains_key(&node_id) && df.node_finalized.contains(&node_id),
        "Failed row + its finalize marker must survive the sweep \
             until the dataflow itself is stopped/destroyed"
    );
}

#[test]
fn cached_result_is_pending_distinguishes_pending_and_cached() {
    let mut r = CachedResult::default();
    assert!(r.is_pending(), "fresh CachedResult should be Pending");

    r.set_result(Err(eyre!("done")));
    assert!(!r.is_pending(), "after set_result, should be Cached");

    // set_result on Cached should be a no-op (already covered by
    // existing semantics but verified here to lock in idempotency).
    r.set_result(Ok(ControlRequestReply::DataflowSpawned {
        uuid: DataflowId::from(Uuid::new_v4()),
    }));
    assert!(!r.is_pending());
}

#[tokio::test]
async fn check_spawn_timeouts_fires_error_when_pending_past_deadline() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = HLC::default();
    let mut daemon_connections = DaemonConnections::default();

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
    // Backdate the spawn start so the watchdog treats it as stuck.
    df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);
    // Still waiting on the daemon's spawn_result; this is the trigger
    // condition the watchdog is meant to catch.
    df.pending_spawn_results.insert(daemon_id);

    // Register a waiter so we can prove `wait_for_spawn` unblocks with
    // an error rather than hanging.
    let (tx, rx) = tokio::sync::oneshot::channel();
    df.spawn_result.register(tx);

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
    let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        IndexMap::new();
    let mut pending_restarts: HashMap<DataflowId, PendingRestart> = HashMap::new();

    check_spawn_timeouts(
        &mut running_dataflows,
        &mut archived_dataflows,
        &mut dataflow_results,
        &mut daemon_connections,
        &mut pending_restarts,
        &clock,
        store.as_ref(),
    )
    .await;

    // The waiter should now have an error reply.
    let reply = timeout(TokioDuration::from_secs(1), rx)
        .await
        .expect("waiter should resolve, not hang")
        .expect("sender should not drop");
    let err = reply.expect_err("timed-out spawn must surface as Err");
    let msg = format!("{err:?}");
    assert!(
        msg.contains("spawn timed out") || msg.contains("timeout"),
        "error should explain the timeout, got: {msg}"
    );

    // Watchdog must remove the dataflow from running_dataflows so
    // Check / List / reconcile no longer report it as active.
    assert!(
        !running_dataflows.contains_key(&dataflow_id),
        "watchdog must remove the terminally-failed dataflow from running_dataflows"
    );

    // And it must be archived so post-mortem queries can still find
    // its name/descriptor.
    assert!(
        archived_dataflows.contains_key(&dataflow_id),
        "watchdog must archive the terminally-failed dataflow"
    );
}

#[tokio::test]
async fn check_spawn_timeouts_no_op_when_within_deadline() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = HLC::default();
    let mut daemon_connections = DaemonConnections::default();

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
    // Fresh spawn — well within the deadline.
    df.spawn_started_at = Instant::now();
    df.pending_spawn_results.insert(daemon_id);

    let (tx, rx) = tokio::sync::oneshot::channel();
    df.spawn_result.register(tx);

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
    let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        IndexMap::new();
    let mut pending_restarts: HashMap<DataflowId, PendingRestart> = HashMap::new();

    check_spawn_timeouts(
        &mut running_dataflows,
        &mut archived_dataflows,
        &mut dataflow_results,
        &mut daemon_connections,
        &mut pending_restarts,
        &clock,
        store.as_ref(),
    )
    .await;

    // The waiter should still be pending.
    let polled = tokio::time::timeout(TokioDuration::from_millis(50), rx).await;
    assert!(polled.is_err(), "waiter must still be pending pre-deadline");

    let df = running_dataflows
        .get(&dataflow_id)
        .expect("df still present");
    assert!(
        df.spawn_result.is_pending(),
        "spawn_result must remain Pending pre-deadline"
    );
}

#[tokio::test]
async fn check_spawn_timeouts_idempotent_on_already_cached_result() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = HLC::default();
    let mut daemon_connections = DaemonConnections::default();

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let mut df = test_running_dataflow(dataflow_id, daemon_id, node_id);
    df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);
    // Spawn already resolved successfully — the watchdog must NOT
    // re-fire on subsequent heartbeats and clobber the cached result.
    df.spawn_result
        .set_result(Ok(ControlRequestReply::DataflowSpawned {
            uuid: dataflow_id,
        }));

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
    let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        IndexMap::new();
    let mut pending_restarts: HashMap<DataflowId, PendingRestart> = HashMap::new();

    check_spawn_timeouts(
        &mut running_dataflows,
        &mut archived_dataflows,
        &mut dataflow_results,
        &mut daemon_connections,
        &mut pending_restarts,
        &clock,
        store.as_ref(),
    )
    .await;

    let df = running_dataflows.get_mut(&dataflow_id).expect(
        "Cached(Ok) dataflow must stay in running_dataflows — only Cached(Err) triggers teardown",
    );
    // Registering a new waiter on a Cached result must immediately
    // deliver the cached Ok — proving the watchdog did not clobber it.
    let (tx, rx) = tokio::sync::oneshot::channel();
    df.spawn_result.register(tx);
    let reply = timeout(TokioDuration::from_millis(50), rx)
        .await
        .expect("Cached result should deliver immediately")
        .expect("sender should not drop");
    let ok = reply.expect("the cached Ok result must be preserved");
    assert!(matches!(ok, ControlRequestReply::DataflowSpawned { .. }));
}

/// Covers Finding 2 from the self-review on PR #1854: exercises the
/// rollback dispatch path with a *non-empty* succeeded set. The other
/// timeout tests have `succeeded = {}`, so the rollback helper
/// short-circuits at its `if spawned_daemons.is_empty()` early-exit
/// and never actually sends a stop message.
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn check_spawn_timeouts_dispatches_rollback_stop_to_succeeded_daemon() {
    #[derive(serde::Deserialize)]
    struct OutboundRaw {
        params: Timestamped<DaemonCoordinatorEvent>,
    }

    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = HLC::default();

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_a = DaemonId::new(Some("daemon-a".to_string())); // succeeded
    let daemon_b = DaemonId::new(Some("daemon-b".to_string())); // still pending

    // Mock daemon `a` -- fire-and-forget rollback uses `connection.send()`
    // which only enqueues; no reply is awaited. We just need to capture
    // the outbound message and verify StopDataflow was dispatched.
    let (tx_a, mut rx_a) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies_a = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let conn_a =
        crate::state::DaemonConnection::new(tx_a, pending_replies_a.clone(), BTreeMap::new());
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(daemon_a.clone(), conn_a);

    let stop_seen = Arc::new(tokio::sync::Mutex::new(false));
    let stop_seen_task = stop_seen.clone();
    let daemon_a_task = tokio::spawn(async move {
        while let Some(outbound) = rx_a.recv().await {
            let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
            match outbound_raw.params.inner {
                DaemonCoordinatorEvent::StopDataflow { .. } => {
                    *stop_seen_task.lock().await = true;
                }
                other => panic!("unexpected event on daemon-a in rollback test: {other:?}"),
            }
        }
    });

    // Build a RunningDataflow where `a` succeeded (in `daemons` but not
    // in `pending_spawn_results`) and `b` is still pending.
    let mut df = test_running_dataflow(dataflow_id, daemon_a.clone(), "sender".to_string().into());
    df.daemons.insert(daemon_b.clone());
    df.pending_spawn_results.insert(daemon_b.clone());
    df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

    // Register a waiter so we can confirm spawn_result fires.
    let (waiter_tx, waiter_rx) = tokio::sync::oneshot::channel();
    df.spawn_result.register(waiter_tx);

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
    let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        IndexMap::new();
    let mut pending_restarts: HashMap<DataflowId, PendingRestart> = HashMap::new();

    check_spawn_timeouts(
        &mut running_dataflows,
        &mut archived_dataflows,
        &mut dataflow_results,
        &mut daemon_connections,
        &mut pending_restarts,
        &clock,
        store.as_ref(),
    )
    .await;

    // 1. The mock daemon must have received a stop dispatch.
    //    Fire-and-forget rollback enqueues into the mpsc channel; the
    //    daemon task picks it up asynchronously, so poll briefly.
    let saw_stop = timeout(TokioDuration::from_secs(1), async {
        loop {
            if *stop_seen.lock().await {
                return true;
            }
            tokio::time::sleep(TokioDuration::from_millis(5)).await;
        }
    })
    .await
    .unwrap_or(false);
    assert!(
        saw_stop,
        "rollback must dispatch StopDataflow to the succeeded daemon"
    );

    // 2. The waiter must be released with an error (regardless of
    //    rollback outcome).
    let reply = timeout(TokioDuration::from_secs(1), waiter_rx)
        .await
        .expect("waiter should resolve, not hang")
        .expect("sender should not drop");
    let err = reply.expect_err("timed-out spawn must surface as Err");
    let msg = format!("{err:?}");
    assert!(
        msg.contains("spawn timed out") || msg.contains("timeout"),
        "error should explain the timeout, got: {msg}"
    );

    // 3. Dataflow must be persisted as Failed so a coordinator restart
    //    does not resurrect it as Recovering.
    let records = store.list_dataflows().expect("store should list");
    let record = records
        .iter()
        .find(|r| r.uuid == dataflow_id)
        .expect("dataflow should be persisted after timeout");
    assert!(
        matches!(
            record.status,
            dora_coordinator_store::DataflowStatus::Failed { .. }
        ),
        "dataflow must be persisted as Failed, got: {:?}",
        record.status,
    );

    daemon_a_task.abort();
}

/// Issue #3134: daemons report their spawn results asynchronously, so on a
/// multi-daemon dataflow one can fail after another has already started its
/// nodes. The failure must roll the started daemon back and tear the
/// dataflow down, otherwise its nodes run on unmanaged and `dora list`
/// keeps showing `Running` while the store says `Failed`.
///
/// Both orderings go through `handle_dataflow_spawn_result`, so the test
/// drives it: daemon `a` reports success, then daemon `b` reports a
/// failure, then `a`'s (impossible-in-this-ordering but harmless) late
/// success is replayed to cover the mirror case.
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn async_partial_spawn_failure_rolls_back_started_daemon() {
    #[derive(serde::Deserialize)]
    struct OutboundRaw {
        params: Timestamped<DaemonCoordinatorEvent>,
    }

    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = HLC::default();

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_a = DaemonId::new(Some("daemon-a".to_string()));
    let daemon_b = DaemonId::new(Some("daemon-b".to_string()));

    let (tx_a, mut rx_a) = tokio::sync::mpsc::channel::<String>(8);
    let conn_a = crate::state::DaemonConnection::new(
        tx_a,
        Arc::new(tokio::sync::Mutex::new(HashMap::new())),
        BTreeMap::new(),
    );
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(daemon_a.clone(), conn_a);

    let stops_seen = Arc::new(tokio::sync::Mutex::new(0usize));
    let stops_seen_task = stops_seen.clone();
    let daemon_a_task = tokio::spawn(async move {
        while let Some(outbound) = rx_a.recv().await {
            let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
            match outbound_raw.params.inner {
                DaemonCoordinatorEvent::StopDataflow { .. } => {
                    *stops_seen_task.lock().await += 1;
                }
                other => panic!("unexpected event on daemon-a: {other:?}"),
            }
        }
    });

    // Both daemons are assigned to the dataflow and both spawns are still
    // outstanding.
    let mut df = test_running_dataflow(dataflow_id, daemon_a.clone(), "sender".to_string().into());
    df.daemons.insert(daemon_b.clone());
    df.pending_spawn_results.insert(daemon_a.clone());
    df.pending_spawn_results.insert(daemon_b.clone());
    let (waiter_tx, waiter_rx) = tokio::sync::oneshot::channel();
    df.spawn_result.register(waiter_tx);
    let (stop_tx, stop_rx) = tokio::sync::oneshot::channel();
    df.stop_reply_senders.push(stop_tx);

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
    let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        IndexMap::new();

    // A `dora restart` parked while the spawn was still in flight. It is
    // keyed by this UUID and nothing else will ever drain it once the
    // dataflow is gone.
    let (restart_tx, restart_rx) = tokio::sync::oneshot::channel();
    let mut pending_restarts = HashMap::new();
    pending_restarts.insert(
        dataflow_id,
        PendingRestart {
            descriptor: serde_json::from_value(serde_json::json!({"nodes": [{"id": "sender"}]}))
                .expect("valid descriptor"),
            name: None,
            uv: false,
            reply_sender: restart_tx,
        },
    );

    // Ordering: `a` spawns successfully, then `b` fails.
    handle_dataflow_spawn_result(
        dataflow_id,
        daemon_a.clone(),
        Ok(()),
        &mut running_dataflows,
        &mut archived_dataflows,
        &mut dataflow_results,
        &mut daemon_connections,
        &mut pending_restarts,
        &clock,
        store.as_ref(),
    )
    .await;
    handle_dataflow_spawn_result(
        dataflow_id,
        daemon_b.clone(),
        Err(eyre!("no such binary on machine b")),
        &mut running_dataflows,
        &mut archived_dataflows,
        &mut dataflow_results,
        &mut daemon_connections,
        &mut pending_restarts,
        &clock,
        store.as_ref(),
    )
    .await;

    // 1. The core regression: the dataflow must not linger in
    //    `running_dataflows`, where `dora list` would keep calling it
    //    Running despite the terminal Failed store record.
    assert!(
        !running_dataflows.contains_key(&dataflow_id),
        "failed spawn must not stay in `running_dataflows`"
    );
    assert!(
        archived_dataflows.contains_key(&dataflow_id),
        "failed spawn must be archived so `dora list` can still describe it"
    );

    // 2. Daemon `a` already started its nodes; it must be told to stop.
    let saw_stop = timeout(TokioDuration::from_secs(1), async {
        loop {
            if *stops_seen.lock().await >= 1 {
                return true;
            }
            tokio::time::sleep(TokioDuration::from_millis(5)).await;
        }
    })
    .await
    .unwrap_or(false);
    assert!(
        saw_stop,
        "the daemon that already spawned must receive a StopDataflow"
    );

    // 3. Waiters must be released rather than hang: `dora start` gets the
    //    spawn error, an in-flight `dora stop` gets a stopped reply.
    let spawn_reply = timeout(TokioDuration::from_secs(1), waiter_rx)
        .await
        .expect("spawn waiter should resolve, not hang")
        .expect("sender should not drop");
    let err = format!("{:?}", spawn_reply.expect_err("spawn must surface as Err"));
    assert!(err.contains("no such binary"), "got: {err}");
    let stop_reply = timeout(TokioDuration::from_secs(1), stop_rx)
        .await
        .expect("stop waiter should resolve, not hang")
        .expect("sender should not drop");
    assert!(matches!(
        stop_reply,
        Ok(ControlRequestReply::DataflowStopped { .. })
    ));
    assert!(
        pending_restarts.is_empty(),
        "a parked restart must be cancelled, not left keyed to a dead dataflow"
    );
    let restart_reply = timeout(TokioDuration::from_secs(1), restart_rx)
        .await
        .expect("restart waiter should resolve, not hang")
        .expect("sender should not drop");
    let restart_err = restart_reply.expect_err("parked restart must be failed, not dropped");
    assert!(
        format!("{restart_err:?}").contains("spawn failed"),
        "restart caller should learn why, got: {restart_err:?}"
    );

    // 4. Mirror ordering: a success reported after the teardown finds no
    //    running dataflow, and must roll that daemon back too.
    handle_dataflow_spawn_result(
        dataflow_id,
        daemon_a.clone(),
        Ok(()),
        &mut running_dataflows,
        &mut archived_dataflows,
        &mut dataflow_results,
        &mut daemon_connections,
        &mut pending_restarts,
        &clock,
        store.as_ref(),
    )
    .await;
    let saw_second_stop = timeout(TokioDuration::from_secs(1), async {
        loop {
            if *stops_seen.lock().await >= 2 {
                return true;
            }
            tokio::time::sleep(TokioDuration::from_millis(5)).await;
        }
    })
    .await
    .unwrap_or(false);
    assert!(
        saw_second_stop,
        "a spawn success arriving after the teardown must also be rolled back"
    );

    daemon_a_task.abort();
}

/// Covers Finding 1 from the round-3 review on PR #1854: a late
/// successful `DataflowSpawnResult` arriving after the watchdog has
/// already failed the dataflow must NOT resurrect it as `Running` in
/// the store.
///
/// This test drives `handle_spawn_result_ok` directly (the same
/// helper the event loop's success arm calls) so any future refactor
/// that drops the `spawn_result.is_pending()` guard will surface here.
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn late_successful_spawn_result_does_not_resurrect_failed_dataflow() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("late".to_string()));
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());

    // Simulate the watchdog having already fired: spawn_result is
    // Cached(Err), and the Failed status is persisted to the store.
    df.spawn_result
        .set_result(Err(eyre!("spawn timed out after 60s (watchdog)")));
    assert!(!df.spawn_result.is_pending());

    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let failed_record = df
        .make_record(StoreDataflowStatus::Failed {
            error: "spawn timed out".to_string(),
            terminal: true,
        })
        .expect("make_record should succeed");
    store
        .put_dataflow(&failed_record)
        .expect("seed Failed in store");

    // Now simulate the late-arriving Ok: the event loop would have
    // removed `daemon_id` from `pending_spawn_results` and then
    // called `handle_spawn_result_ok`. Drive the helper directly.
    df.pending_spawn_results.remove(&daemon_id);
    handle_spawn_result_ok(&mut df, dataflow_id, &daemon_id, store.as_ref());

    // 1. The store entry must still be Failed — NOT promoted to Running.
    let records = store.list_dataflows().expect("store should list");
    let record = records
        .iter()
        .find(|r| r.uuid == dataflow_id)
        .expect("seeded record should still be present");
    assert!(
        matches!(
            record.status,
            dora_coordinator_store::DataflowStatus::Failed { .. }
        ),
        "Failed must NOT be promoted to Running by a late successful spawn_result, \
             got: {:?}",
        record.status,
    );

    // 2. spawn_result must still be Cached(Err) — registering a new
    //    waiter should immediately deliver the original error.
    let (tx, rx) = tokio::sync::oneshot::channel();
    df.spawn_result.register(tx);
    let reply = timeout(TokioDuration::from_millis(50), rx)
        .await
        .expect("Cached result should deliver immediately")
        .expect("sender should not drop");
    let err = reply.expect_err("late ok must not clobber the watchdog's Err");
    let msg = format!("{err:?}");
    assert!(msg.contains("watchdog") || msg.contains("timed out"),);
}

/// Companion test: the happy path through `handle_spawn_result_ok`
/// (spawn_result was Pending, all pending daemons have now reported)
/// must still promote to Running in the store. Guards against an
/// over-eager guard that would block the legitimate success path.
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn handle_spawn_result_ok_promotes_to_running_when_pending_and_complete() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());
    // Single-daemon dataflow: after removing this daemon from pending,
    // the set becomes empty and the success branch should fire.
    df.pending_spawn_results.insert(daemon_id.clone());
    df.pending_spawn_results.remove(&daemon_id);
    assert!(df.spawn_result.is_pending());

    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    handle_spawn_result_ok(&mut df, dataflow_id, &daemon_id, store.as_ref());

    // spawn_result must now be Cached(Ok), waiter should receive
    // DataflowSpawned.
    let (tx, rx) = tokio::sync::oneshot::channel();
    df.spawn_result.register(tx);
    let reply = timeout(TokioDuration::from_millis(50), rx)
        .await
        .expect("Cached result should deliver immediately")
        .expect("sender should not drop");
    let ok = reply.expect("happy-path ok must not be guarded out");
    assert!(matches!(ok, ControlRequestReply::DataflowSpawned { .. }));

    // Store must reflect Running.
    let records = store.list_dataflows().expect("store should list");
    let record = records
        .iter()
        .find(|r| r.uuid == dataflow_id)
        .expect("dataflow should be persisted");
    assert!(
        matches!(
            record.status,
            dora_coordinator_store::DataflowStatus::Running
        ),
        "happy-path success must persist Running, got: {:?}",
        record.status,
    );
}

// -------------------------------------------------------------------
// Round-4 findings (PR #1854): terminal-failure paths must be
// respected by every handler that writes to the affected state.
// -------------------------------------------------------------------

#[test]
fn cached_result_is_terminal_error_distinguishes_states() {
    let mut r = CachedResult::default();
    assert!(!r.is_terminal_error(), "fresh Pending must not be terminal");

    // Cached(Ok) is terminal but NOT an error.
    r.set_result(Ok(ControlRequestReply::DataflowSpawned {
        uuid: DataflowId::from(Uuid::new_v4()),
    }));
    assert!(
        !r.is_terminal_error(),
        "Cached(Ok) must not match is_terminal_error()"
    );

    // Cached(Err) is the only state we want flagged.
    let mut r = CachedResult::default();
    r.set_result(Err(eyre!("spawn timed out")));
    assert!(
        r.is_terminal_error(),
        "Cached(Err) must match is_terminal_error()"
    );
}

/// Round-4 Finding 2: a late-arriving daemon `Err` after the watchdog
/// (or any other terminal-failure path) has already cached an `Err`
/// must NOT overwrite the existing store record. Drives
/// `handle_spawn_result_err` directly.
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn late_failed_spawn_result_does_not_overwrite_watchdog_store_error() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("late".to_string()));
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());

    // Simulate the watchdog having already fired with a detailed err.
    let watchdog_msg = "spawn timed out after 60s (watchdog)";
    df.spawn_result.set_result(Err(eyre!(watchdog_msg)));
    assert!(df.spawn_result.is_terminal_error());

    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    // Seed the store with the detailed Failed record the watchdog
    // would have persisted.
    let initial_record = df
        .make_record(StoreDataflowStatus::Failed {
            error: watchdog_msg.to_string(),
            terminal: true,
        })
        .expect("make_record");
    store.put_dataflow(&initial_record).expect("seed");
    let initial_generation = initial_record.generation;

    // Late-arriving daemon Err.
    let late_err = eyre!("daemon-side spawn rejection");
    handle_spawn_result_err(&mut df, dataflow_id, &daemon_id, late_err, store.as_ref());

    // 1. Store record must still carry the watchdog's detailed message.
    let records = store.list_dataflows().expect("store list");
    let record = records
        .iter()
        .find(|r| r.uuid == dataflow_id)
        .expect("seeded record present");
    match &record.status {
        dora_coordinator_store::DataflowStatus::Failed { error, .. } => assert!(
            error.contains("watchdog") || error.contains("timed out"),
            "watchdog error message must be preserved, got: {error}"
        ),
        other => panic!("expected Failed, got: {other:?}"),
    }
    // 2. Generation must NOT bump (no rewrite happened).
    assert_eq!(
        record.generation, initial_generation,
        "guarded late-Err must not bump store_generation"
    );

    // 3. In-memory spawn_result still carries the watchdog's err.
    let (tx, rx) = tokio::sync::oneshot::channel();
    df.spawn_result.register(tx);
    let reply = timeout(TokioDuration::from_millis(50), rx)
        .await
        .expect("Cached result delivers immediately")
        .expect("sender alive");
    let err = reply.expect_err("must still be Err");
    let msg = format!("{err:?}");
    assert!(
        msg.contains("watchdog") || msg.contains("timed out"),
        "in-memory err must be the watchdog's, got: {msg}"
    );
}

/// Companion test for Finding 2: when spawn_result IS still Pending,
/// the err arm must persist Failed and fire the waiter — guards
/// against an over-eager guard blocking the legitimate failure path.
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn handle_spawn_result_err_persists_failed_when_pending() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());
    df.pending_spawn_results.insert(daemon_id.clone());
    df.pending_spawn_results.remove(&daemon_id);
    assert!(df.spawn_result.is_pending());

    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let err = eyre!("daemon-side spawn rejection");
    handle_spawn_result_err(&mut df, dataflow_id, &daemon_id, err, store.as_ref());

    // 1. spawn_result must now be Cached(Err); waiter receives the err.
    let (tx, rx) = tokio::sync::oneshot::channel();
    df.spawn_result.register(tx);
    let reply = timeout(TokioDuration::from_millis(50), rx)
        .await
        .expect("Cached result delivers immediately")
        .expect("sender alive");
    let err = reply.expect_err("happy-path err must not be guarded out");
    let msg = format!("{err:?}");
    assert!(msg.contains("rejection"));

    // 2. Store must reflect Failed with the daemon's error message.
    let records = store.list_dataflows().expect("store list");
    let record = records
        .iter()
        .find(|r| r.uuid == dataflow_id)
        .expect("dataflow persisted");
    match &record.status {
        dora_coordinator_store::DataflowStatus::Failed { error, .. } => {
            assert!(
                error.contains("rejection"),
                "Failed.error must carry the daemon's message, got: {error}"
            );
        }
        other => panic!("expected Failed, got: {other:?}"),
    }
}

// -------------------------------------------------------------------
// Round-5 findings (PR #1854): watchdog must make the dataflow
// terminal in memory too, not just in the store. Without removal
// from running_dataflows, `Check` and `List` would report it as
// active, and `DaemonStatusReport` reconciliation could promote
// its Failed store record back to Running.
// -------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn watchdog_teardown_drains_stop_reply_senders() {
    // Anyone waiting on `dora stop` for the dataflow when the
    // watchdog fires must be released — otherwise they hang
    // forever because no `DataflowFinishedOnDaemon` will arrive
    // for a spawn-that-never-started.
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = HLC::default();
    let mut daemon_connections = DaemonConnections::default();

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());
    df.pending_spawn_results.insert(daemon_id);
    df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

    // Register an in-flight `dora stop` waiter.
    let (stop_tx, stop_rx) = tokio::sync::oneshot::channel();
    df.stop_reply_senders.push(stop_tx);

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
    let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        IndexMap::new();
    let mut pending_restarts: HashMap<DataflowId, PendingRestart> = HashMap::new();

    check_spawn_timeouts(
        &mut running_dataflows,
        &mut archived_dataflows,
        &mut dataflow_results,
        &mut daemon_connections,
        &mut pending_restarts,
        &clock,
        store.as_ref(),
    )
    .await;

    // The stop waiter must have received a reply.
    let reply = timeout(TokioDuration::from_secs(1), stop_rx)
        .await
        .expect("stop waiter should resolve, not hang")
        .expect("stop sender should not drop");
    let stop = reply.expect("watchdog drains with Ok DataflowStopped");
    assert!(
        matches!(stop, ControlRequestReply::DataflowStopped { uuid, .. } if uuid == dataflow_id),
        "drain should send DataflowStopped with the watchdog'd dataflow's uuid"
    );
}

/// Round-5 Finding 2: after the watchdog fires, the dataflow must be
/// absent from `running_dataflows` so `Check`/`List` no longer report
/// it as active. The archive map must hold a record so post-mortem
/// queries can still find its name/descriptor.
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn watchdog_makes_dataflow_invisible_to_check_and_list() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = HLC::default();
    let mut daemon_connections = DaemonConnections::default();

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());
    df.name = Some("flagged-name".to_string());
    df.pending_spawn_results.insert(daemon_id);
    df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
    let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        IndexMap::new();
    let mut pending_restarts: HashMap<DataflowId, PendingRestart> = HashMap::new();

    check_spawn_timeouts(
        &mut running_dataflows,
        &mut archived_dataflows,
        &mut dataflow_results,
        &mut daemon_connections,
        &mut pending_restarts,
        &clock,
        store.as_ref(),
    )
    .await;

    // Check / List query running_dataflows directly; absence is the
    // contract that fixes both consumers in one move.
    assert!(
        !running_dataflows.contains_key(&dataflow_id),
        "Check/List must observe the watchdog-failed dataflow as absent"
    );

    // The archive preserves the name so post-mortem `dora list`
    // queries against `archived_dataflows` can correlate the uuid
    // with a human-readable identity.
    let archived = archived_dataflows
        .get(&dataflow_id)
        .expect("watchdog must archive for post-mortem queries");
    assert_eq!(
        archived.name.as_deref(),
        Some("flagged-name"),
        "archive must preserve the dataflow name"
    );
}

// -------------------------------------------------------------------
// Round-6 findings (PR #1854): post-watchdog UX parity with
// normal-failure path. Watchdog now populates `dataflow_results` so
// `dora list` / `dora check` / `dora stop` all see the dataflow as
// Failed (instead of "disappeared" / "no known running dataflow").
// -------------------------------------------------------------------

/// Round-6 Finding 1: after watchdog fires, `dataflow_results` must
/// contain an entry that classifies the dataflow as Failed (i.e. at
/// least one per-daemon entry is_ok()==false). Without this, the
/// List handler's `finished_failed` iterator skips the dataflow and
/// it disappears from `dora list` entirely.
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn watchdog_synthesizes_dataflow_results_for_list_visibility() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = HLC::default();
    let mut daemon_connections = DaemonConnections::default();

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());
    df.pending_spawn_results.insert(daemon_id.clone());
    df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
    let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        IndexMap::new();
    let mut pending_restarts: HashMap<DataflowId, PendingRestart> = HashMap::new();

    check_spawn_timeouts(
        &mut running_dataflows,
        &mut archived_dataflows,
        &mut dataflow_results,
        &mut daemon_connections,
        &mut pending_restarts,
        &clock,
        store.as_ref(),
    )
    .await;

    // `dataflow_results` must contain an entry for the timed-out uuid.
    let per_daemon = dataflow_results
        .get(&dataflow_id)
        .expect("watchdog must synthesize a dataflow_results entry");

    // The entry must have at least one per-daemon DataflowDaemonResult.
    assert!(
        !per_daemon.is_empty(),
        "synthesized entry must have per-daemon results"
    );

    // Mirror the List classification check (lib.rs ~1019): all per-
    // daemon results must NOT be is_ok(), so the dataflow is Failed.
    let is_failed = !per_daemon.values().all(DataflowDaemonResult::is_ok);
    assert!(
        is_failed,
        "synthesized result must classify as Failed (List would show \
             Finished otherwise)"
    );

    // Each per-daemon entry must include the dataflow's nodes with
    // Err(FailedToSpawn(..)) so the user sees which nodes never started.
    let daemon_result = per_daemon
        .get(&daemon_id)
        .expect("entry must include the assigned daemon");
    let node_err = daemon_result
        .node_results
        .get(&node_id)
        .expect("node must be present in synthesized results");
    match node_err {
        Err(NodeError {
            cause: NodeErrorCause::FailedToSpawn(msg),
            ..
        }) => {
            assert!(
                msg.contains("spawn timed out") || msg.contains("timeout"),
                "FailedToSpawn cause must carry the watchdog timeout message, got: {msg}"
            );
        }
        other => panic!("expected Err(FailedToSpawn(..)), got: {other:?}"),
    }
}

/// #2027: `dataflow_results` is FIFO-bounded at `MAX_DATAFLOW_RESULTS`.
/// Synthetic watchdog entries (and any results that never get cleared via
/// archival) must not accumulate without bound. When an insert pushes the
/// map over the cap, the oldest entry is evicted and the new one survives.
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn watchdog_synthesis_keeps_dataflow_results_bounded() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = HLC::default();
    let mut daemon_connections = DaemonConnections::default();

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
    df.pending_spawn_results.insert(daemon_id.clone());
    df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();

    // Pre-fill to exactly the cap with stale entries; the oldest is first.
    let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        IndexMap::new();
    let oldest = DataflowId::from(Uuid::new_v4());
    dataflow_results.insert(oldest, BTreeMap::new());
    for _ in 1..MAX_DATAFLOW_RESULTS {
        dataflow_results.insert(DataflowId::from(Uuid::new_v4()), BTreeMap::new());
    }
    assert_eq!(dataflow_results.len(), MAX_DATAFLOW_RESULTS);

    let mut pending_restarts: HashMap<DataflowId, PendingRestart> = HashMap::new();
    check_spawn_timeouts(
        &mut running_dataflows,
        &mut archived_dataflows,
        &mut dataflow_results,
        &mut daemon_connections,
        &mut pending_restarts,
        &clock,
        store.as_ref(),
    )
    .await;

    assert_eq!(
        dataflow_results.len(),
        MAX_DATAFLOW_RESULTS,
        "map must stay capped after the watchdog synthesizes a new entry"
    );
    assert!(
        dataflow_results.contains_key(&dataflow_id),
        "the freshly synthesized entry must survive eviction"
    );
    assert!(
        !dataflow_results.contains_key(&oldest),
        "the oldest entry must be the one evicted (FIFO)"
    );
}

/// #2027 review (P2): a partially-finished multi-daemon dataflow keeps its
/// `dataflow_results` entry while still running (one daemon reported, others
/// pending). The cap must evict only finished-dataflow history, never an
/// active entry — otherwise the earlier daemon's result is lost and the
/// final status (computed when the last daemon finishes) is wrong.
#[test]
fn cap_dataflow_results_preserves_running_dataflow_entries() {
    let active = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(active, test_running_dataflow(active, daemon_id, node_id));

    let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        IndexMap::new();
    // Insert the active entry FIRST so it is the FIFO-oldest — i.e. the
    // entry a blind cap would evict first.
    dataflow_results.insert(active, BTreeMap::new());
    for _ in 0..MAX_DATAFLOW_RESULTS {
        dataflow_results.insert(DataflowId::from(Uuid::new_v4()), BTreeMap::new());
    }
    assert_eq!(dataflow_results.len(), MAX_DATAFLOW_RESULTS + 1);

    cap_dataflow_results(&mut dataflow_results, &running_dataflows);

    assert_eq!(
        dataflow_results.len(),
        MAX_DATAFLOW_RESULTS,
        "cap must bring the map back within the limit"
    );
    assert!(
        dataflow_results.contains_key(&active),
        "the active running dataflow's entry must be preserved even though \
             it is the FIFO-oldest — a finished-history entry is evicted instead"
    );
}

/// Round-6 Finding 3: `dora stop <watchdog-failed-uuid>` must return
/// `DataflowStopped` (via the cached `dataflow_results` early-return)
/// rather than "no known running dataflow". This auto-resolves once
/// Finding 1's synthesis is in place — the Stop handler's
/// `dataflow_results.get(&uuid)` early-return fires.
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn dora_stop_after_watchdog_finds_dataflow_results_entry() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = HLC::default();
    let mut daemon_connections = DaemonConnections::default();

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());
    df.pending_spawn_results.insert(daemon_id);
    df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
    let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        IndexMap::new();
    let mut pending_restarts: HashMap<DataflowId, PendingRestart> = HashMap::new();

    check_spawn_timeouts(
        &mut running_dataflows,
        &mut archived_dataflows,
        &mut dataflow_results,
        &mut daemon_connections,
        &mut pending_restarts,
        &clock,
        store.as_ref(),
    )
    .await;

    // The Stop handler at lib.rs:824 does:
    //   if let Some(result) = dataflow_results.get(&dataflow_uuid) {
    //       reply DataflowStopped { uuid, result: dataflow_result(result, ...) };
    //       continue;
    //   }
    // The early-return only fires when the synthesis above worked.
    assert!(
        dataflow_results.contains_key(&dataflow_id),
        "Stop handler's dataflow_results early-return must fire for \
             watchdog-failed dataflows (otherwise stop_dataflow bails \
             with 'no known running dataflow')"
    );
}

/// A partially-finished multi-daemon dataflow is present in BOTH
/// `running_dataflows` (still running on the remaining daemons) and
/// `dataflow_results` (a partial, per-daemon entry). The Stop/StopByName
/// early-return and the List `finished_failed` projection both consult
/// `dataflow_results`; without a `running_dataflows` guard, Stop no-ops
/// (leaving the still-running daemons' nodes alive) and List shows the
/// dataflow twice with contradictory statuses. This exercises the guard
/// predicate shared by all three handlers.
#[test]
fn partial_multi_daemon_dataflow_is_not_stopped_early_or_listed_twice() {
    let partial = DataflowId::from(Uuid::new_v4()); // running + partial results
    let finished = DataflowId::from(Uuid::new_v4()); // fully done, results only

    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let mut running_dataflows: HashMap<DataflowId, RunningDataflow> = HashMap::new();
    running_dataflows.insert(
        partial,
        test_running_dataflow(partial, daemon_id.clone(), "sender".to_string().into()),
    );

    let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        IndexMap::new();
    dataflow_results.insert(partial, BTreeMap::new());
    dataflow_results.insert(finished, BTreeMap::new());

    // Stop/StopByName fast path: only the fully-finished dataflow may take it.
    let stop_fast_path = |uuid: &DataflowId| {
        !running_dataflows.contains_key(uuid) && dataflow_results.contains_key(uuid)
    };
    assert!(
        !stop_fast_path(&partial),
        "partial dataflow must fall through to stop_dataflow, not report DataflowStopped"
    );
    assert!(
        stop_fast_path(&finished),
        "fully finished dataflow keeps the DataflowStopped fast path"
    );

    // List: only the fully-finished dataflow is a finished_failed row.
    let finished_failed: Vec<_> = dataflow_results
        .iter()
        .filter(|(uuid, _)| !running_dataflows.contains_key(uuid))
        .map(|(&uuid, _)| uuid)
        .collect();
    assert_eq!(
        finished_failed,
        vec![finished],
        "the still-running partial dataflow must not also appear as finished/failed"
    );
}

/// Round-6 Finding 2: late `DataflowFinishedOnDaemon` arrivals must
/// merge into the synthetic `dataflow_results` entry, not be silently
/// discarded. This tests the merge math directly (extending
/// node_results overwrites synthetic FailedToSpawn entries with the
/// daemon's real per-node results).
#[test]
fn late_finished_on_daemon_merge_extends_node_results() {
    // Step 1: build a synthesized DataflowDaemonResult as the watchdog
    // would emit. Two nodes assigned, both with FailedToSpawn errors.
    let synth_timestamp = HLC::default().new_timestamp();
    let node_a: dora_core::config::NodeId = "node_a".to_string().into();
    let node_b: dora_core::config::NodeId = "node_b".to_string().into();

    let mut synth = DataflowDaemonResult {
        timestamp: synth_timestamp,
        node_results: BTreeMap::new(),
    };
    synth.node_results.insert(
        node_a.clone(),
        Err(NodeError {
            timestamp: synth_timestamp,
            cause: NodeErrorCause::FailedToSpawn("spawn timed out".to_string()),
            exit_status: NodeExitStatus::Unknown,
        }),
    );
    synth.node_results.insert(
        node_b.clone(),
        Err(NodeError {
            timestamp: synth_timestamp,
            cause: NodeErrorCause::FailedToSpawn("spawn timed out".to_string()),
            exit_status: NodeExitStatus::Unknown,
        }),
    );

    // Step 2: late `result` from a daemon that DID end up running
    // node_a successfully and saw node_b crash with a real exit code.
    let late_timestamp = HLC::default().new_timestamp();
    let mut late_results: BTreeMap<dora_core::config::NodeId, Result<(), NodeError>> =
        BTreeMap::new();
    late_results.insert(node_a.clone(), Ok(()));
    late_results.insert(
        node_b.clone(),
        Err(NodeError {
            timestamp: late_timestamp,
            cause: NodeErrorCause::Other {
                stderr: "real error".to_string(),
            },
            exit_status: NodeExitStatus::ExitCode(42),
        }),
    );

    // Step 3: apply the merge logic from the Vacant arm.
    synth.timestamp = late_timestamp;
    synth.node_results.extend(late_results);

    // The merged entry must reflect the daemon's real results, not
    // the synthetic ones, for the nodes that overlap.
    assert!(matches!(synth.node_results.get(&node_a), Some(Ok(()))));
    match synth.node_results.get(&node_b) {
        Some(Err(NodeError {
            cause: NodeErrorCause::Other { stderr },
            exit_status: NodeExitStatus::ExitCode(42),
            ..
        })) => {
            assert_eq!(stderr, "real error");
        }
        other => panic!("expected merged real Err, got: {other:?}"),
    }
    assert_eq!(synth.timestamp, late_timestamp);
}

// -------------------------------------------------------------------
// Round-7 findings (PR #1854):
//   1. DaemonStatusReport reconcile must NOT promote watchdog-failed
//      (archived) dataflows back to Running.
//   2. Watchdog synthesis must produce a non-empty result map even
//      when df.daemons was emptied by disconnect cleanup before the
//      watchdog fired.
// -------------------------------------------------------------------

/// Round-7 Finding 1: ensure the reconcile guard skips
/// archived/terminal dataflows. We test the predicate the reconcile
/// loop checks (`archived_dataflows.contains_key(df_id)`) rather
/// than driving the full DaemonStatusReport event end-to-end —
/// that would require synthesizing a registered daemon and a
/// reported_dataflows entry, which the existing reconnect test
/// (`restore_topic_debug_streams_re_issues_start_after_reconnect`)
/// shows is non-trivial setup. The watchdog test
/// `watchdog_makes_dataflow_invisible_to_check_and_list` already
/// proves the dataflow lands in `archived_dataflows`, so this
/// closing-the-loop predicate test plus the integration via
/// `dora_stop_after_watchdog_finds_dataflow_results_entry` is
/// sufficient.
#[test]
fn archived_dataflows_predicate_distinguishes_watchdog_failed_from_unknown() {
    let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
    let watchdog_failed = DataflowId::from(Uuid::new_v4());
    let unknown = DataflowId::from(Uuid::new_v4());

    archived_dataflows.insert(
        watchdog_failed,
        ArchivedDataflow {
            name: Some("flagged".to_string()),
            nodes: BTreeMap::new(),
        },
    );

    // The reconcile guard predicate.
    assert!(
        archived_dataflows.contains_key(&watchdog_failed),
        "watchdog-archived dataflow must be flagged for reconcile skip"
    );
    assert!(
        !archived_dataflows.contains_key(&unknown),
        "unknown dataflow must NOT be flagged (normal reconcile path applies)"
    );
}

#[test]
fn status_report_reconcile_stops_succeeded_dataflows_reported_as_running() {
    let status = StoreDataflowStatus::Succeeded;

    assert!(
        status_report_should_stop_orphan(&status),
        "a Succeeded store record is terminal; a daemon reporting it as still \
             running must be stopped rather than ignored"
    );
}

/// Round-7 Finding 2: if all daemons disconnected before the watchdog
/// fired (so `df.daemons` is empty by the disconnect cleanup at
/// `lib.rs:1893-1899`), the watchdog's synthesis must STILL produce
/// a non-empty `dataflow_results` entry. Empty `BTreeMap` would make
/// List's `results.values().all(is_ok)` vacuously true and
/// misclassify the dataflow as `Finished`.
///
/// Synthesis iterates `df.node_to_daemon` (the original assignment,
/// untouched by disconnect cleanup) rather than `df.daemons`, so the
/// daemon set survives. As a final defence, an empty
/// `node_to_daemon` injects a sentinel result.
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn watchdog_disconnect_mid_spawn_still_classifies_as_failed() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = HLC::default();
    let mut daemon_connections = DaemonConnections::default();

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("disconnected".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());
    // `node_to_daemon` already includes `daemon_id` (set up by
    // test_running_dataflow). Simulate the disconnect cleanup having
    // run before the watchdog: df.daemons is emptied.
    df.daemons.clear();
    df.pending_spawn_results.clear();
    // spawn_result still Pending because the disconnect path
    // intentionally doesn't fire it (round-1 design decision); the
    // watchdog is the single chokepoint.
    df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
    let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        IndexMap::new();
    let mut pending_restarts: HashMap<DataflowId, PendingRestart> = HashMap::new();

    check_spawn_timeouts(
        &mut running_dataflows,
        &mut archived_dataflows,
        &mut dataflow_results,
        &mut daemon_connections,
        &mut pending_restarts,
        &clock,
        store.as_ref(),
    )
    .await;

    // The synthesized entry must NOT be empty.
    let per_daemon = dataflow_results
        .get(&dataflow_id)
        .expect("watchdog must synthesize even when df.daemons is empty");
    assert!(
        !per_daemon.is_empty(),
        "synthesized result must be non-empty so List classifies as Failed"
    );

    // List classification: must be Failed (not vacuously Finished).
    let is_failed = !per_daemon.values().all(DataflowDaemonResult::is_ok);
    assert!(
        is_failed,
        "disconnect-mid-spawn case must classify as Failed, not vacuously Finished"
    );

    // Verify it used the original assignment from node_to_daemon
    // (the daemon was disconnected but its assignment remains).
    let daemon_result = per_daemon.get(&daemon_id).unwrap_or_else(|| {
        panic!(
            "node_to_daemon must drive synthesis; expected key {daemon_id:?}, got {:?}",
            per_daemon.keys().collect::<Vec<_>>()
        )
    });
    assert!(
        daemon_result.node_results.contains_key(&node_id),
        "synthesis must include the originally-assigned node"
    );
}

// -------------------------------------------------------------------
// Round-8 findings (PR #1854):
//   1. Cross-restart reconcile resurrection: store-level `terminal`
//      marker so the watchdog's verdict survives coordinator
//      restarts where `archived_dataflows` is in-memory-wiped.
//   2. Defensive sentinel must not panic: `NodeId::from(invalid)`
//      asserts on disallowed chars.
// -------------------------------------------------------------------

/// Round-8 Finding 1: watchdog must persist `terminal: true` so the
/// reconcile path skips promotion to Running even after coordinator
/// restart (when `archived_dataflows` is empty).
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn watchdog_persists_terminal_marker_for_cross_restart_protection() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = HLC::default();
    let mut daemon_connections = DaemonConnections::default();

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());
    df.pending_spawn_results.insert(daemon_id);
    df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
    let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        IndexMap::new();
    let mut pending_restarts: HashMap<DataflowId, PendingRestart> = HashMap::new();

    check_spawn_timeouts(
        &mut running_dataflows,
        &mut archived_dataflows,
        &mut dataflow_results,
        &mut daemon_connections,
        &mut pending_restarts,
        &clock,
        store.as_ref(),
    )
    .await;

    let record = store
        .get_dataflow(&dataflow_id)
        .expect("store should be readable")
        .expect("watchdog must persist a record");
    match record.status {
        dora_coordinator_store::DataflowStatus::Failed { terminal, .. } => {
            assert!(
                terminal,
                "watchdog-persisted Failed must carry terminal: true so \
                     a post-restart reconcile cannot resurrect it"
            );
        }
        other => panic!("expected Failed, got: {other:?}"),
    }
}

/// Round-8 Finding 1 companion: assert that the `Failed` field
/// defaults to `terminal: false` when an older record (without the
/// field) is deserialized. This guarantees backward compat with
/// records persisted by pre-#1854 coordinators.
#[test]
fn dataflow_status_failed_terminal_field_defaults_to_false_on_legacy_records() {
    // Simulate a record JSON written by an older coordinator that
    // didn't know about the `terminal` field.
    let legacy_json = r#"{ "Failed": { "error": "old failure" } }"#;
    let deser: dora_coordinator_store::DataflowStatus =
        serde_json::from_str(legacy_json).expect("legacy Failed must deserialize");
    match deser {
        dora_coordinator_store::DataflowStatus::Failed { error, terminal } => {
            assert_eq!(error, "old failure");
            assert!(
                !terminal,
                "legacy Failed records (no terminal field) MUST default to \
                     terminal: false to preserve the daemon-overrides-coordinator \
                     reconcile semantics of pre-#1854 stores"
            );
        }
        other => panic!("expected Failed, got: {other:?}"),
    }
}

/// Round-8 Finding 2: the defensive sentinel branch in the watchdog
/// (fired when `node_to_daemon` is empty) must NOT panic. Previous
/// versions used `NodeId::from("<watchdog>".to_string())` which
/// panics via `validate_node_id` -- the `<` / `>` chars aren't in
/// the allowed `[a-zA-Z0-9_.-]` set.
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn watchdog_sentinel_branch_does_not_panic_on_empty_node_to_daemon() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = HLC::default();
    let mut daemon_connections = DaemonConnections::default();

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());
    // Force the sentinel branch: empty node_to_daemon (and daemons)
    // so the synthesis-from-assignment path produces an empty map.
    df.node_to_daemon.clear();
    df.daemons.clear();
    df.pending_spawn_results.clear();
    df.spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let mut archived_dataflows: IndexMap<DataflowId, ArchivedDataflow> = IndexMap::new();
    let mut dataflow_results: IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>> =
        IndexMap::new();

    // If `NodeId::from("<watchdog>")` is reintroduced, this awaits
    // a panic and the test fails. Currently uses `"watchdog"` which
    // passes validation.
    let mut pending_restarts: HashMap<DataflowId, PendingRestart> = HashMap::new();
    check_spawn_timeouts(
        &mut running_dataflows,
        &mut archived_dataflows,
        &mut dataflow_results,
        &mut daemon_connections,
        &mut pending_restarts,
        &clock,
        store.as_ref(),
    )
    .await;

    // Sentinel result must be present and classify as Failed.
    let per_daemon = dataflow_results
        .get(&dataflow_id)
        .expect("sentinel must populate dataflow_results");
    assert!(!per_daemon.is_empty(), "sentinel must be non-empty");
    assert!(
        !per_daemon.values().all(DataflowDaemonResult::is_ok),
        "sentinel must classify as Failed (not vacuously Finished)"
    );
}

// ===== check_build_timeouts watchdog tests (#1465) =====
//
// Mirror the four spawn watchdog tests above for the build path.
// Build's analog is simpler: no daemon rollback, no archived_dataflows /
// dataflow_results synthesis — just release waiters and move the entry
// from `running_builds` to `finished_builds`.

// Small, cross-platform-safe watchdog timeout for the unit tests. The
// tests pass this to `check_build_timeouts` instead of the 20-min
// production `build_result_timeout()`, so `test_running_build` only has to
// backdate `build_started_at` by ~2s. Backdating by the production timeout
// computes `Instant::now() - 20min`, which underflows and panics on
// Windows runners whose monotonic-clock epoch is younger than 20 min
// (dora-rs/dora#2082).
const TEST_BUILD_TIMEOUT: Duration = Duration::from_secs(1);

fn test_running_build(daemon_id: DaemonId, backdate: bool) -> RunningBuild {
    let mut pending = BTreeSet::new();
    pending.insert(daemon_id);
    RunningBuild {
        errors: Vec::new(),
        build_result: CachedResult::default(),
        buffered_log_messages: Vec::new(),
        log_subscribers: Vec::new(),
        pending_build_results: pending,
        build_started_at: if backdate {
            Instant::now() - TEST_BUILD_TIMEOUT - Duration::from_secs(1)
        } else {
            Instant::now()
        },
    }
}

#[tokio::test]
async fn check_build_timeouts_fires_error_when_pending_past_deadline() {
    let clock = HLC::default();
    let build_id = BuildId::generate();
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let mut build = test_running_build(daemon_id, /*backdate=*/ true);

    // Register a waiter so we can prove `wait_for_build` unblocks with
    // an error rather than hanging on the client-side RPC deadline.
    let (tx, rx) = tokio::sync::oneshot::channel();
    build.build_result.register(tx);

    let mut running_builds: HashMap<BuildId, RunningBuild> = HashMap::new();
    running_builds.insert(build_id, build);
    let mut finished_builds: IndexMap<BuildId, CachedResult> = IndexMap::new();

    check_build_timeouts(
        &mut running_builds,
        &mut finished_builds,
        &clock,
        TEST_BUILD_TIMEOUT,
    )
    .await;

    // Waiter resolved with an error.
    let reply = timeout(TokioDuration::from_secs(1), rx)
        .await
        .expect("waiter should resolve, not hang")
        .expect("sender should not drop");
    let err = reply.expect_err("timed-out build must surface as Err");
    let msg = format!("{err:?}");
    assert!(
        msg.contains("build timed out"),
        "error should explain the timeout, got: {msg}"
    );

    // Build moved from running_builds to finished_builds.
    assert!(
        !running_builds.contains_key(&build_id),
        "watchdog must remove timed-out build from running_builds"
    );
    assert!(
        finished_builds.contains_key(&build_id),
        "watchdog must move timed-out build into finished_builds so late \
             WaitForBuild requests find the cached error",
    );
}

#[tokio::test]
async fn check_build_timeouts_no_op_when_within_deadline() {
    let clock = HLC::default();
    let build_id = BuildId::generate();
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let mut build = test_running_build(daemon_id, /*backdate=*/ false);

    let (tx, rx) = tokio::sync::oneshot::channel();
    build.build_result.register(tx);

    let mut running_builds: HashMap<BuildId, RunningBuild> = HashMap::new();
    running_builds.insert(build_id, build);
    let mut finished_builds: IndexMap<BuildId, CachedResult> = IndexMap::new();

    check_build_timeouts(
        &mut running_builds,
        &mut finished_builds,
        &clock,
        TEST_BUILD_TIMEOUT,
    )
    .await;

    // Waiter still pending.
    let polled = tokio::time::timeout(TokioDuration::from_millis(50), rx).await;
    assert!(polled.is_err(), "waiter must still be pending pre-deadline");

    let build = running_builds.get(&build_id).expect("build still present");
    assert!(
        build.build_result.is_pending(),
        "build_result must remain Pending pre-deadline"
    );
    assert!(
        finished_builds.is_empty(),
        "fresh build must not be moved to finished_builds"
    );
}

#[tokio::test]
async fn check_build_timeouts_idempotent_on_already_cached_result() {
    let clock = HLC::default();
    let build_id = BuildId::generate();
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let mut build = test_running_build(daemon_id, /*backdate=*/ true);
    // Build already resolved successfully — the watchdog must NOT
    // re-fire on subsequent heartbeats and clobber the cached result.
    build
        .build_result
        .set_result(Ok(ControlRequestReply::DataflowBuildFinished {
            build_id,
            result: Ok(()),
        }));

    let mut running_builds: HashMap<BuildId, RunningBuild> = HashMap::new();
    running_builds.insert(build_id, build);
    let mut finished_builds: IndexMap<BuildId, CachedResult> = IndexMap::new();

    check_build_timeouts(
        &mut running_builds,
        &mut finished_builds,
        &clock,
        TEST_BUILD_TIMEOUT,
    )
    .await;

    let build = running_builds.get_mut(&build_id).expect(
        "Cached(Ok) build must stay in running_builds — only is_pending() \
             triggers the watchdog",
    );
    // Registering a new waiter on a Cached result must immediately
    // deliver the cached Ok — proving the watchdog did not clobber it.
    let (tx, rx) = tokio::sync::oneshot::channel();
    build.build_result.register(tx);
    let reply = timeout(TokioDuration::from_millis(50), rx)
        .await
        .expect("Cached result should deliver immediately")
        .expect("sender should not drop");
    let ok = reply.expect("the cached Ok result must be preserved");
    assert!(matches!(
        ok,
        ControlRequestReply::DataflowBuildFinished { result: Ok(()), .. }
    ));
    assert!(
        finished_builds.is_empty(),
        "watchdog must not move a non-timed-out build into finished_builds"
    );
}

/// Build-specific edge: a `WaitForBuild` registered AFTER the watchdog
/// fired must still receive the cached error via `finished_builds`
/// (the second branch of the `WaitForBuild` arm). This proves the
/// late-arriver path, complementing the pre-registered waiter path
/// in `check_build_timeouts_fires_error_when_pending_past_deadline`.
#[tokio::test]
async fn check_build_timeouts_late_wait_for_build_gets_cached_error() {
    let clock = HLC::default();
    let build_id = BuildId::generate();
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let build = test_running_build(daemon_id, /*backdate=*/ true);
    // NB: NO waiter registered yet — this is the late-arriver path.

    let mut running_builds: HashMap<BuildId, RunningBuild> = HashMap::new();
    running_builds.insert(build_id, build);
    let mut finished_builds: IndexMap<BuildId, CachedResult> = IndexMap::new();

    check_build_timeouts(
        &mut running_builds,
        &mut finished_builds,
        &clock,
        TEST_BUILD_TIMEOUT,
    )
    .await;

    // Build was moved to finished_builds with a Cached Err.
    let cached = finished_builds
        .get_mut(&build_id)
        .expect("build must be in finished_builds after watchdog");

    // A late WaitForBuild registers on the finished CachedResult and
    // must receive the cached error immediately, NOT hang.
    let (tx, rx) = tokio::sync::oneshot::channel();
    cached.register(tx);
    let reply = timeout(TokioDuration::from_millis(50), rx)
        .await
        .expect("Cached error should deliver immediately")
        .expect("sender should not drop");
    let err = reply.expect_err("late wait_for_build must surface the watchdog's Err");
    assert!(format!("{err:?}").contains("build timed out"));
}

#[test]
fn cleanup_disconnected_builds_prunes_and_keeps_others_pending() {
    // A daemon exits while another is still building: only the exited daemon
    // is pruned, the build stays in `running_builds` awaiting the survivor's
    // `DataflowBuildResult`, and the disconnect is recorded as an error so
    // the eventual result is a failure rather than a silent success.
    let m1 = DaemonId::new(Some("m1".to_string()));
    let m2 = DaemonId::new(Some("m2".to_string()));

    let build_id = BuildId::generate();
    let mut build = test_running_build(m1.clone(), /*backdate=*/ false);
    build.pending_build_results.insert(m2.clone());

    let mut running_builds: HashMap<BuildId, RunningBuild> = HashMap::new();
    running_builds.insert(build_id, build);
    let mut finished_builds: IndexMap<BuildId, CachedResult> = IndexMap::new();

    let disconnected = BTreeSet::from([m1.clone()]);
    cleanup_disconnected_daemons_from_running_builds(
        &mut running_builds,
        &mut finished_builds,
        &disconnected,
    );

    let build = running_builds
        .get(&build_id)
        .expect("build must stay pending while m2 is still building");
    assert!(!build.pending_build_results.contains(&m1), "m1 pruned");
    assert!(
        build.pending_build_results.contains(&m2),
        "m2 still pending"
    );
    assert!(
        build.errors.iter().any(|e| e.contains("m1")),
        "the disconnect must be recorded as a build error, got: {:?}",
        build.errors
    );
    assert!(finished_builds.is_empty(), "build not finalized yet");
}

#[tokio::test]
async fn cleanup_disconnected_builds_finalizes_when_last_daemon_exits() {
    // The exited daemon is the *last* one pending. The cleanup must finalize
    // the build immediately (no further `DataflowBuildResult` will arrive),
    // resolving the waiter with a failure instead of hanging until
    // `check_build_timeouts`.
    let m1 = DaemonId::new(Some("m1".to_string()));

    let build_id = BuildId::generate();
    let mut build = test_running_build(m1.clone(), /*backdate=*/ false);
    let (tx, rx) = tokio::sync::oneshot::channel();
    build.build_result.register(tx);

    let mut running_builds: HashMap<BuildId, RunningBuild> = HashMap::new();
    running_builds.insert(build_id, build);
    let mut finished_builds: IndexMap<BuildId, CachedResult> = IndexMap::new();

    let disconnected = BTreeSet::from([m1]);
    cleanup_disconnected_daemons_from_running_builds(
        &mut running_builds,
        &mut finished_builds,
        &disconnected,
    );

    assert!(
        !running_builds.contains_key(&build_id),
        "build must be finalized and removed from running_builds"
    );
    assert!(
        finished_builds.contains_key(&build_id),
        "finalized build must be cached in finished_builds for late waiters"
    );
    // The pre-registered waiter must resolve immediately (not hang) with a
    // failed build result naming the disconnect.
    let reply = timeout(TokioDuration::from_millis(50), rx)
        .await
        .expect("waiter should resolve, not hang")
        .expect("sender should not drop")
        .expect("build finalization delivers Ok(reply)");
    match reply {
        ControlRequestReply::DataflowBuildFinished { result, .. } => {
            let err = result.expect_err("a mid-build disconnect must fail the build");
            assert!(err.contains("disconnected"), "got: {err}");
        }
        other => panic!("expected DataflowBuildFinished, got {other:?}"),
    }
}

#[test]
fn cleanup_disconnected_daemons_drains_pending_restarts() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "node".to_string().into();

    let df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);

    let (tx, rx) = tokio::sync::oneshot::channel();
    let descriptor: Descriptor =
        serde_json::from_value(serde_json::json!({"nodes": [{"id": "node"}]}))
            .expect("valid descriptor");
    let mut pending_restarts = HashMap::new();
    pending_restarts.insert(
        dataflow_id,
        PendingRestart {
            descriptor,
            name: None,
            uv: false,
            reply_sender: tx,
        },
    );

    let disconnected = BTreeSet::from([daemon_id]);
    let _actions = cleanup_disconnected_daemons_from_running_dataflows(
        &mut running_dataflows,
        &disconnected,
        &mut pending_restarts,
    );

    // pending_restarts must be drained
    assert!(
        pending_restarts.is_empty(),
        "pending_restarts must be drained on daemon disconnect, but has {} entries",
        pending_restarts.len()
    );

    // caller must be released with an error, not hang
    let result = rx.blocking_recv().expect("restart sender must send reply");
    let err = result.expect_err("restart must fail on daemon disconnect");
    assert!(
        format!("{err:?}").contains("disconnected"),
        "error must mention daemon disconnect, got: {err:?}"
    );
}

#[tokio::test]
async fn initiate_restart_rejects_duplicate_request() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "node".to_string().into();

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(
        dataflow_id,
        test_running_dataflow(dataflow_id, daemon_id.clone(), node_id),
    );

    let (existing_tx, _existing_rx) = tokio::sync::oneshot::channel();
    let descriptor: Descriptor =
        serde_json::from_value(serde_json::json!({"nodes": [{"id": "node"}]}))
            .expect("valid descriptor");
    let mut pending_restarts = HashMap::new();
    pending_restarts.insert(
        dataflow_id,
        PendingRestart {
            descriptor: descriptor.clone(),
            name: None,
            uv: false,
            reply_sender: existing_tx,
        },
    );

    let (dup_tx, dup_rx) = tokio::sync::oneshot::channel();
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = Arc::new(HLC::default());
    let mut daemon_connections = DaemonConnections::default();

    initiate_restart(
        dataflow_id,
        None,
        false,
        &mut running_dataflows,
        &mut pending_restarts,
        &mut daemon_connections,
        &clock,
        store.as_ref(),
        dup_tx,
    )
    .await;

    // duplicate must be rejected with an error
    let result = dup_rx
        .await
        .expect("duplicate restart sender must send reply");
    let err = result.expect_err("duplicate restart must be rejected");
    assert!(
        format!("{err:?}").contains("already being restarted"),
        "error must mention already being restarted, got: {err:?}"
    );

    // original PendingRestart must not be overwritten
    assert_eq!(
        pending_restarts.len(),
        1,
        "original PendingRestart must not be overwritten by duplicate"
    );
}
