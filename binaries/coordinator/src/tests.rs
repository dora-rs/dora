use super::*;
use crate::state::ParamTarget;
use crate::topic_debug::topic_outputs_by_daemon;
use crate::{
    daemon_liveness::{
        DisconnectAction, HeartbeatSendOutcome, MAX_CONSECUTIVE_HEARTBEAT_SEND_TIMEOUTS,
        heartbeat_disconnect_decision,
    },
    params::replay_persisted_params_for_daemon,
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

/// A node for `dora node add` / `dora node replace` comes from the user's
/// YAML, and `path_fallback` is internal (only module expansion sets it).
#[test]
fn dynamic_node_cannot_set_path_fallback() {
    let running = running_descriptor_with_operator_producer(serde_json::json!(null));
    let added: Node = serde_json::from_value(serde_json::json!({
        "id": "consumer",
        "path": "modules/tool",
        "path_fallback": "tool",
    }))
    .expect("valid node");

    let err = resolve_single_node(added, &running).expect_err("path_fallback must be rejected");
    assert_eq!(
        format!("{err:#}"),
        "node `consumer` sets `path_fallback`, which is internal \
         (module expansion sets it); remove it"
    );
}

#[test]
fn dynamic_node_with_invalid_timing_is_rejected() {
    // `dora node add` / `dora node replace` never run whole-dataflow
    // validation, so a negative or non-finite timing value used to reach the
    // daemon, where `Duration::from_secs_f64` panics and takes down every
    // dataflow on it. `resolve_single_node` must reject it up front.
    let running = running_descriptor_with_operator_producer(serde_json::json!(null));

    for (field, node) in [
        (
            "health_check_timeout",
            serde_json::json!({
                "id": "consumer",
                "path": "consumer",
                "health_check_timeout": -1.0,
            }),
        ),
        (
            "restart_delay",
            serde_json::json!({
                "id": "consumer",
                "path": "consumer",
                "restart_delay": 1e30,
            }),
        ),
        (
            "input_timeout",
            serde_json::json!({
                "id": "consumer",
                "path": "consumer",
                "inputs": {
                    "reading": { "source": "producer/result", "input_timeout": -1.0 },
                },
            }),
        ),
    ] {
        let added: Node = serde_json::from_value(node).expect("valid node");
        let err = resolve_single_node(added, &running)
            .expect_err("invalid timing must be rejected")
            .to_string();
        assert!(err.contains(field), "error should name `{field}`: {err}");
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
        persisted_placement: BTreeMap::new(),
        node_metrics: BTreeMap::new(),
        node_finalized: BTreeSet::new(),
        exited_nodes: BTreeSet::new(),
        departed_daemons: BTreeMap::new(),
        node_stopped_at: BTreeMap::new(),
        network_metrics: BTreeMap::new(),
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
        uv: false,
        launch: crate::state::LaunchContext::unknown(),
        state_log_sequence: 0,
        state_log: Vec::new(),
        daemon_ack_sequence: BTreeMap::new(),
    }
}

/// A store whose `get_dataflow` always fails (simulating a transient
/// read error), delegating every other method to a backing
/// [`InMemoryStore`]. Used to prove the ready-barrier release is still
/// persisted when the status read fails (#3115). With `fail_param_list`
/// set, `list_node_params` fails as well.
struct FailingReadStore {
    inner: InMemoryStore,
    fail_param_list: bool,
}

impl FailingReadStore {
    fn new() -> Self {
        Self {
            inner: InMemoryStore::new(),
            fail_param_list: false,
        }
    }

    fn with_failing_param_list() -> Self {
        Self {
            fail_param_list: true,
            ..Self::new()
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
        if self.fail_param_list {
            return Err(eyre!("simulated transient store read error"));
        }
        self.inner.list_node_params(dataflow_id, node_id)
    }
    fn delete_node_param(&self, dataflow_id: &Uuid, node_id: &NodeId, key: &str) -> Result<()> {
        self.inner.delete_node_param(dataflow_id, node_id, key)
    }
    fn list_node_param_tombstones(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
    ) -> Result<Vec<(String, Uuid)>> {
        self.inner.list_node_param_tombstones(dataflow_id, node_id)
    }
    fn clear_node_param_tombstone(
        &self,
        dataflow_id: &Uuid,
        node_id: &NodeId,
        key: &str,
        token: Uuid,
    ) -> Result<()> {
        self.inner
            .clear_node_param_tombstone(dataflow_id, node_id, key, token)
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

/// `dora topic pub` must publish under the id the daemons subscribe to: a
/// single-operator runtime node's bare `image` went out unresolved, as
/// `.../control/single/image`, so the check passed and the message was lost.
#[test]
fn topic_publish_resolves_a_single_operator_output_id() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let descriptor: Descriptor = serde_json::from_value(serde_json::json!({
        "nodes": [{
            "id": "single",
            "operator": { "python": "single.py", "outputs": ["image"] },
        }],
    }))
    .expect("valid test descriptor");
    let single_node: dora_core::config::NodeId = "single".to_string().into();
    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id, single_node.clone());
    dataflow.descriptor = descriptor;
    dataflow.nodes = dataflow
        .descriptor
        .resolve_aliases_and_set_defaults()
        .expect("test descriptor should resolve");
    let mut running_dataflows = HashMap::from([(dataflow_id, dataflow)]);
    let publish = |running_dataflows: &HashMap<_, _>, output: &str| {
        topic_publish_output_id(
            running_dataflows,
            dataflow_id,
            single_node.clone(),
            output.to_string().into(),
        )
    };

    // Debug publishing is off by default.
    assert_eq!(publish(&running_dataflows, "image"), None);

    running_dataflows
        .get_mut(&dataflow_id)
        .unwrap()
        .descriptor
        .debug
        .enable_debug_inspection = true;
    let resolved = publish(&running_dataflows, "image").expect("bare output resolves");
    assert_eq!(resolved.to_string(), "op/image");
    assert!(
        dora_core::topics::zenoh_daemon_control_topic(dataflow_id, &single_node, &resolved)
            .ends_with("/control/single/op/image")
    );
    assert_eq!(publish(&running_dataflows, "op/image"), Some(resolved));
    assert_eq!(publish(&running_dataflows, "missing"), None);
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
        &mut HashMap::new(),
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
        &mut HashMap::new(),
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
        &mut HashMap::new(),
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
        &mut HashMap::new(),
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
        &mut HashMap::new(),
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
        &mut HashMap::new(),
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
        &mut HashMap::new(),
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
        &mut HashMap::new(),
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
        &daemon_connections,
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

/// A connected daemon on `machine`, with the receiver for what is sent to it.
fn connected_test_daemon(
    daemon_connections: &mut DaemonConnections,
    machine: &str,
) -> (DaemonId, tokio::sync::mpsc::Receiver<String>) {
    let daemon_id = DaemonId::new(Some(machine.to_string()));
    let (tx, rx) = tokio::sync::mpsc::channel::<String>(8);
    let connection = crate::state::DaemonConnection::new(
        tx,
        Arc::new(tokio::sync::Mutex::new(HashMap::new())),
        BTreeMap::new(),
    );
    daemon_connections.add(daemon_id.clone(), connection);
    (daemon_id, rx)
}

/// Every coordinator event queued on `rx`.
fn received_daemon_events(
    rx: &mut tokio::sync::mpsc::Receiver<String>,
) -> Vec<DaemonCoordinatorEvent> {
    #[derive(serde::Deserialize)]
    struct OutboundRaw {
        params: Timestamped<DaemonCoordinatorEvent>,
    }
    let mut events = Vec::new();
    while let Ok(outbound) = rx.try_recv() {
        let outbound: OutboundRaw = serde_json::from_str(&outbound).unwrap();
        events.push(outbound.params.inner);
    }
    events
}

/// The `exited_before_subscribe` of every `AllNodesReady` queued on `rx`.
fn received_all_nodes_ready(rx: &mut tokio::sync::mpsc::Receiver<String>) -> Vec<Vec<NodeId>> {
    received_daemon_events(rx)
        .into_iter()
        .filter_map(|event| match event {
            DaemonCoordinatorEvent::AllNodesReady {
                exited_before_subscribe,
                ..
            } => Some(exited_before_subscribe),
            _ => None,
        })
        .collect()
}

/// A two-daemon dataflow whose ready barrier already released, with a
/// `sender` node on `m1` and a `receiver` node on `m2`.
fn released_two_daemon_dataflow(
    dataflow_id: DataflowId,
    m1: &DaemonId,
    m2: &DaemonId,
) -> RunningDataflow {
    let mut dataflow = test_running_dataflow(dataflow_id, m1.clone(), "sender".to_string().into());
    dataflow.daemons.insert(m2.clone());
    dataflow
        .node_to_daemon
        .insert("receiver".to_string().into(), m2.clone());
    dataflow.ready_barrier_released = true;
    dataflow
}

#[tokio::test]
async fn a_ready_report_after_the_barrier_released_does_not_release_it_again() {
    // #3885: the barrier can release before a daemon reports (the last
    // pending daemon disconnected, #2028), and that daemon still reports its
    // nodes ready once it reconnects. That late report must not broadcast a
    // second, possibly different verdict, rewrite the stored status, or
    // replay params to every daemon again. The reporting daemon gets the
    // released verdict back, and no one else gets anything.
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let mut daemon_connections = DaemonConnections::default();
    let (m1, mut m1_rx) = connected_test_daemon(&mut daemon_connections, "m1");
    let (m2, mut m2_rx) = connected_test_daemon(&mut daemon_connections, "m2");

    let mut dataflow = released_two_daemon_dataflow(dataflow_id, &m1, &m2);
    // The dataflow moved on since the release; the late report must not
    // put the durable record back to `Running`.
    let seeded = dataflow
        .make_record(StoreDataflowStatus::Stopping)
        .expect("seed record");
    store.put_dataflow(&seeded).expect("seed");

    let mut coordinator = test_coordinator(
        HashMap::from([(dataflow_id, dataflow)]),
        daemon_connections,
        store.clone(),
    );
    // An empty verdict is the one a re-release would persist as `Running`.
    coordinator
        .handle_dataflow_event(
            dataflow_id,
            DataflowEvent::ReadyOnDaemon {
                daemon_id: m2.clone(),
                exited_before_subscribe: vec![],
            },
        )
        .await
        .unwrap();
    assert_eq!(
        store
            .get_dataflow(&dataflow_id)
            .expect("read back")
            .expect("record present")
            .status,
        StoreDataflowStatus::Stopping,
        "the stored status must not be rewritten"
    );

    // A late report of a node that exited must not change the verdict.
    coordinator
        .handle_dataflow_event(
            dataflow_id,
            DataflowEvent::ReadyOnDaemon {
                daemon_id: m2,
                exited_before_subscribe: vec!["receiver".to_string().into()],
            },
        )
        .await
        .unwrap();
    assert!(
        coordinator.running_dataflows[&dataflow_id]
            .exited_before_subscribe
            .is_empty(),
        "the released verdict stays frozen, so reconnect replays match the broadcast"
    );

    assert_eq!(
        received_all_nodes_ready(&mut m2_rx),
        vec![Vec::<NodeId>::new(); 2],
        "the reporting daemon is answered with the released verdict each time"
    );
    assert!(
        received_all_nodes_ready(&mut m1_rx).is_empty(),
        "no other daemon may be sent a second AllNodesReady"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn auto_recovered_nodes_are_released_after_the_barrier_released() {
    // Auto-recovery re-spawns a daemon's nodes into the same dataflow, whose
    // barrier released long ago. That daemon's re-spawned nodes wait for a
    // coordinator `AllNodesReady`, and its `ReadyOnDaemon` is the only thing
    // that triggers one: it must not be dropped as a stale report.
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let mut daemon_connections = DaemonConnections::default();
    let (m1, mut m1_rx) = connected_test_daemon(&mut daemon_connections, "m1");
    let (m2, mut m2_rx) = connected_test_daemon(&mut daemon_connections, "m2");

    let mut coordinator = test_coordinator(
        HashMap::from([(
            dataflow_id,
            released_two_daemon_dataflow(dataflow_id, &m1, &m2),
        )]),
        daemon_connections,
        store,
    );
    // `m1` restarted and reports nothing running, so its nodes are re-spawned.
    coordinator
        .handle_daemon_status_report(m1.clone(), vec![])
        .await
        .expect("status report should be handled");
    assert!(received_all_nodes_ready(&mut m1_rx).is_empty());

    coordinator
        .handle_dataflow_event(
            dataflow_id,
            DataflowEvent::ReadyOnDaemon {
                daemon_id: m1,
                exited_before_subscribe: vec![],
            },
        )
        .await
        .unwrap();

    assert_eq!(
        received_all_nodes_ready(&mut m1_rx),
        vec![Vec::<NodeId>::new()],
        "the re-spawned daemon must be released"
    );
    assert!(received_all_nodes_ready(&mut m2_rx).is_empty());
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
        "the cleanup only prunes membership; failing the spawn is \
         apply_disconnect_actions' job"
    );
}

// #3736: a daemon of a spawn-pending dataflow disconnects -> the cleanup asks
// to fail the spawn (never to reclaim it), and leaves the teardown itself to
// `apply_disconnect_actions`.
#[test]
fn disconnect_of_spawn_pending_dataflow_asks_to_fail_the_spawn() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("gone".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id);
    // spawn_result left Pending (CachedResult::default()).

    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);
    let disconnected = BTreeSet::from([daemon_id.clone()]);

    let actions = cleanup_disconnected_daemons_from_running_dataflows(
        &mut running_dataflows,
        &disconnected,
        &mut HashMap::new(),
    );
    assert!(
        matches!(
            actions.as_slice(),
            [DisconnectAction::FailPendingSpawn { uuid, daemons }]
                if *uuid == dataflow_id && *daemons == disconnected
        ),
        "a spawn-pending dataflow that lost a daemon must be failed, got {} action(s)",
        actions.len()
    );
    assert!(
        running_dataflows.contains_key(&dataflow_id),
        "the cleanup itself must not tear the dataflow down"
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

/// A coordinator running one dataflow on daemons `a` (node `sender`) and `b`
/// (node `receiver`), whose spawn is pending on both and whose ready barrier
/// waits on both. `wait_for_spawn` is parked on the returned receiver.
#[allow(clippy::type_complexity)]
fn coordinator_with_two_daemon_pending_spawn(
    dataflow_id: DataflowId,
) -> (
    Coordinator,
    (DaemonId, Uuid),
    (DaemonId, tokio::sync::mpsc::Receiver<String>),
    tokio::sync::oneshot::Receiver<eyre::Result<ControlRequestReply>>,
) {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let mut daemon_connections = DaemonConnections::default();
    let (daemon_a, _rx_a) = connected_test_daemon(&mut daemon_connections, "a");
    let (daemon_b, rx_b) = connected_test_daemon(&mut daemon_connections, "b");
    let connection_a = daemon_connections
        .connection_id_of(&daemon_a)
        .expect("a is connected");

    let mut df = test_running_dataflow(dataflow_id, daemon_a.clone(), "sender".to_string().into());
    df.daemons.insert(daemon_b.clone());
    df.node_to_daemon
        .insert("receiver".to_string().into(), daemon_b.clone());
    df.pending_daemons = BTreeSet::from([daemon_a.clone(), daemon_b.clone()]);
    df.pending_spawn_results = BTreeSet::from([daemon_a.clone(), daemon_b.clone()]);
    let (waiter_tx, waiter_rx) = tokio::sync::oneshot::channel();
    df.spawn_result.register(waiter_tx);

    let coordinator = test_coordinator(
        HashMap::from([(dataflow_id, df)]),
        daemon_connections,
        store,
    );
    (
        coordinator,
        (daemon_a, connection_a),
        (daemon_b, rx_b),
        waiter_rx,
    )
}

/// After `a` disconnected mid-spawn: the spawn failed naming `a`, the dataflow
/// is torn down as terminally failed, `b` was told to stop it, and the ready
/// barrier was never released.
async fn assert_spawn_failed_by_disconnect(
    coordinator: &Coordinator,
    dataflow_id: DataflowId,
    daemon_a: &DaemonId,
    rx_b: &mut tokio::sync::mpsc::Receiver<String>,
    waiter_rx: tokio::sync::oneshot::Receiver<eyre::Result<ControlRequestReply>>,
) {
    let reply = timeout(TokioDuration::from_secs(1), waiter_rx)
        .await
        .expect("wait_for_spawn must be released right away, not by the watchdog")
        .expect("waiter must not be dropped");
    let err = format!("{:?}", reply.expect_err("the spawn must fail"));
    assert!(
        err.contains(&format!("daemon `{daemon_a}` disconnected during spawn")),
        "the error must name the disconnected daemon, got: {err}"
    );

    assert!(!coordinator.running_dataflows.contains_key(&dataflow_id));
    assert!(coordinator.archived_dataflows.contains_key(&dataflow_id));
    let record = coordinator
        .store
        .get_dataflow(&dataflow_id)
        .expect("store lookup")
        .expect("record present");
    assert!(
        matches!(
            record.status,
            StoreDataflowStatus::Failed { terminal: true, .. }
        ),
        "the store must record a terminal failure, got {:?}",
        record.status
    );

    let events = received_daemon_events(rx_b);
    assert!(
        events.iter().any(|e| matches!(
            e,
            DaemonCoordinatorEvent::StopDataflow { dataflow_id: id, force: true, .. }
                if *id == dataflow_id
        )),
        "the survivor must be told to stop the dataflow, got {events:?}"
    );
    assert!(
        !events
            .iter()
            .any(|e| matches!(e, DaemonCoordinatorEvent::AllNodesReady { .. })),
        "the survivor's nodes must not be released, got {events:?}"
    );
}

/// A survivor's report arriving after the disconnect failed the spawn must
/// not bring the dataflow back or release its nodes. Returns the events the
/// survivor was sent in response.
fn assert_not_resurrected(
    coordinator: &Coordinator,
    dataflow_id: DataflowId,
    rx_b: &mut tokio::sync::mpsc::Receiver<String>,
) -> Vec<DaemonCoordinatorEvent> {
    assert!(!coordinator.running_dataflows.contains_key(&dataflow_id));
    let record = coordinator
        .store
        .get_dataflow(&dataflow_id)
        .expect("store lookup")
        .expect("record present");
    assert!(
        matches!(
            record.status,
            StoreDataflowStatus::Failed { terminal: true, .. }
        ),
        "a late report must not resurrect the dataflow, got {:?}",
        record.status
    );
    let events = received_daemon_events(rx_b);
    assert!(
        !events
            .iter()
            .any(|e| matches!(e, DaemonCoordinatorEvent::AllNodesReady { .. })),
        "a late report must not release the survivor's nodes, got {events:?}"
    );
    events
}

// #3736: survivor `b` reports ready before its spawn result, then `a`
// disconnects mid-spawn. The spawn must fail at once and `b` be stopped. On
// `main` `b`'s later Ok resolved the spawn with the barrier never released,
// so `b`'s nodes waited forever under a `Running` dataflow.
#[tokio::test]
async fn disconnect_mid_spawn_after_survivor_ready_fails_the_spawn() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let (mut coordinator, (daemon_a, connection_a), (daemon_b, mut rx_b), waiter_rx) =
        coordinator_with_two_daemon_pending_spawn(dataflow_id);

    coordinator
        .handle_dataflow_event(
            dataflow_id,
            DataflowEvent::ReadyOnDaemon {
                daemon_id: daemon_b.clone(),
                exited_before_subscribe: vec![],
            },
        )
        .await
        .unwrap();
    coordinator
        .handle_daemon_exit(daemon_a.clone(), connection_a)
        .await
        .unwrap();
    assert_spawn_failed_by_disconnect(&coordinator, dataflow_id, &daemon_a, &mut rx_b, waiter_rx)
        .await;

    coordinator
        .handle_spawn_result(dataflow_id, daemon_b, Ok(()))
        .await
        .unwrap();
    let events = assert_not_resurrected(&coordinator, dataflow_id, &mut rx_b);
    // `b` may have created the dataflow only after the first stop reached it,
    // so its late Ok is answered with another stop.
    assert!(
        events.iter().any(|e| matches!(
            e,
            DaemonCoordinatorEvent::StopDataflow { dataflow_id: id, force: true, .. }
                if *id == dataflow_id
        )),
        "a late spawn Ok must stop the survivor again, got {events:?}"
    );
}

// The other order of the same race, and the common one: `b`'s spawn Ok
// arrives while the spawn is still pending on `a`, then `a` disconnects. The
// spawn must fail at once as well, rather than release `b`'s nodes and
// force-stop them when the spawn watchdog fires.
#[tokio::test]
async fn disconnect_mid_spawn_after_survivor_spawned_fails_the_spawn() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let (mut coordinator, (daemon_a, connection_a), (daemon_b, mut rx_b), waiter_rx) =
        coordinator_with_two_daemon_pending_spawn(dataflow_id);

    coordinator
        .handle_spawn_result(dataflow_id, daemon_b.clone(), Ok(()))
        .await
        .unwrap();
    assert!(
        coordinator.running_dataflows[&dataflow_id]
            .spawn_result
            .is_pending(),
        "precondition: the spawn is still pending on `a`"
    );
    coordinator
        .handle_daemon_exit(daemon_a.clone(), connection_a)
        .await
        .unwrap();
    assert_spawn_failed_by_disconnect(&coordinator, dataflow_id, &daemon_a, &mut rx_b, waiter_rx)
        .await;

    coordinator
        .handle_dataflow_event(
            dataflow_id,
            DataflowEvent::ReadyOnDaemon {
                daemon_id: daemon_b,
                exited_before_subscribe: vec![],
            },
        )
        .await
        .unwrap();
    let _ = assert_not_resurrected(&coordinator, dataflow_id, &mut rx_b);
}

// A spawn the disconnect path failed is not failed a second time by the
// spawn-timeout watchdog, even once its deadline has passed.
#[tokio::test]
async fn spawn_watchdog_skips_a_spawn_failed_by_a_disconnect() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let (mut coordinator, (daemon_a, connection_a), (_daemon_b, mut rx_b), waiter_rx) =
        coordinator_with_two_daemon_pending_spawn(dataflow_id);
    coordinator
        .running_dataflows
        .get_mut(&dataflow_id)
        .unwrap()
        .spawn_started_at = Instant::now() - spawn_result_timeout() - Duration::from_secs(1);

    coordinator
        .handle_daemon_exit(daemon_a.clone(), connection_a)
        .await
        .unwrap();
    assert_spawn_failed_by_disconnect(&coordinator, dataflow_id, &daemon_a, &mut rx_b, waiter_rx)
        .await;
    let record_before = coordinator
        .store
        .get_dataflow(&dataflow_id)
        .unwrap()
        .unwrap();
    let results_before = format!("{:?}", coordinator.dataflow_results.get(&dataflow_id));

    check_spawn_timeouts(
        &mut coordinator.running_dataflows,
        &mut coordinator.archived_dataflows,
        &mut coordinator.dataflow_results,
        &mut coordinator.daemon_connections,
        &mut coordinator.pending_restarts,
        &coordinator.clock,
        coordinator.store.as_ref(),
    )
    .await;

    let events = received_daemon_events(&mut rx_b);
    assert!(
        events.is_empty(),
        "the watchdog must not roll the survivor back a second time, got {events:?}"
    );
    let record_after = coordinator
        .store
        .get_dataflow(&dataflow_id)
        .unwrap()
        .unwrap();
    assert_eq!(
        format!("{:?}", record_after.status),
        format!("{:?}", record_before.status),
        "the watchdog must not overwrite the disconnect verdict"
    );
    assert_eq!(
        format!("{:?}", coordinator.dataflow_results.get(&dataflow_id)),
        results_before,
        "the watchdog must not synthesize a second failure"
    );
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
        &mut HashMap::new(),
        &mut daemon_connections,
        &mut IndexMap::new(),
        &mut IndexMap::new(),
        &mut HashMap::new(),
        &mut HashMap::new(),
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

/// An orphan reclaim removes the live entry, and the persisted record does
/// not carry the launch context. It must survive the reclaim anyway, or the
/// entry rebuilt when the daemon reconnects would make a later `dora restart`
/// relaunch with no build, a fresh session and no working directory.
#[tokio::test]
async fn launch_context_survives_orphan_reclaim_and_reconnect() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = Arc::new(HLC::default());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("blip".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let launch = crate::state::LaunchContext {
        build_id: Some(BuildId::generate()),
        session_id: dora_message::SessionId::generate(),
        local_working_dir: Some("/home/user/project".into()),
        write_events_to: Some("/home/user/events".into()),
    };
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());
    df.launch = launch.clone();
    df.spawn_result
        .set_result(Ok(ControlRequestReply::DataflowSpawned {
            uuid: dataflow_id,
        }));
    store
        .put_dataflow(&df.make_record(StoreDataflowStatus::Running).unwrap())
        .unwrap();
    let mut running_dataflows = HashMap::from([(dataflow_id, df)]);
    let mut reclaimed_launches = HashMap::new();

    // The dataflow's only daemon drops: reclaim.
    let actions = cleanup_disconnected_daemons_from_running_dataflows(
        &mut running_dataflows,
        &BTreeSet::from([daemon_id.clone()]),
        &mut HashMap::new(),
    );
    apply_disconnect_actions(
        actions,
        &mut running_dataflows,
        &mut reclaimed_launches,
        &mut DaemonConnections::default(),
        &mut IndexMap::new(),
        &mut IndexMap::new(),
        &mut HashMap::new(),
        &mut HashMap::new(),
        &store,
        &clock,
    )
    .await
    .unwrap();
    assert!(!running_dataflows.contains_key(&dataflow_id));

    // It reconnects and re-reports the dataflow: the rebuilt entry keeps the
    // launch context.
    let record = store.get_dataflow(&dataflow_id).unwrap().unwrap();
    let _ = reestablish_running_dataflow(
        &mut running_dataflows,
        &mut reclaimed_launches,
        &record,
        &daemon_id,
        std::slice::from_ref(&node_id),
    );
    assert_eq!(running_dataflows[&dataflow_id].launch, launch);
    assert!(reclaimed_launches.is_empty());
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

/// A forwarded `dora param set`/`delete` that the daemon applied must not
/// stay in the state catch-up log: acks used to move only on reconnect, so on
/// a healthy cluster the log grew to its hard cap and every reconnect
/// replayed the whole history. A daemon that missed an entry for its own
/// node keeps it until it is gone from the dataflow; one whose missed entries
/// all target other daemons' nodes advances.
#[tokio::test]
async fn applied_param_changes_are_acked_and_pruned_from_the_state_log() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let owner = DaemonId::new(Some("owner".to_string()));
    let peer = DaemonId::new(Some("peer".to_string()));
    let lagging = DaemonId::new(Some("lagging".to_string()));
    let node_id: NodeId = "sender".to_string().into();

    // A daemon that answers every request with `Ok`.
    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let conn = crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
    tokio::spawn(async move {
        #[derive(serde::Deserialize)]
        struct OutboundRaw {
            id: String,
            params: Timestamped<DaemonCoordinatorEvent>,
        }
        while let Some(outbound) = rx.recv().await {
            let outbound: OutboundRaw = serde_json::from_str(&outbound).unwrap();
            let reply = match outbound.params.inner {
                DaemonCoordinatorEvent::SetParam { .. } => {
                    DaemonCoordinatorReply::SetParamResult(Ok(()))
                }
                DaemonCoordinatorEvent::DeleteParam { .. } => {
                    DaemonCoordinatorReply::DeleteParamResult(Ok(()))
                }
                other => panic!("unexpected event: {other:?}"),
            };
            let request_id = Uuid::parse_str(&outbound.id).unwrap();
            if let Some(reply_tx) = pending_replies.lock().await.remove(&request_id) {
                let _ = reply_tx.send(serde_json::to_string(&reply).unwrap());
            }
        }
    });
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(owner.clone(), conn);

    let lagging_node: NodeId = "lagging-node".to_string().into();
    let mut df = test_running_dataflow(dataflow_id, owner.clone(), node_id.clone());
    df.daemons.extend([peer.clone(), lagging.clone()]);
    df.node_to_daemon
        .insert(lagging_node.clone(), lagging.clone());
    // An earlier entry for `lagging`'s node, which only `owner` has acked
    // (`peer` has nothing to apply for it).
    df.append_state_log(StateCatchUpOperation::DeleteParam {
        node_id: lagging_node,
        key: "old".to_string(),
    });
    df.daemon_ack_sequence = BTreeMap::from([(owner.clone(), 1)]);

    let mut coordinator = Coordinator {
        resent_finish_reports: Default::default(),
        running_builds: HashMap::new(),
        finished_builds: IndexMap::new(),
        running_dataflows: HashMap::from([(dataflow_id, df)]),
        pending_restarts: HashMap::new(),
        reclaimed_launches: HashMap::new(),
        dataflow_results: IndexMap::new(),
        archived_dataflows: IndexMap::new(),
        daemon_connections,
        clock: Arc::new(HLC::default()),
        store: Arc::new(InMemoryStore::new()),
        span_store: SpanStore::default(),
        daemon_peer_addrs: Default::default(),
        #[cfg(feature = "metrics")]
        otel_metrics: crate::otel_metrics::new_shared(),
        abort_handle: futures::stream::abortable(futures::stream::empty::<()>()).1,
        topology: crate::topology_lane::TopologyLanes::detached(),
    };

    let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
    coordinator
        .handle_set_param(
            dataflow_id,
            node_id.clone(),
            "gain".to_string(),
            serde_json::json!(2),
            reply_tx,
        )
        .await
        .unwrap();
    reply_rx.await.unwrap().expect("set is applied");

    let df = &coordinator.running_dataflows[&dataflow_id];
    assert_eq!(df.state_log_sequence, 2);
    assert_eq!(df.daemon_ack_sequence[&owner], 2);
    assert_eq!(df.daemon_ack_sequence[&peer], 2);
    assert_eq!(
        df.daemon_ack_sequence.get(&lagging).copied().unwrap_or(0),
        0,
        "a daemon that missed entry 1 for its own node must not skip it"
    );
    assert_eq!(df.state_log.len(), 2, "kept for the lagging daemon");

    // Once the lagging daemon is gone, the next applied change empties the log.
    coordinator
        .running_dataflows
        .get_mut(&dataflow_id)
        .unwrap()
        .daemons
        .remove(&lagging);
    let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
    coordinator
        .handle_delete_param(dataflow_id, node_id, "gain".to_string(), reply_tx)
        .await
        .unwrap();
    reply_rx.await.unwrap().expect("delete is applied");

    let df = &coordinator.running_dataflows[&dataflow_id];
    assert_eq!(df.daemon_ack_sequence[&owner], 3);
    assert_eq!(df.daemon_ack_sequence[&peer], 3);
    assert!(df.state_log.is_empty(), "every live daemon applied it");
}

/// A `WaitForSpawn` that lands after a failed spawn already tore the
/// dataflow down must report the failure. It used to get "unknown
/// dataflow", which `dora start` reads as "started and finished", so a
/// dataflow that never started was reported as a success.
#[tokio::test]
async fn wait_for_spawn_after_a_failed_spawn_reports_the_failure() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());
    df.pending_spawn_results.insert(daemon_id.clone());

    let mut coordinator = Coordinator {
        resent_finish_reports: Default::default(),
        running_builds: HashMap::new(),
        finished_builds: IndexMap::new(),
        running_dataflows: HashMap::from([(dataflow_id, df)]),
        pending_restarts: HashMap::new(),
        reclaimed_launches: HashMap::new(),
        dataflow_results: IndexMap::new(),
        archived_dataflows: IndexMap::new(),
        daemon_connections: DaemonConnections::default(),
        clock: Arc::new(HLC::default()),
        store: Arc::new(InMemoryStore::new()),
        span_store: SpanStore::default(),
        daemon_peer_addrs: Default::default(),
        #[cfg(feature = "metrics")]
        otel_metrics: crate::otel_metrics::new_shared(),
        abort_handle: futures::stream::abortable(futures::stream::empty::<()>()).1,
        topology: crate::topology_lane::TopologyLanes::detached(),
    };
    coordinator
        .handle_spawn_result(dataflow_id, daemon_id, Err(eyre!("no such binary")))
        .await
        .unwrap();
    assert!(!coordinator.running_dataflows.contains_key(&dataflow_id));

    let wait_for_spawn = |dataflow_id| {
        let (reply_sender, reply) = tokio::sync::oneshot::channel();
        let event = ControlEvent::IncomingRequest {
            request: Box::new(
                dora_message::cli_to_coordinator::ControlRequest::WaitForSpawn { dataflow_id },
            ),
            reply_sender,
        };
        (event, reply)
    };

    let (event, reply) = wait_for_spawn(dataflow_id);
    coordinator.handle_control_event(event).await.unwrap();
    let err = format!("{:?}", reply.await.unwrap().expect_err("spawn failed"));
    assert!(err.contains("no such binary"), "got: {err}");
    assert!(
        !error_reads_as_finished(&err),
        "must not read as a dataflow that ran to completion: {err}"
    );

    // A dataflow the coordinator never knew still answers "unknown".
    let (event, reply) = wait_for_spawn(DataflowId::from(Uuid::new_v4()));
    coordinator.handle_control_event(event).await.unwrap();
    let err = format!("{:?}", reply.await.unwrap().expect_err("unknown"));
    assert!(err.contains("unknown dataflow"), "got: {err}");
}

/// Runs a dataflow whose only node exits with `node_result` before its spawn
/// result is recorded, then sends the `WaitForSpawn` that `dora start` sends
/// once the coordinator has archived it, and returns the reply.
async fn wait_for_spawn_after_an_early_node_exit(
    node_result: Result<(), NodeError>,
) -> eyre::Result<ControlRequestReply> {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: NodeId = "sender".to_string().into();
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());
    df.pending_spawn_results.insert(daemon_id.clone());

    let mut coordinator = Coordinator {
        resent_finish_reports: Default::default(),
        running_builds: HashMap::new(),
        finished_builds: IndexMap::new(),
        running_dataflows: HashMap::from([(dataflow_id, df)]),
        pending_restarts: HashMap::new(),
        reclaimed_launches: HashMap::new(),
        dataflow_results: IndexMap::new(),
        archived_dataflows: IndexMap::new(),
        daemon_connections: DaemonConnections::default(),
        clock: Arc::new(HLC::default()),
        store: Arc::new(InMemoryStore::new()),
        span_store: SpanStore::default(),
        daemon_peer_addrs: Default::default(),
        #[cfg(feature = "metrics")]
        otel_metrics: crate::otel_metrics::new_shared(),
        abort_handle: futures::stream::abortable(futures::stream::empty::<()>()).1,
        topology: crate::topology_lane::TopologyLanes::detached(),
    };
    let result = DataflowDaemonResult {
        timestamp: coordinator.clock.new_timestamp(),
        node_results: BTreeMap::from([(node_id, node_result)]),
    };
    coordinator
        .handle_dataflow_event(
            dataflow_id,
            DataflowEvent::DataflowFinishedOnDaemon { daemon_id, result },
        )
        .await
        .unwrap();
    assert!(!coordinator.running_dataflows.contains_key(&dataflow_id));

    let (reply_sender, reply) = tokio::sync::oneshot::channel();
    coordinator
        .handle_control_event(ControlEvent::IncomingRequest {
            request: Box::new(
                dora_message::cli_to_coordinator::ControlRequest::WaitForSpawn { dataflow_id },
            ),
            reply_sender,
        })
        .await
        .unwrap();
    reply.await.unwrap()
}

/// The same late `WaitForSpawn`, for a dataflow whose node crashed before
/// the spawn completed: the coordinator archives it from
/// `DataflowFinishedOnDaemon` and the request must still see the failure.
#[tokio::test]
async fn wait_for_spawn_after_an_early_node_exit_reports_the_failure() {
    let err = wait_for_spawn_after_an_early_node_exit(Err(NodeError {
        timestamp: HLC::default().new_timestamp(),
        cause: NodeErrorCause::Other {
            stderr: "segfault at startup".to_string(),
        },
        exit_status: NodeExitStatus::ExitCode(139),
    }))
    .await
    .expect_err("spawn failed");
    let err = format!("{err:?}");
    assert!(err.contains("failed to start"), "got: {err}");
    assert!(
        !error_reads_as_finished(&err),
        "must not read as a dataflow that ran to completion: {err}"
    );
}

/// A dataflow whose nodes all exited cleanly before the spawn result came in
/// did start: a late `WaitForSpawn` must keep reading as "started and
/// finished", as it does for any other finished dataflow.
#[tokio::test]
async fn wait_for_spawn_after_a_clean_early_exit_reads_as_finished() {
    let err = wait_for_spawn_after_an_early_node_exit(Ok(()))
        .await
        .expect_err("the dataflow is no longer running");
    let err = format!("{err:?}");
    assert!(
        error_reads_as_finished(&err),
        "must read as a dataflow that ran to completion: {err}"
    );
}

/// Mirrors the CLI's `error_indicates_dataflow_finished`.
fn error_reads_as_finished(msg: &str) -> bool {
    msg.contains("no running dataflow with id") || msg.contains("unknown dataflow")
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

/// Fake daemon for replay tests: answers every `SetParam` with `Ok`, after
/// running `on_request` on it, and returns the `(key, value)` pairs it saw.
/// A `DeleteParam` is answered with `Ok` and not recorded.
fn spawn_set_param_acking_daemon(
    mut rx: tokio::sync::mpsc::Receiver<String>,
    pending_replies: Arc<tokio::sync::Mutex<HashMap<Uuid, tokio::sync::oneshot::Sender<String>>>>,
    mut on_request: impl FnMut(&str) + Send + 'static,
) -> tokio::task::JoinHandle<Vec<(String, serde_json::Value)>> {
    #[derive(serde::Deserialize)]
    struct OutboundRaw {
        id: String,
        params: Timestamped<DaemonCoordinatorEvent>,
    }
    tokio::spawn(async move {
        let mut seen = Vec::new();
        while let Some(outbound) = rx.recv().await {
            let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
            let reply = match outbound_raw.params.inner {
                DaemonCoordinatorEvent::SetParam { key, value, .. } => {
                    on_request(&key);
                    seen.push((key, value));
                    DaemonCoordinatorReply::SetParamResult(Ok(()))
                }
                DaemonCoordinatorEvent::DeleteParam { .. } => {
                    DaemonCoordinatorReply::DeleteParamResult(Ok(()))
                }
                other => panic!("unexpected replay event: {other:?}"),
            };
            let request_id = Uuid::parse_str(&outbound_raw.id).expect("valid request id");
            let reply = serde_json::to_string(&reply).unwrap();
            let reply_tx = pending_replies
                .lock()
                .await
                .remove(&request_id)
                .expect("pending reply sender should exist");
            let _ = reply_tx.send(reply);
        }
        seen
    })
}

#[tokio::test]
async fn replay_sends_current_value_of_param_changed_after_snapshot() {
    // #3683: a `dora param set`/`delete` that lands while the replay is
    // sending earlier items must not be overwritten by the value the replay
    // listed at its start, nor a deleted key be resurrected.
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();
    let one = serde_json::to_vec(&serde_json::json!(1)).unwrap();
    for key in ["a", "b", "c"] {
        store
            .put_node_param(&dataflow_id, &node_id, key, &one)
            .unwrap();
    }

    let (tx, rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let connection =
        crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());

    let daemon_store = store.clone();
    let daemon_node = node_id.clone();
    let daemon_task = spawn_set_param_acking_daemon(rx, pending_replies, move |key| {
        if key == "a" {
            let two = serde_json::to_vec(&serde_json::json!(2)).unwrap();
            daemon_store
                .put_node_param(&dataflow_id, &daemon_node, "b", &two)
                .unwrap();
            daemon_store
                .delete_node_param(&dataflow_id, &daemon_node, "c")
                .unwrap();
        }
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
    // `a` and `b` are set, then the delete of `c` is replayed.
    assert_eq!(summary.attempted, 3);
    assert_eq!(summary.failed, 0);

    let seen = tokio::time::timeout(Duration::from_secs(10), daemon_task)
        .await
        .expect("daemon task did not finish")
        .unwrap();
    assert_eq!(
        seen,
        vec![
            ("a".to_string(), serde_json::json!(1)),
            ("b".to_string(), serde_json::json!(2)),
        ]
    );
}

#[tokio::test]
async fn replay_releases_param_write_lock_while_waiting_for_the_daemon() {
    // The replay holds the connection's lock only up to queuing each param
    // (#3683). Held while waiting for the reply, a slow daemon would stall
    // every `dora param set` to it, on the coordinator's event loop.
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();
    let value = serde_json::to_vec(&serde_json::json!(1)).unwrap();
    store
        .put_node_param(&dataflow_id, &node_id, "gain", &value)
        .unwrap();

    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let connection = crate::state::DaemonConnection::new(tx, pending_replies, BTreeMap::new());
    let lock = connection.param_write_lock.clone();

    let replay = tokio::spawn(replay_persisted_params_for_daemon(
        dataflow_id,
        daemon_id,
        vec![node_id],
        store,
        connection,
        Arc::new(HLC::default()),
    ));

    // The daemon receives the param but never answers it.
    tokio::time::timeout(Duration::from_secs(10), rx.recv())
        .await
        .expect("replay did not send the param")
        .expect("connection closed");
    let _released = tokio::time::timeout(Duration::from_secs(10), lock.lock())
        .await
        .expect("replay must release the lock before waiting for the reply");
    assert!(
        !replay.is_finished(),
        "replay is still waiting for the reply"
    );
    replay.abort();
}

#[tokio::test]
async fn replay_takes_param_write_lock_before_reading_the_param() {
    // #3683, replay side: a `set` holding the connection's lock may be
    // between persisting its value and queuing it. The replay must wait for
    // it before re-reading the store, or it could queue the old value after
    // the `set`'s new one.
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();
    let one = serde_json::to_vec(&serde_json::json!(1)).unwrap();
    store
        .put_node_param(&dataflow_id, &node_id, "gain", &one)
        .unwrap();

    let (tx, rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let connection =
        crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
    let lock = connection.param_write_lock.clone();
    let daemon_task = spawn_set_param_acking_daemon(rx, pending_replies, |_| {});

    let set_guard = lock.lock().await;
    let replay = tokio::spawn(replay_persisted_params_for_daemon(
        dataflow_id,
        daemon_id,
        vec![node_id.clone()],
        store.clone(),
        connection,
        Arc::new(HLC::default()),
    ));
    tokio::time::sleep(Duration::from_millis(200)).await;
    assert!(!replay.is_finished(), "replay must wait for the set's lock");
    // The `set` persists its value while it holds the lock.
    let two = serde_json::to_vec(&serde_json::json!(2)).unwrap();
    store
        .put_node_param(&dataflow_id, &node_id, "gain", &two)
        .unwrap();
    drop(set_guard);

    let summary = tokio::time::timeout(Duration::from_secs(10), replay)
        .await
        .expect("replay did not finish")
        .unwrap();
    assert_eq!(summary.attempted, 1);
    assert_eq!(summary.failed, 0);
    let seen = tokio::time::timeout(Duration::from_secs(10), daemon_task)
        .await
        .expect("daemon task did not finish")
        .unwrap();
    assert_eq!(seen, vec![("gain".to_string(), serde_json::json!(2))]);
}

// -------------------------------------------------------------------
// Param deletes a disconnected daemon missed
// -------------------------------------------------------------------

/// A daemon connection whose fake daemon answers each request with `reply`
/// (no answer for `None`) and hands every coordinator command it receives to
/// the returned channel.
fn param_daemon_connection(
    reply: fn(&DaemonCoordinatorEvent) -> Option<DaemonCoordinatorReply>,
) -> (
    crate::state::DaemonConnection,
    tokio::sync::mpsc::UnboundedReceiver<DaemonCoordinatorEvent>,
) {
    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(64);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let connection =
        crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
    let (seen_tx, seen_rx) = tokio::sync::mpsc::unbounded_channel();
    tokio::spawn(async move {
        #[derive(serde::Deserialize)]
        struct Outbound {
            id: String,
            params: serde_json::Value,
        }
        while let Some(outbound) = rx.recv().await {
            let outbound: Outbound = serde_json::from_str(&outbound).unwrap();
            // The register reply is not a `DaemonCoordinatorEvent`.
            let Ok(command) =
                serde_json::from_value::<Timestamped<DaemonCoordinatorEvent>>(outbound.params)
            else {
                continue;
            };
            if let Some(reply) = reply(&command.inner) {
                let request_id = Uuid::parse_str(&outbound.id).unwrap();
                if let Some(reply_tx) = pending_replies.lock().await.remove(&request_id) {
                    let _ = reply_tx.send(serde_json::to_string(&reply).unwrap());
                }
            }
            // Keep answering when nobody watches what this daemon receives.
            let _ = seen_tx.send(command.inner);
        }
    });
    (connection, seen_rx)
}

/// A daemon connection whose fake daemon answers every `SetParam` and
/// `DeleteParam` with `Ok`.
fn param_acking_daemon_connection() -> (
    crate::state::DaemonConnection,
    tokio::sync::mpsc::UnboundedReceiver<DaemonCoordinatorEvent>,
) {
    param_daemon_connection(|command| match command {
        DaemonCoordinatorEvent::SetParam { .. } => {
            Some(DaemonCoordinatorReply::SetParamResult(Ok(())))
        }
        DaemonCoordinatorEvent::DeleteParam { .. } => {
            Some(DaemonCoordinatorReply::DeleteParamResult(Ok(())))
        }
        _ => None,
    })
}

/// Register a daemon through the coordinator's `Register` handler, as a
/// (re)connecting daemon does, and return the id it was given.
async fn register_daemon(
    coordinator: &mut Coordinator,
    machine_id: &str,
    connection: crate::state::DaemonConnection,
) -> DaemonId {
    let (daemon_id_tx, daemon_id_rx) = tokio::sync::oneshot::channel();
    coordinator
        .handle_daemon_request(DaemonRequest::Register {
            version_check_result: Ok(()),
            machine_id: Some(machine_id.to_string()),
            labels: BTreeMap::new(),
            connection,
            daemon_id_tx,
        })
        .await
        .unwrap();
    daemon_id_rx.await.expect("registration succeeds")
}

fn spawned_running_dataflow(mut df: RunningDataflow) -> RunningDataflow {
    let uuid = df.uuid;
    df.spawn_result
        .set_result(Ok(ControlRequestReply::DataflowSpawned { uuid }));
    df
}

async fn set_param(
    coordinator: &mut Coordinator,
    dataflow_id: DataflowId,
    node_id: &NodeId,
    key: &str,
) -> eyre::Result<ControlRequestReply> {
    let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
    coordinator
        .handle_set_param(
            dataflow_id,
            node_id.clone(),
            key.to_string(),
            serde_json::json!(1),
            reply_tx,
        )
        .await
        .unwrap();
    reply_rx.await.unwrap()
}

async fn delete_param(
    coordinator: &mut Coordinator,
    dataflow_id: DataflowId,
    node_id: &NodeId,
    key: &str,
) -> eyre::Result<ControlRequestReply> {
    let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
    coordinator
        .handle_delete_param(dataflow_id, node_id.clone(), key.to_string(), reply_tx)
        .await
        .unwrap();
    reply_rx.await.unwrap()
}

/// Re-register daemon `machine_id` under a new id and report `sender` of
/// `dataflow_id` running on it, as a daemon does on reconnect, then return
/// the param commands its full param replay sent it: the first `expected`
/// ones, waited for under a long deadline, and any that follow shortly after.
async fn reconnect_daemon_and_collect_param_commands(
    coordinator: &mut Coordinator,
    machine_id: &str,
    old_id: &DaemonId,
    dataflow_id: DataflowId,
    expected: usize,
) -> Vec<DaemonCoordinatorEvent> {
    let (conn, mut seen) = param_acking_daemon_connection();
    let daemon_id = register_daemon(coordinator, machine_id, conn).await;
    assert_ne!(&daemon_id, old_id, "a daemon that reconnects gets a new id");
    coordinator
        .handle_daemon_status_report(
            daemon_id.clone(),
            vec![dora_message::daemon_to_coordinator::DataflowStatusEntry {
                dataflow_id,
                running_nodes: vec!["sender".to_string().into()],
            }],
        )
        .await
        .unwrap();
    assert!(
        coordinator.running_dataflows[&dataflow_id]
            .daemons
            .contains(&daemon_id),
        "the status report links the daemon back into the dataflow"
    );
    let is_param_command = |command: &DaemonCoordinatorEvent| {
        matches!(
            command,
            DaemonCoordinatorEvent::SetParam { .. } | DaemonCoordinatorEvent::DeleteParam { .. }
        )
    };
    let mut commands = Vec::new();
    timeout(TokioDuration::from_secs(10), async {
        while commands.len() < expected {
            let command = seen.recv().await.expect("connection closed");
            if is_param_command(&command) {
                commands.push(command);
            }
        }
    })
    .await
    .unwrap_or_else(|_| panic!("expected {expected} param commands, got {commands:#?}"));
    while let Ok(Some(command)) = timeout(TokioDuration::from_millis(300), seen.recv()).await {
        if is_param_command(&command) {
            commands.push(command);
        }
    }
    commands
}

fn deleted_keys(commands: &[DaemonCoordinatorEvent]) -> Vec<&str> {
    commands
        .iter()
        .filter_map(|c| match c {
            DaemonCoordinatorEvent::DeleteParam { key, .. } => Some(key.as_str()),
            _ => None,
        })
        .collect()
}

/// A delete aimed at a node whose daemon is disconnected is persisted, then
/// pruned from the state log by another daemon's activity. The daemon that
/// reconnects falls back to the full param replay, which must delete the key
/// on the node and leave the key that is still set alone.
#[tokio::test]
async fn reconnect_after_pruned_delete_deletes_the_param_on_the_node() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let mut coordinator = coordinator_with_store(store.clone());
    let (conn_a, _seen_a) = param_acking_daemon_connection();
    let conn_a_id = conn_a.connection_id;
    let a = register_daemon(&mut coordinator, "a", conn_a).await;
    let (conn_b, _seen_b) = param_acking_daemon_connection();
    let b = register_daemon(&mut coordinator, "b", conn_b).await;
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let mut df = spawned_running_dataflow(released_two_daemon_dataflow(dataflow_id, &a, &b));
    df.daemon_ack_sequence = BTreeMap::from([(a.clone(), 0), (b, 0)]);
    store
        .put_dataflow(&df.make_record(StoreDataflowStatus::Running).unwrap())
        .unwrap();
    coordinator.running_dataflows.insert(dataflow_id, df);

    let sender: NodeId = "sender".to_string().into();
    let receiver: NodeId = "receiver".to_string().into();
    for key in ["gain", "fps"] {
        set_param(&mut coordinator, dataflow_id, &sender, key)
            .await
            .expect("applied while connected");
    }
    coordinator
        .handle_daemon_exit(a.clone(), conn_a_id)
        .await
        .unwrap();
    assert!(
        delete_param(&mut coordinator, dataflow_id, &sender, "gain")
            .await
            .is_err(),
        "`a` is not connected, so the delete is only persisted"
    );
    set_param(&mut coordinator, dataflow_id, &receiver, "rate")
        .await
        .expect("applied on `b`");
    assert!(
        !coordinator.running_dataflows[&dataflow_id]
            .state_log
            .iter()
            .any(|e| matches!(&e.operation, StateCatchUpOperation::DeleteParam { .. })),
        "precondition: `b`'s ack pruned the delete `a` never got"
    );

    let commands =
        reconnect_daemon_and_collect_param_commands(&mut coordinator, "a", &a, dataflow_id, 2)
            .await;
    assert_eq!(deleted_keys(&commands), vec!["gain"], "{commands:#?}");
    assert!(
        commands.iter().any(|c| matches!(
            c,
            DaemonCoordinatorEvent::SetParam { key, .. } if key == "fps"
        )),
        "the param that is still set is replayed: {commands:#?}"
    );
}

/// When every daemon of a dataflow disconnects, the dataflow and its state
/// log are dropped (orphan reclaim) and a delete only reaches the store. The
/// daemon that comes back must still get it.
#[tokio::test]
async fn reconnect_after_orphan_reclaim_deletes_the_param_on_the_node() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let mut coordinator = coordinator_with_store(store.clone());
    let (conn, _old_seen) = param_acking_daemon_connection();
    let conn_id = conn.connection_id;
    let a = register_daemon(&mut coordinator, "a", conn).await;
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let sender: NodeId = "sender".to_string().into();
    let mut df = spawned_running_dataflow(test_running_dataflow(
        dataflow_id,
        a.clone(),
        sender.clone(),
    ));
    df.ready_barrier_released = true;
    store
        .put_dataflow(&df.make_record(StoreDataflowStatus::Running).unwrap())
        .unwrap();
    coordinator.running_dataflows.insert(dataflow_id, df);
    for key in ["gain", "fps"] {
        set_param(&mut coordinator, dataflow_id, &sender, key)
            .await
            .expect("applied while connected");
    }

    coordinator
        .handle_daemon_exit(a.clone(), conn_id)
        .await
        .unwrap();
    assert!(
        !coordinator.running_dataflows.contains_key(&dataflow_id),
        "precondition: the orphaned dataflow is reclaimed, state log and all"
    );
    delete_param(&mut coordinator, dataflow_id, &sender, "gain")
        .await
        .expect("a delete for a reclaimed dataflow is persisted only");

    let commands =
        reconnect_daemon_and_collect_param_commands(&mut coordinator, "a", &a, dataflow_id, 2)
            .await;
    assert_eq!(deleted_keys(&commands), vec!["gain"], "{commands:#?}");
}

/// A store in which param `gain` of node `sender` was set and then deleted.
fn store_with_deleted_param() -> (Arc<dyn CoordinatorStore>, DataflowId, NodeId) {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let node_id: NodeId = "sender".to_string().into();
    let one = serde_json::to_vec(&serde_json::json!(1)).unwrap();
    store
        .put_node_param(&dataflow_id, &node_id, "gain", &one)
        .unwrap();
    store
        .delete_node_param(&dataflow_id, &node_id, "gain")
        .unwrap();
    (store, dataflow_id, node_id)
}

/// Run the full param replay of `node_id` over `connection`.
async fn replay_node_params(
    store: &Arc<dyn CoordinatorStore>,
    dataflow_id: DataflowId,
    node_id: &NodeId,
    connection: crate::state::DaemonConnection,
) -> crate::params::ParamReplaySummary {
    replay_persisted_params_for_daemon(
        dataflow_id,
        DaemonId::new(Some("a".to_string())),
        vec![node_id.clone()],
        store.clone(),
        connection,
        Arc::new(HLC::default()),
    )
    .await
}

/// A `dora param set` of a deleted key that lands while the replay waits for
/// the connection's write lock wins: the replay re-checks the store under the
/// lock and does not delete the value the set just forwarded.
#[tokio::test]
async fn replay_does_not_delete_a_param_set_again_while_it_waits_for_the_lock() {
    let (store, dataflow_id, node_id) = store_with_deleted_param();
    let (connection, mut seen) = param_acking_daemon_connection();
    let lock = connection.param_write_lock.clone();

    let set_guard = lock.lock().await;
    let replay = tokio::spawn({
        let store = store.clone();
        let node_id = node_id.clone();
        async move { replay_node_params(&store, dataflow_id, &node_id, connection).await }
    });
    tokio::time::sleep(TokioDuration::from_millis(200)).await;
    assert!(!replay.is_finished(), "replay must wait for the set's lock");
    // The `set` persists its value while it holds the lock.
    let one = serde_json::to_vec(&serde_json::json!(1)).unwrap();
    store
        .put_node_param(&dataflow_id, &node_id, "gain", &one)
        .unwrap();
    drop(set_guard);

    let summary = timeout(TokioDuration::from_secs(10), replay)
        .await
        .expect("replay did not finish")
        .unwrap();
    assert_eq!((summary.attempted, summary.failed), (0, 0));
    assert!(
        seen.try_recv().is_err(),
        "nothing may be sent for a key set again"
    );
}

/// A delete the daemon rejects counts as a failed replay item and keeps its
/// tombstone.
#[tokio::test]
async fn replay_counts_a_rejected_delete_as_failed() {
    let (store, dataflow_id, node_id) = store_with_deleted_param();
    let (connection, _seen) = param_daemon_connection(|command| match command {
        DaemonCoordinatorEvent::DeleteParam { .. } => Some(
            DaemonCoordinatorReply::DeleteParamResult(Err("node `sender` not connected".into())),
        ),
        _ => None,
    });

    let summary = timeout(
        TokioDuration::from_secs(10),
        replay_node_params(&store, dataflow_id, &node_id, connection),
    )
    .await
    .expect("replay did not finish");
    assert_eq!((summary.attempted, summary.failed), (1, 1));
    assert_eq!(
        store
            .list_node_param_tombstones(&dataflow_id, &node_id)
            .unwrap()
            .len(),
        1,
        "an undelivered delete keeps its tombstone"
    );
}

/// Once a replay has delivered a delete, its tombstone is gone and the next
/// replay sends nothing for the key.
#[tokio::test]
async fn replay_drops_the_tombstone_of_a_delivered_delete() {
    let (store, dataflow_id, node_id) = store_with_deleted_param();
    for expected_deletes in [1, 0] {
        let (connection, _seen) = param_acking_daemon_connection();
        let summary = timeout(
            TokioDuration::from_secs(10),
            replay_node_params(&store, dataflow_id, &node_id, connection),
        )
        .await
        .expect("replay did not finish");
        assert_eq!((summary.attempted, summary.failed), (expected_deletes, 0));
        assert!(
            store
                .list_node_param_tombstones(&dataflow_id, &node_id)
                .unwrap()
                .is_empty()
        );
    }
}

/// Ask a coordinator whose default daemon connected over `daemon` whether a
/// CLI connected over `cli` shares its machine, through the event loop and
/// the control connection's completion of the reply, as `dora start` does.
async fn cli_and_default_daemon_on_same_machine(
    cli: crate::ws_server::PeerInfo,
    daemon: Option<crate::ws_server::PeerInfo>,
) -> bool {
    let mut daemon_connections = DaemonConnections::default();
    if let Some(daemon) = daemon {
        let (tx, _rx) = tokio::sync::mpsc::channel::<String>(8);
        let mut connection = crate::state::DaemonConnection::new(
            tx,
            Arc::new(tokio::sync::Mutex::new(HashMap::new())),
            BTreeMap::new(),
        );
        connection.peer_addr = Some(daemon.remote);
        connection.coordinator_local_addr = daemon.local;
        daemon_connections.add(DaemonId::new(None), connection);
    }
    let mut coordinator = test_coordinator(
        HashMap::new(),
        daemon_connections,
        Arc::new(InMemoryStore::new()),
    );
    let (reply_sender, reply) = tokio::sync::oneshot::channel();
    coordinator
        .handle_control_event(ControlEvent::IncomingRequest {
            request: Box::new(
                dora_message::cli_to_coordinator::ControlRequest::CliAndDefaultDaemonOnSameMachine,
            ),
            reply_sender,
        })
        .await
        .unwrap();
    let reply = crate::ws_control::fill_in_cli_ip(reply.await.unwrap().unwrap(), cli);
    // The CLI's check in `binaries/cli/src/common.rs`.
    let ControlRequestReply::CliAndDefaultDaemonIps {
        default_daemon,
        cli,
    } = reply
    else {
        panic!("unexpected reply {reply:?}")
    };
    default_daemon.is_some() && default_daemon == cli
}

fn peer(remote: &str, local: &str) -> crate::ws_server::PeerInfo {
    crate::ws_server::PeerInfo {
        remote: remote.parse().unwrap(),
        local: Some(local.parse().unwrap()),
    }
}

/// A CLI on another host must not be told it shares the default daemon's
/// machine: it would send its own working directory, which does not exist
/// there. The coordinator used to answer `127.0.0.1` for both whenever an
/// unnamed daemon was connected.
#[tokio::test]
async fn a_remote_cli_does_not_share_the_default_daemons_machine() {
    // Daemon next to the coordinator, CLI on a laptop.
    assert!(
        !cli_and_default_daemon_on_same_machine(
            peer("192.0.2.7:50000", "192.0.2.1:6013"),
            Some(peer("127.0.0.1:40000", "127.0.0.1:6013")),
        )
        .await
    );
    // Daemon on another host, CLI next to the coordinator.
    assert!(
        !cli_and_default_daemon_on_same_machine(
            peer("127.0.0.1:50000", "127.0.0.1:6013"),
            Some(peer("192.0.2.8:40000", "192.0.2.1:6013")),
        )
        .await
    );
    // No default daemon at all.
    assert!(
        !cli_and_default_daemon_on_same_machine(peer("127.0.0.1:50000", "127.0.0.1:6013"), None)
            .await
    );
}

#[tokio::test]
async fn a_local_cli_shares_the_default_daemons_machine() {
    // Both over loopback.
    assert!(
        cli_and_default_daemon_on_same_machine(
            peer("127.0.0.1:50000", "127.0.0.1:6013"),
            Some(peer("127.0.0.1:40000", "127.0.0.1:6013")),
        )
        .await
    );
    // The daemon dialed the host's LAN IP, the CLI loopback.
    assert!(
        cli_and_default_daemon_on_same_machine(
            peer("127.0.0.1:50000", "127.0.0.1:6013"),
            Some(peer("192.168.1.5:40000", "192.168.1.5:6013")),
        )
        .await
    );
}

/// Deliver the answer of the topology change in flight, as the event loop
/// would: the handlers no longer wait for the daemon, they report back through
/// an `Event::TopologyReply`.
async fn settle_topology_change(
    coordinator: &mut Coordinator,
    events: &mut tokio::sync::mpsc::Receiver<Event>,
) {
    let Some(Event::TopologyReply {
        dataflow_id,
        change_id,
    }) = events.recv().await
    else {
        panic!("expected a topology reply");
    };
    coordinator
        .handle_topology_reply(dataflow_id, change_id)
        .await
        .unwrap();
}

/// A `Coordinator` around the given state, with everything else empty.
fn test_coordinator(
    running_dataflows: HashMap<DataflowId, RunningDataflow>,
    daemon_connections: DaemonConnections,
    store: Arc<dyn CoordinatorStore>,
) -> Coordinator {
    Coordinator {
        resent_finish_reports: Default::default(),
        running_builds: HashMap::new(),
        finished_builds: IndexMap::new(),
        running_dataflows,
        pending_restarts: HashMap::new(),
        reclaimed_launches: HashMap::new(),
        dataflow_results: IndexMap::new(),
        archived_dataflows: IndexMap::new(),
        daemon_connections,
        clock: Arc::new(HLC::default()),
        store,
        span_store: SpanStore::default(),
        daemon_peer_addrs: Default::default(),
        #[cfg(feature = "metrics")]
        otel_metrics: crate::otel_metrics::new_shared(),
        abort_handle: futures::stream::abortable(futures::stream::empty::<()>()).1,
        topology: crate::topology_lane::TopologyLanes::detached(),
    }
}

#[tokio::test]
async fn param_set_waits_for_a_replay_queuing_to_the_same_daemon() {
    // #3683, handler side: a `set` takes the connection's lock before it
    // persists. A replay holding it has read a param from the store and is
    // queuing it; the `set` must neither persist nor forward until then, or
    // the replay could queue the old value after the new one.
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: NodeId = "sender".to_string().into();

    let (tx, rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let connection =
        crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
    let lock = connection.param_write_lock.clone();
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(daemon_id.clone(), connection);
    let daemon_task = spawn_set_param_acking_daemon(rx, pending_replies, |_| {});

    let df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
    let mut coordinator = test_coordinator(
        HashMap::from([(dataflow_id, df)]),
        daemon_connections,
        store.clone(),
    );

    let replay_guard = lock.lock().await;
    let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
    let set = tokio::spawn(async move {
        coordinator
            .handle_set_param(
                dataflow_id,
                node_id,
                "gain".to_string(),
                serde_json::json!(2),
                reply_tx,
            )
            .await
            .unwrap();
        coordinator
    });
    tokio::time::sleep(Duration::from_millis(200)).await;
    assert!(!set.is_finished(), "set must wait for the replay's lock");
    assert_eq!(
        store
            .get_node_param(&dataflow_id, &"sender".to_string().into(), "gain")
            .unwrap(),
        None,
        "set must not persist while a replay is queuing"
    );

    drop(replay_guard);
    let coordinator = tokio::time::timeout(Duration::from_secs(10), set)
        .await
        .expect("set did not finish")
        .unwrap();
    reply_rx.await.unwrap().expect("set is applied");
    drop(coordinator);
    let seen = tokio::time::timeout(Duration::from_secs(10), daemon_task)
        .await
        .expect("daemon task did not finish")
        .unwrap();
    assert_eq!(seen, vec![("gain".to_string(), serde_json::json!(2))]);
}

/// Runs a `dora param set` (or `delete`) against a daemon that never
/// answers, and checks that the handler frees the connection's lock once the
/// forward is queued. Held while waiting for the reply, a slow daemon would
/// stall every replay to it, and the `set` itself runs on the event loop.
async fn assert_param_write_releases_lock_while_waiting(delete: bool) {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: NodeId = "sender".to_string().into();
    let one = serde_json::to_vec(&serde_json::json!(1)).unwrap();
    store
        .put_node_param(&dataflow_id, &node_id, "gain", &one)
        .unwrap();

    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let connection = crate::state::DaemonConnection::new(tx, pending_replies, BTreeMap::new());
    let lock = connection.param_write_lock.clone();
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(daemon_id.clone(), connection);

    let df = test_running_dataflow(dataflow_id, daemon_id, node_id.clone());
    let mut coordinator = test_coordinator(
        HashMap::from([(dataflow_id, df)]),
        daemon_connections,
        store,
    );

    let (reply_tx, _reply_rx) = tokio::sync::oneshot::channel();
    let write = tokio::spawn(async move {
        if delete {
            coordinator
                .handle_delete_param(dataflow_id, node_id, "gain".to_string(), reply_tx)
                .await
        } else {
            coordinator
                .handle_set_param(
                    dataflow_id,
                    node_id,
                    "gain".to_string(),
                    serde_json::json!(2),
                    reply_tx,
                )
                .await
        }
    });

    // The daemon receives the forward but never answers it.
    tokio::time::timeout(Duration::from_secs(10), rx.recv())
        .await
        .expect("handler did not forward the change")
        .expect("connection closed");
    let _replay_guard = tokio::time::timeout(Duration::from_secs(10), lock.lock())
        .await
        .expect("handler must release the lock before waiting for the reply");
    assert!(
        !write.is_finished(),
        "handler is still waiting for the reply"
    );
    write.abort();
}

#[tokio::test]
async fn param_set_releases_param_write_lock_while_waiting_for_the_daemon() {
    assert_param_write_releases_lock_while_waiting(false).await;
}

#[tokio::test]
async fn param_delete_releases_param_write_lock_while_waiting_for_the_daemon() {
    assert_param_write_releases_lock_while_waiting(true).await;
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
async fn replay_reports_failure_when_params_cannot_be_loaded() {
    // A store read error must count as a failed replay. Skipping the node
    // silently yields `failed == 0`, and the pruned-log fallback then marks
    // the daemon caught up although it never received the node's params —
    // and never retries, since its ack now equals the log head.
    let store: Arc<dyn CoordinatorStore> = Arc::new(FailingReadStore::with_failing_param_list());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "camera".to_string().into();

    let (tx, _rx) = tokio::sync::mpsc::channel::<String>(8);
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
    assert_eq!(
        summary.failed, 1,
        "the unreadable node must count as failed"
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

/// A dataflow whose state log was pruned past `daemon_id`'s ack of 3.
fn dataflow_with_pruned_log(
    dataflow_id: DataflowId,
    daemon_id: &DaemonId,
    node_id: &NodeId,
) -> RunningDataflow {
    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());
    dataflow.ready_barrier_released = true;
    dataflow.state_log_sequence = 10;
    dataflow.daemon_ack_sequence.insert(daemon_id.clone(), 3);
    assert!(dataflow.state_log_delta_for_daemon(3, daemon_id).is_none());
    dataflow
}

#[test]
fn pruned_catch_up_keeps_ack_when_daemon_is_disconnected() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: NodeId = "camera".to_string().into();
    let mut dataflow = dataflow_with_pruned_log(dataflow_id, &daemon_id, &node_id);

    catch_up_daemon_behind_pruned_log(
        dataflow_id,
        &mut dataflow,
        &daemon_id,
        false,
        &DaemonConnections::default(),
        &store,
        &Arc::new(HLC::default()),
    );

    assert_eq!(dataflow.daemon_ack_sequence.get(&daemon_id), Some(&3));
}

/// #3684: the ready-barrier replay of the same status report already sends
/// every persisted param, so the pruned-log catch-up must not send them a
/// second time, and must not wait for that replay on the event loop.
#[tokio::test]
async fn pruned_catch_up_reuses_the_ready_barrier_replay() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: NodeId = "camera".to_string().into();
    let value = serde_json::to_vec(&serde_json::json!(1)).unwrap();
    store
        .put_node_param(&dataflow_id, &node_id, "gain", &value)
        .unwrap();
    let mut dataflow = dataflow_with_pruned_log(dataflow_id, &daemon_id, &node_id);

    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(
        daemon_id.clone(),
        crate::state::DaemonConnection::new(tx, pending_replies, BTreeMap::new()),
    );

    catch_up_daemon_behind_pruned_log(
        dataflow_id,
        &mut dataflow,
        &daemon_id,
        true,
        &daemon_connections,
        &store,
        &Arc::new(HLC::default()),
    );

    assert_eq!(dataflow.daemon_ack_sequence.get(&daemon_id), Some(&10));
    tokio::task::yield_now().await;
    assert!(rx.try_recv().is_err(), "no second replay");
}

#[test]
fn pruned_catch_up_before_barrier_release_leaves_the_replay_to_the_release() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: NodeId = "camera".to_string().into();
    let mut dataflow = dataflow_with_pruned_log(dataflow_id, &daemon_id, &node_id);
    dataflow.ready_barrier_released = false;

    catch_up_daemon_behind_pruned_log(
        dataflow_id,
        &mut dataflow,
        &daemon_id,
        false,
        &DaemonConnections::default(),
        &store,
        &Arc::new(HLC::default()),
    );

    assert_eq!(dataflow.daemon_ack_sequence.get(&daemon_id), Some(&10));
}

/// #3684: when nothing else replays the params to the daemon, the catch-up
/// starts a replay itself, in the background: an unresponsive daemon must not
/// stall the coordinator's event loop.
#[tokio::test]
async fn pruned_catch_up_replays_in_the_background_when_nothing_else_does() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: NodeId = "camera".to_string().into();
    let value = serde_json::to_vec(&serde_json::json!(1)).unwrap();
    store
        .put_node_param(&dataflow_id, &node_id, "gain", &value)
        .unwrap();
    let mut dataflow = dataflow_with_pruned_log(dataflow_id, &daemon_id, &node_id);

    // The daemon never answers.
    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(
        daemon_id.clone(),
        crate::state::DaemonConnection::new(tx, pending_replies, BTreeMap::new()),
    );

    catch_up_daemon_behind_pruned_log(
        dataflow_id,
        &mut dataflow,
        &daemon_id,
        false,
        &daemon_connections,
        &store,
        &Arc::new(HLC::default()),
    );
    assert_eq!(dataflow.daemon_ack_sequence.get(&daemon_id), Some(&10));

    #[derive(serde::Deserialize)]
    struct OutboundRaw {
        params: Timestamped<DaemonCoordinatorEvent>,
    }
    let outbound = tokio::time::timeout(Duration::from_secs(10), rx.recv())
        .await
        .expect("replay did not send the param")
        .expect("connection closed");
    let outbound: OutboundRaw = serde_json::from_str(&outbound).unwrap();
    assert!(matches!(
        outbound.params.inner,
        DaemonCoordinatorEvent::SetParam { key, .. } if key == "gain"
    ));
}

/// After the pruned-log catch-up, a `dora param set` that the daemon applies
/// advances its ack like on any other daemon, so it neither stays behind nor
/// holds back log pruning for the dataflow. Acking only when a background
/// replay finished would leave a set applied meanwhile unacked forever.
#[test]
fn param_set_after_pruned_catch_up_is_acked_and_pruned() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: NodeId = "camera".to_string().into();
    let mut dataflow = dataflow_with_pruned_log(dataflow_id, &daemon_id, &node_id);

    catch_up_daemon_behind_pruned_log(
        dataflow_id,
        &mut dataflow,
        &daemon_id,
        true,
        &DaemonConnections::default(),
        &store,
        &Arc::new(HLC::default()),
    );
    let sequence = dataflow.append_state_log(StateCatchUpOperation::SetParam {
        node_id,
        key: "gain".to_string(),
        value: serde_json::json!(2),
    });
    dataflow.ack_applied_state_log_entry(sequence, &daemon_id);

    assert_eq!(dataflow.daemon_ack_sequence.get(&daemon_id), Some(&11));
    assert!(dataflow.state_log.is_empty());
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
        &daemon_connections,
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

/// #3722: an unreachable daemon during rollback must neither stop the
/// remaining daemons from getting their stop, nor replace the daemon's
/// original reason for rejecting the subscription.
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn start_topic_debug_stream_rollback_reaches_every_started_daemon() {
    #[derive(serde::Deserialize)]
    struct OutboundRaw {
        id: String,
        params: Timestamped<DaemonCoordinatorEvent>,
    }

    #[derive(Clone, Copy, PartialEq)]
    enum Behavior {
        RejectStart,
        DropStartReply,
        DropStopReply,
        Accept,
    }

    let dataflow_id = Uuid::new_v4();
    let data_id: dora_core::config::DataId = "message".to_string().into();
    let (frame_tx, _frame_rx) =
        tokio::sync::mpsc::channel::<crate::topic_subscriber::TopicFrame>(4);

    // Daemons are visited in `DaemonId` order, so `daemon-b` (whose stop
    // fails) comes before `daemon-c` (which must still get one). `daemon-d`
    // may have started streaming although its start reply got lost.
    let daemons = [
        ("daemon-a", "sender", Behavior::RejectStart),
        ("daemon-b", "relay", Behavior::DropStopReply),
        ("daemon-c", "sink", Behavior::Accept),
        ("daemon-d", "logger", Behavior::DropStartReply),
    ];

    let mut daemon_connections = DaemonConnections::default();
    let mut daemon_tasks = Vec::new();
    let stopped_daemons = Arc::new(std::sync::Mutex::new(BTreeSet::new()));
    let mut node_daemons = Vec::new();
    for (name, node, behavior) in daemons {
        let daemon_id = DaemonId::new(Some(name.to_string()));
        node_daemons.push((
            dora_message::id::NodeId::from(node.to_string()),
            daemon_id.clone(),
        ));
        let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
        let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
        let connection =
            crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
        daemon_connections.add(daemon_id, connection);

        let stopped_daemons = stopped_daemons.clone();
        daemon_tasks.push(tokio::spawn(async move {
            while let Some(outbound) = rx.recv().await {
                let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
                let request_id = Uuid::parse_str(&outbound_raw.id).expect("valid request id");
                let reply_tx = pending_replies
                    .lock()
                    .await
                    .remove(&request_id)
                    .expect("pending reply sender should exist");
                let reply = match outbound_raw.params.inner {
                    DaemonCoordinatorEvent::StartTopicDebugStream { .. } => {
                        if behavior == Behavior::DropStartReply {
                            continue;
                        }
                        DaemonCoordinatorReply::StartTopicDebugStreamResult(
                            if behavior == Behavior::RejectStart {
                                Err("daemon rejected debug stream".to_string())
                            } else {
                                Ok(())
                            },
                        )
                    }
                    DaemonCoordinatorEvent::StopTopicDebugStream { .. } => {
                        match behavior {
                            // Dropping the reply sender fails the coordinator's
                            // `send_and_receive` like a closed connection does.
                            Behavior::DropStopReply => continue,
                            Behavior::Accept | Behavior::DropStartReply => {
                                stopped_daemons.lock().unwrap().insert(name);
                            }
                            Behavior::RejectStart => {
                                panic!("a daemon that rejected the start must not be stopped")
                            }
                        }
                        DaemonCoordinatorReply::StopTopicDebugStreamResult(Ok(()))
                    }
                    other => panic!("unexpected daemon event during rollback test: {other:?}"),
                };
                let _ = reply_tx.send(serde_json::to_string(&reply).unwrap());
            }
        }));
    }

    let mut running_dataflows = HashMap::new();
    let (first_node, first_daemon) = node_daemons[0].clone();
    let mut dataflow = test_running_dataflow(dataflow_id, first_daemon, first_node);
    dataflow.descriptor.debug.enable_debug_inspection = true;
    let mut descriptor_json = serde_json::to_value(&dataflow.descriptor).unwrap();
    let nodes = descriptor_json
        .get_mut("nodes")
        .and_then(serde_json::Value::as_array_mut)
        .expect("descriptor nodes array");
    for (node_id, daemon_id) in &node_daemons[1..] {
        dataflow
            .node_to_daemon
            .insert(node_id.clone(), daemon_id.clone());
        nodes.push(serde_json::json!({
            "id": node_id,
            "path": node_id,
            "outputs": [data_id.clone()],
        }));
    }
    dataflow.descriptor = serde_json::from_value(descriptor_json).unwrap();
    dataflow.nodes = dataflow
        .descriptor
        .resolve_aliases_and_set_defaults()
        .expect("test descriptor should resolve");
    running_dataflows.insert(dataflow_id, dataflow);

    let err = start_topic_debug_stream(
        &mut running_dataflows,
        &mut daemon_connections,
        dataflow_id,
        node_daemons
            .iter()
            .map(|(node_id, _)| (node_id.clone(), data_id.clone()))
            .collect(),
        frame_tx,
        &HLC::default(),
    )
    .await
    .expect_err("subscription should fail");

    assert!(
        format!("{err:#}").contains("daemon rejected debug stream"),
        "the daemon's rejection must be returned, got: {err:#}"
    );
    // The rollback stops are sent from a spawned task.
    let expected = BTreeSet::from(["daemon-c", "daemon-d"]);
    let stopped = timeout(TokioDuration::from_secs(10), async {
        while *stopped_daemons.lock().unwrap() != expected {
            tokio::time::sleep(TokioDuration::from_millis(5)).await;
        }
    })
    .await;
    assert!(
        stopped.is_ok(),
        "daemon-c must be stopped even though daemon-b's stop failed, and \
         daemon-d although its start reply got lost; stopped: {:?}",
        stopped_daemons.lock().unwrap()
    );
    assert!(running_dataflows[&dataflow_id].topic_subscribers.is_empty());

    for task in daemon_tasks {
        task.abort();
    }
}

/// #3723: `TopicUnsubscribe` runs inside the serial event loop, and the
/// daemon may not answer the stop while it is still flooding the coordinator
/// with frames. The unsubscribe must return with the subscriber removed and
/// the stop in flight, instead of waiting up to `TCP_READ_TIMEOUT` for the
/// reply.
#[tokio::test(flavor = "current_thread", start_paused = true)]
async fn stop_topic_debug_stream_does_not_wait_for_daemon_reply() {
    #[derive(serde::Deserialize)]
    struct OutboundRaw {
        params: Timestamped<DaemonCoordinatorEvent>,
    }

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let data_id: dora_core::config::DataId = "message".to_string().into();
    let subscription_id = Uuid::new_v4();

    // The daemon never answers: nothing ever takes the reply sender out of
    // `pending_replies`.
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
    outputs_by_daemon.insert(daemon_id, vec![(node_id, data_id)]);
    dataflow.topic_subscribers.insert(
        subscription_id,
        crate::topic_subscriber::TopicSubscriber::new(outputs_by_daemon, frame_tx),
    );
    running_dataflows.insert(dataflow_id, dataflow);

    let started = tokio::time::Instant::now();
    stop_topic_debug_stream(
        &mut running_dataflows,
        &mut daemon_connections,
        subscription_id,
        &HLC::default(),
    )
    .expect("unsubscribe should succeed");
    assert!(
        running_dataflows[&dataflow_id].topic_subscribers.is_empty(),
        "the subscriber must be removed right away"
    );

    // The stop still reaches the daemon, from the spawned task.
    let outbound = rx.recv().await.expect("daemon should receive the stop");
    let outbound_raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
    match outbound_raw.params.inner {
        DaemonCoordinatorEvent::StopTopicDebugStream {
            dataflow_id: stop_df,
            subscription_id: stop_sub,
        } => {
            assert_eq!(stop_df, dataflow_id);
            assert_eq!(stop_sub, subscription_id);
        }
        other => panic!("unexpected event on unsubscribe: {other:?}"),
    }
    assert!(
        started.elapsed() < dora_message::TCP_READ_TIMEOUT,
        "unsubscribe must not wait for the daemon's reply"
    );
    assert_eq!(
        pending_replies.lock().await.len(),
        1,
        "the stop's reply should still be outstanding"
    );
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

// The state log is dataflow-wide, but a reconnecting daemon stops replaying
// at the first entry for a node it has no channel for. Entries for another
// daemon's node (or a removed node) must not be sent to it, or they block
// every later entry for its own nodes on every reconnect.
#[test]
fn state_log_delta_for_daemon_keeps_only_its_own_nodes() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let d1 = DaemonId::new(Some("m1".to_string()));
    let d2 = DaemonId::new(Some("m2".to_string()));
    let node_a: dora_core::config::NodeId = "a".to_string().into();
    let node_b: dora_core::config::NodeId = "b".to_string().into();
    let removed: dora_core::config::NodeId = "removed".to_string().into();

    let mut df = test_running_dataflow(dataflow_id, d1.clone(), node_a.clone());
    df.daemons.insert(d2.clone());
    df.node_to_daemon.insert(node_b.clone(), d2.clone());

    for node_id in [&node_b, &removed, &node_a] {
        df.append_state_log(StateCatchUpOperation::SetParam {
            node_id: node_id.clone(),
            key: "k".to_string(),
            value: serde_json::json!(1),
        });
    }

    let for_d1 = df.state_log_delta_for_daemon(0, &d1).expect("not pruned");
    assert_eq!(for_d1.len(), 1);
    assert_eq!(for_d1[0].sequence, 3);

    let for_d2 = df.state_log_delta_for_daemon(0, &d2).expect("not pruned");
    assert_eq!(for_d2.len(), 1);
    assert_eq!(for_d2[0].sequence, 1);

    // Nothing after seq 1 targets d2: an empty delta, not a pruned one.
    let for_d2 = df.state_log_delta_for_daemon(1, &d2).expect("not pruned");
    assert!(for_d2.is_empty());
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
            launch: crate::state::LaunchContext::unknown(),
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
            node_to_daemon: BTreeMap::new(),
            spawn_error: None,
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
            launch: crate::state::LaunchContext::unknown(),
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

/// `dora restart` must relaunch with the original launch context. The
/// replacement used to be spawned with no working dir, a fresh session and
/// no build, so the daemon ran it in `_work/<new session>`: relative node
/// paths stopped resolving and built nodes lost their build.
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn restart_relaunches_with_the_original_launch_context() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    // Unnamed, so nodes without a `deploy` block resolve to it.
    let daemon_id = DaemonId::new(None);

    // A daemon that acks every request and records each `Spawn` it gets.
    let spawns = Arc::new(tokio::sync::Mutex::new(Vec::new()));
    let pending: Arc<tokio::sync::Mutex<HashMap<Uuid, tokio::sync::oneshot::Sender<String>>>> =
        Default::default();
    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let conn = crate::state::DaemonConnection::new(tx, pending.clone(), BTreeMap::new());
    let spawns_task = spawns.clone();
    tokio::spawn(async move {
        #[derive(serde::Deserialize)]
        struct OutboundRaw {
            id: Uuid,
            params: Timestamped<DaemonCoordinatorEvent>,
        }
        while let Some(outbound) = rx.recv().await {
            let outbound: OutboundRaw = serde_json::from_str(&outbound).unwrap();
            let reply = match outbound.params.inner {
                DaemonCoordinatorEvent::Spawn(spawn) => {
                    spawns_task.lock().await.push(spawn);
                    DaemonCoordinatorReply::TriggerSpawnResult(Ok(()))
                }
                DaemonCoordinatorEvent::StopDataflow { .. } => {
                    DaemonCoordinatorReply::StopResult(Ok(()))
                }
                other => panic!("unexpected event: {other:?}"),
            };
            if let Some(sender) = pending.lock().await.remove(&outbound.id) {
                let _ = sender.send(serde_json::to_string(&reply).unwrap());
            }
        }
    });
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(daemon_id.clone(), conn);

    let launch = crate::state::LaunchContext {
        build_id: Some(BuildId::generate()),
        session_id: dora_message::SessionId::generate(),
        local_working_dir: Some("/home/user/project".into()),
        write_events_to: Some("/home/user/events".into()),
    };
    let mut df = test_running_dataflow(dataflow_id, daemon_id, "sender".to_string().into());
    df.launch = launch.clone();
    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);

    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let clock = Arc::new(HLC::default());
    let mut pending_restarts = HashMap::new();
    let (reply_tx, _reply_rx) = tokio::sync::oneshot::channel();
    initiate_restart(
        dataflow_id,
        None,
        false,
        &mut running_dataflows,
        &mut pending_restarts,
        &mut daemon_connections,
        &clock,
        store.as_ref(),
        reply_tx,
    )
    .await;

    // What the event loop does once every daemon reports the old one done.
    let restart = pending_restarts
        .remove(&dataflow_id)
        .expect("restart should be pending");
    let new_dataflow = crate::handlers::start_dataflow(
        restart.descriptor,
        restart.launch,
        restart.name,
        &mut daemon_connections,
        &clock,
        restart.uv,
    )
    .await
    .expect("restart spawn should succeed");

    let spawns = spawns.lock().await;
    let [spawn] = spawns.as_slice() else {
        panic!("expected exactly one spawn, got {}", spawns.len());
    };
    assert_eq!(spawn.local_working_dir, launch.local_working_dir);
    assert_eq!(spawn.session_id, launch.session_id);
    assert_eq!(spawn.build_id, launch.build_id);
    assert_eq!(spawn.write_events_to, launch.write_events_to);
    // ... and kept on the replacement, so a second restart works too.
    assert_eq!(new_dataflow.launch, launch);
}

/// Auto-recovery re-spawns a dataflow on a daemon that reconnects without
/// reporting it. The re-spawn reuses the dataflow's UUID, so it must also
/// reuse its launch context: with a fresh session and no working dir, the
/// daemon ran the re-spawned nodes in a different `_work/<session>` dir
/// than the rest of the dataflow, without its build.
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn auto_recovery_respawns_with_the_original_launch_context() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(None);

    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let conn = crate::state::DaemonConnection::new(tx, Default::default(), BTreeMap::new());
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(daemon_id.clone(), conn);

    let launch = crate::state::LaunchContext {
        build_id: Some(BuildId::generate()),
        session_id: dora_message::SessionId::generate(),
        local_working_dir: Some("/home/user/project".into()),
        write_events_to: Some("/home/user/events".into()),
    };
    let mut df = test_running_dataflow(dataflow_id, daemon_id.clone(), "sender".to_string().into());
    df.launch = launch.clone();
    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);

    let mut coordinator = Coordinator {
        running_builds: HashMap::new(),
        finished_builds: IndexMap::new(),
        running_dataflows,
        pending_restarts: HashMap::new(),
        reclaimed_launches: HashMap::new(),
        dataflow_results: IndexMap::new(),
        resent_finish_reports: HashMap::new(),
        archived_dataflows: IndexMap::new(),
        daemon_connections,
        clock: Arc::new(HLC::default()),
        store: Arc::new(InMemoryStore::new()),
        span_store: SpanStore::default(),
        daemon_peer_addrs: Default::default(),
        #[cfg(feature = "metrics")]
        otel_metrics: crate::otel_metrics::new_shared(),
        abort_handle: futures::stream::abortable(futures::stream::empty::<()>()).1,
        topology: crate::topology_lane::TopologyLanes::detached(),
    };

    // The daemon reconnects reporting no running dataflows.
    coordinator
        .handle_daemon_status_report(daemon_id, vec![])
        .await
        .expect("status report should be handled");

    #[derive(serde::Deserialize)]
    struct OutboundRaw {
        params: Timestamped<DaemonCoordinatorEvent>,
    }
    let mut spawns = Vec::new();
    while let Ok(outbound) = rx.try_recv() {
        let outbound: OutboundRaw = serde_json::from_str(&outbound).unwrap();
        if let DaemonCoordinatorEvent::Spawn(spawn) = outbound.params.inner {
            spawns.push(spawn);
        }
    }
    let [spawn] = spawns.as_slice() else {
        panic!("expected exactly one re-spawn, got {}", spawns.len());
    };
    assert_eq!(spawn.dataflow_id, dataflow_id);
    assert_eq!(spawn.local_working_dir, launch.local_working_dir);
    assert_eq!(spawn.session_id, launch.session_id);
    assert_eq!(spawn.build_id, launch.build_id);
    assert_eq!(spawn.write_events_to, launch.write_events_to);
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
            launch: crate::state::LaunchContext::unknown(),
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

fn deploy_from(value: serde_json::Value) -> dora_message::descriptor::Deploy {
    serde_json::from_value(value).unwrap()
}

#[test]
fn add_node_daemon_honours_deploy_placement() {
    // #3672: `dora node add` used to spawn on whichever daemon of the
    // dataflow sorted first, ignoring the node's `deploy` placement.
    let daemon_a = DaemonId::new(Some("machine-a".to_string()));
    let daemon_b = DaemonId::new(Some("machine-b".to_string()));
    let outsider = DaemonId::new(Some("machine-c".to_string()));

    let mut daemon_connections = DaemonConnections::default();
    for (id, labels) in [
        (&daemon_a, BTreeMap::new()),
        (
            &daemon_b,
            BTreeMap::from([("camera".to_string(), "true".to_string())]),
        ),
        (
            &outsider,
            BTreeMap::from([("gpu".to_string(), "true".to_string())]),
        ),
    ] {
        let (tx, _rx) = tokio::sync::mpsc::channel::<String>(1);
        let pending = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
        daemon_connections.add(
            id.clone(),
            crate::state::DaemonConnection::new(tx, pending, labels),
        );
    }
    let dataflow_daemons = BTreeSet::from([daemon_a.clone(), daemon_b.clone()]);

    // machine placement picks the named daemon, not the first one
    let by_machine = deploy_from(serde_json::json!({ "machine": "machine-b" }));
    assert_eq!(
        daemon_connections
            .resolve_add_node_daemon(&dataflow_daemons, &BTreeSet::new(), Some(&by_machine))
            .unwrap(),
        daemon_b
    );

    // label placement picks the daemon whose labels match
    let by_labels = deploy_from(serde_json::json!({ "labels": { "camera": "true" } }));
    assert_eq!(
        daemon_connections
            .resolve_add_node_daemon(&dataflow_daemons, &BTreeSet::new(), Some(&by_labels))
            .unwrap(),
        daemon_b
    );

    // a placement only a daemon outside the dataflow satisfies is rejected
    let other_machine = deploy_from(serde_json::json!({ "machine": "machine-c" }));
    assert!(
        daemon_connections
            .resolve_add_node_daemon(&dataflow_daemons, &BTreeSet::new(), Some(&other_machine))
            .is_err()
    );
    let other_labels = deploy_from(serde_json::json!({ "labels": { "gpu": "true" } }));
    assert!(
        daemon_connections
            .resolve_add_node_daemon(&dataflow_daemons, &BTreeSet::new(), Some(&other_labels))
            .is_err()
    );

    // no placement keeps working on a dataflow without unnamed daemons
    assert_eq!(
        daemon_connections
            .resolve_add_node_daemon(&dataflow_daemons, &BTreeSet::new(), None)
            .unwrap(),
        daemon_a
    );
}

#[test]
fn add_node_daemon_must_run_beside_its_upstream_producers() {
    // `AddNode` only wires the new node's inputs on the daemon it is sent
    // to, so placing it on another daemon than its producers would leave it
    // without inputs while the CLI reports `NodeAdded`.
    let daemon_a = DaemonId::new(Some("machine-a".to_string()));
    let daemon_b = DaemonId::new(Some("machine-b".to_string()));
    let dataflow_daemons = BTreeSet::from([daemon_a.clone(), daemon_b.clone()]);
    let on_b = BTreeSet::from([daemon_b.clone()]);
    let daemon_connections = DaemonConnections::default();

    // without a placement, the producers' daemon is picked over the first one
    assert_eq!(
        daemon_connections
            .resolve_add_node_daemon(&dataflow_daemons, &on_b, None)
            .unwrap(),
        daemon_b
    );

    // a placement on the producers' daemon is accepted
    let by_machine_b = deploy_from(serde_json::json!({ "machine": "machine-b" }));
    assert_eq!(
        daemon_connections
            .resolve_add_node_daemon(&dataflow_daemons, &on_b, Some(&by_machine_b))
            .unwrap(),
        daemon_b
    );

    // a placement away from the producers is rejected, not silently starved
    let by_machine_a = deploy_from(serde_json::json!({ "machine": "machine-a" }));
    let err = daemon_connections
        .resolve_add_node_daemon(&dataflow_daemons, &on_b, Some(&by_machine_a))
        .unwrap_err();
    assert!(format!("{err}").contains("not supported"), "{err}");

    // producers spread over several daemons can't all be local to one node
    assert!(
        daemon_connections
            .resolve_add_node_daemon(&dataflow_daemons, &dataflow_daemons, None)
            .is_err()
    );
}

#[test]
fn add_node_daemon_without_placement_prefers_unnamed_daemon() {
    let named = DaemonId::new(Some("a-named".to_string()));
    let unnamed = DaemonId::new(None);
    let dataflow_daemons = BTreeSet::from([named.clone(), unnamed.clone()]);
    let daemon_connections = DaemonConnections::default();

    assert_eq!(
        daemon_connections
            .resolve_add_node_daemon(&dataflow_daemons, &BTreeSet::new(), None)
            .unwrap(),
        unnamed
    );
    assert!(
        daemon_connections
            .resolve_add_node_daemon(&BTreeSet::new(), &BTreeSet::new(), None)
            .is_err()
    );
}

/// `dora node add` / `remove` / `replace` change the running descriptor, and
/// a coordinator restarted on a persistent store rebuilds `nodes` from the
/// stored `descriptor_json` (`reestablish_running_dataflow`). So each change
/// has to reach the store, or the restarted coordinator forgets an added
/// node (dropping it from `dora node list` and topic echo/publish) and
/// resurrects a removed one. The stored status must be left as it was.
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn node_topology_changes_are_persisted() {
    #[derive(serde::Deserialize)]
    struct OutboundRaw {
        id: String,
        params: Timestamped<DaemonCoordinatorEvent>,
    }

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(None);

    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let conn = crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(daemon_id.clone(), conn);

    // A daemon that accepts every topology change.
    let daemon_task = tokio::spawn(async move {
        while let Some(outbound) = rx.recv().await {
            let raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
            let reply = match raw.params.inner {
                DaemonCoordinatorEvent::AddNode { .. } => {
                    DaemonCoordinatorReply::AddNodeResult(Ok(()))
                }
                DaemonCoordinatorEvent::RemoveNode { .. } => {
                    DaemonCoordinatorReply::RemoveNodeResult(Ok(()))
                }
                DaemonCoordinatorEvent::ReplaceNode { .. } => {
                    DaemonCoordinatorReply::ReplaceNodeResult(Ok(()))
                }
                other => panic!("unexpected daemon event {other:?}"),
            };
            let request_id = Uuid::parse_str(&raw.id).expect("valid request id");
            if let Some(reply_tx) = pending_replies.lock().await.remove(&request_id) {
                let _ = reply_tx.send(serde_json::to_string(&reply).unwrap());
            }
        }
    });

    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let mut df = test_running_dataflow(dataflow_id, daemon_id, "sender".to_string().into());
    store
        .put_dataflow(&df.make_record(StoreDataflowStatus::Running).unwrap())
        .unwrap();
    let mut running_dataflows = HashMap::new();
    running_dataflows.insert(dataflow_id, df);

    let (topology_tx, mut topology_rx) = tokio::sync::mpsc::channel::<Event>(8);
    let mut coordinator = Coordinator {
        running_builds: HashMap::new(),
        finished_builds: IndexMap::new(),
        running_dataflows,
        pending_restarts: HashMap::new(),
        reclaimed_launches: HashMap::new(),
        dataflow_results: IndexMap::new(),
        resent_finish_reports: HashMap::new(),
        archived_dataflows: IndexMap::new(),
        daemon_connections,
        clock: Arc::new(HLC::default()),
        store: store.clone(),
        span_store: SpanStore::default(),
        daemon_peer_addrs: Default::default(),
        #[cfg(feature = "metrics")]
        otel_metrics: crate::otel_metrics::new_shared(),
        abort_handle: futures::stream::abortable(futures::stream::empty::<()>()).1,
        topology: crate::topology_lane::TopologyLanes::new(topology_tx),
    };

    let stored_nodes = || {
        let record = store.get_dataflow(&dataflow_id).unwrap().expect("record");
        assert_eq!(record.status, StoreDataflowStatus::Running);
        let descriptor: Descriptor = serde_json::from_str(&record.descriptor_json).unwrap();
        descriptor
            .nodes
            .iter()
            .map(|n| (n.id.to_string(), n.path.clone()))
            .collect::<Vec<_>>()
    };
    let node = |id: &str, path: &str| -> dora_message::descriptor::Node {
        serde_json::from_value(serde_json::json!({ "id": id, "path": path })).unwrap()
    };

    let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
    coordinator
        .handle_add_node(dataflow_id, node("extra", "extra-v1"), reply_tx)
        .await
        .unwrap();
    settle_topology_change(&mut coordinator, &mut topology_rx).await;
    reply_rx.await.unwrap().expect("add should succeed");
    assert_eq!(
        stored_nodes(),
        [
            ("sender".to_string(), Some("sender".to_string())),
            ("extra".to_string(), Some("extra-v1".to_string()))
        ]
    );

    let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
    coordinator
        .handle_replace_node(dataflow_id, node("extra", "extra-v2"), None, reply_tx)
        .await
        .unwrap();
    settle_topology_change(&mut coordinator, &mut topology_rx).await;
    reply_rx.await.unwrap().expect("replace should succeed");
    assert_eq!(
        stored_nodes(),
        [
            ("sender".to_string(), Some("sender".to_string())),
            ("extra".to_string(), Some("extra-v2".to_string()))
        ]
    );

    let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
    coordinator
        .handle_remove_node(dataflow_id, "extra".to_string().into(), None, reply_tx)
        .await
        .unwrap();
    settle_topology_change(&mut coordinator, &mut topology_rx).await;
    reply_rx.await.unwrap().expect("remove should succeed");
    assert_eq!(
        stored_nodes(),
        [("sender".to_string(), Some("sender".to_string()))]
    );

    // A dataflow rebuilt after a coordinator restart only maps the nodes its
    // reconnected daemons still run (`RunningDataflow::recovered`), so an
    // exited `sender` is in the descriptor but not in `node_to_daemon`.
    // Re-adding it must be rejected, as it is without a restart, and the
    // stored descriptor left untouched: a duplicate id makes the next
    // re-establish fail to resolve it (#3759).
    coordinator
        .running_dataflows
        .get_mut(&dataflow_id)
        .unwrap()
        .node_to_daemon
        .clear();
    let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
    coordinator
        .handle_add_node(dataflow_id, node("sender", "sender-v2"), reply_tx)
        .await
        .unwrap();
    let err = reply_rx
        .await
        .unwrap()
        .expect_err("re-adding an existing id must be rejected");
    assert!(
        err.to_string().contains("already exists"),
        "unexpected error: {err:?}"
    );
    assert_eq!(
        stored_nodes(),
        [("sender".to_string(), Some("sender".to_string()))]
    );

    drop(coordinator);
    daemon_task.await.unwrap();
}

/// #3872: the WS handler's per-connection subscription list is not told when
/// the coordinator drops a subscriber on its own (a stalled reader evicted,
/// the dataflow ended). `live_topic_subscriptions` reports which of the ids
/// are still registered, so the handler can free the slots of the rest.
#[test]
fn live_topic_subscriptions_keeps_only_registered_ids() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let live_id = Uuid::new_v4();
    let evicted_id = Uuid::new_v4();

    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id, node_id);
    let (frame_tx, _frame_rx) =
        tokio::sync::mpsc::channel::<crate::topic_subscriber::TopicFrame>(4);
    dataflow.topic_subscribers.insert(
        live_id,
        crate::topic_subscriber::TopicSubscriber::new(BTreeMap::new(), frame_tx),
    );
    let running_dataflows = HashMap::from([(dataflow_id, dataflow)]);

    let live =
        crate::topic_debug::live_topic_subscriptions(&running_dataflows, vec![live_id, evicted_id]);

    assert_eq!(live, vec![live_id]);
}

/// #3872: a subscriber the coordinator evicts on its own, here because its
/// receiver went away, must stop being reported live. That is what lets the
/// WS handler free the slot instead of counting it against the cap forever.
#[tokio::test]
async fn evicted_topic_subscriber_is_no_longer_live() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let node_id: dora_core::config::NodeId = "sender".to_string().into();
    let live_id = Uuid::new_v4();
    let evicted_id = Uuid::new_v4();

    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id, node_id);
    let (live_tx, _live_rx) = tokio::sync::mpsc::channel::<crate::topic_subscriber::TopicFrame>(4);
    let (evicted_tx, evicted_rx) =
        tokio::sync::mpsc::channel::<crate::topic_subscriber::TopicFrame>(4);
    drop(evicted_rx);
    dataflow.topic_subscribers.insert(
        live_id,
        crate::topic_subscriber::TopicSubscriber::new(BTreeMap::new(), live_tx),
    );
    dataflow.topic_subscribers.insert(
        evicted_id,
        crate::topic_subscriber::TopicSubscriber::new(BTreeMap::new(), evicted_tx),
    );
    let mut running_dataflows = HashMap::from([(dataflow_id, dataflow)]);
    let ids = vec![live_id, evicted_id];
    assert_eq!(
        crate::topic_debug::live_topic_subscriptions(&running_dataflows, ids.clone()),
        ids,
        "both subscribers are live before the frame"
    );

    crate::topic_debug::forward_topic_frames(
        &mut running_dataflows,
        &mut DaemonConnections::default(),
        dataflow_id,
        ids.clone(),
        vec![1, 2, 3],
        &HLC::default(),
    )
    .await;

    assert_eq!(
        crate::topic_debug::live_topic_subscriptions(&running_dataflows, ids),
        vec![live_id],
        "the subscriber whose receiver closed must no longer be live"
    );
}

/// `dora clean` must leave a dataflow alone while it waits for a daemon to
/// reconnect. Once one daemon of a multi-daemon dataflow has finished, the
/// dataflow has partial `dataflow_results`; if the remaining daemon then drops
/// off, the orphan reclaim removes it from `running_dataflows` and persists
/// `Recovering`. `clean` used to take the partial results for a finished
/// dataflow and delete the stored record, so the reconnecting daemon found
/// nothing to reconcile and its nodes kept running unmanaged.
#[tokio::test]
async fn clean_keeps_a_dataflow_awaiting_reclaim() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let finished_daemon = DaemonId::new(Some("a".to_string()));
    let remaining_daemon = DaemonId::new(Some("b".to_string()));
    let df = test_running_dataflow(dataflow_id, remaining_daemon, "sender".to_string().into());
    let mut coordinator = test_coordinator(
        HashMap::from([(dataflow_id, df)]),
        Default::default(),
        store.clone(),
    );
    coordinator.dataflow_results.insert(
        dataflow_id,
        BTreeMap::from([(
            finished_daemon,
            DataflowDaemonResult {
                timestamp: coordinator.clock.new_timestamp(),
                node_results: BTreeMap::new(),
            },
        )]),
    );
    crate::daemon_liveness::begin_orphaned_dataflow_reclaim(
        dataflow_id,
        &mut coordinator.running_dataflows,
        &mut coordinator.reclaimed_launches,
        &mut coordinator.dataflow_results,
        &mut coordinator.resent_finish_reports,
        &coordinator.clock,
        store.as_ref(),
    )
    .await;
    assert!(matches!(
        store.get_dataflow(&dataflow_id).unwrap().map(|r| r.status),
        Some(StoreDataflowStatus::Recovering)
    ));

    let (reply_sender, reply) = tokio::sync::oneshot::channel();
    coordinator.handle_clean(reply_sender).await.unwrap();
    let Ok(ControlRequestReply::CleanResult { cleaned, failed }) = reply.await.unwrap() else {
        panic!("expected a clean result");
    };
    assert!(failed.is_empty());
    assert!(
        cleaned.0.iter().all(|entry| entry.id.uuid != dataflow_id),
        "a dataflow awaiting reclaim must not be cleaned: {cleaned:?}"
    );
    assert!(
        store.get_dataflow(&dataflow_id).unwrap().is_some(),
        "the Recovering record must survive `dora clean`"
    );
}

// -------------------------------------------------------------------
// Resent finish reports for a dataflow left `Recovering` (dora-rs/dora#3631)
// -------------------------------------------------------------------

/// A `Coordinator` with empty state and the given store, mirroring what
/// `start_inner` builds, without starting the event loop. Handler-level tests
/// that need one event's worth of state use this instead of the WS harness.
fn coordinator_with_store(store: Arc<dyn CoordinatorStore>) -> Coordinator {
    test_coordinator(HashMap::new(), DaemonConnections::default(), store)
}

/// The record `begin_orphaned_dataflow_reclaim` leaves behind: the dataflow is
/// out of `running_dataflows` and `Recovering` in the store, waiting for a
/// daemon to reconnect and say what happened to it. The descriptor lists every
/// node `node_to_daemon` assigns, which is what the coordinator checks a
/// resent report against.
fn recovering_record(
    dataflow_id: DataflowId,
    node_to_daemon: BTreeMap<NodeId, DaemonId>,
) -> dora_coordinator_store::DataflowRecord {
    let descriptor_nodes = node_to_daemon
        .keys()
        .map(|node_id| {
            let id = node_id.to_string();
            serde_json::json!({"id": id, "path": id, "outputs": ["message"]})
        })
        .collect();
    recovering_record_with_nodes(dataflow_id, node_to_daemon, descriptor_nodes)
}

/// Like [`recovering_record`], but with the persisted descriptor written out
/// explicitly. `node_to_daemon` is what the record assigns each node to, which
/// after an earlier recovery can cover fewer nodes than the descriptor lists.
fn recovering_record_with_nodes(
    dataflow_id: DataflowId,
    node_to_daemon: BTreeMap<NodeId, DaemonId>,
    descriptor_nodes: Vec<serde_json::Value>,
) -> dora_coordinator_store::DataflowRecord {
    dora_coordinator_store::DataflowRecord {
        uuid: dataflow_id,
        name: Some("recovering".to_string()),
        descriptor_json: serde_json::json!({ "nodes": descriptor_nodes }).to_string(),
        status: StoreDataflowStatus::Recovering,
        daemon_ids: Vec::new(),
        node_to_daemon: node_to_daemon
            .into_iter()
            .map(|(node, daemon)| (node.to_string(), daemon.to_string()))
            .collect(),
        uv: false,
        ready_barrier_released: true,
        barrier_exited_before_subscribe: Vec::new(),
        generation: 1,
        created_at: 0,
        updated_at: crate::state::now_millis(),
    }
}

fn finish_result(
    clock: &HLC,
    node_results: BTreeMap<NodeId, Result<(), NodeError>>,
) -> DataflowDaemonResult {
    DataflowDaemonResult {
        timestamp: clock.new_timestamp(),
        node_results,
    }
}

/// A finish report the daemon queued on a connection that died is resent when
/// it reconnects (#3612). If the coordinator has already processed the
/// disconnect, the orphan reclaim has taken the dataflow out of
/// `running_dataflows` and left the store record `Recovering`. The resent
/// report must finalize it there instead of being dropped and letting the
/// recovery timeout rewrite it as a terminal `Failed` (#3631).
#[tokio::test]
async fn resent_finish_report_finalizes_a_recovering_dataflow() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("machine".to_string()));
    let node_id: NodeId = "sender".to_string().into();
    store
        .put_dataflow(&recovering_record(
            dataflow_id,
            BTreeMap::from([(node_id.clone(), daemon_id.clone())]),
        ))
        .expect("seed recovering record");
    let mut coordinator = coordinator_with_store(store.clone());
    let clock = coordinator.clock.clone();

    coordinator
        .handle_dataflow_event(
            dataflow_id,
            DataflowEvent::DataflowFinishedOnDaemon {
                daemon_id: daemon_id.clone(),
                result: finish_result(&clock, BTreeMap::from([(node_id, Ok(()))])),
            },
        )
        .await
        .expect("handle the resent finish report");

    let record = store
        .get_dataflow(&dataflow_id)
        .expect("store lookup")
        .expect("record present");
    assert!(
        matches!(record.status, StoreDataflowStatus::Succeeded),
        "a resent finish report must finalize the recovered dataflow, got {:?}",
        record.status
    );
    assert!(
        coordinator.dataflow_results.contains_key(&dataflow_id),
        "the daemon's per-node results must be kept"
    );
}

/// `Recovering` can also mean every daemon dropped while some were still
/// running. A report that covers only part of the dataflow must not finalize
/// it, or the daemon that has not reported yet is stopped as an orphan even
/// though its dataflow is still running.
#[tokio::test]
async fn partial_resent_finish_report_leaves_the_dataflow_recovering() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let finished_daemon = DaemonId::new(Some("done".to_string()));
    let running_daemon = DaemonId::new(Some("running".to_string()));
    let finished_node: NodeId = "sender".to_string().into();
    let running_node: NodeId = "receiver".to_string().into();
    store
        .put_dataflow(&recovering_record(
            dataflow_id,
            BTreeMap::from([
                (finished_node.clone(), finished_daemon.clone()),
                (running_node, running_daemon),
            ]),
        ))
        .expect("seed recovering record");
    let mut coordinator = coordinator_with_store(store.clone());
    let clock = coordinator.clock.clone();

    coordinator
        .handle_dataflow_event(
            dataflow_id,
            DataflowEvent::DataflowFinishedOnDaemon {
                daemon_id: finished_daemon.clone(),
                result: finish_result(&clock, BTreeMap::from([(finished_node, Ok(()))])),
            },
        )
        .await
        .expect("handle the resent finish report");

    let record = store
        .get_dataflow(&dataflow_id)
        .expect("store lookup")
        .expect("record present");
    assert!(
        matches!(record.status, StoreDataflowStatus::Recovering),
        "a report that does not cover the whole dataflow must not finalize it, got {:?}",
        record.status
    );
    // The partial report is parked for the daemons still to report, but it must
    // stay out of `dataflow_results`: every reader there treats an entry as a
    // finished dataflow, so a partial one would make `dora list` say Finished
    // and `dora stop` reply success without stopping the daemon still running.
    assert!(
        !coordinator.dataflow_results.contains_key(&dataflow_id),
        "a partial resent report must not look like a finished dataflow"
    );
    assert!(
        coordinator.resent_finish_reports.contains_key(&dataflow_id),
        "the partial report is parked for the daemons still to report"
    );

    let (tx, rx) = tokio::sync::oneshot::channel();
    coordinator.handle_list(tx).await.expect("handle list");
    let reply = rx.await.expect("list reply").expect("list ok");
    let ControlRequestReply::DataflowList(list) = reply else {
        panic!("expected a dataflow list, got {reply:?}");
    };
    assert!(
        !list.0.iter().any(|entry| entry.id.uuid == dataflow_id),
        "a still-recovering dataflow must not be listed as finished: {:?}",
        list.0
    );
}

/// A daemon never reports a result for a `path: dynamic` node — it sends no
/// `SpawnedNodeResult` — so a coverage check that requires one never passes and
/// the dataflow times out as `Failed`. The dynamic node must be skipped.
#[tokio::test]
async fn resent_finish_report_finalizes_a_dataflow_with_a_dynamic_node() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("machine".to_string()));
    let sender: NodeId = "sender".to_string().into();
    let dynamic: NodeId = "viz".to_string().into();
    store
        .put_dataflow(&recovering_record_with_nodes(
            dataflow_id,
            BTreeMap::from([
                (sender.clone(), daemon_id.clone()),
                (dynamic.clone(), daemon_id.clone()),
            ]),
            vec![
                serde_json::json!({"id": "sender", "path": "sender", "outputs": ["message"]}),
                serde_json::json!({"id": "viz", "path": "dynamic", "outputs": ["message"]}),
            ],
        ))
        .expect("seed recovering record");
    let mut coordinator = coordinator_with_store(store.clone());
    let clock = coordinator.clock.clone();

    coordinator
        .handle_dataflow_event(
            dataflow_id,
            DataflowEvent::DataflowFinishedOnDaemon {
                daemon_id,
                // `viz` is dynamic, so the daemon reports no result for it.
                result: finish_result(&clock, BTreeMap::from([(sender, Ok(()))])),
            },
        )
        .await
        .expect("handle the resent finish report");

    let record = store
        .get_dataflow(&dataflow_id)
        .expect("store lookup")
        .expect("record present");
    assert!(
        matches!(record.status, StoreDataflowStatus::Succeeded),
        "a dataflow whose only unreported node is dynamic must still finalize, got {:?}",
        record.status
    );
}

/// A daemon whose whole share of a dataflow is `path: dynamic` nodes never
/// finishes on its own: `should_finish` (`binaries/daemon/src/node_events.rs`)
/// is only evaluated when a node stops, and its dynamic node is still running.
/// It keeps reporting the dataflow as running. Skipping every dynamic node in
/// the coverage check would settle such a dataflow while that daemon still runs
/// it, and the daemon's next status report would then be stopped as an orphan.
/// Only a dynamic node whose own daemon has reported may be skipped.
#[tokio::test]
async fn resent_finish_report_waits_for_a_dynamic_only_daemon() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    // `saved_*` is what the reclaim persisted; the reports arrive from the
    // fresh `DaemonId`s the daemons registered with after reconnecting. Only
    // the machine id survives that, which is what the check has to match on.
    let saved_a = DaemonId::new(Some("a".to_string()));
    let saved_b = DaemonId::new(Some("b".to_string()));
    let reconnected_a = DaemonId::new(Some("a".to_string()));
    let reconnected_b = DaemonId::new(Some("b".to_string()));
    let sender: NodeId = "sender".to_string().into();
    let dynamic: NodeId = "viz".to_string().into();
    store
        .put_dataflow(&recovering_record_with_nodes(
            dataflow_id,
            BTreeMap::from([(sender.clone(), saved_a), (dynamic.clone(), saved_b)]),
            vec![
                serde_json::json!({"id": "sender", "path": "sender", "outputs": ["message"]}),
                serde_json::json!({"id": "viz", "path": "dynamic", "outputs": ["message"]}),
            ],
        ))
        .expect("seed recovering record");
    let mut coordinator = coordinator_with_store(store.clone());
    let clock = coordinator.clock.clone();

    // `sender` finished on daemon A, but `viz` is still running on daemon B,
    // which hasn't reported. The dataflow must stay `Recovering` so B's status
    // report can re-establish it instead of it being archived and B stopped.
    coordinator
        .handle_dataflow_event(
            dataflow_id,
            DataflowEvent::DataflowFinishedOnDaemon {
                daemon_id: reconnected_a,
                result: finish_result(&clock, BTreeMap::from([(sender.clone(), Ok(()))])),
            },
        )
        .await
        .expect("handle daemon A's resent finish report");

    let record = store
        .get_dataflow(&dataflow_id)
        .expect("store lookup")
        .expect("record present");
    assert!(
        matches!(record.status, StoreDataflowStatus::Recovering),
        "a dynamic node on a daemon that hasn't reported must keep the dataflow \
         Recovering, got {:?}",
        record.status
    );
    assert!(
        !coordinator.dataflow_results.contains_key(&dataflow_id),
        "the partial report must not look like a finished dataflow"
    );

    // Daemon B reports too. It has no node result to send — `viz` is dynamic —
    // but its report covers the dataflow, so it now settles.
    coordinator
        .handle_dataflow_event(
            dataflow_id,
            DataflowEvent::DataflowFinishedOnDaemon {
                daemon_id: reconnected_b,
                result: finish_result(&clock, BTreeMap::new()),
            },
        )
        .await
        .expect("handle daemon B's resent finish report");

    let record = store
        .get_dataflow(&dataflow_id)
        .expect("store lookup")
        .expect("record present");
    assert!(
        matches!(record.status, StoreDataflowStatus::Succeeded),
        "a report from the dynamic node's daemon must let the dataflow settle, got {:?}",
        record.status
    );
    assert!(
        coordinator.dataflow_results.contains_key(&dataflow_id),
        "the settled dataflow keeps the per-daemon results"
    );
}

/// After a dataflow has been recovered once, `node_to_daemon` holds only the
/// reconnecting daemon's nodes (`RunningDataflow::recovered`). A report that
/// covers those must not settle a dataflow whose descriptor still lists a node
/// on another daemon, or that daemon's live nodes are stopped as orphans.
#[tokio::test]
async fn resent_finish_report_does_not_settle_a_dataflow_another_daemon_still_runs() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_a = DaemonId::new(Some("a".to_string()));
    let node_a: NodeId = "a".to_string().into();
    // The record was reclaimed after `a`'s daemon re-established the dataflow
    // on its own, so `node_to_daemon` covers only `a` while the descriptor
    // still lists `b` too.
    store
        .put_dataflow(&recovering_record_with_nodes(
            dataflow_id,
            BTreeMap::from([(node_a.clone(), daemon_a.clone())]),
            vec![
                serde_json::json!({"id": "a", "path": "a", "outputs": ["message"]}),
                serde_json::json!({"id": "b", "path": "b", "outputs": ["message"]}),
            ],
        ))
        .expect("seed recovering record");
    let mut coordinator = coordinator_with_store(store.clone());
    let clock = coordinator.clock.clone();

    coordinator
        .handle_dataflow_event(
            dataflow_id,
            DataflowEvent::DataflowFinishedOnDaemon {
                daemon_id: daemon_a.clone(),
                result: finish_result(&clock, BTreeMap::from([(node_a, Ok(()))])),
            },
        )
        .await
        .expect("handle the resent finish report");

    let record = store
        .get_dataflow(&dataflow_id)
        .expect("store lookup")
        .expect("record present");
    assert!(
        matches!(record.status, StoreDataflowStatus::Recovering),
        "a report covering only the reconnecting daemon's nodes must not settle \
         a dataflow the descriptor still assigns elsewhere, got {:?}",
        record.status
    );
}

/// A dataflow settled from a resent report must be archived like one settled
/// by the normal finish path, or `dora list` loses its name and name-based
/// `logs` / `stop` stop resolving it.
#[tokio::test]
async fn resent_finish_report_archives_the_settled_dataflow() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("machine".to_string()));
    let node_id: NodeId = "sender".to_string().into();
    store
        .put_dataflow(&recovering_record(
            dataflow_id,
            BTreeMap::from([(node_id.clone(), daemon_id.clone())]),
        ))
        .expect("seed recovering record");
    let mut coordinator = coordinator_with_store(store.clone());
    let clock = coordinator.clock.clone();

    coordinator
        .handle_dataflow_event(
            dataflow_id,
            DataflowEvent::DataflowFinishedOnDaemon {
                daemon_id,
                result: finish_result(&clock, BTreeMap::from([(node_id, Ok(()))])),
            },
        )
        .await
        .expect("handle the resent finish report");

    assert!(
        coordinator.archived_dataflows.contains_key(&dataflow_id),
        "a settled dataflow must be archived so its name survives"
    );
    assert_eq!(
        coordinator
            .archived_dataflows
            .get(&dataflow_id)
            .and_then(|archived| archived.name.as_deref()),
        Some("recovering"),
    );
    let resolved = crate::handlers::resolve_name(
        "recovering".to_string(),
        &coordinator.running_dataflows,
        &coordinator.archived_dataflows,
    )
    .expect("the settled dataflow must resolve by name");
    assert_eq!(resolved, dataflow_id);
}

/// When another daemon re-establishes a `Recovering` dataflow as `Running`, the
/// partial reports parked for daemons that already finished belong in
/// `dataflow_results`; otherwise the eventual finish settles without them.
#[tokio::test]
async fn reestablishing_a_dataflow_keeps_parked_finish_reports() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let finished_daemon = DaemonId::new(Some("done".to_string()));
    let running_daemon = DaemonId::new(Some("running".to_string()));
    let finished_node: NodeId = "sender".to_string().into();
    let running_node: NodeId = "receiver".to_string().into();
    let mut record = recovering_record_with_nodes(
        dataflow_id,
        BTreeMap::from([
            (finished_node.clone(), finished_daemon.clone()),
            (running_node.clone(), running_daemon.clone()),
        ]),
        vec![
            serde_json::json!({"id": "sender", "path": "sender", "outputs": ["message"]}),
            serde_json::json!({"id": "receiver", "path": "receiver", "outputs": ["message"]}),
        ],
    );
    record.ready_barrier_released = false;
    store.put_dataflow(&record).expect("seed recovering record");
    let mut coordinator = coordinator_with_store(store.clone());
    let clock = coordinator.clock.clone();

    // `sender`'s daemon finished and its report arrived while the dataflow was
    // still `Recovering`, so it is parked rather than in `dataflow_results`.
    coordinator
        .handle_dataflow_event(
            dataflow_id,
            DataflowEvent::DataflowFinishedOnDaemon {
                daemon_id: finished_daemon.clone(),
                result: finish_result(&clock, BTreeMap::from([(finished_node.clone(), Ok(()))])),
            },
        )
        .await
        .expect("park the partial report");
    assert!(coordinator.resent_finish_reports.contains_key(&dataflow_id));

    // The other daemon reconnects with its node still running.
    coordinator
        .handle_daemon_status_report(
            running_daemon,
            vec![dora_message::daemon_to_coordinator::DataflowStatusEntry {
                dataflow_id,
                running_nodes: vec![running_node],
            }],
        )
        .await
        .expect("handle the status report");

    assert!(
        coordinator.running_dataflows.contains_key(&dataflow_id),
        "the dataflow must be re-established as running"
    );
    assert!(
        coordinator.dataflow_results.contains_key(&dataflow_id),
        "the parked report must move into dataflow_results on re-establish"
    );
    assert!(
        !coordinator.resent_finish_reports.contains_key(&dataflow_id),
        "the parked report must not be left behind"
    );
}

/// A multi-daemon dataflow where one daemon finished before the other dropped
/// off. The orphan reclaim takes it out of `running_dataflows`. Left in
/// `dataflow_results`, the finished daemon's results would read as a finished
/// dataflow: `dora list` would show it Finished, `dora stop` would reply
/// success without stopping the daemon still running it, and
/// `cap_dataflow_results` could evict the results before the reconnecting
/// daemon's resent report needs them to settle the record.
#[tokio::test]
async fn orphan_reclaim_keeps_earlier_finish_reports_out_of_dataflow_results() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let finished_daemon = DaemonId::new(Some("a".to_string()));
    let remaining_daemon = DaemonId::new(Some("b".to_string()));
    let finished_node: NodeId = "early".to_string().into();
    let remaining_node: NodeId = "sender".to_string().into();
    let df = test_running_dataflow(
        dataflow_id,
        remaining_daemon.clone(),
        remaining_node.clone(),
    );
    let mut coordinator = test_coordinator(
        HashMap::from([(dataflow_id, df)]),
        Default::default(),
        store.clone(),
    );
    let clock = coordinator.clock.clone();
    coordinator.dataflow_results.insert(
        dataflow_id,
        BTreeMap::from([(
            finished_daemon.clone(),
            finish_result(&clock, BTreeMap::from([(finished_node.clone(), Ok(()))])),
        )]),
    );
    crate::daemon_liveness::begin_orphaned_dataflow_reclaim(
        dataflow_id,
        &mut coordinator.running_dataflows,
        &mut coordinator.reclaimed_launches,
        &mut coordinator.dataflow_results,
        &mut coordinator.resent_finish_reports,
        &coordinator.clock,
        store.as_ref(),
    )
    .await;
    assert!(matches!(
        store.get_dataflow(&dataflow_id).unwrap().map(|r| r.status),
        Some(StoreDataflowStatus::Recovering)
    ));

    // `dora list` must not show the recovering dataflow as finished.
    let (tx, rx) = tokio::sync::oneshot::channel();
    coordinator.handle_list(tx).await.expect("handle list");
    let ControlRequestReply::DataflowList(list) = rx.await.unwrap().expect("list ok") else {
        panic!("expected a dataflow list");
    };
    assert!(
        !list.0.iter().any(|entry| entry.id.uuid == dataflow_id),
        "a Recovering dataflow must not be listed as finished: {:?}",
        list.0
    );

    // `dora stop` must not report a successful stop it never sent.
    let (tx, rx) = tokio::sync::oneshot::channel();
    coordinator
        .handle_stop(dataflow_id, None, false, tx)
        .await
        .expect("handle stop");
    let reply = rx.await.unwrap();
    assert!(
        !matches!(reply, Ok(ControlRequestReply::DataflowStopped { .. })),
        "stop must not reply success for a dataflow still Recovering: {reply:?}"
    );

    // Finished history filling up must not evict the earlier daemon's results.
    for _ in 0..=MAX_DATAFLOW_RESULTS {
        coordinator
            .dataflow_results
            .insert(DataflowId::from(Uuid::new_v4()), BTreeMap::new());
    }
    cap_dataflow_results(
        &mut coordinator.dataflow_results,
        &coordinator.running_dataflows,
    );

    // The remaining daemon reconnects and resends its finish report: the
    // record settles with both daemons' results.
    coordinator
        .handle_dataflow_event(
            dataflow_id,
            DataflowEvent::DataflowFinishedOnDaemon {
                daemon_id: remaining_daemon.clone(),
                result: finish_result(&clock, BTreeMap::from([(remaining_node, Ok(()))])),
            },
        )
        .await
        .expect("handle the resent finish report");
    let record = store.get_dataflow(&dataflow_id).unwrap().expect("record");
    assert!(
        matches!(record.status, StoreDataflowStatus::Succeeded),
        "the resent report must settle the record, got {:?}",
        record.status
    );
    let results = coordinator
        .dataflow_results
        .get(&dataflow_id)
        .expect("settled results");
    assert!(
        results
            .get(&finished_daemon)
            .is_some_and(|r| r.node_results.contains_key(&finished_node)),
        "the earlier daemon's results must survive the reclaim window: {results:?}"
    );
    assert!(results.contains_key(&remaining_daemon));
}

/// The orphan reclaim parks the results of daemons that already finished,
/// and its `Recovering` write can fail, leaving the record `Running`. A daemon
/// re-reporting the dataflow then takes the `Running` reconcile path, which
/// must merge the parked results back too, or the dataflow settles without
/// them.
#[tokio::test]
async fn reestablishing_a_running_record_keeps_parked_finish_reports() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let finished_daemon = DaemonId::new(Some("done".to_string()));
    let running_daemon = DaemonId::new(Some("running".to_string()));
    let finished_node: NodeId = "sender".to_string().into();
    let running_node: NodeId = "receiver".to_string().into();
    let mut record = recovering_record_with_nodes(
        dataflow_id,
        BTreeMap::from([
            (finished_node.clone(), finished_daemon.clone()),
            (running_node.clone(), running_daemon.clone()),
        ]),
        vec![
            serde_json::json!({"id": "sender", "path": "sender", "outputs": ["message"]}),
            serde_json::json!({"id": "receiver", "path": "receiver", "outputs": ["message"]}),
        ],
    );
    record.status = StoreDataflowStatus::Running;
    store.put_dataflow(&record).expect("seed running record");
    let mut coordinator = coordinator_with_store(store.clone());
    let clock = coordinator.clock.clone();
    coordinator.resent_finish_reports.insert(
        dataflow_id,
        BTreeMap::from([(
            finished_daemon.clone(),
            finish_result(&clock, BTreeMap::from([(finished_node, Ok(()))])),
        )]),
    );

    coordinator
        .handle_daemon_status_report(
            running_daemon,
            vec![dora_message::daemon_to_coordinator::DataflowStatusEntry {
                dataflow_id,
                running_nodes: vec![running_node],
            }],
        )
        .await
        .expect("handle the status report");

    assert!(coordinator.running_dataflows.contains_key(&dataflow_id));
    assert!(
        coordinator
            .dataflow_results
            .get(&dataflow_id)
            .is_some_and(|results| results.contains_key(&finished_daemon)),
        "the parked results must move into dataflow_results on re-establish"
    );
    assert!(!coordinator.resent_finish_reports.contains_key(&dataflow_id));
}

/// A report is only a resend for a dataflow the coordinator lost track of.
/// Once the record has settled — a normal finish, or the recovery-timeout
/// verdict — a late report must keep being dropped, as it was before.
#[tokio::test]
async fn finish_report_for_a_settled_dataflow_is_still_dropped() {
    for settled in [
        StoreDataflowStatus::Succeeded,
        StoreDataflowStatus::Failed {
            error: "recovery timeout (60s): no daemon reconnected".to_string(),
            terminal: true,
        },
    ] {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let daemon_id = DaemonId::new(Some("machine".to_string()));
        let node_id: NodeId = "sender".to_string().into();
        let mut record = recovering_record(
            dataflow_id,
            BTreeMap::from([(node_id.clone(), daemon_id.clone())]),
        );
        record.status = settled.clone();
        store.put_dataflow(&record).expect("seed settled record");
        let mut coordinator = coordinator_with_store(store.clone());
        let clock = coordinator.clock.clone();

        coordinator
            .handle_dataflow_event(
                dataflow_id,
                DataflowEvent::DataflowFinishedOnDaemon {
                    daemon_id,
                    result: finish_result(&clock, BTreeMap::from([(node_id, Ok(()))])),
                },
            )
            .await
            .expect("handle the late finish report");

        let after = store
            .get_dataflow(&dataflow_id)
            .expect("store lookup")
            .expect("record present");
        assert_eq!(
            after.status, settled,
            "a late report must not rewrite a settled verdict"
        );
        assert!(
            !coordinator.dataflow_results.contains_key(&dataflow_id),
            "a settled dataflow must not gain a fresh result entry"
        );
    }
}

/// The first report for a dataflow the coordinator still holds as running goes
/// through the normal finalize path: results recorded, store `Succeeded`,
/// archived so `dora list` still knows its name.
#[tokio::test]
async fn first_finish_report_still_finalizes_a_running_dataflow() {
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("machine".to_string()));
    let node_id: NodeId = "sender".to_string().into();
    let mut dataflow = test_running_dataflow(dataflow_id, daemon_id.clone(), node_id.clone());
    dataflow
        .spawn_result
        .set_result(Ok(ControlRequestReply::DataflowSpawned {
            uuid: dataflow_id,
        }));
    store
        .put_dataflow(&dataflow.make_record(StoreDataflowStatus::Running).unwrap())
        .expect("seed running record");
    let mut coordinator = coordinator_with_store(store.clone());
    coordinator.running_dataflows.insert(dataflow_id, dataflow);
    let clock = coordinator.clock.clone();

    coordinator
        .handle_dataflow_event(
            dataflow_id,
            DataflowEvent::DataflowFinishedOnDaemon {
                daemon_id,
                result: finish_result(&clock, BTreeMap::from([(node_id, Ok(()))])),
            },
        )
        .await
        .expect("handle the first finish report");

    let record = store.get_dataflow(&dataflow_id).unwrap().unwrap();
    assert!(
        matches!(record.status, StoreDataflowStatus::Succeeded),
        "first report must still finalize a running dataflow, got {:?}",
        record.status
    );
    assert!(
        coordinator.archived_dataflows.contains_key(&dataflow_id),
        "the normal path still archives the finished dataflow"
    );
    assert!(coordinator.dataflow_results.contains_key(&dataflow_id));
}

#[test]
fn upstream_node_ids_lists_node_sources_only() {
    let running = running_descriptor_with_operator_producer(serde_json::json!(null));
    let added: Node = serde_json::from_value(serde_json::json!({
        "id": "consumer",
        "path": "consumer",
        "inputs": {
            "reading": "producer/result",
            "tick": "dora/timer/millis/100",
        },
    }))
    .expect("valid node");
    let (_, resolved) = resolve_single_node(added, &running).expect("node resolves");

    let sources: Vec<String> = crate::control_requests::upstream_node_ids(&resolved)
        .map(|id| id.to_string())
        .collect();
    assert_eq!(sources, ["producer"]);
}

#[test]
fn producer_daemons_waits_only_while_the_producers_daemon_is_away() {
    // After a coordinator restart or reclaim, `node_to_daemon` only holds
    // nodes whose daemon has reported them as running. An unmapped producer
    // must block the add while its last daemon is away (else the placement
    // check fails open), but not once that daemon is back: then the producer
    // has exited, and waiting for it would block the add for good.
    let running = running_descriptor_with_operator_producer(serde_json::json!(null));
    let added: Node = serde_json::from_value(serde_json::json!({
        "id": "consumer",
        "path": "consumer",
        "inputs": { "reading": "producer/result" },
    }))
    .expect("valid node");
    let (_, resolved) = resolve_single_node(added, &running).expect("node resolves");

    let producer: NodeId = "producer".to_string().into();
    let producer_node = ResolvedNode::new(
        producer.clone(),
        dora_message::descriptor::CoreNodeKind::Runtime(dora_message::descriptor::RuntimeNode {
            operators: Vec::new(),
        }),
    );
    let nodes = BTreeMap::from([(producer.clone(), producer_node)]);
    let old_daemon = DaemonId::new(Some("machine-a".to_string()));
    let persisted = BTreeMap::from([(producer.clone(), old_daemon.to_string())]);
    let producer_daemons = |node_to_daemon: &BTreeMap<NodeId, DaemonId>,
                            persisted: &BTreeMap<NodeId, String>,
                            connections: &DaemonConnections| {
        crate::control_requests::producer_daemons(
            &resolved,
            &nodes,
            node_to_daemon,
            &node_to_daemon.values().cloned().collect(),
            &BTreeSet::new(),
            &BTreeMap::from([(old_daemon.clone(), std::time::Instant::now())]),
            persisted,
            connections,
        )
    };

    // the producer's machine has no daemon connected: wait
    let mut connections = DaemonConnections::default();
    let err = producer_daemons(&BTreeMap::new(), &persisted, &connections).unwrap_err();
    assert!(format!("{err}").contains("has not reconnected"), "{err}");

    // the machine is back under a new id without reporting the producer:
    // it has exited and doesn't constrain the placement
    let new_daemon = DaemonId::new(Some("machine-a".to_string()));
    let (tx, _rx) = tokio::sync::mpsc::channel::<String>(1);
    let pending = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    connections.add(
        new_daemon.clone(),
        crate::state::DaemonConnection::new(tx, pending, BTreeMap::new()),
    );
    // registered, but its status report (which maps its running nodes) has
    // not been reconciled yet: the producer may still be running there
    let err = producer_daemons(&BTreeMap::new(), &persisted, &connections).unwrap_err();
    assert!(format!("{err}").contains("has not reconnected"), "{err}");
    connections.get_mut(&new_daemon).unwrap().status_reported = true;
    assert!(
        producer_daemons(&BTreeMap::new(), &persisted, &connections)
            .unwrap()
            .is_empty()
    );

    // no stored placement: never block the add for good
    assert!(
        producer_daemons(
            &BTreeMap::new(),
            &BTreeMap::new(),
            &DaemonConnections::default()
        )
        .unwrap()
        .is_empty()
    );

    // a mapped producer is placed by its live daemon
    let mapped = BTreeMap::from([(producer, new_daemon.clone())]);
    assert_eq!(
        producer_daemons(&mapped, &persisted, &DaemonConnections::default()).unwrap(),
        BTreeSet::from([new_daemon])
    );
}

#[tokio::test]
async fn producer_daemons_waits_for_a_reclaimed_dataflows_daemon() {
    // Drive the real reclaim path: the disconnect cleanup empties
    // `df.daemons`, so the reclaim persists no `daemon_ids`. The persisted
    // `node_to_daemon` must still name the producer's machine, so an add
    // made before that daemon is back waits instead of failing open.
    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon = DaemonId::new(Some("machine-a".to_string()));
    let producer: NodeId = "sender".to_string().into();
    let df = test_running_dataflow(dataflow_id, daemon, producer.clone());
    let descriptor = df.descriptor.clone();
    let nodes = df.nodes.clone();
    let mut coordinator = test_coordinator(
        HashMap::from([(dataflow_id, df)]),
        Default::default(),
        store.clone(),
    );
    coordinator
        .running_dataflows
        .get_mut(&dataflow_id)
        .unwrap()
        .daemons
        .clear();
    crate::daemon_liveness::begin_orphaned_dataflow_reclaim(
        dataflow_id,
        &mut coordinator.running_dataflows,
        &mut coordinator.reclaimed_launches,
        &mut coordinator.dataflow_results,
        &mut coordinator.resent_finish_reports,
        &coordinator.clock,
        store.as_ref(),
    )
    .await;
    let record = store.get_dataflow(&dataflow_id).unwrap().expect("record");
    assert!(record.daemon_ids.is_empty());

    let added: Node = serde_json::from_value(serde_json::json!({
        "id": "consumer",
        "path": "consumer",
        "inputs": { "reading": "sender/message" },
    }))
    .expect("valid node");
    let (_, resolved) = resolve_single_node(added, &descriptor).expect("node resolves");

    let err = crate::control_requests::producer_daemons(
        &resolved,
        &nodes,
        &BTreeMap::new(),
        &BTreeSet::new(),
        &BTreeSet::new(),
        &record
            .node_to_daemon
            .values()
            .filter_map(|id| DaemonId::from_display_str(id))
            .map(|id| (id, std::time::Instant::now()))
            .collect(),
        &record
            .node_to_daemon
            .iter()
            .map(|(node, daemon)| (NodeId::from(node.clone()), daemon.clone()))
            .collect(),
        &DaemonConnections::default(),
    )
    .unwrap_err();
    assert!(format!("{err}").contains("`sender`"), "{err}");
}

#[test]
fn recovered_dataflow_keeps_unreported_placement_when_re_persisted() {
    // After a coordinator restart, `recovered` maps only the nodes a
    // reconnected daemon reported. The next `make_record` (e.g. after a
    // successful `dora node add`) must not drop the stored placement of a
    // producer whose daemon is still away, or a later add would treat that
    // producer as exited and place the node away from it (#3672).
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_a = DaemonId::new(Some("machine-a".to_string()));
    let producer: NodeId = "sender".to_string().into();
    let mut original = test_running_dataflow(dataflow_id, daemon_a.clone(), producer.clone());
    let record = original
        .make_record(StoreDataflowStatus::Running)
        .expect("record");

    let daemon_b = DaemonId::new(Some("machine-b".to_string()));
    let mut recovered = RunningDataflow::recovered(
        &record,
        original.descriptor.clone(),
        original.nodes.clone(),
        daemon_b,
        &[],
    );
    assert!(recovered.node_to_daemon.is_empty());
    let re_persisted = recovered
        .make_record(StoreDataflowStatus::Running)
        .expect("record");
    assert_eq!(
        re_persisted.node_to_daemon.get("sender"),
        Some(&daemon_a.to_string())
    );

    let added: Node = serde_json::from_value(serde_json::json!({
        "id": "consumer",
        "path": "consumer",
        "inputs": { "reading": "sender/message" },
    }))
    .expect("valid node");
    let (_, resolved) = resolve_single_node(added, &recovered.descriptor).expect("node resolves");
    assert!(
        crate::control_requests::producer_daemons(
            &resolved,
            &recovered.nodes,
            &recovered.node_to_daemon,
            &recovered.daemons,
            &recovered.exited_nodes,
            &recovered.departed_daemons,
            &RunningDataflow::recovered(
                &re_persisted,
                recovered.descriptor.clone(),
                recovered.nodes.clone(),
                DaemonId::new(Some("machine-b".to_string())),
                &[],
            )
            .persisted_placement,
            &DaemonConnections::default(),
        )
        .is_err(),
        "a producer on a daemon that is still away must keep blocking the add"
    );

    // once its daemon reports it, the live mapping takes over
    recovered.node_to_daemon.insert(producer, daemon_a.clone());
    assert_eq!(
        recovered
            .make_record(StoreDataflowStatus::Running)
            .expect("record")
            .node_to_daemon
            .get("sender"),
        Some(&daemon_a.to_string())
    );
}

/// A connection whose status report has been reconciled.
fn reported_connection(labels: BTreeMap<String, String>) -> crate::state::DaemonConnection {
    let (tx, _rx) = tokio::sync::mpsc::channel::<String>(1);
    let pending = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let mut connection = crate::state::DaemonConnection::new(tx, pending, labels);
    connection.status_reported = true;
    connection
}

#[test]
fn producer_daemons_ignores_exited_producers() {
    // An exited producer sends nothing more, so it must not pin the new
    // node to its daemon, whether the coordinator saw it exit (finalized)
    // or its daemon's whole share finished (removed from `daemons` but
    // still mapped and connected).
    let descriptor: Descriptor = serde_json::from_value(serde_json::json!({
        "nodes": [
            { "id": "camera", "path": "camera", "outputs": ["image"] },
            { "id": "config_loader", "path": "config_loader", "outputs": ["config"] },
        ]
    }))
    .expect("valid descriptor");
    let nodes = descriptor
        .resolve_aliases_and_set_defaults()
        .expect("resolves");
    let added: Node = serde_json::from_value(serde_json::json!({
        "id": "consumer",
        "path": "consumer",
        "inputs": { "image": "camera/image", "config": "config_loader/config" },
    }))
    .expect("valid node");
    let (_, resolved) = resolve_single_node(added, &descriptor).expect("node resolves");

    let daemon_a = DaemonId::new(Some("machine-a".to_string()));
    let daemon_b = DaemonId::new(Some("machine-b".to_string()));
    let node_to_daemon = BTreeMap::from([
        ("camera".to_string().into(), daemon_a.clone()),
        ("config_loader".to_string().into(), daemon_b.clone()),
    ]);
    let mut connections = DaemonConnections::default();
    connections.add(daemon_a.clone(), reported_connection(BTreeMap::new()));
    connections.add(daemon_b.clone(), reported_connection(BTreeMap::new()));
    let both = BTreeSet::from([daemon_a.clone(), daemon_b.clone()]);
    let producer_daemons = |dataflow_daemons: &BTreeSet<DaemonId>,
                            finalized: &BTreeSet<NodeId>,
                            connections: &DaemonConnections| {
        crate::control_requests::producer_daemons(
            &resolved,
            &nodes,
            &node_to_daemon,
            dataflow_daemons,
            finalized,
            &BTreeMap::from([(daemon_b.clone(), std::time::Instant::now())]),
            &BTreeMap::new(),
            connections,
        )
    };

    // both producers running: both daemons constrain the placement
    assert_eq!(
        producer_daemons(&both, &BTreeSet::new(), &connections).unwrap(),
        both
    );
    // `config_loader` exited while machine-b keeps running other nodes
    let finalized = BTreeSet::from(["config_loader".to_string().into()]);
    assert_eq!(
        producer_daemons(&both, &finalized, &connections).unwrap(),
        BTreeSet::from([daemon_a.clone()])
    );
    // machine-b's whole share finished: it left `daemons`, stays connected
    let only_a = BTreeSet::from([daemon_a.clone()]);
    assert_eq!(
        producer_daemons(&only_a, &BTreeSet::new(), &connections).unwrap(),
        BTreeSet::from([daemon_a.clone()])
    );
    // machine-b left `daemons` because it disconnected: it may come back
    let mut without_b = DaemonConnections::default();
    without_b.add(daemon_a, reported_connection(BTreeMap::new()));
    assert!(producer_daemons(&only_a, &BTreeSet::new(), &without_b).is_err());
}

#[test]
fn add_node_daemon_label_placement_prefers_the_producers_daemon() {
    // When several daemons match the labels, picking the first one could
    // fail the co-location check although the producers' daemon matches too.
    let daemon_a = DaemonId::new(Some("machine-a".to_string()));
    let daemon_b = DaemonId::new(Some("machine-b".to_string()));
    let gpu = BTreeMap::from([("gpu".to_string(), "true".to_string())]);
    let mut connections = DaemonConnections::default();
    connections.add(daemon_a.clone(), reported_connection(gpu.clone()));
    connections.add(daemon_b.clone(), reported_connection(gpu));
    let dataflow_daemons = BTreeSet::from([daemon_a, daemon_b.clone()]);

    let by_labels = deploy_from(serde_json::json!({ "labels": { "gpu": "true" } }));
    assert_eq!(
        connections
            .resolve_add_node_daemon(
                &dataflow_daemons,
                &BTreeSet::from([daemon_b.clone()]),
                Some(&by_labels),
            )
            .unwrap(),
        daemon_b
    );
}

#[tokio::test]
async fn status_report_marks_the_connection_as_reported() {
    // `dora node add` only treats an unreported producer as exited once a
    // daemon of its machine has reconciled its status report.
    let daemon_id = DaemonId::new(Some("machine-a".to_string()));
    let (tx, _rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(
        daemon_id.clone(),
        crate::state::DaemonConnection::new(tx, pending, BTreeMap::new()),
    );
    let mut coordinator = test_coordinator(
        HashMap::new(),
        daemon_connections,
        Arc::new(InMemoryStore::new()),
    );
    assert!(
        !coordinator
            .daemon_connections
            .has_reported_daemon_on_machine_of(&daemon_id)
    );

    coordinator
        .handle_daemon_status_report(daemon_id.clone(), vec![])
        .await
        .expect("status report should be handled");
    assert!(
        coordinator
            .daemon_connections
            .has_reported_daemon_on_machine_of(&daemon_id)
    );
}

#[tokio::test]
async fn exited_producer_stays_ignored_after_the_stopped_grace_period() {
    // `expire_stopped_nodes` clears `node_finalized` for a cleanly stopped
    // node after `NODE_STOPPED_GRACE`, but the node keeps its
    // `node_to_daemon` entry. The placement check must still treat it as
    // exited afterwards, or it pins the new node to that daemon again.
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_b = DaemonId::new(Some("machine-b".to_string()));
    let producer: NodeId = "sender".to_string().into();
    let df = test_running_dataflow(dataflow_id, daemon_b.clone(), producer.clone());
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(daemon_b.clone(), reported_connection(BTreeMap::new()));
    let mut coordinator = test_coordinator(
        HashMap::from([(dataflow_id, df)]),
        daemon_connections,
        Arc::new(InMemoryStore::new()),
    );

    coordinator
        .handle_daemon_node_stopped(daemon_b.clone(), dataflow_id, producer.clone(), true)
        .await
        .expect("node stop should be handled");
    let dataflow = coordinator.running_dataflows.get_mut(&dataflow_id).unwrap();
    let stopped_at = dataflow
        .node_stopped_at
        .get_mut(&producer)
        .expect("stopped");
    *stopped_at -= NODE_STOPPED_GRACE + std::time::Duration::from_secs(1);
    crate::topology::expire_stopped_nodes(dataflow);
    assert!(!dataflow.node_finalized.contains(&producer));
    assert_eq!(dataflow.node_to_daemon.get(&producer), Some(&daemon_b));

    let added: Node = serde_json::from_value(serde_json::json!({
        "id": "consumer",
        "path": "consumer",
        "inputs": { "reading": "sender/message" },
    }))
    .expect("valid node");
    let (_, resolved) = resolve_single_node(added, &dataflow.descriptor).expect("node resolves");
    assert!(
        crate::control_requests::producer_daemons(
            &resolved,
            &dataflow.nodes,
            &dataflow.node_to_daemon,
            &dataflow.daemons,
            &dataflow.exited_nodes,
            &dataflow.departed_daemons,
            &BTreeMap::new(),
            &coordinator.daemon_connections,
        )
        .unwrap()
        .is_empty(),
        "an exited producer must not constrain the placement"
    );
}

#[test]
fn producer_daemons_stops_waiting_for_a_daemon_that_never_returns() {
    // A daemon that disconnects for good (crash, retired machine) leaves its
    // nodes in `node_to_daemon` without a `NodeStopped`. `dora node add`
    // may wait for it to come back, but not forever.
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_a = DaemonId::new(Some("machine-a".to_string()));
    let daemon_b = DaemonId::new(Some("machine-b".to_string()));
    let producer: NodeId = "sender".to_string().into();
    let mut df = test_running_dataflow(dataflow_id, daemon_b.clone(), producer.clone());
    df.daemons.insert(daemon_a.clone());
    let mut running_dataflows = HashMap::from([(dataflow_id, df)]);

    // the real disconnect cleanup records when machine-b was last seen
    crate::daemon_liveness::cleanup_disconnected_daemons_from_running_dataflows(
        &mut running_dataflows,
        &BTreeSet::from([daemon_b.clone()]),
        &mut HashMap::new(),
    );
    let dataflow = running_dataflows.get_mut(&dataflow_id).unwrap();
    assert!(!dataflow.daemons.contains(&daemon_b));
    assert!(dataflow.departed_daemons.contains_key(&daemon_b));

    let added: Node = serde_json::from_value(serde_json::json!({
        "id": "consumer",
        "path": "consumer",
        "inputs": { "reading": "sender/message" },
    }))
    .expect("valid node");
    let (_, resolved) = resolve_single_node(added, &dataflow.descriptor).expect("node resolves");
    let mut connections = DaemonConnections::default();
    connections.add(daemon_a, reported_connection(BTreeMap::new()));
    let producer_daemons = |departed: &BTreeMap<DaemonId, std::time::Instant>| {
        crate::control_requests::producer_daemons(
            &resolved,
            &dataflow.nodes,
            &dataflow.node_to_daemon,
            &dataflow.daemons,
            &dataflow.exited_nodes,
            departed,
            &BTreeMap::new(),
            &connections,
        )
    };

    // just disconnected: wait, and say how to get out
    let err = producer_daemons(&dataflow.departed_daemons).unwrap_err();
    assert!(format!("{err}").contains("dora node remove"), "{err}");

    // gone for longer than the grace: the producer counts as gone
    let long_ago = BTreeMap::from([(
        daemon_b,
        std::time::Instant::now()
            - crate::control_requests::PRODUCER_DAEMON_RECONNECT_GRACE
            - std::time::Duration::from_secs(1),
    )]);
    assert!(producer_daemons(&long_ago).unwrap().is_empty());
}

/// Each daemon pushes its own cross-daemon Zenoh counters for a dataflow, so
/// a node's `network` row must come from the daemon running it — not from
/// whichever daemon of a multi-daemon dataflow happened to push last.
#[tokio::test]
async fn node_info_reports_the_network_counters_of_the_nodes_own_daemon() {
    use dora_message::daemon_to_coordinator::NetworkMetrics;

    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_a = DaemonId::new(Some("a".to_string()));
    let daemon_b = DaemonId::new(Some("b".to_string()));
    let sender: NodeId = "sender".to_string().into();
    let receiver: NodeId = "receiver".to_string().into();

    let mut df = test_running_dataflow(dataflow_id, daemon_a.clone(), sender.clone());
    let receiver_node = df.nodes[&sender].clone();
    df.nodes.insert(receiver.clone(), receiver_node);
    df.node_to_daemon.insert(receiver.clone(), daemon_b.clone());
    df.daemons.insert(daemon_b.clone());

    let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
    let mut coordinator = test_coordinator(
        HashMap::from([(dataflow_id, df)]),
        DaemonConnections::default(),
        store,
    );
    let net = |bytes_sent| NetworkMetrics {
        bytes_sent,
        ..Default::default()
    };
    coordinator
        .handle_node_metrics(dataflow_id, daemon_a, BTreeMap::new(), Some(net(1)))
        .await
        .unwrap();
    coordinator
        .handle_node_metrics(dataflow_id, daemon_b, BTreeMap::new(), Some(net(2)))
        .await
        .unwrap();

    let (reply_sender, reply) = tokio::sync::oneshot::channel();
    coordinator
        .handle_get_node_info(reply_sender)
        .await
        .unwrap();
    let Ok(ControlRequestReply::NodeInfoList(infos)) = reply.await.unwrap() else {
        panic!("expected a NodeInfoList reply");
    };
    let bytes_sent = |node: &NodeId| {
        infos
            .iter()
            .find(|info| &info.node_id == node)
            .and_then(|info| info.network.as_ref())
            .map(|net| net.bytes_sent)
    };
    assert_eq!(bytes_sent(&sender), Some(1), "{infos:?}");
    assert_eq!(bytes_sent(&receiver), Some(2), "{infos:?}");
}

/// Module expansion sets `path_fallback`, but only spawning and local
/// validation read it, and a CLI built with an older dora parses the
/// `DataflowInfo` descriptor with `deny_unknown_fields` (`dora node info`,
/// `dora topic echo`), so the reply carries the descriptor without the field.
#[tokio::test]
async fn dataflow_info_leaves_out_path_fallback() {
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let daemon_id = DaemonId::new(Some("m1".to_string()));
    let mut df = test_running_dataflow(dataflow_id, daemon_id, "m.sender".to_string().into());
    let node = r#"{"id": "m.sender", "path": "modules/sender", "outputs": ["message"]}"#;
    let without: Descriptor =
        serde_json::from_str(&format!(r#"{{"nodes": [{node}]}}"#)).expect("valid descriptor");
    df.descriptor = serde_json::from_str(&format!(
        r#"{{"nodes": [{}]}}"#,
        node.replace('}', r#", "path_fallback": "sender"}"#)
    ))
    .expect("valid descriptor");
    assert_eq!(
        df.descriptor.nodes[0].path_fallback.as_deref(),
        Some("sender")
    );
    let mut coordinator = test_coordinator(
        HashMap::from([(dataflow_id, df)]),
        DaemonConnections::default(),
        Arc::new(InMemoryStore::new()),
    );

    let (reply_sender, reply) = tokio::sync::oneshot::channel();
    coordinator
        .handle_control_event(ControlEvent::IncomingRequest {
            request: Box::new(dora_message::cli_to_coordinator::ControlRequest::Info {
                dataflow_uuid: dataflow_id,
            }),
            reply_sender,
        })
        .await
        .unwrap();
    let ControlRequestReply::DataflowInfo { descriptor, .. } = reply.await.unwrap().unwrap() else {
        panic!("expected a DataflowInfo reply");
    };
    assert_eq!(
        serde_json::to_value(&descriptor).unwrap(),
        serde_json::to_value(&without).unwrap()
    );
}

/// A coordinator whose dataflow runs on machine-a (accepting every
/// `AddNode`) and machine-b, with `sender` on machine-b.
fn two_daemon_add_node_coordinator(dataflow_id: DataflowId) -> (Coordinator, DaemonId, DaemonId) {
    let daemon_a = DaemonId::new(Some("machine-a".to_string()));
    let daemon_b = DaemonId::new(Some("machine-b".to_string()));
    let (tx, mut rx) = tokio::sync::mpsc::channel::<String>(8);
    let pending_replies = Arc::new(tokio::sync::Mutex::new(HashMap::new()));
    let mut conn_a =
        crate::state::DaemonConnection::new(tx, pending_replies.clone(), BTreeMap::new());
    conn_a.status_reported = true;
    tokio::spawn(async move {
        #[derive(serde::Deserialize)]
        struct OutboundRaw {
            id: String,
            params: Timestamped<DaemonCoordinatorEvent>,
        }
        while let Some(outbound) = rx.recv().await {
            let raw: OutboundRaw = serde_json::from_str(&outbound).unwrap();
            let reply = match raw.params.inner {
                DaemonCoordinatorEvent::AddNode { .. } => {
                    DaemonCoordinatorReply::AddNodeResult(Ok(()))
                }
                other => panic!("unexpected daemon event {other:?}"),
            };
            let request_id = Uuid::parse_str(&raw.id).expect("valid request id");
            if let Some(reply_tx) = pending_replies.lock().await.remove(&request_id) {
                let _ = reply_tx.send(serde_json::to_string(&reply).unwrap());
            }
        }
    });
    let mut daemon_connections = DaemonConnections::default();
    daemon_connections.add(daemon_a.clone(), conn_a);
    daemon_connections.add(daemon_b.clone(), reported_connection(BTreeMap::new()));

    let mut df = test_running_dataflow(dataflow_id, daemon_b.clone(), "sender".to_string().into());
    df.daemons.insert(daemon_a.clone());
    let coordinator = test_coordinator(
        HashMap::from([(dataflow_id, df)]),
        daemon_connections,
        Arc::new(InMemoryStore::new()),
    );
    (coordinator, daemon_a, daemon_b)
}

async fn add_consumer(
    coordinator: &mut Coordinator,
    dataflow_id: DataflowId,
    node: serde_json::Value,
) -> eyre::Result<ControlRequestReply> {
    let node: Node = serde_json::from_value(node).expect("valid node");
    let (topology_tx, mut topology_rx) = tokio::sync::mpsc::channel::<Event>(8);
    coordinator.topology = crate::topology_lane::TopologyLanes::new(topology_tx);
    let (reply_tx, mut reply_rx) = tokio::sync::oneshot::channel();
    coordinator
        .handle_add_node(dataflow_id, node, reply_tx)
        .await
        .unwrap();
    // A placement the coordinator rejects is answered at once; an accepted add
    // is answered once the daemon's reply comes back as an event, which the
    // event loop would deliver.
    loop {
        tokio::select! {
            reply = &mut reply_rx => return reply.unwrap(),
            Some(Event::TopologyReply { dataflow_id, change_id }) = topology_rx.recv() => {
                coordinator
                    .handle_topology_reply(dataflow_id, change_id)
                    .await
                    .unwrap();
            }
        }
    }
}

#[tokio::test]
async fn add_node_ignores_a_producer_that_exited_past_the_stopped_grace() {
    // Through `handle_add_node`: `sender` exited on machine-b and its
    // stopped grace expired, so a node placed on machine-a is accepted.
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let (mut coordinator, daemon_a, daemon_b) = two_daemon_add_node_coordinator(dataflow_id);
    let producer: NodeId = "sender".to_string().into();
    coordinator
        .handle_daemon_node_stopped(daemon_b, dataflow_id, producer.clone(), true)
        .await
        .expect("node stop should be handled");
    let dataflow = coordinator.running_dataflows.get_mut(&dataflow_id).unwrap();
    *dataflow.node_stopped_at.get_mut(&producer).unwrap() -=
        NODE_STOPPED_GRACE + std::time::Duration::from_secs(1);
    crate::topology::expire_stopped_nodes(dataflow);

    add_consumer(
        &mut coordinator,
        dataflow_id,
        serde_json::json!({
            "id": "consumer",
            "path": "consumer",
            "inputs": { "reading": "sender/message" },
            "deploy": { "machine": "machine-a" },
        }),
    )
    .await
    .expect("an exited producer must not constrain the placement");
    assert_eq!(
        coordinator.running_dataflows[&dataflow_id].node_to_daemon
            [&NodeId::from("consumer".to_string())],
        daemon_a
    );
}

#[tokio::test]
async fn add_node_waits_for_a_producer_daemon_that_just_disconnected() {
    // Through `handle_add_node`: machine-b just disconnected with `sender`
    // on it, so the add must wait rather than land on machine-a.
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let (mut coordinator, _daemon_a, daemon_b) = two_daemon_add_node_coordinator(dataflow_id);
    crate::daemon_liveness::cleanup_disconnected_daemons_from_running_dataflows(
        &mut coordinator.running_dataflows,
        &BTreeSet::from([daemon_b.clone()]),
        &mut HashMap::new(),
    );
    coordinator.daemon_connections.remove(&daemon_b);

    let err = add_consumer(
        &mut coordinator,
        dataflow_id,
        serde_json::json!({
            "id": "consumer",
            "path": "consumer",
            "inputs": { "reading": "sender/message" },
        }),
    )
    .await
    .unwrap_err();
    assert!(
        format!("{err:#}").contains("has not reconnected"),
        "{err:#}"
    );
}

#[tokio::test]
async fn auto_recovery_respawn_makes_an_exited_producer_count_again() {
    // `sender` crashed on machine-b, then machine-b's daemon re-registered
    // (still in `daemons`) without the dataflow, so auto-recovery re-spawns
    // `sender`. It runs again, so a node placed on machine-a must be
    // rejected instead of silently missing its input.
    let dataflow_id = DataflowId::from(Uuid::new_v4());
    let (mut coordinator, _daemon_a, daemon_b) = two_daemon_add_node_coordinator(dataflow_id);
    let producer: NodeId = "sender".to_string().into();
    coordinator
        .handle_daemon_node_stopped(daemon_b.clone(), dataflow_id, producer.clone(), false)
        .await
        .expect("node stop should be handled");
    assert!(
        coordinator.running_dataflows[&dataflow_id]
            .exited_nodes
            .contains(&producer)
    );

    coordinator
        .handle_daemon_status_report(daemon_b, vec![])
        .await
        .expect("status report should be handled");
    assert!(
        !coordinator.running_dataflows[&dataflow_id]
            .exited_nodes
            .contains(&producer)
    );

    let err = add_consumer(
        &mut coordinator,
        dataflow_id,
        serde_json::json!({
            "id": "consumer",
            "path": "consumer",
            "inputs": { "reading": "sender/message" },
            "deploy": { "machine": "machine-a" },
        }),
    )
    .await
    .unwrap_err();
    assert!(format!("{err:#}").contains("not supported yet"), "{err:#}");
}

// ---------------------------------------------------------------------------
// Node-lifecycle changes (`AddNode`, `RemoveNode`, `ReplaceNode`, `AddMapping`,
// `RemoveMapping`) must not hold the serial coordinator event loop while they
// wait for a daemon (see `topology_lane`).
//
// The tests drive the REAL handlers, `handle_topology_reply`,
// `handle_heartbeat_interval` and `handle_daemon_heartbeat` through a loop with
// the same shape as `start_inner` (futures_concurrency `Merge` of the external
// events, the 3 s heartbeat interval and the topology replies, strictly
// sequential dispatch), against fake daemons that delay or never reply.
// ---------------------------------------------------------------------------

mod lifecycle_stall {
    use super::*;
    use dora_message::{
        cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply,
    };
    use tokio::sync::{mpsc, oneshot};

    type PendingReplies = Arc<tokio::sync::Mutex<HashMap<Uuid, oneshot::Sender<String>>>>;
    type CliReply = oneshot::Receiver<eyre::Result<ControlRequestReply>>;

    #[derive(serde::Deserialize)]
    struct FrameRaw {
        id: String,
        params: Timestamped<DaemonCoordinatorEvent>,
    }

    #[derive(Clone, Copy)]
    enum ReplyPolicy {
        /// A hung daemon: the command is received and never answered or applied.
        Never,
        /// Applies every command after the delay, then answers. If the
        /// coordinator has stopped waiting by then, the answer is lost (like
        /// the real daemon: `let _ = reply_tx.send(..)`).
        After(Duration),
        /// Per command name: `None` hangs, `Some(delay)` as in `After`.
        Custom(fn(&'static str) -> Option<Duration>),
    }

    struct FakeDaemon {
        connection: crate::state::DaemonConnection,
        heartbeats: Arc<std::sync::Mutex<Vec<tokio::time::Instant>>>,
        /// Commands the daemon applied, in order.
        applied: Arc<std::sync::Mutex<Vec<&'static str>>>,
    }

    fn fake_daemon(policy: ReplyPolicy) -> FakeDaemon {
        let (tx, mut rx) = mpsc::channel::<String>(64);
        let pending: PendingReplies = Default::default();
        let connection = crate::state::DaemonConnection::new(tx, pending.clone(), BTreeMap::new());
        let heartbeats = Arc::new(std::sync::Mutex::new(Vec::new()));
        let applied = Arc::new(std::sync::Mutex::new(Vec::new()));
        let (hb, ap) = (heartbeats.clone(), applied.clone());
        tokio::spawn(async move {
            while let Some(frame) = rx.recv().await {
                let Ok(raw) = serde_json::from_str::<FrameRaw>(&frame) else {
                    continue;
                };
                let (name, reply) = match raw.params.inner {
                    DaemonCoordinatorEvent::Heartbeat => {
                        hb.lock().unwrap().push(tokio::time::Instant::now());
                        continue;
                    }
                    DaemonCoordinatorEvent::AddNode { .. } => {
                        ("AddNode", DaemonCoordinatorReply::AddNodeResult(Ok(())))
                    }
                    DaemonCoordinatorEvent::RemoveNode { .. } => (
                        "RemoveNode",
                        DaemonCoordinatorReply::RemoveNodeResult(Ok(())),
                    ),
                    DaemonCoordinatorEvent::ReplaceNode { .. } => (
                        "ReplaceNode",
                        DaemonCoordinatorReply::ReplaceNodeResult(Ok(())),
                    ),
                    DaemonCoordinatorEvent::AddMapping { .. } => (
                        "AddMapping",
                        DaemonCoordinatorReply::AddMappingResult(Ok(())),
                    ),
                    DaemonCoordinatorEvent::RemoveMapping { .. } => (
                        "RemoveMapping",
                        DaemonCoordinatorReply::RemoveMappingResult(Ok(())),
                    ),
                    _ => continue,
                };
                let delay = match policy {
                    ReplyPolicy::Never => None,
                    ReplyPolicy::After(d) => Some(d),
                    ReplyPolicy::Custom(f) => f(name),
                };
                let Some(delay) = delay else {
                    continue; // hung: neither applies nor answers
                };
                let (pending, ap) = (pending.clone(), ap.clone());
                tokio::spawn(async move {
                    tokio::time::sleep(delay).await;
                    ap.lock().unwrap().push(name);
                    let id = Uuid::parse_str(&raw.id).unwrap();
                    if let Some(tx) = pending.lock().await.remove(&id) {
                        let _ = tx.send(serde_json::to_string(&reply).unwrap());
                    }
                });
            }
        });
        FakeDaemon {
            connection,
            heartbeats,
            applied,
        }
    }

    /// Same dispatch shape as `start_inner` (lib.rs). Returns the coordinator
    /// once `Event::CtrlC` arrives.
    async fn serial_loop(
        mut c: Coordinator,
        events: mpsc::Receiver<Event>,
        topology: mpsc::Receiver<Event>,
    ) -> Coordinator {
        let tick = tokio_stream::wrappers::IntervalStream::new(tokio::time::interval(
            Duration::from_secs(3),
        ))
        .map(|_| Event::DaemonHeartbeatInterval);
        let mut events = (
            ReceiverStream::new(events),
            tick,
            ReceiverStream::new(topology),
        )
            .merge();
        while let Some(event) = events.next().await {
            match event {
                Event::Control(e) => c.handle_control_event(e).await.unwrap(),
                Event::DaemonHeartbeat {
                    daemon_id,
                    ft_stats,
                } => c
                    .handle_daemon_heartbeat(daemon_id, ft_stats)
                    .await
                    .unwrap(),
                Event::DaemonHeartbeatInterval => c.handle_heartbeat_interval().await.unwrap(),
                Event::DaemonExit {
                    daemon_id,
                    connection_id,
                } => c
                    .handle_daemon_exit(daemon_id, connection_id)
                    .await
                    .unwrap(),
                Event::TopologyReply {
                    dataflow_id,
                    change_id,
                } => c
                    .handle_topology_reply(dataflow_id, change_id)
                    .await
                    .unwrap(),
                Event::DaemonNodeStopped {
                    daemon_id,
                    dataflow_id,
                    node_id,
                    clean_stop,
                } => c
                    .handle_daemon_node_stopped(daemon_id, dataflow_id, node_id, clean_stop)
                    .await
                    .unwrap(),
                Event::CtrlC => break,
                other => panic!("unexpected event in test loop: {other:?}"),
            }
            c.fail_topology_changes_of_gone_daemons().await;
        }
        c
    }

    struct Fixture {
        coordinator: Coordinator,
        dataflow_id: DataflowId,
        a_id: DaemonId,
        b_id: DaemonId,
        a_connection_id: Uuid,
        a_applied: Arc<std::sync::Mutex<Vec<&'static str>>>,
        b_heartbeats: Arc<std::sync::Mutex<Vec<tokio::time::Instant>>>,
        topology_rx: mpsc::Receiver<Event>,
    }

    /// Daemon `a` owns the dataflow's only node (`sender`); daemon `b` is a
    /// healthy bystander that runs nothing and only receives heartbeats.
    fn fixture(policy_a: ReplyPolicy) -> Fixture {
        fixture_with(policy_a, ReplyPolicy::Never)
    }

    fn fixture_with(policy_a: ReplyPolicy, policy_b: ReplyPolicy) -> Fixture {
        let store: Arc<dyn CoordinatorStore> = Arc::new(InMemoryStore::new());
        let dataflow_id = DataflowId::from(Uuid::new_v4());
        let a_id = DaemonId::new(Some("a".to_string()));
        let b_id = DaemonId::new(Some("b".to_string()));
        let a = fake_daemon(policy_a);
        let b = fake_daemon(policy_b);
        let a_connection_id = a.connection.connection_id;
        let (a_applied, b_heartbeats) = (a.applied.clone(), b.heartbeats.clone());
        let mut conns = DaemonConnections::default();
        conns.add(a_id.clone(), a.connection);
        conns.add(b_id.clone(), b.connection);
        let node: dora_core::config::NodeId = "sender".to_string().into();
        let mut flows = HashMap::new();
        flows.insert(
            dataflow_id,
            test_running_dataflow(dataflow_id, a_id.clone(), node),
        );
        let (topology_tx, topology_rx) = mpsc::channel::<Event>(64);
        let mut coordinator = test_coordinator(flows, conns, store);
        coordinator.topology = crate::topology_lane::TopologyLanes::new(topology_tx);
        Fixture {
            coordinator,
            dataflow_id,
            a_id,
            b_id,
            a_connection_id,
            a_applied,
            b_heartbeats,
            topology_rx,
        }
    }

    async fn send_control(events: &mpsc::Sender<Event>, request: ControlRequest) -> CliReply {
        let (reply_sender, reply_rx) = oneshot::channel();
        events
            .send(Event::Control(ControlEvent::IncomingRequest {
                request: Box::new(request),
                reply_sender,
            }))
            .await
            .unwrap();
        reply_rx
    }

    fn node_json(id: &str, path: &str, outputs: &[&str]) -> dora_message::descriptor::Node {
        serde_json::from_value(serde_json::json!({
            "id": id, "path": path, "outputs": outputs,
        }))
        .unwrap()
    }

    fn request(case: &str, dataflow_id: DataflowId) -> ControlRequest {
        let sender: dora_core::config::NodeId = "sender".to_string().into();
        let out: dora_core::config::DataId = "message".to_string().into();
        let inp: dora_core::config::DataId = "in".to_string().into();
        match case {
            "AddNode" => ControlRequest::AddNode {
                dataflow_id,
                node: node_json("added", "added", &["out"]),
            },
            "AddNodeX" => ControlRequest::AddNode {
                dataflow_id,
                node: node_json("x", "x", &["out"]),
            },
            "RemoveNodeX" => ControlRequest::RemoveNode {
                dataflow_id,
                node_id: "x".to_string().into(),
                grace_duration: None,
            },
            "RemoveNode" => ControlRequest::RemoveNode {
                dataflow_id,
                node_id: sender,
                grace_duration: None,
            },
            "ReplaceNode" => ControlRequest::ReplaceNode {
                dataflow_id,
                node: node_json("sender", "sender-v2", &["message"]),
                grace_duration: None,
            },
            "AddMapping" => ControlRequest::AddMapping {
                dataflow_id,
                source_node: sender.clone(),
                source_output: out,
                target_node: sender,
                target_input: inp,
            },
            "RemoveMapping" => ControlRequest::RemoveMapping {
                dataflow_id,
                source_node: sender.clone(),
                source_output: out,
                target_node: sender,
                target_input: inp,
            },
            other => panic!("unknown case {other}"),
        }
    }

    fn describe(outcome: &eyre::Result<ControlRequestReply>) -> String {
        match outcome {
            Ok(_) => "Ok".to_string(),
            Err(e) => format!("Err({e})"),
        }
    }

    fn tracks(coordinator: &Coordinator, dataflow_id: DataflowId, node: &str) -> (bool, bool) {
        let id: dora_core::config::NodeId = node.to_string().into();
        let df = coordinator.running_dataflows.get(&dataflow_id).unwrap();
        (
            df.node_to_daemon.contains_key(&id),
            df.descriptor.nodes.iter().any(|n| n.id == id),
        )
    }

    // ---- the event loop keeps running while a daemon does not answer ----

    /// While the owning daemon never answers, does the bystander keep getting
    /// coordinator heartbeats? (virtual time, so the 30 s wait is instant)
    async fn heartbeats_during_wait(case: &str) -> (usize, usize, String) {
        let Fixture {
            coordinator,
            dataflow_id,
            b_heartbeats,
            topology_rx,
            ..
        } = fixture(ReplyPolicy::Never);
        let (events_tx, events_rx) = mpsc::channel::<Event>(64);
        let driver = async {
            // normal operation first: ticks at t=0,3,6
            tokio::time::sleep(Duration::from_secs(7)).await;
            let before = b_heartbeats.lock().unwrap().len();
            let t0 = tokio::time::Instant::now();
            let reply = send_control(&events_tx, request(case, dataflow_id)).await;
            // 25 s into the (up to 30 s) wait
            tokio::time::sleep(Duration::from_secs(25)).await;
            let during = b_heartbeats
                .lock()
                .unwrap()
                .iter()
                .filter(|t| **t > t0)
                .count();
            let outcome = reply.await.expect("reply sender dropped");
            let elapsed = t0.elapsed();
            events_tx.send(Event::CtrlC).await.unwrap();
            (
                before,
                during,
                format!("{} after {elapsed:?}", describe(&outcome)),
            )
        };
        let (_c, result) = tokio::join!(serial_loop(coordinator, events_rx, topology_rx), driver);
        result
    }

    macro_rules! starvation_test {
        ($name:ident, $case:literal) => {
            #[tokio::test(flavor = "current_thread", start_paused = true)]
            async fn $name() {
                let (before, during, outcome) = heartbeats_during_wait($case).await;
                eprintln!(
                    "[{}] heartbeats to a HEALTHY daemon: {before} in the 7 s before the request, \
                     {during} in the 25 s while the owning daemon's reply is pending; caller got {outcome}",
                    $case
                );
                assert!(
                    before >= 2,
                    "loop should heartbeat normally first (got {before})"
                );
                assert!(
                    during >= 5,
                    "{}: the coordinator sent {during} heartbeats to a healthy daemon during a \
                     25 s wait on another daemon's reply (expected ~8 at one per 3 s)",
                    $case
                );
                assert!(
                    outcome.contains("outcome of") && outcome.contains("is unknown"),
                    "{}: an unanswered change must say its outcome is unknown, got {outcome}",
                    $case
                );
            }
        };
    }

    starvation_test!(add_node_wait_does_not_starve_heartbeats, "AddNode");
    starvation_test!(remove_node_wait_does_not_starve_heartbeats, "RemoveNode");
    starvation_test!(replace_node_wait_does_not_starve_heartbeats, "ReplaceNode");
    starvation_test!(add_mapping_wait_does_not_starve_heartbeats, "AddMapping");
    starvation_test!(
        remove_mapping_wait_does_not_starve_heartbeats,
        "RemoveMapping"
    );

    /// Control for the starvation tests: the same request, but the owning
    /// daemon answers within 10 ms. Heartbeats flow either way, and the
    /// change commits.
    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn prompt_reply_commits_and_heartbeats_keep_flowing() {
        let Fixture {
            coordinator,
            dataflow_id,
            b_heartbeats,
            topology_rx,
            ..
        } = fixture(ReplyPolicy::After(Duration::from_millis(10)));
        let (events_tx, events_rx) = mpsc::channel::<Event>(64);
        let driver = async {
            tokio::time::sleep(Duration::from_secs(7)).await;
            let t0 = tokio::time::Instant::now();
            let reply = send_control(&events_tx, request("RemoveNode", dataflow_id)).await;
            tokio::time::sleep(Duration::from_secs(25)).await;
            let during = b_heartbeats
                .lock()
                .unwrap()
                .iter()
                .filter(|t| **t > t0)
                .count();
            let outcome = reply.await.expect("reply sender dropped");
            events_tx.send(Event::CtrlC).await.unwrap();
            (during, outcome)
        };
        let (c, (during, outcome)) =
            tokio::join!(serial_loop(coordinator, events_rx, topology_rx), driver);
        assert!(during >= 5, "got {during}");
        assert!(outcome.is_ok(), "{}", describe(&outcome));
        assert_eq!(tracks(&c, dataflow_id, "sender"), (false, false));
    }

    // ---- one change at a time per dataflow ----

    /// `add x`, `add x`, `remove x` submitted back to back: they reach the
    /// daemon one at a time and in order, and the duplicate is refused against
    /// the state the first `add` left behind, never sent to the daemon.
    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn changes_to_one_dataflow_run_one_at_a_time_in_order() {
        let Fixture {
            coordinator,
            dataflow_id,
            a_applied,
            topology_rx,
            ..
        } = fixture(ReplyPolicy::After(Duration::from_secs(2)));
        let (events_tx, events_rx) = mpsc::channel::<Event>(64);
        let driver = async {
            let t0 = tokio::time::Instant::now();
            let first = send_control(&events_tx, request("AddNodeX", dataflow_id)).await;
            let duplicate = send_control(&events_tx, request("AddNodeX", dataflow_id)).await;
            let remove = send_control(&events_tx, request("RemoveNodeX", dataflow_id)).await;
            let first = first.await.unwrap();
            let after_first = t0.elapsed();
            let duplicate = duplicate.await.unwrap();
            let remove = remove.await.unwrap();
            let total = t0.elapsed();
            events_tx.send(Event::CtrlC).await.unwrap();
            (first, after_first, duplicate, remove, total)
        };
        let (c, (first, after_first, duplicate, remove, total)) =
            tokio::join!(serial_loop(coordinator, events_rx, topology_rx), driver);
        assert!(first.is_ok(), "{}", describe(&first));
        assert!(after_first >= Duration::from_secs(2));
        let duplicate = duplicate.expect_err("the second add must be refused");
        assert!(
            duplicate.to_string().contains("already exists"),
            "got {duplicate}"
        );
        assert!(remove.is_ok(), "{}", describe(&remove));
        assert_eq!(
            *a_applied.lock().unwrap(),
            ["AddNode", "RemoveNode"],
            "the duplicate must never reach the daemon, and the rest in order"
        );
        assert!(
            total >= Duration::from_secs(4),
            "remove must wait for the add to finish, got {total:?}"
        );
        assert_eq!(tracks(&c, dataflow_id, "x"), (false, false));
        assert!(c.topology.is_idle());
    }

    /// A change stuck on one dataflow's daemon does not hold up another
    /// dataflow's changes.
    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn lanes_are_per_dataflow() {
        let Fixture {
            mut coordinator,
            dataflow_id: df1,
            b_id,
            topology_rx,
            ..
        } = fixture_with(
            ReplyPolicy::Never,
            ReplyPolicy::After(Duration::from_secs(1)),
        );
        let df2 = DataflowId::from(Uuid::new_v4());
        coordinator.running_dataflows.insert(
            df2,
            test_running_dataflow(df2, b_id, "sender".to_string().into()),
        );
        let (events_tx, events_rx) = mpsc::channel::<Event>(64);
        let driver = async {
            let t0 = tokio::time::Instant::now();
            let stuck = send_control(&events_tx, request("RemoveNode", df1)).await;
            let other = send_control(&events_tx, request("RemoveNode", df2)).await;
            let other = other.await.unwrap();
            let other_took = t0.elapsed();
            let stuck = stuck.await.unwrap();
            events_tx.send(Event::CtrlC).await.unwrap();
            (other, other_took, stuck)
        };
        let (_c, (other, other_took, stuck)) =
            tokio::join!(serial_loop(coordinator, events_rx, topology_rx), driver);
        assert!(other.is_ok(), "{}", describe(&other));
        assert!(
            other_took < Duration::from_secs(5),
            "the other dataflow's change took {other_took:?} behind a stuck one"
        );
        assert!(stuck.is_err());
    }

    // ---- late replies, disconnects, cancellation ----

    fn add_slow_remove_fast(name: &'static str) -> Option<Duration> {
        match name {
            "AddNode" => Some(Duration::from_secs(35)),
            _ => Some(Duration::from_secs(1)),
        }
    }

    /// The daemon needs 35 s for `AddNode` (the coordinator gives up at 30 s).
    /// The caller is told the outcome is unknown and nothing is committed; the
    /// change queued behind it still runs; and the daemon's late answer (lost:
    /// its pending entry is gone) changes nothing.
    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn a_late_reply_is_ignored_and_the_next_change_still_runs() {
        let Fixture {
            coordinator,
            dataflow_id,
            a_applied,
            topology_rx,
            ..
        } = fixture(ReplyPolicy::Custom(add_slow_remove_fast));
        let (events_tx, events_rx) = mpsc::channel::<Event>(64);
        let driver = async {
            let t0 = tokio::time::Instant::now();
            let add = send_control(&events_tx, request("AddNode", dataflow_id)).await;
            let remove = send_control(&events_tx, request("RemoveNode", dataflow_id)).await;
            let add = add.await.unwrap();
            let add_took = t0.elapsed();
            let remove = remove.await.unwrap();
            let remove_took = t0.elapsed();
            // let the slow daemon's late answer arrive
            tokio::time::sleep(Duration::from_secs(10)).await;
            events_tx.send(Event::CtrlC).await.unwrap();
            (add, add_took, remove, remove_took)
        };
        let (c, (add, add_took, remove, remove_took)) =
            tokio::join!(serial_loop(coordinator, events_rx, topology_rx), driver);
        let add = add.expect_err("an unanswered AddNode must not report success");
        assert!(
            add.to_string()
                .contains("outcome of AddNode `added` is unknown"),
            "{add}"
        );
        assert_eq!(add_took, Duration::from_secs(30));
        assert!(remove.is_ok(), "{}", describe(&remove));
        assert!(remove_took >= Duration::from_secs(31) && remove_took < Duration::from_secs(33));
        assert_eq!(
            tracks(&c, dataflow_id, "added"),
            (false, false),
            "a late reply must not commit the node"
        );
        assert_eq!(tracks(&c, dataflow_id, "sender"), (false, false));
        assert_eq!(*a_applied.lock().unwrap(), ["RemoveNode", "AddNode"]);
        assert!(c.topology.is_idle());
    }

    /// The owning daemon disconnects while the change is in flight: the caller
    /// hears about it at once (not after the 30 s timeout) and the lane moves on.
    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn daemon_disconnect_fails_the_change_without_waiting_for_the_timeout() {
        let Fixture {
            coordinator,
            dataflow_id,
            a_id,
            a_connection_id,
            topology_rx,
            ..
        } = fixture(ReplyPolicy::Never);
        let (events_tx, events_rx) = mpsc::channel::<Event>(64);
        let driver = async {
            let t0 = tokio::time::Instant::now();
            let reply = send_control(&events_tx, request("RemoveNode", dataflow_id)).await;
            tokio::time::sleep(Duration::from_secs(2)).await;
            events_tx
                .send(Event::DaemonExit {
                    daemon_id: a_id.clone(),
                    connection_id: a_connection_id,
                })
                .await
                .unwrap();
            let outcome = reply.await.unwrap();
            let took = t0.elapsed();
            events_tx.send(Event::CtrlC).await.unwrap();
            (outcome, took)
        };
        let (c, (outcome, took)) =
            tokio::join!(serial_loop(coordinator, events_rx, topology_rx), driver);
        let err = outcome.expect_err("the change cannot have succeeded");
        assert!(
            err.to_string().contains("disconnected before it replied"),
            "{err}"
        );
        assert!(err.to_string().contains("is unknown"), "{err}");
        assert!(took < Duration::from_secs(10), "took {took:?}");
        // (The fixture's only daemon is gone, so the dataflow itself is
        // removed with it; the lane is what this test is about.)
        assert!(c.topology.is_idle());
    }

    // ---- interleavings with other events ----

    /// Run `request_case` against a daemon that answers after 1 s, deliver
    /// `NodeStopped` for `node` while the change is in flight (so the loop
    /// handles it before the change's reply, the order the unordered merge of
    /// the event streams allows), and return the coordinator afterwards.
    async fn stop_during_change(
        request_case: &str,
        node: &str,
    ) -> (Coordinator, DataflowId, eyre::Result<ControlRequestReply>) {
        let Fixture {
            coordinator,
            dataflow_id,
            a_id,
            topology_rx,
            ..
        } = fixture(ReplyPolicy::After(Duration::from_secs(1)));
        let (events_tx, events_rx) = mpsc::channel::<Event>(64);
        let driver = async {
            let reply = send_control(&events_tx, request(request_case, dataflow_id)).await;
            tokio::time::sleep(Duration::from_millis(500)).await;
            events_tx
                .send(Event::DaemonNodeStopped {
                    daemon_id: a_id.clone(),
                    dataflow_id,
                    node_id: node.to_string().into(),
                    clean_stop: false,
                })
                .await
                .unwrap();
            let outcome = reply.await.unwrap();
            events_tx.send(Event::CtrlC).await.unwrap();
            outcome
        };
        let (c, outcome) = tokio::join!(serial_loop(coordinator, events_rx, topology_rx), driver);
        (c, dataflow_id, outcome)
    }

    fn recorded_as_failed(c: &Coordinator, dataflow_id: DataflowId, node: &str) -> bool {
        let id: dora_core::config::NodeId = node.to_string().into();
        let df = &c.running_dataflows[&dataflow_id];
        df.exited_nodes.contains(&id)
            && df.node_finalized.contains(&id)
            && df.node_metrics.get(&id).is_some_and(|m| {
                m.status == dora_message::daemon_to_coordinator::NodeStatus::Failed
            })
    }

    /// An added node that crashes right after it was spawned: its exit is
    /// reported before the add's reply is handled. It used to be dropped (the
    /// node was not known yet), leaving a dead node listed as running.
    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn exit_of_an_added_node_reported_before_the_commit_is_kept() {
        let (c, dataflow_id, outcome) = stop_during_change("AddNode", "added").await;
        assert!(outcome.is_ok(), "{}", describe(&outcome));
        assert_eq!(tracks(&c, dataflow_id, "added"), (true, true));
        assert!(recorded_as_failed(&c, dataflow_id, "added"));
        assert!(c.topology.is_idle());
    }

    /// The replacement crashes right after it was spawned: the commit used to
    /// clear the markers its exit had just set.
    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn exit_of_a_replaced_node_reported_before_the_commit_is_kept() {
        let (c, dataflow_id, outcome) = stop_during_change("ReplaceNode", "sender").await;
        assert!(outcome.is_ok(), "{}", describe(&outcome));
        assert_eq!(tracks(&c, dataflow_id, "sender"), (true, true));
        assert!(recorded_as_failed(&c, dataflow_id, "sender"));
        assert!(c.topology.is_idle());
    }

    /// A removed node's exit reported before the commit must leave nothing
    /// behind for a node that is no longer in the dataflow (the same result as
    /// when the loop waited for the reply and handled the exit afterwards).
    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn exit_of_a_removed_node_reported_before_the_commit_leaves_no_state() {
        let (c, dataflow_id, outcome) = stop_during_change("RemoveNode", "sender").await;
        assert!(outcome.is_ok(), "{}", describe(&outcome));
        assert_eq!(tracks(&c, dataflow_id, "sender"), (false, false));
        let id: dora_core::config::NodeId = "sender".to_string().into();
        let df = &c.running_dataflows[&dataflow_id];
        assert!(!df.exited_nodes.contains(&id));
        assert!(!df.node_finalized.contains(&id));
        assert!(!df.node_metrics.contains_key(&id));
        assert!(!df.node_stopped_at.contains_key(&id));
    }

    /// The stop of a node that no change is about is handled at once.
    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn exit_of_an_unrelated_node_is_not_held_back() {
        let (c, dataflow_id, outcome) = stop_during_change("AddNode", "sender").await;
        assert!(outcome.is_ok(), "{}", describe(&outcome));
        assert!(recorded_as_failed(&c, dataflow_id, "sender"));
    }

    /// A `restart` copies the descriptor when it arrives, so one that arrives
    /// while a change is in flight would restart the dataflow without a
    /// change the caller is told was applied. It is refused instead, and the
    /// dataflow keeps running with the change.
    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn restart_is_refused_while_a_change_is_in_flight() {
        let Fixture {
            coordinator,
            dataflow_id,
            topology_rx,
            ..
        } = fixture(ReplyPolicy::After(Duration::from_secs(1)));
        let (events_tx, events_rx) = mpsc::channel::<Event>(64);
        let driver = async {
            let add = send_control(&events_tx, request("AddNode", dataflow_id)).await;
            tokio::time::sleep(Duration::from_millis(500)).await;
            let restart = send_control(
                &events_tx,
                ControlRequest::Restart {
                    dataflow_uuid: dataflow_id,
                    grace_duration: None,
                    force: false,
                },
            )
            .await;
            let restart = restart.await.unwrap();
            let add = add.await.unwrap();
            events_tx.send(Event::CtrlC).await.unwrap();
            (restart, add)
        };
        let (c, (restart, add)) =
            tokio::join!(serial_loop(coordinator, events_rx, topology_rx), driver);
        let err = restart.expect_err("a restart must not overtake the change");
        assert!(err.to_string().contains("still in progress"), "{err}");
        assert!(add.is_ok(), "{}", describe(&add));
        assert_eq!(tracks(&c, dataflow_id, "added"), (true, true));
        assert!(c.pending_restarts.is_empty());
        assert!(
            c.ensure_no_topology_change(dataflow_id).is_ok(),
            "a restart is allowed once the change has completed"
        );
    }

    /// The daemon answers and then disconnects, and the loop sees the
    /// disconnect first. The answer is already on its way, so the change
    /// commits instead of being reported as unknown.
    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn a_reply_already_received_survives_the_daemon_disconnecting() {
        let Fixture {
            mut coordinator,
            dataflow_id,
            a_id,
            a_connection_id,
            mut topology_rx,
            ..
        } = fixture(ReplyPolicy::After(Duration::from_millis(10)));
        // A second daemon keeps the dataflow alive when `a` leaves.
        coordinator
            .running_dataflows
            .get_mut(&dataflow_id)
            .unwrap()
            .daemons
            .insert(DaemonId::new(Some("b".to_string())));
        let (reply_tx, reply_rx) = oneshot::channel();
        coordinator
            .handle_control_event(ControlEvent::IncomingRequest {
                request: Box::new(request("RemoveNode", dataflow_id)),
                reply_sender: reply_tx,
            })
            .await
            .unwrap();
        // The daemon's answer reaches the topology stream ...
        let Some(Event::TopologyReply {
            dataflow_id: id,
            change_id,
        }) = topology_rx.recv().await
        else {
            panic!("expected the daemon's reply");
        };
        // ... but the loop handles the daemon's exit first. The exit commits
        // the answered change before it tears the dataflow down (#4161).
        coordinator
            .handle_daemon_exit(a_id, a_connection_id)
            .await
            .unwrap();
        coordinator.fail_topology_changes_of_gone_daemons().await;
        assert!(coordinator.topology.is_idle());
        let reply = reply_rx.await.unwrap();
        assert!(
            matches!(reply, Ok(ControlRequestReply::NodeRemoved { .. })),
            "{}",
            describe(&reply)
        );
        // The late `TopologyReply` finds nothing in flight and is ignored.
        coordinator
            .handle_topology_reply(id, change_id)
            .await
            .unwrap();
        assert!(coordinator.topology.is_idle());
    }

    /// The only daemon of a running dataflow confirms a change and then
    /// disconnects, and the loop handles the disconnect before the change's
    /// `TopologyReply`. The disconnect parks the dataflow as `Recovering` (to
    /// be rebuilt from its record when the daemon reconnects), so the record
    /// must already contain the change the daemon confirmed (#4161).
    async fn confirmed_change_is_in_the_recovering_record(
        case: &str,
    ) -> dora_coordinator_store::DataflowRecord {
        let Fixture {
            mut coordinator,
            dataflow_id,
            a_id,
            a_connection_id,
            mut topology_rx,
            ..
        } = fixture(ReplyPolicy::After(Duration::from_millis(10)));
        // A dataflow that spawned successfully is reclaimed, not failed, when
        // its last daemon leaves.
        coordinator
            .running_dataflows
            .get_mut(&dataflow_id)
            .unwrap()
            .spawn_result
            .set_result(Ok(ControlRequestReply::DataflowSpawned {
                uuid: dataflow_id,
            }));
        let (reply_tx, reply_rx) = oneshot::channel();
        coordinator
            .handle_control_event(ControlEvent::IncomingRequest {
                request: Box::new(request(case, dataflow_id)),
                reply_sender: reply_tx,
            })
            .await
            .unwrap();
        // The daemon applies the change and answers; its exit is then handled
        // before the `TopologyReply` the answer produced.
        tokio::time::sleep(Duration::from_millis(50)).await;
        coordinator
            .handle_daemon_exit(a_id, a_connection_id)
            .await
            .unwrap();
        coordinator.fail_topology_changes_of_gone_daemons().await;
        assert!(
            !coordinator.running_dataflows.contains_key(&dataflow_id),
            "the dataflow lost its only daemon and is reclaimed"
        );
        // Then the `TopologyReply`, which finds nothing left to do.
        let Some(Event::TopologyReply {
            dataflow_id: id,
            change_id,
        }) = topology_rx.recv().await
        else {
            panic!("expected the waiter's report");
        };
        coordinator
            .handle_topology_reply(id, change_id)
            .await
            .unwrap();
        assert!(coordinator.topology.is_idle());
        let reply = reply_rx.await.unwrap();
        assert!(reply.is_ok(), "{case}: {}", describe(&reply));

        let record = coordinator
            .store
            .get_dataflow(&dataflow_id)
            .unwrap()
            .expect("the reclaimed dataflow is persisted");
        assert!(
            matches!(record.status, StoreDataflowStatus::Recovering),
            "{:?}",
            record.status
        );
        record
    }

    fn record_descriptor(record: &dora_coordinator_store::DataflowRecord) -> Descriptor {
        serde_json::from_str(&record.descriptor_json).unwrap()
    }

    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn add_node_confirmed_before_a_disconnect_is_persisted() {
        let record = confirmed_change_is_in_the_recovering_record("AddNode").await;
        let added: dora_core::config::NodeId = "added".to_string().into();
        assert!(
            record_descriptor(&record)
                .nodes
                .iter()
                .any(|n| n.id == added)
        );
        assert!(record.node_to_daemon.contains_key("added"));
    }

    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn remove_node_confirmed_before_a_disconnect_is_persisted() {
        let record = confirmed_change_is_in_the_recovering_record("RemoveNode").await;
        assert!(record_descriptor(&record).nodes.is_empty());
        assert!(!record.node_to_daemon.contains_key("sender"));
    }

    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn replace_node_confirmed_before_a_disconnect_is_persisted() {
        let record = confirmed_change_is_in_the_recovering_record("ReplaceNode").await;
        let descriptor = record_descriptor(&record);
        let sender = descriptor
            .nodes
            .iter()
            .find(|n| n.id.to_string() == "sender")
            .expect("the replaced node stays in the descriptor");
        assert_eq!(sender.path.as_deref(), Some("sender-v2"));
    }

    /// The owning daemon reported the node's exit (held back for the change)
    /// and then disconnected without replying. The change fails as unknown,
    /// but the exit must not be lost with it: on `main` it was processed once
    /// the failed wait returned.
    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn held_exit_survives_the_change_failing_on_a_disconnect() {
        let Fixture {
            mut coordinator,
            dataflow_id,
            a_id,
            ..
        } = fixture(ReplyPolicy::Never);
        let (reply_tx, reply_rx) = oneshot::channel();
        coordinator
            .handle_control_event(ControlEvent::IncomingRequest {
                request: Box::new(request("RemoveNode", dataflow_id)),
                reply_sender: reply_tx,
            })
            .await
            .unwrap();
        coordinator
            .handle_daemon_node_stopped(
                a_id.clone(),
                dataflow_id,
                "sender".to_string().into(),
                false,
            )
            .await
            .unwrap();
        // The daemon's connection goes away (the dataflow itself stays, so
        // the exit has somewhere to be recorded).
        coordinator.daemon_connections.remove(&a_id);
        coordinator.fail_topology_changes_of_gone_daemons().await;

        let err = reply_rx.await.unwrap().unwrap_err().to_string();
        assert!(err.contains("disconnected before it replied"), "{err}");
        assert!(
            recorded_as_failed(&coordinator, dataflow_id, "sender"),
            "the held exit must still be applied"
        );
        assert!(coordinator.topology.is_idle());
    }

    /// The CLI hangs up while its change is in flight: the change still commits.
    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn cli_cancellation_does_not_lose_the_commit() {
        let Fixture {
            coordinator,
            dataflow_id,
            topology_rx,
            ..
        } = fixture(ReplyPolicy::After(Duration::from_secs(1)));
        let (events_tx, events_rx) = mpsc::channel::<Event>(64);
        let driver = async {
            let reply = send_control(&events_tx, request("AddNode", dataflow_id)).await;
            drop(reply);
            tokio::time::sleep(Duration::from_secs(5)).await;
            events_tx.send(Event::CtrlC).await.unwrap();
        };
        let (c, ()) = tokio::join!(serial_loop(coordinator, events_rx, topology_rx), driver);
        assert_eq!(tracks(&c, dataflow_id, "added"), (true, true));
        assert!(c.topology.is_idle());
    }

    /// The dataflow ends while its change is in flight: the daemon's
    /// confirmation has nothing to commit to, the caller is told so, and the
    /// changes queued behind it fail cleanly instead of leaking.
    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn dataflow_gone_before_the_reply_commits_nothing() {
        let Fixture {
            mut coordinator,
            dataflow_id,
            mut topology_rx,
            ..
        } = fixture(ReplyPolicy::After(Duration::from_millis(10)));
        let (first_tx, first_rx) = oneshot::channel();
        coordinator
            .handle_add_node(dataflow_id, node_json("added", "added", &["out"]), first_tx)
            .await
            .unwrap();
        let (queued_tx, queued_rx) = oneshot::channel();
        coordinator
            .handle_remove_node(dataflow_id, "sender".to_string().into(), None, queued_tx)
            .await
            .unwrap();
        coordinator.running_dataflows.remove(&dataflow_id);
        settle_topology_change(&mut coordinator, &mut topology_rx).await;
        let first = first_rx.await.unwrap().expect_err("nothing to commit to");
        assert!(first.to_string().contains("no longer running"), "{first}");
        let queued = queued_rx.await.unwrap().expect_err("dataflow is gone");
        assert!(
            queued.to_string().contains("no running dataflow"),
            "{queued}"
        );
        assert!(coordinator.topology.is_idle());
    }

    /// A bound on the queue: a daemon that never answers cannot make the
    /// coordinator hold an unbounded number of waiting changes.
    #[tokio::test(flavor = "current_thread", start_paused = true)]
    async fn the_waiting_queue_is_bounded() {
        let Fixture {
            mut coordinator,
            dataflow_id,
            ..
        } = fixture(ReplyPolicy::Never);
        let mut replies = Vec::new();
        for _ in 0..40 {
            let (tx, rx) = oneshot::channel();
            coordinator
                .handle_add_mapping(
                    dataflow_id,
                    "sender".to_string().into(),
                    "message".to_string().into(),
                    "sender".to_string().into(),
                    "in".to_string().into(),
                    tx,
                )
                .await
                .unwrap();
            replies.push(rx);
        }
        // 1 in flight + 32 waiting; the rest are refused immediately.
        let mut refused = 0;
        for rx in replies.iter_mut() {
            if let Ok(Err(e)) = rx.try_recv() {
                assert!(e.to_string().contains("too many topology changes"), "{e}");
                refused += 1;
            }
        }
        assert_eq!(refused, 40 - 1 - 32);
    }

    // ---- real time: the coordinator's own watchdog ----

    /// REAL TIME (~40 s): after one daemon's reply times out, are healthy
    /// daemons disconnected by the coordinator's own watchdog?
    /// `DaemonConnection::last_heartbeat` is a `std::time::Instant`, so this
    /// cannot run under a paused tokio clock. Run with `-- --ignored --nocapture`.
    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    #[ignore = "takes ~40 s of real time"]
    async fn healthy_daemon_survives_another_daemons_reply_timeout() {
        const ROUNDS: usize = 12;
        let rounds = (0..ROUNDS).map(|_| async {
            let Fixture {
                coordinator,
                dataflow_id,
                a_id,
                b_id,
                topology_rx,
                ..
            } = fixture(ReplyPolicy::Never);
            let (events_tx, events_rx) = mpsc::channel::<Event>(256);
            let driver = async {
                // both daemons look fresh at start
                for id in [&a_id, &b_id] {
                    events_tx
                        .send(Event::DaemonHeartbeat {
                            daemon_id: id.clone(),
                            ft_stats: None,
                        })
                        .await
                        .unwrap();
                }
                tokio::time::sleep(Duration::from_secs(1)).await;
                let reply = send_control(&events_tx, request("RemoveNode", dataflow_id)).await;
                // daemon `b` keeps heartbeating every 5 s like the real daemon
                for _ in 0..7 {
                    tokio::time::sleep(Duration::from_secs(5)).await;
                    events_tx
                        .send(Event::DaemonHeartbeat {
                            daemon_id: b_id.clone(),
                            ft_stats: None,
                        })
                        .await
                        .unwrap();
                }
                let _ = reply.await;
                tokio::time::sleep(Duration::from_secs(6)).await;
                events_tx.send(Event::CtrlC).await.unwrap();
            };
            let (c, ()) = tokio::join!(serial_loop(coordinator, events_rx, topology_rx), driver);
            c.daemon_connections.get(&b_id).is_some()
        });
        let survived = futures::future::join_all(rounds).await;
        let ok = survived.iter().filter(|s| **s).count();
        eprintln!("healthy bystander daemon still connected after the stall: {ok}/{ROUNDS} rounds");
        assert_eq!(
            ok,
            ROUNDS,
            "a healthy daemon was disconnected in {} of {ROUNDS} rounds",
            ROUNDS - ok
        );
    }
}
