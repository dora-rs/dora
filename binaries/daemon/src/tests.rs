use super::*;
use crate::coordinator_events::{
    apply_state_catch_up_entries, deliver_param_delete_strict, deliver_param_update_strict,
};
use crate::node_exit::{DEFAULT_FINISH_DRAIN_GRACE, parse_finish_drain_grace};
use crate::pending::DataflowStatus;
use crate::running_dataflow::{HandleReplacement, StopProcessPolicy};
use std::sync::atomic::AtomicU32;
use std::sync::{Arc, Mutex};

use aligned_vec::AVec;
use crossbeam::queue::ArrayQueue;
use dora_core::config::InputMapping;
use dora_message::{
    coordinator_to_daemon::{StateCatchUpEntry, StateCatchUpOperation},
    daemon_to_node::{NodeConfig, NodeEvent},
    descriptor::{Descriptor, RestartPolicy},
    metadata,
};
use std::sync::atomic::{AtomicBool, AtomicU64};

fn test_dataflow() -> RunningDataflow {
    let descriptor = Descriptor::new(vec![]);
    RunningDataflow::new(Uuid::nil(), DaemonId::new(None), descriptor)
}

// dora-rs/dora: a node added to an already-running dataflow (via
// `AddNode`) may register a timer input on an interval no existing node
// uses. `start()` is the only place timer tasks are spawned, so the
// `AddNode` handler re-invokes it. This test locks in the property that
// makes that safe: `start()` is idempotent for existing intervals yet
// still spawns a task for a newly-added one.
#[tokio::test]
async fn start_spawns_task_for_interval_added_after_first_start() {
    let mut df = test_dataflow();
    let clock = Arc::new(HLC::default());
    let (events_tx, _events_rx) = mpsc::channel(8);

    let first = Duration::from_millis(100);
    df.timers
        .entry(first)
        .or_default()
        .insert((NodeId::from("a".to_string()), DataId::from("t".to_string())));
    df.start(&events_tx, &clock).await.unwrap();
    assert!(df._timer_handles.contains_key(&first));

    // Simulate a node added later that registers a brand-new interval.
    let second = Duration::from_millis(250);
    df.timers
        .entry(second)
        .or_default()
        .insert((NodeId::from("b".to_string()), DataId::from("t".to_string())));
    df.start(&events_tx, &clock).await.unwrap();

    assert!(df._timer_handles.contains_key(&first));
    assert!(df._timer_handles.contains_key(&second));
    assert_eq!(df._timer_handles.len(), 2);
}

// dora-rs/dora: removing the last subscriber of a timer interval (via
// `RemoveNode`) must cancel that interval's timer task and forget the
// entry, so it doesn't keep ticking to an empty subscriber set for the
// rest of the dataflow's life (#2585). An interval that still has other
// subscribers must be left running.
#[tokio::test]
async fn unsubscribe_last_subscriber_cancels_timer_task() {
    let mut df = test_dataflow();
    let clock = Arc::new(HLC::default());
    let (events_tx, _events_rx) = mpsc::channel(8);

    let node_a = NodeId::from("a".to_string());
    let node_b = NodeId::from("b".to_string());
    let solo = Duration::from_millis(100); // only `a` subscribes
    let shared = Duration::from_millis(250); // `a` and `b` subscribe

    df.timers
        .entry(solo)
        .or_default()
        .insert((node_a.clone(), DataId::from("t".to_string())));
    df.timers
        .entry(shared)
        .or_default()
        .insert((node_a.clone(), DataId::from("t".to_string())));
    df.timers
        .entry(shared)
        .or_default()
        .insert((node_b.clone(), DataId::from("t".to_string())));
    df.start(&events_tx, &clock).await.unwrap();
    assert!(df._timer_handles.contains_key(&solo));
    assert!(df._timer_handles.contains_key(&shared));

    df.unsubscribe_node_from_timers(&node_a);

    // `solo` lost its only subscriber: entry and task both gone.
    assert!(!df.timers.contains_key(&solo));
    assert!(!df._timer_handles.contains_key(&solo));
    // `shared` still has `b`: it keeps its subscriber set and its task.
    assert_eq!(
        df.timers.get(&shared).map(|s| s.len()),
        Some(1),
        "shared interval must keep its remaining subscriber"
    );
    assert!(df._timer_handles.contains_key(&shared));
}

// dora-rs/dora: the `AddNode` handler guards its re-invocation of
// `start()` on `dataflow_started`, so that flag must be set on *every*
// start path. It used to be set only at the single-daemon `Subscribe`
// readiness call site, leaving it `false` for the whole life of a
// distributed dataflow (which starts via the coordinator `AllNodesReady`
// path instead) — so a node added later was silently starved of ticks.
// `start()` now sets the flag itself; this pins that invariant.
#[tokio::test]
async fn start_marks_dataflow_started() {
    let mut df = test_dataflow();
    let clock = Arc::new(HLC::default());
    let (events_tx, _events_rx) = mpsc::channel(8);

    assert!(!df.dataflow_started);
    df.start(&events_tx, &clock).await.unwrap();
    assert!(df.dataflow_started);
}

fn test_logger(clock: Arc<HLC>) -> DaemonLogger {
    Logger {
        destination: log::LogDestination::Tracing,
        daemon_id: DaemonId::new(None),
        clock,
    }
    .for_daemon(DaemonId::new(None))
}

// dora-rs/dora#3053: a startup-barrier completion can arrive *during*
// teardown — a pending cohort member subscribing inside the stop grace
// window, that member being killed when the window expires (the death
// trigger from #2970), or a `RemoveNode` dropping the last pending member.
// `should_start_on_barrier_completion` gates all three on `!stop_sent`, so
// a stopping dataflow is never (re)started. Drives `stop_all` rather than
// assigning `stop_sent`, so the test tracks the real teardown entry point.
#[tokio::test]
async fn barrier_completion_does_not_start_a_stopping_dataflow() {
    let clock = Arc::new(HLC::default());
    let mut daemon_logger = test_logger(clock.clone());
    let mut logger = daemon_logger.for_dataflow(Uuid::nil());
    let mut df = test_dataflow();

    // A fresh dataflow whose barrier just completed should start.
    assert!(df.should_start_on_barrier_completion(&DataflowStatus::AllNodesReady));

    // A `Pending` status never starts.
    assert!(!df.should_start_on_barrier_completion(&DataflowStatus::Pending));

    // Once the dataflow is stopping, the same completion must not start it.
    let finish_when = df
        .stop_all(&mut None, &clock, None, false, &mut logger)
        .await
        .unwrap();
    assert!(matches!(finish_when, FinishDataflowWhen::Now));
    assert!(df.stop_sent, "stop_all must record that stop was sent");
    assert!(
        !df.should_start_on_barrier_completion(&DataflowStatus::AllNodesReady),
        "a stopping dataflow must not be started by a completing startup barrier"
    );

    // An already-started dataflow is not started again either.
    let mut df = test_dataflow();
    df.dataflow_started = true;
    assert!(!df.should_start_on_barrier_completion(&DataflowStatus::AllNodesReady));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn finish_dataflow_cleans_local_state_when_coordinator_send_fails() {
    let (coordinator_sender, coordinator_rx) = coordinator::CoordinatorSender::for_test();
    drop(coordinator_rx);
    let (mut daemon, _events_rx) = Daemon::build_daemon(
        None,
        Some(coordinator_sender),
        DaemonId::new(None),
        None,
        Arc::new(HLC::default()),
        None,
        BTreeMap::new(),
        LogDestination::Tracing,
        None,
        Vec::new(),
        None,
        Vec::new(),
        ZenohBind::Derived(LOCALHOST),
        false,
        false,
        None,
    )
    .await
    .expect("daemon should build");

    let dataflow_id = Uuid::new_v4();
    let dataflow = test_dataflow();
    let mut listener_shutdown = dataflow.listener_shutdown_tx.subscribe();
    daemon.running.insert(dataflow_id, dataflow);

    #[cfg(feature = "tensor-pool")]
    {
        unsafe { std::env::set_var("DORA_MEMORY_POOL_CROSS_MACHINE", "1") };
        daemon.pool_subscribe_dataflow(dataflow_id);
        unsafe { std::env::remove_var("DORA_MEMORY_POOL_CROSS_MACHINE") };
        assert!(
            daemon.pool.has_subscriber(&dataflow_id),
            "test setup must create a memory-pool subscriber"
        );
    }

    let result = daemon.finish_dataflow(dataflow_id).await;

    assert!(
        result.is_err(),
        "closed coordinator sender should still report the send failure"
    );
    assert!(
        !daemon.running.contains_key(&dataflow_id),
        "finished dataflow must be removed locally even if coordinator reporting fails"
    );
    assert!(
        daemon.pending_finished_dataflows.contains_key(&dataflow_id),
        "failed finish report must be retained for reconnect retry"
    );
    listener_shutdown
        .changed()
        .await
        .expect("finish_dataflow should signal listener shutdown");
    assert!(*listener_shutdown.borrow());
    #[cfg(feature = "tensor-pool")]
    assert!(
        !daemon.pool.has_subscriber(&dataflow_id),
        "memory-pool subscriber must be removed even when coordinator reporting fails"
    );

    let (coordinator_sender, mut coordinator_rx) = coordinator::CoordinatorSender::for_test();
    daemon.coordinator_sender = Some(coordinator_sender);
    daemon
        .report_pending_finished_dataflows()
        .await
        .expect("retrying pending finish report should succeed");
    assert!(
        !daemon.pending_finished_dataflows.contains_key(&dataflow_id),
        "pending finish report should be removed after successful retry"
    );

    let retried = coordinator_rx
        .recv()
        .await
        .expect("retry should send an event to the coordinator");
    assert!(retried.contains("AllNodesFinished"), "{retried}");
    assert!(retried.contains(&dataflow_id.to_string()), "{retried}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn failed_pending_finish_retry_does_not_abort_reconnect_cycle() {
    let (coordinator_sender, coordinator_rx) = coordinator::CoordinatorSender::for_test();
    drop(coordinator_rx);

    let clock = Arc::new(HLC::default());
    let (mut daemon, _events_rx) = Daemon::build_daemon(
        None,
        Some(coordinator_sender),
        DaemonId::new(None),
        None,
        clock.clone(),
        None,
        BTreeMap::new(),
        LogDestination::Tracing,
        None,
        Vec::new(),
        None,
        Vec::new(),
        ZenohBind::Derived(LOCALHOST),
        false,
        false,
        None,
    )
    .await
    .expect("daemon should build");

    let dataflow_id = Uuid::new_v4();
    daemon.pending_finished_dataflows.insert(
        dataflow_id,
        DataflowDaemonResult {
            timestamp: clock.new_timestamp(),
            node_results: BTreeMap::new(),
        },
    );

    let external_events = futures::stream::iter([Timestamped {
        inner: Event::CtrlC,
        timestamp: clock.new_timestamp(),
    }]);
    let (_dora_events_tx, mut dora_events_rx) = mpsc::channel(1);

    let result = daemon
        .run_inner(external_events, &mut dora_events_rx, None)
        .await;

    assert!(
        result.is_ok(),
        "a failed pending finish retry must not abort the reconnect cycle: {result:?}"
    );
    assert!(
        daemon.pending_finished_dataflows.contains_key(&dataflow_id),
        "failed retry must keep the finish report pending for the next reconnect"
    );
}

fn test_running_node() -> RunningNode {
    RunningNode {
        process: None,
        restart_loop_start: None,
        _listener_shutdown: None,
        generation: 7,
        generation_counter: Arc::new(AtomicU64::new(7)),
        node_config: NodeConfig {
            dataflow_id: Uuid::nil(),
            node_id: NodeId::from("test".to_string()),
            run_config: NodeRunConfig::default(),
            daemon_communication: None,
            dataflow_descriptor: serde_yaml::Value::Null,
            dynamic: false,
            write_events_to: None,
            restart_count: 0,
            output_routing: None,
        },
        pid: None,
        restart_count: Arc::new(AtomicU32::new(0)),
        restart_policy: RestartPolicy::Never,
        disable_restart: Arc::new(AtomicBool::new(false)),
        force_restart_next: Arc::new(AtomicBool::new(false)),
        last_activity: Arc::new(AtomicU64::new(0)),
        spawned_at: Arc::new(AtomicU64::new(0)),
        startup_kill_sent: Arc::new(AtomicBool::new(false)),
        health_check_timeout: None,
        startup_timeout: None,
        finish_grace_secs: None,
    }
}

fn add_input(df: &mut RunningDataflow, consumer: &str, input: &str, source: &str) {
    df.running_nodes
        .get_mut(&NodeId::from(consumer.to_owned()))
        .unwrap()
        .node_config
        .run_config
        .inputs
        .insert(input.to_owned().into(), user_input(source, "out", None));
}

#[test]
fn dynamic_join_dials_both_local_producers_and_consumers() {
    let mut df = test_dataflow();
    for name in ["producer", "joining", "consumer", "unrelated"] {
        let id = NodeId::from(name.to_owned());
        let mut node = test_running_node();
        node.node_config.node_id = id.clone();
        node.node_config.dynamic = name == "joining";
        df.running_nodes.insert(id, node);
    }
    let joining = NodeId::from("joining".to_owned());
    add_input(&mut df, "joining", "in", "producer");
    add_input(&mut df, "consumer", "in", "joining");
    // A remote producer is deliberately absent from the local running set.
    add_input(&mut df, "joining", "remote", "remote");
    for (name, port) in [
        ("producer", 12001),
        ("consumer", 12002),
        ("unrelated", 12003),
        ("remote", 12004),
    ] {
        Arc::make_mut(&mut df.zenoh_peering).insert(
            name.to_owned().into(),
            crate::spawn::NodeZenohPeering {
                listen: vec![format!("tcp/127.0.0.1:{port}")],
                connect: vec![],
                routable: false,
            },
        );
    }
    let (_, plan) = df
        .dynamic_node_config(&joining, Some("tcp/127.0.0.1:12000"))
        .unwrap();
    assert_eq!(
        plan.connect,
        [
            "tcp/127.0.0.1:12000",
            "tcp/127.0.0.1:12001",
            "tcp/127.0.0.1:12002"
        ]
    );
    assert!(plan.listen.starts_with("tcp/127.0.0.1:"));
    let again = df.dynamic_node_config(&joining, None).unwrap().1;
    assert_ne!(
        again.listen, plan.listen,
        "a restarted node must not inherit a port nothing held while it was down"
    );
    assert_eq!(df.zenoh_peering[&joining].listen, [again.listen]);
}

#[test]
fn dynamic_join_publishes_listener_before_next_configuration_request() {
    let mut df = test_dataflow();
    let a = NodeId::from("a".to_owned());
    let b = NodeId::from("b".to_owned());
    for id in [&a, &b] {
        let mut node = test_running_node();
        node.node_config.node_id = id.clone();
        node.node_config.dynamic = true;
        df.running_nodes.insert(id.clone(), node);
    }
    add_input(&mut df, "b", "in", "a");
    let first = df.dynamic_node_config(&a, None).unwrap().1;
    let second = df.dynamic_node_config(&b, None).unwrap().1;
    assert_eq!(
        second.connect.as_slice(),
        std::slice::from_ref(&first.listen)
    );
    assert_ne!(first.listen, second.listen);
    // Once both have asked, either order of reconnect has explicit endpoints.
    assert_eq!(
        df.dynamic_node_config(&a, None).unwrap().1.connect,
        [second.listen]
    );
}

#[test]
fn dynamic_join_rejects_static_nodes_and_stopping_dataflows() {
    let mut df = test_dataflow();
    let id = NodeId::from("test".to_owned());
    df.running_nodes.insert(id.clone(), test_running_node());
    assert!(
        df.dynamic_node_config(&id, None)
            .unwrap_err()
            .to_string()
            .contains("not dynamic")
    );
    assert!(df.zenoh_peering.is_empty());
    df.running_nodes.get_mut(&id).unwrap().node_config.dynamic = true;
    df.stop_sent = true;
    assert!(
        df.dynamic_node_config(&id, None)
            .unwrap_err()
            .to_string()
            .contains("stopping")
    );
    assert!(df.zenoh_peering.is_empty());
}

fn user_input(
    source: &str,
    output: &str,
    policy: Option<dora_message::config::QueuePolicy>,
) -> Input {
    Input {
        mapping: InputMapping::User(dora_message::config::UserInputMapping {
            source: NodeId::from(source.to_string()),
            output: DataId::from(output.to_string()),
        }),
        queue_size: None,
        input_timeout: None,
        queue_policy: policy,
    }
}

fn running_node_with(
    inputs: BTreeMap<DataId, Input>,
    output_routing: Option<BTreeMap<DataId, dora_message::daemon_to_node::OutputRouting>>,
) -> RunningNode {
    let mut node = test_running_node();
    node.node_config.run_config.inputs = inputs;
    node.node_config.output_routing = output_routing;
    node
}

/// The live predicate behind `added_node_output_routing`'s
/// `requires_backpressure` argument (dora-rs/dora#3428): it reads the
/// receiver's registered config, and a receiver with no entry counts as
/// not requiring anything.
#[test]
fn input_requires_backpressure_reads_the_live_node_config() {
    use dora_message::config::QueuePolicy;
    let mut df = test_dataflow();
    let sink = NodeId::from("sink".to_string());
    let camera = DataId::from("camera".to_string());
    assert!(
        !df.input_requires_backpressure(&sink, &camera),
        "no entry (exited or never here)"
    );
    for (policy, expected) in [
        (None, false),
        (Some(QueuePolicy::DropOldest), false),
        (Some(QueuePolicy::Backpressure), true),
    ] {
        let inputs = BTreeMap::from([(camera.clone(), user_input("src", "image", policy))]);
        df.running_nodes
            .insert(sink.clone(), running_node_with(inputs, None));
        assert_eq!(df.input_requires_backpressure(&sink, &camera), expected);
    }
    assert!(!df.input_requires_backpressure(&sink, &DataId::from("other".to_string())));
}

/// `dora node add`/`replace` refuse a backpressure input only when its
/// producer runs here with the output already on the direct path — the
/// one case nothing can re-pin (dora-rs/dora#3428 review).
#[test]
fn backpressure_consumer_is_refused_only_for_an_unpinned_running_producer() {
    use dora_message::{config::QueuePolicy, daemon_to_node::OutputRouting};
    let mut df = test_dataflow();
    let sink = NodeId::from("sink".to_string());
    let src = NodeId::from("src".to_string());
    let image = DataId::from("image".to_string());
    let camera = DataId::from("camera".to_string());
    let wants = BTreeMap::from([(
        camera.clone(),
        user_input("src", "image", Some(QueuePolicy::Backpressure)),
    )]);

    assert_eq!(
        df.unpinnable_backpressure_input(&sink, &wants),
        None,
        "producer not on this daemon: its own daemon decides"
    );

    let pinned = BTreeMap::from([(
        image.clone(),
        OutputRouting {
            daemon_only: true,
            ..Default::default()
        },
    )]);
    df.running_nodes.insert(
        src.clone(),
        running_node_with(BTreeMap::new(), Some(pinned)),
    );
    assert_eq!(df.unpinnable_backpressure_input(&sink, &wants), None);

    df.running_nodes
        .insert(src.clone(), running_node_with(BTreeMap::new(), None));
    assert_eq!(
        df.unpinnable_backpressure_input(&sink, &wants),
        None,
        "no routing at all keeps every output on the daemon path"
    );

    let direct = BTreeMap::from([(image.clone(), OutputRouting::default())]);
    df.running_nodes.insert(
        src.clone(),
        running_node_with(BTreeMap::new(), Some(direct)),
    );
    assert_eq!(
        df.unpinnable_backpressure_input(&sink, &wants),
        Some((camera.clone(), OutputId(src.clone(), image.clone())))
    );

    let lossy = BTreeMap::from([(
        camera.clone(),
        user_input("src", "image", Some(QueuePolicy::DropOldest)),
    )]);
    assert_eq!(df.unpinnable_backpressure_input(&sink, &lossy), None);

    let self_loop = BTreeMap::from([(
        DataId::from("again".to_string()),
        user_input("sink", "out", Some(QueuePolicy::Backpressure)),
    )]);
    let own_direct = BTreeMap::from([(DataId::from("out".to_string()), OutputRouting::default())]);
    df.running_nodes.insert(
        sink.clone(),
        running_node_with(BTreeMap::new(), Some(own_direct)),
    );
    assert_eq!(
        df.unpinnable_backpressure_input(&sink, &self_loop),
        None,
        "a self-loop is the entering node's own routing"
    );
}

/// dora-rs/dora#2988 review, finding 1 (still upheld) and dora-rs/dora#2997:
/// the Event::Node gate accepts only the entry's own lineage — its current
/// generation or the successor its own restart loop announced into
/// `generation_counter`. It must let a fast successor's early events through
/// (or the incarnation stays disconnected forever) yet reject a stranger's
/// higher generation (or a replace-vs-restart zombie corrupts the
/// replacement).
#[test]
fn node_event_gate_accepts_only_own_lineage() {
    // No pending successor announced: counter == entry generation.
    // Predecessor's connection after a swap advanced the entry: stale.
    assert!(event_generation_is_stale(8, 8, 7));
    // The entry's current incarnation: fresh.
    assert!(!event_generation_is_stale(8, 8, 8));
    // A higher generation with no announced successor is a stranger: stale.
    assert!(
        event_generation_is_stale(8, 8, 9),
        "a newer generation the entry never announced must be dropped"
    );

    // The entry's own restart loop announced successor generation 9 (into
    // `generation_counter`) before spawning it. Its early Subscribe races
    // ahead of `ProcessHandleReplaced`, so the entry still reads generation
    // 8; that successor's events must pass.
    assert!(
        !event_generation_is_stale(8, 9, 9),
        "a successor's early events must not be dropped while the \
             entry still holds the predecessor's generation"
    );

    // #2997: `dora node replace` minted the replacement at generation 10
    // (its counter is 10, no successor announced), while a concurrent
    // restart of the outgoing node minted a zombie at generation 11 in a
    // DIFFERENT lineage's counter. The zombie's events (gen 11) must not be
    // applied to the replacement, even though 11 > 10.
    assert!(
        event_generation_is_stale(10, 10, 11),
        "a stranger incarnation's higher generation must be rejected, \
             not misattributed to the replacement"
    );
}

// The old `patch_descriptor_entry` unit test was removed together with the
// function: `ReplaceNode` now assigns the descriptor entry wholesale from
// the original YAML-shape `Node` the coordinator ships, so there is no
// per-field enumeration left to unit-test — the type-level identity of the
// assignment IS the correctness guarantee.

#[test]
fn running_node_rejects_stale_generation() {
    let mut node = test_running_node();
    let reused_pid = Arc::new(AtomicU32::new(42));
    node.pid = Some(reused_pid);

    assert!(node.matches_generation(7));
    assert!(
        !node.matches_generation(6),
        "a reused PID must not make an older generation current"
    );
}

// The registration gate itself is tested through `restart_loop` in
// `spawn::prepared::tests` (`restart_loop_aborts_when_registration_never_happens`
// and `cancelled_restart_settles_terminal_exit`), which drive the real
// loop rather than restating oneshot-channel semantics here.

#[test]
fn planned_stop_markers_are_scoped_to_generation() {
    let dataflow = test_dataflow();
    let node_id: NodeId = "readded".to_string().into();

    dataflow.grace_duration_kills.insert((node_id.clone(), 7));

    assert!(
        dataflow
            .grace_duration_kills
            .contains(&(node_id.clone(), 7))
    );
    assert!(
        !dataflow
            .grace_duration_kills
            .contains(&(node_id.clone(), 8)),
        "a successor must not inherit its predecessor's planned-stop marker"
    );
    dataflow.grace_duration_kills.remove(&(node_id.clone(), 6));
    assert!(
        dataflow
            .grace_duration_kills
            .contains(&(node_id.clone(), 7)),
        "cleaning a stale event must not clear another generation's marker"
    );
}

#[test]
fn successful_readd_clears_only_the_previous_node_result() {
    let dataflow_id = Uuid::new_v4();
    let other_dataflow_id = Uuid::new_v4();
    let single_result_dataflow_id = Uuid::new_v4();
    let node_id: NodeId = "readded".to_string().into();
    let other_node_id: NodeId = "other".to_string().into();
    let mut results = BTreeMap::from([
        (
            dataflow_id,
            BTreeMap::from([(node_id.clone(), Ok(())), (other_node_id.clone(), Ok(()))]),
        ),
        (
            other_dataflow_id,
            BTreeMap::from([(node_id.clone(), Ok(()))]),
        ),
        (
            single_result_dataflow_id,
            BTreeMap::from([(node_id.clone(), Ok(()))]),
        ),
    ]);

    clear_node_result(&mut results, dataflow_id, &node_id);

    assert!(!results[&dataflow_id].contains_key(&node_id));
    assert!(results[&dataflow_id].contains_key(&other_node_id));
    assert!(results[&other_dataflow_id].contains_key(&node_id));

    clear_node_result(&mut results, single_result_dataflow_id, &node_id);
    assert!(!results.contains_key(&single_result_dataflow_id));
}

#[test]
fn stale_handle_replacement_preserves_live_process() {
    let mut node = test_running_node();
    let (live_tx, live_rx) = flume::bounded(2);
    node.process = Some(ProcessHandle::new(live_tx));

    let (stale_tx, stale_rx) = flume::bounded(2);
    let outcome = node.replace_process_handle(6, 8, ProcessHandle::new(stale_tx));

    assert!(
        matches!(outcome, HandleReplacement::RejectedStale),
        "a dead incarnation must not replace a live handle"
    );
    assert!(
        node.matches_generation(7),
        "a stale rejection must leave the live generation untouched"
    );
    assert!(
        live_rx.try_recv().is_err(),
        "rejecting a stale replacement must not kill the live successor"
    );
    assert!(
        matches!(stale_rx.try_recv(), Ok(ProcessOperation::Kill)),
        "the orphan process owned by the stale event should be killed"
    );

    // Avoid sending a drop-time Kill into `live_rx` after the assertions.
    let _ = node.process.take();
}

#[test]
fn teardown_rejects_matching_handle_replacement() {
    let mut node = test_running_node();
    let (live_tx, live_rx) = flume::bounded(2);
    node.process = Some(ProcessHandle::new(live_tx));
    node.disable_restart();

    let (replacement_tx, replacement_rx) = flume::bounded(2);
    let outcome = node.replace_process_handle(7, 8, ProcessHandle::new(replacement_tx));

    let HandleReplacement::RejectedTeardown(replacement) = outcome else {
        panic!("teardown must win a race with process replacement");
    };
    assert!(
        replacement_rx.try_recv().is_err(),
        "the caller must retain the replacement for planned-stop routing"
    );
    drop(replacement);
    assert!(
        matches!(replacement_rx.try_recv(), Ok(ProcessOperation::Kill)),
        "teardown must win a race with process replacement"
    );
    // The generation must still advance: the restart loop now speaks
    // generation 8, and its terminal SpawnedNodeResult has to match this
    // entry or the node would stay registered forever and the dataflow
    // could never finish.
    assert!(node.matches_generation(8));
    assert!(!node.matches_generation(7));
    assert!(
        node.process.is_some(),
        "teardown rejection must not install the replacement handle"
    );
    assert!(live_rx.try_recv().is_err());
    let _ = node.process.take();
}

#[tokio::test(start_paused = true)]
async fn teardown_replacement_uses_configured_grace_period() {
    let mut dataflow = test_dataflow();
    dataflow.stop_process_policy = Some(StopProcessPolicy::Graceful(Duration::from_secs(4)));
    let mut node = test_running_node();
    node.disable_restart();

    let (replacement_tx, replacement_rx) = flume::bounded(4);
    let outcome = node.replace_process_handle(7, 8, ProcessHandle::new(replacement_tx));
    let HandleReplacement::RejectedTeardown(replacement) = outcome else {
        panic!("teardown must retain ownership of the replacement handle");
    };

    let node_id: NodeId = "test".to_string().into();
    dataflow.stop_rejected_replacement(&node_id, 8, replacement);
    tokio::task::yield_now().await;

    assert!(
        replacement_rx.try_recv().is_err(),
        "a racing replacement must not be killed immediately"
    );
    tokio::time::advance(Duration::from_secs(3)).await;
    assert!(replacement_rx.try_recv().is_err());

    tokio::time::advance(Duration::from_secs(1)).await;
    tokio::task::yield_now().await;
    assert!(matches!(
        replacement_rx.try_recv(),
        Ok(ProcessOperation::SoftKill)
    ));
    assert!(
        dataflow
            .grace_duration_kills
            .contains(&(node_id.clone(), 8)),
        "the replacement stop must be classified as daemon-initiated"
    );

    tokio::time::advance(Duration::from_secs(2)).await;
    tokio::task::yield_now().await;
    assert!(matches!(
        replacement_rx.try_recv(),
        Ok(ProcessOperation::Kill)
    ));
}

#[test]
fn current_handle_replacement_advances_generation() {
    let mut node = test_running_node();
    let (old_tx, old_rx) = flume::bounded(2);
    node.process = Some(ProcessHandle::new(old_tx));

    let (new_tx, new_rx) = flume::bounded(2);
    let outcome = node.replace_process_handle(7, 8, ProcessHandle::new(new_tx));

    assert!(matches!(outcome, HandleReplacement::Replaced));
    assert!(node.matches_generation(8));
    assert!(
        matches!(old_rx.try_recv(), Ok(ProcessOperation::Kill)),
        "replacing the current incarnation should retire its old handle"
    );
    assert!(
        new_rx.try_recv().is_err(),
        "the replacement process must remain live"
    );

    let _ = node.process.take();
}

fn test_clock() -> HLC {
    HLC::default()
}

fn drain_events(rx: &mut mpsc::Receiver<Timestamped<NodeEvent>>) -> Vec<NodeEvent> {
    let mut events = Vec::new();
    while let Ok(timestamped) = rx.try_recv() {
        events.push(timestamped.inner);
    }
    events
}

fn matches_event(event: &NodeEvent, expected: &str) -> bool {
    matches!(
        (event, expected),
        (NodeEvent::InputClosed { .. }, "InputClosed")
            | (NodeEvent::InputRecovered { .. }, "InputRecovered")
            | (NodeEvent::AllInputsClosed, "AllInputsClosed")
            | (NodeEvent::Input { .. }, "Input")
    )
}

// -- Test 1: close_input removes input, sends InputClosed, no AllInputsClosed with remaining inputs --

/// Regression test for #241. The daemon used to carry two inverse maps
/// (`debug_topic_subscriptions: Uuid → Set<OutputId>` and
/// `debug_topic_watchers: OutputId → Set<Uuid>`). The inverse map was
/// dropped — unsubscribe now scans `debug_topic_watchers` with `retain`.
/// This test locks in the invariants the inverse map used to provide:
/// after stop, the subscription_id must be gone from every watcher set,
/// and outputs with no remaining watchers must be removed entirely.
#[test]
fn stop_debug_stream_scan_cleans_up_watchers() {
    let mut df = test_dataflow();
    let node_a: NodeId = "node_a".to_string().into();
    let out_1: DataId = "out_1".to_string().into();
    let out_2: DataId = "out_2".to_string().into();
    let sub_a = uuid::Uuid::new_v4();
    let sub_b = uuid::Uuid::new_v4();

    // Two subs, both watching out_1; only sub_a watches out_2.
    df.debug_topic_watchers
        .entry(OutputId(node_a.clone(), out_1.clone()))
        .or_default()
        .extend([sub_a, sub_b]);
    df.debug_topic_watchers
        .entry(OutputId(node_a.clone(), out_2.clone()))
        .or_default()
        .insert(sub_a);

    // Mirror the production stop path exactly.
    df.debug_topic_watchers.retain(|_output_id, watchers| {
        watchers.remove(&sub_a);
        !watchers.is_empty()
    });

    // out_1 still has sub_b → entry retained with just sub_b.
    let out_1_watchers = df
        .debug_topic_watchers
        .get(&OutputId(node_a.clone(), out_1))
        .expect("out_1 entry must remain because sub_b still watches it");
    assert_eq!(out_1_watchers.len(), 1);
    assert!(out_1_watchers.contains(&sub_b));

    // out_2 had only sub_a → entry removed entirely after scan.
    assert!(
        !df.debug_topic_watchers
            .contains_key(&OutputId(node_a, out_2)),
        "out_2 must be dropped because its only watcher was sub_a"
    );
}

#[test]
fn close_input_removes_from_open_inputs() {
    let mut df = test_dataflow();
    let clock = test_clock();
    let node_a: NodeId = "node_a".to_string().into();
    let input_x: DataId = "input_x".to_string().into();
    let input_y: DataId = "input_y".to_string().into();

    // Setup: node_a has two open inputs
    df.open_inputs
        .entry(node_a.clone())
        .or_default()
        .insert(input_x.clone());
    df.open_inputs
        .entry(node_a.clone())
        .or_default()
        .insert(input_y.clone());

    let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    df.subscribe_channels.insert(node_a.clone(), tx);

    // Act: permanently close input_x
    close_input(&mut df, &node_a, &input_x, &clock);

    // Assert: input_x removed, input_y still open
    let open = df.open_inputs(&node_a);
    assert!(!open.contains(&input_x));
    assert!(open.contains(&input_y));

    // Assert: only InputClosed sent (no AllInputsClosed)
    let events = drain_events(&mut rx);
    assert_eq!(events.len(), 1);
    assert!(matches_event(&events[0], "InputClosed"));
}

// -- dora#2270: finish-straggler watchdog must spare timer/log-fed nodes --

/// Insert a connected node that has been silent since the epoch, so
/// `finish_stragglers` sees it as long-idle regardless of grace.
fn insert_silent_node(df: &mut RunningDataflow, node: &NodeId) {
    let running = test_running_node();
    running.last_activity.store(1, atomic::Ordering::Release);
    df.running_nodes.insert(node.clone(), running);
    // a real running node has subscribed (can receive finish events)
    let (tx, _rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    df.subscribe_channels.insert(node.clone(), tx);
    df.connected_nodes.insert(node.clone());
}

/// Record a real `User` data input for `node`, exactly as the spawn and
/// `AddNode` paths do. `node_never_finishes` classifies "is this a source"
/// from `data_inputs`, so a fixture that skips this models a *source*, and
/// a source vetoes the watchdog for the whole dataflow — every assertion
/// below about a user-input node needs the input recorded to mean anything.
fn register_data_input(df: &mut RunningDataflow, node: &NodeId) {
    df.data_inputs
        .entry(node.clone())
        .or_default()
        .insert("value".to_string().into());
}

#[test]
fn timer_fed_node_is_not_a_finish_straggler() {
    // A long-running timer-only node never drains and sends no daemon
    // traffic, so it looks "silent + never drained" exactly like a wedge —
    // but it is alive by design and must NOT be force-killed (#2270).
    let mut df = test_dataflow();
    let timer_node: NodeId = "timer_node".to_string().into();
    insert_silent_node(&mut df, &timer_node);
    df.timers
        .entry(Duration::from_millis(100))
        .or_default()
        .insert((timer_node.clone(), "tick".to_string().into()));

    let now = node_communication::current_millis();
    let selected = df.finish_stragglers(Duration::from_millis(1), now);
    assert!(
        selected.is_empty(),
        "timer-fed node must not be escalated: {selected:?}"
    );
}

#[test]
fn wedged_user_input_node_is_a_finish_straggler() {
    // A connected node with no timer/log input that has gone silent past
    // grace while the rest of the dataflow finished IS a straggler.
    let mut df = test_dataflow();
    let stuck: NodeId = "stuck".to_string().into();
    insert_silent_node(&mut df, &stuck);
    register_data_input(&mut df, &stuck);

    let now = node_communication::current_millis();
    let selected = df.finish_stragglers(Duration::from_millis(1), now);
    assert_eq!(selected, vec![stuck]);
}

#[test]
fn unconnected_slow_starting_node_is_not_a_finish_straggler() {
    // `last_activity` is seeded at spawn, so a node that has not subscribed
    // yet looks long-silent — but it is still starting up (e.g. loading a
    // model) and must not be force-killed (#2270 review).
    let mut df = test_dataflow();
    let starting: NodeId = "slow_loader".to_string().into();
    let running = test_running_node();
    running.last_activity.store(1, atomic::Ordering::Release);
    df.running_nodes.insert(starting.clone(), running);
    register_data_input(&mut df, &starting);
    // NOTE: deliberately NOT added to subscribe_channels (not connected).

    let now = node_communication::current_millis();
    let selected = df.finish_stragglers(Duration::from_millis(1), now);
    assert!(
        selected.is_empty(),
        "a node that has not subscribed must not be escalated: {selected:?}"
    );
}

#[test]
fn dropped_stream_node_is_still_a_finish_straggler() {
    // A node that connected, then dropped its event stream (channel removed)
    // but kept its process alive, is still a wedge candidate — `connected`
    // tracks `connected_nodes`, not current channel presence (#2270 review).
    let mut df = test_dataflow();
    let stuck: NodeId = "stuck".to_string().into();
    let running = test_running_node();
    running.last_activity.store(1, atomic::Ordering::Release);
    df.running_nodes.insert(stuck.clone(), running);
    df.connected_nodes.insert(stuck.clone());
    register_data_input(&mut df, &stuck);
    // NOTE: no subscribe_channels entry — the event stream was dropped.

    let now = node_communication::current_millis();
    let selected = df.finish_stragglers(Duration::from_millis(1), now);
    assert_eq!(selected, vec![stuck]);
}

#[test]
fn removed_node_id_is_not_connected_on_reuse() {
    // RemoveNode clears connected_nodes, so a re-added node ID starts fresh:
    // its slow-starting new incarnation must not be selected before it
    // subscribes, even though the previous incarnation had connected.
    let mut df = test_dataflow();
    let node_a: NodeId = "node_a".to_string().into();
    df.connected_nodes.insert(node_a.clone());
    df.connected_nodes.remove(&node_a); // (the RemoveNode cleanup line)

    let running = test_running_node();
    running.last_activity.store(1, atomic::Ordering::Release);
    df.running_nodes.insert(node_a.clone(), running);
    register_data_input(&mut df, &node_a);

    let now = node_communication::current_millis();
    let selected = df.finish_stragglers(Duration::from_millis(1), now);
    assert!(
        selected.is_empty(),
        "a re-added node ID must not be selected before its new incarnation subscribes"
    );
}

#[test]
fn cleared_timer_state_makes_reused_id_escalatable() {
    // A timer-fed node is never-finishing (vetoed). RemoveNode clears its
    // timer subscription, so a re-added user-input node under the same ID is
    // classified by its own inputs and can be escalated (#2270 review).
    let mut df = test_dataflow();
    let node: NodeId = "reused".to_string().into();
    insert_silent_node(&mut df, &node);
    register_data_input(&mut df, &node);
    df.timers
        .entry(Duration::from_millis(100))
        .or_default()
        .insert((node.clone(), "tick".to_string().into()));

    let now = node_communication::current_millis();
    // as a timer node → vetoed
    assert!(
        df.finish_stragglers(Duration::from_millis(1), now)
            .is_empty(),
        "timer-fed node must be vetoed"
    );

    // RemoveNode clears the timer subscription
    for receivers in df.timers.values_mut() {
        receivers.retain(|(nid, _)| nid != &node);
    }
    assert_eq!(
        df.finish_stragglers(Duration::from_millis(1), now),
        vec![node],
        "once its timer state is cleared the node is classified by its own inputs"
    );
}

#[test]
fn reopened_input_clears_stale_drain_timestamp() {
    // A node drained long ago is eligible via the drained arm. Reopening a
    // mapping must clear that timestamp so the node — now actively receiving
    // again, not silent — is not force-stopped on a stale drain (#2270 review).
    let mut df = test_dataflow();
    let node: NodeId = "node_a".to_string().into();
    let running = test_running_node();
    // recent activity → not silent (only a stale drain clock could select it)
    running.last_activity.store(
        node_communication::current_millis(),
        atomic::Ordering::Release,
    );
    df.running_nodes.insert(node.clone(), running);
    df.connected_nodes.insert(node.clone());
    register_data_input(&mut df, &node);
    df.all_inputs_closed_at.insert(
        node.clone(),
        std::time::Instant::now() - Duration::from_secs(1),
    );

    let grace = Duration::from_millis(500);
    let now = node_communication::current_millis();
    // drained past grace → selected
    assert_eq!(df.finish_stragglers(grace, now), vec![node.clone()]);

    // AddMapping reopen clears the drain clock
    df.all_inputs_closed_at.remove(&node);
    assert!(
        df.finish_stragglers(grace, now).is_empty(),
        "an active node with a reopened input must not be selected on a stale drain"
    );
}

#[tokio::test]
async fn restart_clears_connected_marker() {
    // A restarting node keeps its ID but is a new incarnation: clear the
    // connected marker so the restarting process isn't silence-escalatable
    // before it re-subscribes (a slow restart / model reload) — #2270 review.
    use crate::running_dataflow::{ProcessHandle, ProcessOperation};
    let mut df = test_dataflow();
    let clock = test_clock();
    let node_a: NodeId = "node_a".to_string().into();

    let mut running = test_running_node();
    let (op_tx, _op_rx) = flume::unbounded::<ProcessOperation>();
    running.process = Some(ProcessHandle::new(op_tx));
    df.running_nodes.insert(node_a.clone(), running);
    df.connected_nodes.insert(node_a.clone());
    df.all_inputs_closed_at
        .insert(node_a.clone(), std::time::Instant::now());

    df.restart_single_node(&node_a, &clock, None).unwrap();

    assert!(
        !df.connected_nodes.contains(&node_a),
        "restart must clear the connected marker so the new incarnation re-subscribes"
    );
}

/// When a node's process exit is observed for a restart, the exit handler
/// resets the incarnation's bookkeeping via `reset_incarnation_state`. That
/// must clear the `dropped_event_streams` marker the exiting incarnation set on
/// its clean `EventStream::drop` — otherwise the respawned node that never
/// re-subscribes would have its upstream deliveries silenced as an intentional
/// drop instead of surfacing the #3201 "failed to re-subscribe" warning.
///
/// The marker is set first (mirroring the real order: the old process only
/// drops its stream after being told to stop, so `EventStreamDropped` runs
/// before the exit is observed), then the exit-path reset runs and must clear
/// it (dora-rs/dora#3558). This exercises the shared exit-handler path, which
/// covers both a `restart_policy` respawn and `dora node restart`.
#[test]
fn restart_exit_reset_clears_dropped_event_stream_marker() {
    let capture = LevelCapture::default();
    let rt = tokio::runtime::Builder::new_current_thread()
        .build()
        .unwrap();

    tracing::subscriber::with_default(capture.clone(), || {
        rt.block_on(async {
            let mut df = test_dataflow();
            let clock = test_clock();
            let sender: NodeId = "sender".to_string().into();
            let output: DataId = "output".to_string().into();
            let node_a: NodeId = "node_a".to_string().into();
            let input: DataId = "input".to_string().into();

            // The node is still a live process (in `running_nodes`) and its old
            // incarnation cleanly dropped its stream before exit — set the
            // marker via the same handler bookkeeping.
            df.running_nodes.insert(node_a.clone(), test_running_node());
            let (tx, _rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
            df.subscribe_channels.insert(node_a.clone(), tx);
            df.mark_event_stream_dropped(&node_a);
            df.mappings.insert(
                OutputId(sender.clone(), output.clone()),
                BTreeSet::from([(node_a.clone(), input.clone())]),
            );

            // The process exit is observed and the node will be restarted: the
            // exit handler resets the incarnation state.
            df.reset_incarnation_state(&node_a, 0);

            assert!(
                !df.dropped_event_streams.contains(&node_a),
                "the restart exit reset must clear the deliberate-drop marker so \
                 a fresh incarnation that fails to re-subscribe is diagnosed"
            );

            // The node is still in `running_nodes` (restart keeps it) but has no
            // channel and is no longer marked as a deliberate drop, so an
            // upstream delivery must WARN — the #3201 case the marker would
            // otherwise hide.
            let metadata = metadata::Metadata::new(clock.new_timestamp());
            let output_id = OutputId(sender, output);
            send_output_to_local_receivers(
                &output_id, &mut df, &metadata, None, &clock, None, false, None,
            )
            .await
            .unwrap();

            let warns = capture
                .levels
                .lock()
                .unwrap()
                .iter()
                .filter(|level| **level == tracing::Level::WARN)
                .count();
            assert_eq!(
                warns, 1,
                "a restarted node that never re-subscribed must WARN on delivery"
            );
        });
    });
}

#[test]
fn finish_drain_grace_defaults_on_with_opt_out() {
    // unset → enabled at the default grace (on by default, dora#2270 step 3)
    assert_eq!(
        parse_finish_drain_grace(None),
        Some(DEFAULT_FINISH_DRAIN_GRACE)
    );
    // explicit opt-out escape hatch → disabled
    assert_eq!(parse_finish_drain_grace(Some("off")), None);
    assert_eq!(parse_finish_drain_grace(Some("disabled")), None);
    assert_eq!(parse_finish_drain_grace(Some("OFF")), None);
    // set → enabled at the given grace
    assert_eq!(
        parse_finish_drain_grace(Some("30")),
        Some(Duration::from_secs(30))
    );
    assert_eq!(
        parse_finish_drain_grace(Some("0")),
        Some(Duration::from_secs(0))
    );
    // set-but-garbage → enabled at the default (the user meant to turn it on)
    assert_eq!(
        parse_finish_drain_grace(Some("not-a-number")),
        Some(DEFAULT_FINISH_DRAIN_GRACE)
    );
}

// -- Test 2: close_input sends AllInputsClosed + disable_restart on last input --

#[test]
fn close_input_sends_all_inputs_closed() {
    let mut df = test_dataflow();
    let clock = test_clock();
    let node_a: NodeId = "node_a".to_string().into();
    let input_x: DataId = "input_x".to_string().into();

    df.open_inputs
        .entry(node_a.clone())
        .or_default()
        .insert(input_x.clone());

    let running = test_running_node();
    let disable_restart = running.disable_restart.clone();
    df.running_nodes.insert(node_a.clone(), running);

    let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    df.subscribe_channels.insert(node_a.clone(), tx);

    close_input(&mut df, &node_a, &input_x, &clock);

    assert!(df.open_inputs(&node_a).is_empty());

    let events = drain_events(&mut rx);
    assert_eq!(events.len(), 2);
    assert!(matches_event(&events[0], "InputClosed"));
    assert!(matches_event(&events[1], "AllInputsClosed"));
    assert!(disable_restart.load(atomic::Ordering::Acquire));
}

/// dora-rs/dora#2920: under `--exit-when-nodes-finish` a node drains
/// once its DATA inputs close, even with a timer still open — and it
/// must have restart disabled at the same moment. Otherwise it exits,
/// gets restarted, is told to finish again, and loops until
/// `max_restarts`.
#[test]
fn opt_in_drains_node_with_open_timer_and_disables_its_restart() {
    let mut df = test_dataflow();
    let clock = test_clock();
    let node_a: NodeId = "node_a".to_string().into();
    let data_in: DataId = "value".to_string().into();
    let timer_in: DataId = "tick".to_string().into();

    df.timers_gate_drain = false;
    for input in [&data_in, &timer_in] {
        df.open_inputs
            .entry(node_a.clone())
            .or_default()
            .insert(input.clone());
    }
    // Only `value` is a real data dependency; `tick` is daemon-fed.
    df.data_inputs
        .entry(node_a.clone())
        .or_default()
        .insert(data_in.clone());
    df.timers
        .entry(Duration::from_millis(100))
        .or_default()
        .insert((node_a.clone(), timer_in.clone()));

    let running = test_running_node();
    let disable_restart = running.disable_restart.clone();
    df.running_nodes.insert(node_a.clone(), running);
    let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    df.subscribe_channels.insert(node_a.clone(), tx);

    close_input(&mut df, &node_a, &data_in, &clock);

    assert!(
        df.open_inputs(&node_a).contains(&timer_in),
        "the timer is still open — that is the whole point of the opt-in"
    );
    let events = drain_events(&mut rx);
    assert!(
        events.iter().any(|e| matches_event(e, "AllInputsClosed")),
        "node should be told to finish once its data inputs closed, got {events:?}"
    );
    assert!(
        disable_restart.load(atomic::Ordering::Acquire),
        "a node told to finish must not be restarted into a finish/exit loop"
    );
}

/// The opt-in must not stop disabling restart for SOURCE nodes.
/// `is_drained` reports false for them by design, so keying the
/// restart decision on it alone would let a source with
/// `restart_policy: always` respawn forever.
#[test]
fn opt_in_still_disables_restart_for_source_nodes() {
    let mut df = test_dataflow();
    let clock = test_clock();
    let node_a: NodeId = "node_a".to_string().into();
    let input_x: DataId = "input_x".to_string().into();

    df.timers_gate_drain = false;
    df.open_inputs
        .entry(node_a.clone())
        .or_default()
        .insert(input_x.clone());
    // No `data_inputs` entry: as far as the drain rule is concerned
    // this node has nothing that can finish, i.e. it is a source.
    assert!(!df.is_drained(&node_a));

    let running = test_running_node();
    let disable_restart = running.disable_restart.clone();
    df.running_nodes.insert(node_a.clone(), running);
    let (tx, _rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    df.subscribe_channels.insert(node_a.clone(), tx);

    close_input(&mut df, &node_a, &input_x, &clock);

    assert!(
        disable_restart.load(atomic::Ordering::Acquire),
        "source nodes must still have restart disabled once nothing is \
             open, independent of the drain opt-in"
    );
}

#[test]
fn forget_node_bookkeeping_purges_only_that_node() {
    // Regression: RemoveNode must drop the removed node's per-node
    // bookkeeping, otherwise stale input_deadlines/broken_inputs entries
    // are re-scanned every tick forever and the stderr queue leaks across
    // repeated dynamic add/remove cycles.
    let mut df = test_dataflow();
    let node_a: NodeId = "node_a".to_string().into();
    let node_b: NodeId = "node_b".to_string().into();
    let input_x: DataId = "input_x".to_string().into();
    let output_m: DataId = "message".to_string().into();
    let timeout = Duration::from_secs(1);

    let upstream: NodeId = "upstream".to_string().into();
    for node in [&node_a, &node_b] {
        df.input_deadlines.insert(
            (node.clone(), input_x.clone()),
            InputDeadline {
                timeout,
                last_received: None,
            },
        );
        df.broken_inputs
            .insert((node.clone(), input_x.clone()), timeout);
        df.node_stderr_most_recent
            .insert(node.clone(), Arc::new(ArrayQueue::new(4)));
        // Both nodes were recorded as cascading victims of `upstream`.
        df.cascading_error_causes
            .report_cascading_error(upstream.clone(), node.clone());
        // Both nodes have a debug-topic watcher on their `message` output.
        df.debug_topic_watchers.insert(
            OutputId(node.clone(), output_m.clone()),
            BTreeSet::from([uuid::Uuid::new_v4()]),
        );
    }

    df.forget_node_bookkeeping(&node_a);

    // node_a's entries are gone …
    assert!(
        !df.input_deadlines
            .contains_key(&(node_a.clone(), input_x.clone()))
    );
    assert!(
        !df.broken_inputs
            .contains_key(&(node_a.clone(), input_x.clone()))
    );
    assert!(!df.node_stderr_most_recent.contains_key(&node_a));
    // … including the stale cascading-error cause, so a re-added/replaced
    // `node_a` incarnation is classified by its own failure, not a stale
    // upstream cause (dora-rs/dora#2927).
    assert_eq!(df.cascading_error_causes.error_caused_by(&node_a), None);
    // … but NOT its debug-topic watchers. This cleanup runs on both the
    // `RemoveNode` and `ReplaceNode` paths, and neither re-registers a
    // watcher afterwards (only a fresh `StartTopicDebugStream` does). Since
    // remove + re-add and replace both resume under the same
    // `OutputId(node_id, output)`, purging here would silently kill an
    // active `dora topic` stream across the cycle. Watchers are never
    // purged by node-id churn anywhere; this pins that.
    assert!(
        df.debug_topic_watchers
            .contains_key(&OutputId(node_a.clone(), output_m.clone())),
        "debug-topic watchers must survive `forget_node_bookkeeping` on \
             every path, so a remove+re-add or ReplaceNode keeps an active \
             debug stream alive"
    );

    // … while node_b's are untouched.
    assert!(
        df.input_deadlines
            .contains_key(&(node_b.clone(), input_x.clone()))
    );
    assert!(
        df.broken_inputs
            .contains_key(&(node_b.clone(), input_x.clone()))
    );
    assert!(df.node_stderr_most_recent.contains_key(&node_b));
    assert_eq!(
        df.cascading_error_causes.error_caused_by(&node_b),
        Some(&upstream)
    );
    assert!(
        df.debug_topic_watchers
            .contains_key(&OutputId(node_b.clone(), output_m.clone()))
    );
}

// -- Test 3: close_input defers AllInputsClosed when broken_inputs exist --

#[test]
fn close_input_deferred_by_broken_inputs() {
    let mut df = test_dataflow();
    let clock = test_clock();
    let node_a: NodeId = "node_a".to_string().into();
    let input_x: DataId = "input_x".to_string().into();
    let input_y: DataId = "input_y".to_string().into();

    // node_a has input_x open, input_y is broken
    df.open_inputs
        .entry(node_a.clone())
        .or_default()
        .insert(input_x.clone());
    df.broken_inputs
        .insert((node_a.clone(), input_y.clone()), Duration::from_secs(10));

    let running = test_running_node();
    let disable_restart = running.disable_restart.clone();
    df.running_nodes.insert(node_a.clone(), running);

    let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    df.subscribe_channels.insert(node_a.clone(), tx);

    // Close last open input — but broken input still exists
    close_input(&mut df, &node_a, &input_x, &clock);

    let events = drain_events(&mut rx);
    // Only InputClosed, NO AllInputsClosed (broken input might recover)
    assert_eq!(events.len(), 1);
    assert!(matches_event(&events[0], "InputClosed"));
    assert!(!disable_restart.load(atomic::Ordering::Acquire));
}

// -- Test 4: close_input on already-broken input --

#[test]
fn close_input_on_already_broken_input() {
    let mut df = test_dataflow();
    let clock = test_clock();
    let node_a: NodeId = "node_a".to_string().into();
    let input_x: DataId = "input_x".to_string().into();

    // input_x is broken (not in open_inputs)
    df.broken_inputs
        .insert((node_a.clone(), input_x.clone()), Duration::from_secs(10));

    let running = test_running_node();
    let disable_restart = running.disable_restart.clone();
    df.running_nodes.insert(node_a.clone(), running);

    let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    df.subscribe_channels.insert(node_a.clone(), tx);

    // Permanently close the broken input (upstream exited)
    close_input(&mut df, &node_a, &input_x, &clock);

    // broken_inputs cleaned up
    assert!(
        !df.broken_inputs
            .contains_key(&(node_a.clone(), input_x.clone()))
    );

    let events = drain_events(&mut rx);
    // No InputClosed (was already sent when it broke), just AllInputsClosed
    assert_eq!(events.len(), 1);
    assert!(matches_event(&events[0], "AllInputsClosed"));
    assert!(disable_restart.load(atomic::Ordering::Acquire));
}

// -- #2968: a closed input's armed deadline must not orphan a broken record --

#[test]
fn close_input_drops_armed_deadline() {
    // Regression (#2968): `close_input` must drop the input's
    // `input_deadlines` entry. Otherwise the stale, still-armed deadline
    // keeps counting after the input has closed; `check_input_timeouts`
    // then fires on the already-closed input, inserting a `broken_inputs`
    // record that `break_input` can never clear.
    let mut df = test_dataflow();
    let clock = test_clock();
    let node_a: NodeId = "node_a".to_string().into();
    let input_x: DataId = "input_x".to_string().into();

    df.open_inputs
        .entry(node_a.clone())
        .or_default()
        .insert(input_x.clone());
    // Armed (a message was received), so it would time out if left behind.
    df.input_deadlines.insert(
        (node_a.clone(), input_x.clone()),
        InputDeadline {
            timeout: Duration::from_millis(1),
            last_received: Some(Instant::now() - Duration::from_secs(10)),
        },
    );

    close_input(&mut df, &node_a, &input_x, &clock);

    assert!(
        !df.input_deadlines
            .contains_key(&(node_a.clone(), input_x.clone())),
        "close_input must drop the input's deadline so it can't time out later"
    );
}

#[test]
fn drained_node_finishes_despite_stale_deadline_timeout() {
    // End-to-end of #2968: a node with two inputs, one carrying an
    // `input_timeout`. The timed input's producer exits first, then the
    // stale deadline "fires" (as `check_input_timeouts` would), then the
    // second producer exits. The node must still be told `AllInputsClosed`
    // — not left with an orphaned `broken_inputs` record that pins
    // `has_broken_input` true forever and gets it SIGKILLed as a straggler.
    let mut df = test_dataflow();
    let clock = test_clock();
    let node_c: NodeId = "node_c".to_string().into();
    let input_a: DataId = "input_a".to_string().into();
    let input_b: DataId = "input_b".to_string().into();

    df.open_inputs
        .entry(node_c.clone())
        .or_default()
        .extend([input_a.clone(), input_b.clone()]);
    // input_a has an armed deadline (producer sent messages before exiting).
    df.input_deadlines.insert(
        (node_c.clone(), input_a.clone()),
        InputDeadline {
            timeout: Duration::from_millis(1),
            last_received: Some(Instant::now() - Duration::from_secs(10)),
        },
    );

    let running = test_running_node();
    df.running_nodes.insert(node_c.clone(), running);
    let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    df.subscribe_channels.insert(node_c.clone(), tx);

    // 1. Producer of input_a exits.
    close_input(&mut df, &node_c, &input_a, &clock);

    // 2. Simulate the stale-deadline timeout firing as `check_input_timeouts`
    //    would: it inserts a broken record then calls break_input. With the
    //    fix, close_input already dropped the deadline, so this loop finds
    //    nothing; we force the worst case anyway to prove break_input does
    //    not leave an orphan even if it is reached on a closed input.
    df.broken_inputs
        .insert((node_c.clone(), input_a.clone()), Duration::from_millis(1));
    break_input(&mut df, &node_c, &input_a, &clock);
    assert!(
        !df.has_broken_input(&node_c),
        "break_input on an already-closed input must not orphan a broken record"
    );

    // 3. Producer of input_b exits — node is now fully drained.
    close_input(&mut df, &node_c, &input_b, &clock);

    let events = drain_events(&mut rx);
    assert!(
        events.iter().any(|e| matches_event(e, "AllInputsClosed")),
        "a cleanly-drained node must be told AllInputsClosed, got {events:?}"
    );
}

// -- Circuit-breaker recovery must be gated on receiver backpressure (#2627) --

/// Helper: wire `sender/output -> receiver/input` and mark the input broken.
/// `channel_capacity` controls whether the receiver counts as "keeping up"
/// (>= CONTROL_EVENT_HEADROOM) or "saturated" (< CONTROL_EVENT_HEADROOM).
fn broken_input_dataflow(
    channel_capacity: usize,
) -> (
    RunningDataflow,
    NodeId,
    DataId,
    NodeId,
    DataId,
    mpsc::Receiver<Timestamped<NodeEvent>>,
) {
    let mut df = test_dataflow();
    let sender: NodeId = "sender".to_string().into();
    let output: DataId = "output".to_string().into();
    let receiver: NodeId = "receiver".to_string().into();
    let input: DataId = "input".to_string().into();

    let mut mapping = BTreeSet::new();
    mapping.insert((receiver.clone(), input.clone()));
    df.mappings
        .insert(OutputId(sender.clone(), output.clone()), mapping);

    let (tx, rx) = mpsc::channel(channel_capacity);
    df.subscribe_channels.insert(receiver.clone(), tx);

    // Input is currently broken (circuit breaker open).
    df.broken_inputs
        .insert((receiver.clone(), input.clone()), Duration::from_secs(5));

    (df, sender, output, receiver, input, rx)
}

/// A bare `OutputSent` is not a delivery confirmation. When the receiver's
/// event channel is still saturated (`capacity() < CONTROL_EVENT_HEADROOM`),
/// recovery must NOT fire — otherwise the circuit breaker flaps
/// `broken ↔ recovered` every `input_timeout` for a persistently-slow
/// consumer without any data actually getting through.
#[test]
fn output_sent_does_not_recover_broken_input_when_receiver_saturated() {
    let clock = test_clock();
    let ft_stats = FaultToleranceStats::default();
    // capacity 1 < CONTROL_EVENT_HEADROOM (50) => not keeping up
    let (mut df, sender, output, receiver, input, mut rx) = broken_input_dataflow(1);

    note_output_sent_to_local_receivers(sender, output, &mut df, &clock, Some(&ft_stats));

    assert!(
        df.broken_inputs.contains_key(&(receiver, input)),
        "broken input must stay broken while the receiver is saturated"
    );
    assert_eq!(
        ft_stats
            .circuit_breaker_recoveries
            .load(atomic::Ordering::Relaxed),
        0,
        "no recovery may be counted for a saturated receiver"
    );
    let events = drain_events(&mut rx);
    assert!(
        !events.iter().any(|e| matches_event(e, "InputRecovered")),
        "no InputRecovered may be emitted while the receiver is saturated"
    );
}

/// Companion case: once the receiver has channel headroom (is keeping up),
/// the same `OutputSent` recovers the broken input exactly as before.
#[test]
fn output_sent_recovers_broken_input_when_receiver_keeping_up() {
    let clock = test_clock();
    let ft_stats = FaultToleranceStats::default();
    // an empty NODE_EVENT_CHANNEL_CAPACITY channel has ample headroom
    let (mut df, sender, output, receiver, input, mut rx) =
        broken_input_dataflow(NODE_EVENT_CHANNEL_CAPACITY);

    note_output_sent_to_local_receivers(sender, output, &mut df, &clock, Some(&ft_stats));

    assert!(
        !df.broken_inputs.contains_key(&(receiver, input)),
        "broken input must recover once the receiver is keeping up"
    );
    assert_eq!(
        ft_stats
            .circuit_breaker_recoveries
            .load(atomic::Ordering::Relaxed),
        1,
        "recovery must be counted once the receiver is keeping up"
    );
    let events = drain_events(&mut rx);
    assert!(
        events.iter().any(|e| matches_event(e, "InputRecovered")),
        "InputRecovered must be emitted on recovery"
    );
}

// -- Test 5: break_input sends InputClosed --

#[test]
fn break_input_sends_input_closed() {
    let mut df = test_dataflow();
    let clock = test_clock();
    let node_a: NodeId = "node_a".to_string().into();
    let input_x: DataId = "input_x".to_string().into();

    df.open_inputs
        .entry(node_a.clone())
        .or_default()
        .insert(input_x.clone());

    // Pre-populate broken_inputs (as check_input_timeouts does before calling break_input)
    df.broken_inputs
        .insert((node_a.clone(), input_x.clone()), Duration::from_secs(5));

    let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    df.subscribe_channels.insert(node_a.clone(), tx);

    break_input(&mut df, &node_a, &input_x, &clock);

    // Removed from open_inputs
    assert!(!df.open_inputs(&node_a).contains(&input_x));

    let events = drain_events(&mut rx);
    assert_eq!(events.len(), 1);
    assert!(matches_event(&events[0], "InputClosed"));
}

// -- Test 6: break_input defers AllInputsClosed when broken_inputs has entry --

#[test]
fn break_input_defers_all_inputs_closed() {
    let mut df = test_dataflow();
    let clock = test_clock();
    let node_a: NodeId = "node_a".to_string().into();
    let input_x: DataId = "input_x".to_string().into();

    // Only one input, and it's open
    df.open_inputs
        .entry(node_a.clone())
        .or_default()
        .insert(input_x.clone());

    // Caller inserts into broken_inputs before calling break_input
    df.broken_inputs
        .insert((node_a.clone(), input_x.clone()), Duration::from_secs(5));

    let running = test_running_node();
    let disable_restart = running.disable_restart.clone();
    df.running_nodes.insert(node_a.clone(), running);

    let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    df.subscribe_channels.insert(node_a.clone(), tx);

    break_input(&mut df, &node_a, &input_x, &clock);

    let events = drain_events(&mut rx);
    // Only InputClosed — AllInputsClosed deferred because broken_inputs has entry
    assert_eq!(events.len(), 1);
    assert!(matches_event(&events[0], "InputClosed"));
    assert!(!disable_restart.load(atomic::Ordering::Acquire));
}

// -- Test 7: Circuit breaker recovery via send_output_to_local_receivers --

#[tokio::test]
async fn circuit_breaker_recovery() {
    let mut df = test_dataflow();
    let clock = test_clock();
    let sender: NodeId = "sender".to_string().into();
    let receiver: NodeId = "receiver".to_string().into();
    let output: DataId = "output".to_string().into();
    let input: DataId = "input".to_string().into();

    // Setup mapping: sender/output -> receiver/input
    df.mappings
        .entry(OutputId(sender.clone(), output.clone()))
        .or_default()
        .insert((receiver.clone(), input.clone()));

    // Input is broken (timed out earlier)
    let timeout = Duration::from_secs(5);
    df.broken_inputs
        .insert((receiver.clone(), input.clone()), timeout);

    let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    df.subscribe_channels.insert(receiver.clone(), tx);

    // Send data from upstream
    let metadata = metadata::Metadata::new(clock.new_timestamp());

    let output_id = OutputId(sender, output);
    let result = send_output_to_local_receivers(
        &output_id, &mut df, &metadata, None, &clock, None, false, None,
    )
    .await;
    assert!(result.is_ok());

    // Assert: broken input recovered
    assert!(
        !df.broken_inputs
            .contains_key(&(receiver.clone(), input.clone()))
    );

    // Assert: re-added to open_inputs
    assert!(df.open_inputs(&receiver).contains(&input));

    // Assert: deadline recreated
    assert!(
        df.input_deadlines
            .contains_key(&(receiver.clone(), input.clone()))
    );
    let deadline = &df.input_deadlines[&(receiver.clone(), input.clone())];
    assert_eq!(deadline.timeout, timeout);

    // Assert: events — Input + InputRecovered
    let events = drain_events(&mut rx);
    assert_eq!(events.len(), 2);
    assert!(matches_event(&events[0], "Input"));
    assert!(matches_event(&events[1], "InputRecovered"));
}

// -- need_data_bytes: payload must be returned for remote forwarding --

#[tokio::test]
async fn data_bytes_returned_with_and_without_local_receivers() {
    let clock = test_clock();
    let payload = [1u8, 2, 3, 4];
    let sender: NodeId = "sender".to_string().into();
    let output: DataId = "output".to_string().into();

    // Without local receivers the data Arc is uniquely owned; the payload
    // is moved out (no copy) but must still be returned for the remote
    // forwarding path.
    let mut df = test_dataflow();
    let metadata = metadata::Metadata::new(clock.new_timestamp());
    let data = DataMessage::Vec(AVec::from_slice(128, &payload));
    let output_id = OutputId(sender.clone(), output.clone());
    let result = send_output_to_local_receivers(
        &output_id,
        &mut df,
        &metadata,
        Some(data),
        &clock,
        None,
        true,
        None,
    )
    .await
    .unwrap();
    assert_eq!(result.as_deref(), Some(&payload[..]));

    // With a local receiver holding a reference, the bytes are cloned and
    // must match the payload delivered to the receiver.
    let mut df = test_dataflow();
    let receiver: NodeId = "receiver".to_string().into();
    let input: DataId = "input".to_string().into();
    df.mappings
        .entry(OutputId(sender.clone(), output.clone()))
        .or_default()
        .insert((receiver.clone(), input.clone()));
    let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    df.subscribe_channels.insert(receiver.clone(), tx);
    let metadata = metadata::Metadata::new(clock.new_timestamp());
    let data = DataMessage::Vec(AVec::from_slice(128, &payload));
    let output_id = OutputId(sender, output);
    let result = send_output_to_local_receivers(
        &output_id,
        &mut df,
        &metadata,
        Some(data),
        &clock,
        None,
        true,
        None,
    )
    .await
    .unwrap();
    assert_eq!(result.as_deref(), Some(&payload[..]));
    let events = drain_events(&mut rx);
    assert_eq!(events.len(), 1);
    assert!(matches_event(&events[0], "Input"));
}

// -- Regression tests for dora-rs/dora#3201: a receiver whose event
//    stream is gone must not be silently starved. --

/// Minimal `tracing::Subscriber` that records the level of every event it
/// receives, so a test can assert whether (and how often) a warning fires.
#[derive(Clone, Default)]
struct LevelCapture {
    levels: Arc<Mutex<Vec<tracing::Level>>>,
}

impl tracing::Subscriber for LevelCapture {
    fn enabled(&self, _metadata: &tracing::Metadata<'_>) -> bool {
        true
    }
    fn new_span(&self, _span: &tracing::span::Attributes<'_>) -> tracing::span::Id {
        tracing::span::Id::from_u64(1)
    }
    fn record(&self, _span: &tracing::span::Id, _values: &tracing::span::Record<'_>) {}
    fn record_follows_from(&self, _span: &tracing::span::Id, _follows: &tracing::span::Id) {}
    fn event(&self, event: &tracing::Event<'_>) {
        self.levels.lock().unwrap().push(*event.metadata().level());
    }
    fn enter(&self, _span: &tracing::span::Id) {}
    fn exit(&self, _span: &tracing::span::Id) {}
}

/// With the receiver's event channel full, a `backpressure` input's message
/// is handed back for the producer's listener to deliver once there is room,
/// while a `drop_oldest` input's is dropped, warned about and counted — and
/// with no listener to hand it to (a remote forward), the backpressure one is
/// dropped and counted as well (dora-rs/dora#3397).
#[test]
fn full_channel_defers_backpressure_inputs_and_counts_the_rest() {
    use dora_message::config::QueuePolicy;
    let capture = LevelCapture::default();
    let rt = tokio::runtime::Builder::new_current_thread()
        .build()
        .unwrap();

    tracing::subscriber::with_default(capture.clone(), || {
        rt.block_on(async {
            let mut df = test_dataflow();
            let clock = test_clock();
            let sender: NodeId = "sender".to_string().into();
            let output: DataId = "output".to_string().into();
            let input: DataId = "input".to_string().into();
            let patient: NodeId = "patient".to_string().into();
            let hasty: NodeId = "hasty".to_string().into();

            df.mappings.insert(
                OutputId(sender.clone(), output.clone()),
                BTreeSet::from([
                    (patient.clone(), input.clone()),
                    (hasty.clone(), input.clone()),
                ]),
            );
            let mut receivers = Vec::new();
            for (receiver, policy) in [
                (&patient, QueuePolicy::Backpressure),
                (&hasty, QueuePolicy::DropOldest),
            ] {
                let inputs =
                    BTreeMap::from([(input.clone(), user_input("sender", "output", Some(policy)))]);
                df.running_nodes
                    .insert(receiver.clone(), running_node_with(inputs, None));
                let (tx, rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
                // Leave fewer free slots than the control-event headroom.
                for _ in 0..NODE_EVENT_CHANNEL_CAPACITY - CONTROL_EVENT_HEADROOM + 1 {
                    tx.try_send(Timestamped {
                        inner: NodeEvent::Stop,
                        timestamp: clock.new_timestamp(),
                    })
                    .unwrap();
                }
                df.subscribe_channels.insert(receiver.clone(), tx);
                df.pending_messages
                    .insert(receiver.clone(), Arc::new(AtomicU64::new(0)));
                df.drain_signals
                    .insert(receiver.clone(), Arc::new(tokio::sync::Notify::new()));
                receivers.push(rx);
            }

            let ft_stats = FaultToleranceStats::default();
            let metadata = metadata::Metadata::new(clock.new_timestamp());
            let output_id = OutputId(sender, output);

            let mut deferred = Vec::new();
            send_output_to_local_receivers(
                &output_id,
                &mut df,
                &metadata,
                None,
                &clock,
                Some(&ft_stats),
                false,
                Some(&mut deferred),
            )
            .await
            .unwrap();
            assert_eq!(deferred.len(), 1, "only the backpressure edge is deferred");
            assert_eq!(deferred[0].receiver, patient);
            assert!(deferred[0].pending.is_some());
            assert_eq!(
                ft_stats.dropped_messages.load(atomic::Ordering::Relaxed),
                1,
                "the drop_oldest edge's message is dropped and counted"
            );
            assert_eq!(
                ft_stats
                    .lost_backpressure_messages
                    .load(atomic::Ordering::Relaxed),
                0,
                "nothing promised was lost"
            );
            assert_eq!(
                df.pending_messages[&patient].load(atomic::Ordering::Relaxed),
                0,
                "a deferred delivery is not pending until it lands"
            );

            // No one to wait for room: both are dropped and counted.
            send_output_to_local_receivers(
                &output_id,
                &mut df,
                &metadata,
                None,
                &clock,
                Some(&ft_stats),
                false,
                None,
            )
            .await
            .unwrap();
            assert_eq!(ft_stats.dropped_messages.load(atomic::Ordering::Relaxed), 3);
            assert_eq!(
                ft_stats
                    .lost_backpressure_messages
                    .load(atomic::Ordering::Relaxed),
                1,
                "only the backpressure edge's drop breaks a promise"
            );

            let levels = capture.levels.lock().unwrap();
            let warns = levels
                .iter()
                .filter(|level| **level == tracing::Level::WARN)
                .count();
            assert_eq!(
                warns, 3,
                "one warning per drop, none for the deferral: {levels:?}"
            );
        });
    });
}

/// A message to a live receiver with no event stream (the #3201 mode: mid-
/// restart, or failed to re-subscribe) is a counted drop — and a lost one on
/// a backpressure input — so `fail_on_lost_backpressure_messages` sees it.
/// A receiver that finished and dropped its stream on purpose is not.
#[test]
fn missing_event_stream_drops_are_counted_per_message() {
    use dora_message::config::QueuePolicy;
    let rt = tokio::runtime::Builder::new_current_thread()
        .build()
        .unwrap();
    rt.block_on(async {
        let mut df = test_dataflow();
        let clock = test_clock();
        let sender: NodeId = "sender".to_string().into();
        let output: DataId = "output".to_string().into();
        let input: DataId = "input".to_string().into();
        let receiver: NodeId = "receiver".to_string().into();
        df.mappings.insert(
            OutputId(sender.clone(), output.clone()),
            BTreeSet::from([(receiver.clone(), input.clone())]),
        );
        let inputs = BTreeMap::from([(
            input.clone(),
            user_input("sender", "output", Some(QueuePolicy::Backpressure)),
        )]);
        df.running_nodes
            .insert(receiver.clone(), running_node_with(inputs, None));

        let ft_stats = FaultToleranceStats::default();
        let metadata = metadata::Metadata::new(clock.new_timestamp());
        let output_id = OutputId(sender, output);
        for _ in 0..2 {
            send_output_to_local_receivers(
                &output_id,
                &mut df,
                &metadata,
                None,
                &clock,
                Some(&ft_stats),
                false,
                None,
            )
            .await
            .unwrap();
        }
        assert_eq!(ft_stats.dropped_messages.load(atomic::Ordering::Relaxed), 2);
        assert_eq!(
            ft_stats
                .lost_backpressure_messages
                .load(atomic::Ordering::Relaxed),
            2,
            "every message to the streamless receiver is a lost promise, not just the warned one"
        );

        // Finished normally: not a drop anyone promised against.
        df.dropped_event_streams.insert(receiver.clone());
        send_output_to_local_receivers(
            &output_id,
            &mut df,
            &metadata,
            None,
            &clock,
            Some(&ft_stats),
            false,
            None,
        )
        .await
        .unwrap();
        assert_eq!(ft_stats.dropped_messages.load(atomic::Ordering::Relaxed), 2);
    });
}

/// A receiver recorded in `mappings` but missing from
/// `subscribe_channels` gets routed to *nothing*: `send_output_to_local_receivers`
/// cannot deliver, and the producer's send still "succeeds". The missing
/// listener must therefore be made visible, exactly once per edge, instead
/// of silently starving the consumer (dora-rs/dora#3201).
#[test]
fn receiver_missing_channel_is_skipped_with_once_per_edge_warning() {
    let capture = LevelCapture::default();
    let rt = tokio::runtime::Builder::new_current_thread()
        .build()
        .unwrap();

    tracing::subscriber::with_default(capture.clone(), || {
        rt.block_on(async {
            let mut df = test_dataflow();
            let clock = test_clock();
            let sender: NodeId = "sender".to_string().into();
            let output: DataId = "output".to_string().into();
            let receiver: NodeId = "receiver".to_string().into();
            let input: DataId = "input".to_string().into();

            df.mappings.insert(
                OutputId(sender.clone(), output.clone()),
                BTreeSet::from([(receiver.clone(), input.clone())]),
            );
            // The receiver is still a live node (in `running_nodes`) but
            // its event stream is gone. This is the #3201 case: a mapped,
            // running consumer that cannot be reached.
            df.running_nodes
                .insert(receiver.clone(), test_running_node());
            // Deliberately no `subscribe_channels` entry: the receiver's
            // event stream is gone (dropped or closed).

            let metadata = metadata::Metadata::new(clock.new_timestamp());
            let output_id = OutputId(sender, output);
            send_output_to_local_receivers(
                &output_id, &mut df, &metadata, None, &clock, None, false, None,
            )
            .await
            .unwrap();
            // And again: the *second* drop of the same edge must not warn.
            send_output_to_local_receivers(
                &output_id, &mut df, &metadata, None, &clock, None, false, None,
            )
            .await
            .unwrap();

            let levels = capture.levels.lock().unwrap();
            let warns = levels
                .iter()
                .filter(|level| **level == tracing::Level::WARN)
                .count();
            assert_eq!(
                warns, 1,
                "the orphaned edge must warn exactly once, got {levels:?}"
            );
        });
    });
}

/// One starved consumer must not take the whole fan-out down: a sibling
/// receiver with a live channel still gets its message while the orphaned
/// one is warned about exactly once.
#[test]
fn healthy_receiver_still_receives_when_peer_channel_is_missing() {
    let capture = LevelCapture::default();
    let rt = tokio::runtime::Builder::new_current_thread()
        .build()
        .unwrap();

    let delivered = tracing::subscriber::with_default(capture.clone(), || {
        rt.block_on(async {
            let mut df = test_dataflow();
            let clock = test_clock();
            let sender: NodeId = "sender".to_string().into();
            let output: DataId = "output".to_string().into();
            let healthy: NodeId = "healthy".to_string().into();
            let orphaned: NodeId = "orphaned".to_string().into();
            let input: DataId = "input".to_string().into();

            df.mappings.insert(
                OutputId(sender.clone(), output.clone()),
                BTreeSet::from([
                    (healthy.clone(), input.clone()),
                    (orphaned.clone(), input.clone()),
                ]),
            );

            let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
            df.subscribe_channels.insert(healthy.clone(), tx);
            // The starved sibling is still a live node whose stream is
            // gone — the #3201 case.
            df.running_nodes
                .insert(orphaned.clone(), test_running_node());

            let metadata = metadata::Metadata::new(clock.new_timestamp());
            let output_id = OutputId(sender, output);
            send_output_to_local_receivers(
                &output_id, &mut df, &metadata, None, &clock, None, false, None,
            )
            .await
            .unwrap();

            let events = drain_events(&mut rx);
            let levels = capture.levels.lock().unwrap();
            let warns = levels
                .iter()
                .filter(|level| **level == tracing::Level::WARN)
                .count();
            assert_eq!(
                warns, 1,
                "the orphaned sibling must warn once while the healthy \
                     receiver is served, got {levels:?}"
            );
            events
        })
    });

    assert_eq!(delivered.len(), 1, "healthy receiver must be served once");
    assert!(matches_event(&delivered[0], "Input"));
}

/// A consumer that finished (or was stopped) normally leaves its
/// receiver-edge mapping behind until the dataflow tears down, so an
/// upstream that keeps sending still reaches the no-channel branch. That
/// is an expected dead edge, not #3201 symptomatology — it must NOT WARN.
#[test]
fn finished_receiver_does_not_warn() {
    let capture = LevelCapture::default();
    let rt = tokio::runtime::Builder::new_current_thread()
        .build()
        .unwrap();

    tracing::subscriber::with_default(capture.clone(), || {
        rt.block_on(async {
            let mut df = test_dataflow();
            let clock = test_clock();
            let sender: NodeId = "sender".to_string().into();
            let output: DataId = "output".to_string().into();
            let finished: NodeId = "finished".to_string().into();
            let input: DataId = "input".to_string().into();

            df.mappings.insert(
                OutputId(sender.clone(), output.clone()),
                BTreeSet::from([(finished.clone(), input.clone())]),
            );
            // Deliberately no `subscribe_channels` entry and NO
            // `running_nodes` entry: `handle_node_stop_inner` removed the
            // node but left its receiver-edge mapping in place.

            let metadata = metadata::Metadata::new(clock.new_timestamp());
            let output_id = OutputId(sender, output);
            send_output_to_local_receivers(
                &output_id, &mut df, &metadata, None, &clock, None, false, None,
            )
            .await
            .unwrap();

            let levels = capture.levels.lock().unwrap();
            let warns = levels
                .iter()
                .filter(|level| **level == tracing::Level::WARN)
                .count();
            assert_eq!(
                warns, 0,
                "a finished (no longer running) receiver must not WARN, got {levels:?}"
            );
        });
    });
}

/// A consumer that finished normally sends `EventStreamDropped`, which removes
/// its `subscribe_channels` entry but leaves it in `running_nodes` until its
/// process exit is observed. In that window an upstream still producing to it
/// must NOT WARN — it is a deliberate drop, not the silent-routing-loss of
/// #3201. This is the gap `finished_receiver_does_not_warn` misses: there the
/// node is already out of `running_nodes`, so it never exercises the
/// still-running window (dora-rs/dora#3556).
#[test]
fn finished_but_still_running_receiver_does_not_warn() {
    let capture = LevelCapture::default();
    let rt = tokio::runtime::Builder::new_current_thread()
        .build()
        .unwrap();

    tracing::subscriber::with_default(capture.clone(), || {
        rt.block_on(async {
            let mut df = test_dataflow();
            let clock = test_clock();
            let sender: NodeId = "sender".to_string().into();
            let output: DataId = "output".to_string().into();
            let finished: NodeId = "finished".to_string().into();
            let input: DataId = "input".to_string().into();

            df.mappings.insert(
                OutputId(sender.clone(), output.clone()),
                BTreeSet::from([(finished.clone(), input.clone())]),
            );
            // The node is still a live process (in `running_nodes`) and had an
            // event stream, then dropped it. Drive the real bookkeeping the
            // `EventStreamDropped` handler runs so this test covers that path
            // rather than reproducing its effect by hand.
            df.running_nodes
                .insert(finished.clone(), test_running_node());
            let (tx, _rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
            df.subscribe_channels.insert(finished.clone(), tx);
            df.mark_event_stream_dropped(&finished);
            assert!(
                !df.subscribe_channels.contains_key(&finished)
                    && df.dropped_event_streams.contains(&finished),
                "mark_event_stream_dropped must drop the channel and set the marker"
            );

            let metadata = metadata::Metadata::new(clock.new_timestamp());
            let output_id = OutputId(sender, output);
            send_output_to_local_receivers(
                &output_id, &mut df, &metadata, None, &clock, None, false, None,
            )
            .await
            .unwrap();

            let levels = capture.levels.lock().unwrap();
            let warns = levels
                .iter()
                .filter(|level| **level == tracing::Level::WARN)
                .count();
            assert_eq!(
                warns, 0,
                "a consumer that dropped its stream but is still running must \
                 not WARN, got {levels:?}"
            );
        });
    });
}

// -- Test 8: Full circuit breaker cycle: open -> break -> recover --

#[tokio::test]
async fn full_circuit_breaker_cycle() {
    let mut df = test_dataflow();
    let clock = test_clock();
    let sender: NodeId = "sender".to_string().into();
    let receiver: NodeId = "receiver".to_string().into();
    let output: DataId = "output".to_string().into();
    let input: DataId = "input".to_string().into();

    // Setup: receiver has one open input with timeout
    df.open_inputs
        .entry(receiver.clone())
        .or_default()
        .insert(input.clone());
    let timeout = Duration::from_secs(5);
    df.input_deadlines.insert(
        (receiver.clone(), input.clone()),
        InputDeadline {
            timeout,
            last_received: Some(Instant::now()),
        },
    );

    // Setup mapping
    df.mappings
        .entry(OutputId(sender.clone(), output.clone()))
        .or_default()
        .insert((receiver.clone(), input.clone()));

    let running = test_running_node();
    let disable_restart = running.disable_restart.clone();
    df.running_nodes.insert(receiver.clone(), running);

    let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    df.subscribe_channels.insert(receiver.clone(), tx);

    // Step 1: Simulate timeout — insert into broken_inputs then break
    df.broken_inputs
        .insert((receiver.clone(), input.clone()), timeout);
    break_input(&mut df, &receiver, &input, &clock);
    df.input_deadlines
        .remove(&(receiver.clone(), input.clone()));

    // Verify broken state
    assert!(!df.open_inputs(&receiver).contains(&input));
    assert!(
        df.broken_inputs
            .contains_key(&(receiver.clone(), input.clone()))
    );
    assert!(!disable_restart.load(atomic::Ordering::Acquire)); // deferred

    let events = drain_events(&mut rx);
    assert_eq!(events.len(), 1);
    assert!(matches_event(&events[0], "InputClosed"));

    // Step 2: Upstream sends data again — recovery
    let metadata = metadata::Metadata::new(clock.new_timestamp());

    let output_id = OutputId(sender, output);
    let result = send_output_to_local_receivers(
        &output_id, &mut df, &metadata, None, &clock, None, false, None,
    )
    .await;
    assert!(result.is_ok());

    // Verify recovered state
    assert!(df.open_inputs(&receiver).contains(&input));
    assert!(
        !df.broken_inputs
            .contains_key(&(receiver.clone(), input.clone()))
    );
    assert!(
        df.input_deadlines
            .contains_key(&(receiver.clone(), input.clone()))
    );
    assert!(!disable_restart.load(atomic::Ordering::Acquire));

    let events = drain_events(&mut rx);
    assert_eq!(events.len(), 2);
    assert!(matches_event(&events[0], "Input"));
    assert!(matches_event(&events[1], "InputRecovered"));
}

// -- Regression test for #2021: `OutputSent` must not refresh an
//    input deadline when the receiver has fallen behind. --

/// Since #1787 data is published directly over zenoh, so the daemon's
/// `OutputSent` notification is not a delivery confirmation. A slow
/// consumer whose event channel is saturated drops the zenoh payloads,
/// so its `input_timeout` deadline must be allowed to expire instead of
/// being refreshed on every `OutputSent`.
#[test]
fn output_sent_does_not_refresh_deadline_for_backpressured_receiver() {
    let mut df = test_dataflow();
    let clock = test_clock();
    let sender: NodeId = "sender".to_string().into();
    let receiver: NodeId = "receiver".to_string().into();
    let output: DataId = "output".to_string().into();
    let input: DataId = "input".to_string().into();

    df.mappings
        .entry(OutputId(sender.clone(), output.clone()))
        .or_default()
        .insert((receiver.clone(), input.clone()));

    // Armed deadline that has already exceeded its timeout.
    let timeout = Duration::from_millis(10);
    let stale = Instant::now() - Duration::from_secs(60);
    df.input_deadlines.insert(
        (receiver.clone(), input.clone()),
        InputDeadline {
            timeout,
            last_received: Some(stale),
        },
    );

    // Saturate the receiver's channel so it has no headroom left —
    // a proxy for "the node is dropping zenoh inputs".
    let (tx, _rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    for _ in 0..NODE_EVENT_CHANNEL_CAPACITY {
        tx.try_send(Timestamped {
            inner: NodeEvent::AllInputsClosed,
            timestamp: clock.new_timestamp(),
        })
        .unwrap();
    }
    df.subscribe_channels.insert(receiver.clone(), tx);

    note_output_sent_to_local_receivers(sender, output, &mut df, &clock, None);

    // The deadline must NOT have been refreshed, so it still times out.
    let deadline = &df.input_deadlines[&(receiver.clone(), input.clone())];
    assert_eq!(
        deadline.last_received,
        Some(stale),
        "OutputSent must not refresh the deadline when the receiver is backpressured"
    );
    assert!(
        deadline.is_timed_out(),
        "input_timeout watchdog must still be able to fire for a slow consumer (#2021)"
    );
}

/// Counterpart to the above: a receiver that is keeping up (channel has
/// headroom) still has its deadline refreshed on `OutputSent`.
#[test]
fn output_sent_refreshes_deadline_when_receiver_keeps_up() {
    let mut df = test_dataflow();
    let clock = test_clock();
    let sender: NodeId = "sender".to_string().into();
    let receiver: NodeId = "receiver".to_string().into();
    let output: DataId = "output".to_string().into();
    let input: DataId = "input".to_string().into();

    df.mappings
        .entry(OutputId(sender.clone(), output.clone()))
        .or_default()
        .insert((receiver.clone(), input.clone()));

    let timeout = Duration::from_secs(5);
    let stale = Instant::now() - Duration::from_secs(60);
    df.input_deadlines.insert(
        (receiver.clone(), input.clone()),
        InputDeadline {
            timeout,
            last_received: Some(stale),
        },
    );

    // Empty channel: full headroom available.
    let (tx, _rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    df.subscribe_channels.insert(receiver.clone(), tx);

    note_output_sent_to_local_receivers(sender, output, &mut df, &clock, None);

    let deadline = &df.input_deadlines[&(receiver.clone(), input.clone())];
    assert!(
        deadline.last_received.is_some_and(|t| t > stale),
        "OutputSent must refresh the deadline when the receiver is keeping up"
    );
    assert!(!deadline.is_timed_out());
}

/// A receiver with no daemon-side channel at all (e.g. after
/// `EventStreamDropped`) is not draining inputs, so `OutputSent` must
/// not refresh its deadline either — otherwise a sender's continued
/// output would keep a disconnected receiver's deadline alive forever.
#[test]
fn output_sent_does_not_refresh_deadline_for_receiver_without_channel() {
    let mut df = test_dataflow();
    let clock = test_clock();
    let sender: NodeId = "sender".to_string().into();
    let receiver: NodeId = "receiver".to_string().into();
    let output: DataId = "output".to_string().into();
    let input: DataId = "input".to_string().into();

    df.mappings
        .entry(OutputId(sender.clone(), output.clone()))
        .or_default()
        .insert((receiver.clone(), input.clone()));

    // Armed deadline that has already exceeded its timeout.
    let timeout = Duration::from_millis(10);
    let stale = Instant::now() - Duration::from_secs(60);
    df.input_deadlines.insert(
        (receiver.clone(), input.clone()),
        InputDeadline {
            timeout,
            last_received: Some(stale),
        },
    );

    // No `subscribe_channels` entry for the receiver — it has dropped
    // its event stream.
    assert!(!df.subscribe_channels.contains_key(&receiver));

    note_output_sent_to_local_receivers(sender, output, &mut df, &clock, None);

    let deadline = &df.input_deadlines[&(receiver.clone(), input.clone())];
    assert_eq!(
        deadline.last_received,
        Some(stale),
        "OutputSent must not refresh the deadline when the receiver has no channel"
    );
    assert!(
        deadline.is_timed_out(),
        "input_timeout watchdog must still fire for a disconnected receiver (#2021)"
    );
}

// -- Test: send_with_timestamp delivers ParamUpdate to subscribed node --

#[test]
fn param_update_delivered_to_node() {
    let _df = test_dataflow();
    let clock = test_clock();
    let _node_id: NodeId = "node_a".to_string().into();

    let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);

    // Simulate sending a ParamUpdate
    let result = send_with_timestamp(
        &tx,
        NodeEvent::ParamUpdate {
            key: "threshold".into(),
            value_json: serde_json::to_vec(&serde_json::json!(42)).unwrap(),
        },
        &clock,
    );
    assert!(result.is_ok());

    let events = drain_events(&mut rx);
    assert_eq!(events.len(), 1);
    match &events[0] {
        NodeEvent::ParamUpdate { key, value_json } => {
            assert_eq!(key, "threshold");
            let value: serde_json::Value = serde_json::from_slice(value_json).unwrap();
            assert_eq!(value, serde_json::json!(42));
        }
        other => panic!("expected ParamUpdate, got {other:?}"),
    }
}

#[test]
fn param_update_fails_on_closed_channel() {
    let clock = test_clock();
    let (tx, rx) = mpsc::channel::<Timestamped<NodeEvent>>(NODE_EVENT_CHANNEL_CAPACITY);
    drop(rx); // close the receiver

    let result = send_with_timestamp(
        &tx,
        NodeEvent::ParamUpdate {
            key: "rate".into(),
            value_json: serde_json::to_vec(&serde_json::json!(10)).unwrap(),
        },
        &clock,
    );
    assert!(result.is_err());
}

#[test]
fn strict_param_update_fails_when_node_not_connected() {
    let df = test_dataflow();
    let clock = test_clock();
    let node_id: NodeId = "node_missing".to_string().into();

    let err = deliver_param_update_strict(
        &df,
        &node_id,
        "threshold".into(),
        serde_json::json!(1),
        &clock,
    )
    .expect_err("strict delivery should fail when node channel is missing");
    assert!(err.to_string().contains("not connected"));
}

#[test]
fn param_delete_delivered_to_node() {
    let clock = test_clock();
    let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);

    let result = send_with_timestamp(
        &tx,
        NodeEvent::ParamDeleted {
            key: "threshold".into(),
        },
        &clock,
    );
    assert!(result.is_ok());

    let events = drain_events(&mut rx);
    assert_eq!(events.len(), 1);
    match &events[0] {
        NodeEvent::ParamDeleted { key } => {
            assert_eq!(key, "threshold");
        }
        other => panic!("expected ParamDeleted, got {other:?}"),
    }
}

#[test]
fn param_delete_fails_on_closed_channel() {
    let clock = test_clock();
    let (tx, rx) = mpsc::channel::<Timestamped<NodeEvent>>(NODE_EVENT_CHANNEL_CAPACITY);
    drop(rx); // close the receiver

    let result = send_with_timestamp(&tx, NodeEvent::ParamDeleted { key: "rate".into() }, &clock);
    assert!(result.is_err());
}

#[test]
fn state_catch_up_stops_at_first_full_channel_and_returns_applied_prefix() {
    let clock = test_clock();
    let mut dataflow = test_dataflow();
    let node_a: NodeId = "node_a".to_string().into();
    let node_b: NodeId = "node_b".to_string().into();

    let (tx_a, mut rx_a) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    let (tx_b, _rx_b) = mpsc::channel::<Timestamped<NodeEvent>>(1);
    tx_b.try_send(Timestamped {
        inner: NodeEvent::Stop,
        timestamp: clock.new_timestamp(),
    })
    .expect("prefill node_b channel");

    dataflow.subscribe_channels.insert(node_a.clone(), tx_a);
    dataflow.subscribe_channels.insert(node_b.clone(), tx_b);

    let applied_through = apply_state_catch_up_entries(
        &dataflow,
        &[
            StateCatchUpEntry {
                sequence: 1,
                operation: StateCatchUpOperation::SetParam {
                    node_id: node_a.clone(),
                    key: "threshold".into(),
                    value: serde_json::json!(42),
                },
            },
            StateCatchUpEntry {
                sequence: 2,
                operation: StateCatchUpOperation::DeleteParam {
                    node_id: node_b,
                    key: "threshold".into(),
                },
            },
        ],
        &clock,
    );

    assert_eq!(applied_through, 1);

    let events = drain_events(&mut rx_a);
    assert_eq!(events.len(), 1);
    match &events[0] {
        NodeEvent::ParamUpdate { key, value_json } => {
            assert_eq!(key, "threshold");
            let value: serde_json::Value = serde_json::from_slice(value_json).unwrap();
            assert_eq!(value, serde_json::json!(42));
        }
        other => panic!("expected ParamUpdate, got {other:?}"),
    }
}

#[test]
fn state_catch_up_returns_zero_when_first_entry_cannot_be_delivered() {
    let clock = test_clock();
    let dataflow = test_dataflow();
    let node_id: NodeId = "node_a".to_string().into();

    let applied_through = apply_state_catch_up_entries(
        &dataflow,
        &[StateCatchUpEntry {
            sequence: 7,
            operation: StateCatchUpOperation::SetParam {
                node_id,
                key: "threshold".into(),
                value: serde_json::json!(42),
            },
        }],
        &clock,
    );

    assert_eq!(applied_through, 0);
}

#[test]
fn strict_param_delete_fails_when_channel_full() {
    let mut df = test_dataflow();
    let clock = test_clock();
    let node_id: NodeId = "node_a".to_string().into();
    let (tx, _rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    df.subscribe_channels.insert(node_id.clone(), tx.clone());

    // Saturate channel so strict delivery observes a dropped send.
    for _ in 0..NODE_EVENT_CHANNEL_CAPACITY {
        let sent = send_with_timestamp(
            &tx,
            NodeEvent::ParamDeleted {
                key: "prefill".into(),
            },
            &clock,
        )
        .unwrap();
        assert!(sent);
    }

    let err = deliver_param_delete_strict(&df, &node_id, "threshold".into(), &clock)
        .expect_err("strict delivery should fail when node channel is full");
    assert!(err.to_string().contains("channel full"));
}

#[test]
fn reconnect_window_retries_then_exits() {
    let window = Duration::from_secs(30);
    let mut deadline = None;

    // First failed reconnect: starts the window, must not exit yet. This
    // is the regression from dora-rs/dora#1998 where a fast TCP refusal
    // (coordinator briefly restarting) exited the daemon immediately.
    let t0 = Instant::now();
    assert!(!reconnect_window_elapsed(t0, &mut deadline, window));
    assert_eq!(deadline, Some(t0 + window));

    // Subsequent failures inside the window keep retrying without
    // resetting the deadline.
    assert!(!reconnect_window_elapsed(
        t0 + Duration::from_secs(5),
        &mut deadline,
        window
    ));
    assert!(!reconnect_window_elapsed(
        t0 + Duration::from_secs(29),
        &mut deadline,
        window
    ));
    assert_eq!(deadline, Some(t0 + window));

    // Once the window has elapsed, the daemon gives up and exits so a
    // permanently-gone coordinator doesn't leave an orphan (#1996).
    assert!(reconnect_window_elapsed(t0 + window, &mut deadline, window));
    assert!(reconnect_window_elapsed(
        t0 + Duration::from_secs(31),
        &mut deadline,
        window
    ));
}

#[test]
fn reconnect_window_resets_after_successful_connect() {
    let window = Duration::from_secs(30);
    let mut deadline = None;

    let t0 = Instant::now();
    assert!(!reconnect_window_elapsed(t0, &mut deadline, window));

    // A successful reconnect clears the deadline (the loop sets
    // `reconnect_deadline = None`), so a later outage gets a full fresh
    // window rather than inheriting the old, possibly-elapsed deadline.
    deadline = None;
    let t1 = t0 + Duration::from_secs(100);
    assert!(!reconnect_window_elapsed(t1, &mut deadline, window));
    assert_eq!(deadline, Some(t1 + window));
    assert!(!reconnect_window_elapsed(
        t1 + Duration::from_secs(29),
        &mut deadline,
        window
    ));
}

// -- dora#3177: extension entries of removed/replaced nodes --

fn ext_key(key: &str) -> ExtensionKey {
    ExtensionKey {
        dataflow_id: Uuid::nil().to_string(),
        namespace: "dora-tensor-pool".into(),
        key: key.into(),
    }
}

/// The daemon-side half of reclamation, which `extension_table.rs`
/// cannot cover: the drop notification actually reaches the reader's
/// subscribe channel, so it can release what it derived from the value.
/// Every reclaim call site — `RemoveNode`, `ReplaceNode`, and both exit
/// paths — goes through this.
#[test]
fn reclaim_drops_owned_entries_and_notifies_readers() {
    let mut df = test_dataflow();
    let clock = test_clock();
    let owner: NodeId = "owner".to_string().into();
    let reader: NodeId = "reader".to_string().into();

    let mut extensions = ExtensionTable::new();
    extensions
        .store(ext_key("pool_owner_1"), b"descriptor".to_vec(), &owner)
        .unwrap();
    extensions.load(&ext_key("pool_owner_1"), &reader).unwrap();

    let (tx, mut rx) = mpsc::channel(NODE_EVENT_CHANNEL_CAPACITY);
    df.subscribe_channels.insert(reader.clone(), tx);

    reclaim_extensions_of_exited_node(&mut extensions, Some(&df), Uuid::nil(), &owner, &clock);

    assert_eq!(extensions.len(), 0, "the owner's entry must be gone");
    let events = drain_events(&mut rx);
    assert!(
        matches!(
            events.as_slice(),
            [NodeEvent::ExtensionDropped { namespace, key }]
                if namespace == "dora-tensor-pool" && key == "pool_owner_1"
        ),
        "reader must be notified exactly once: {events:?}"
    );
}
