//! Background metrics sampling for `dora top`: per-node process stats plus
//! per-dataflow network and disk-I/O rates, collected off the event loop.

use crate::{Daemon, coordinator};
use dora_core::{config::NodeId, uhlc};
use dora_message::{
    DataflowId,
    common::DaemonId,
    daemon_to_coordinator::{CoordinatorRequest, DaemonEvent},
    node_to_daemon::Timestamped,
};
use std::{
    collections::{BTreeMap, HashMap},
    sync::{
        Arc,
        atomic::{self, AtomicU32, AtomicU64},
    },
    time::{Duration, Instant},
};

pub(crate) const METRICS_INTERVAL: Duration = Duration::from_secs(2);
/// Shortest sampling window (seconds) we trust for a disk-I/O rate. A window
/// below this either has no prior baseline (first sample) or is so short that
/// dividing by it would blow the rate up into a meaningless spike, so we report
/// no rate for that sample instead.
pub(crate) const MIN_METRICS_WINDOW_SECS: f64 = 0.1;

/// Snapshot of a single node for background metrics collection.
pub(crate) struct NodeSnapshot {
    node_id: NodeId,
    pid: Option<Arc<AtomicU32>>,
    restart_count: Arc<AtomicU32>,
    restarts_disabled: bool,
    broken_inputs: Vec<String>,
    pending_messages: u64,
}

/// Snapshot of a running dataflow for background metrics collection.
pub(crate) struct DataflowMetricsSnapshot {
    dataflow_id: DataflowId,
    nodes: Vec<NodeSnapshot>,
    net_bytes_sent: Arc<AtomicU64>,
    net_bytes_received: Arc<AtomicU64>,
    net_messages_sent: Arc<AtomicU64>,
    net_messages_received: Arc<AtomicU64>,
    net_publish_failures: Arc<AtomicU64>,
}

/// Everything the metrics sampler carries from one refresh to the next: the
/// sysinfo `System` and the record of what its per-process I/O counters are
/// measured against.
///
/// These belong together. `Process::disk_usage()` reports bytes since the
/// *previous refresh of that process*, so a delta only becomes a rate alongside
/// the `System` that produced it (`last_refresh` gives the divisor) and only
/// counts as a delta for processes that `System` had already seen
/// (`processes`). A process observed for the first time — a node in a second
/// dataflow on a long-lived daemon, a restart with a fresh PID, a freshly
/// spawned descendant — reports everything it has read or written since it
/// started, which divided by a two-second window is a large one-sample spike.
///
/// Holding all three in one mutex is what keeps that pairing honest: a
/// collection takes the whole state at once, so a concurrent collection starts
/// from `Default` — an empty `System` *and* an empty baseline — and reports no
/// rate, rather than dividing one collection's total-since-start bytes by the
/// other's plausible-looking window.
#[derive(Default)]
pub(crate) struct MetricsState {
    /// The sysinfo handle whose per-process counters the deltas come from.
    system: sysinfo::System,
    /// Wall-clock instant of the refresh that produced `system`. `None` when
    /// there is no baseline: before the first refresh, and after a panicked
    /// refresh resets the state.
    last_refresh: Option<Instant>,
    /// `pid -> start_time` for every process `system` observed. The start time
    /// distinguishes a long-lived process from a recycled PID, which sysinfo
    /// treats as a new process with no baseline.
    processes: HashMap<sysinfo::Pid, u64>,
}

/// Record which processes the just-refreshed `System` observed, to serve as the
/// baseline for the next cycle.
pub(crate) fn snapshot_processes(sys: &sysinfo::System) -> HashMap<sysinfo::Pid, u64> {
    sys.processes()
        .iter()
        .map(|(pid, process)| (*pid, process.start_time()))
        .collect()
}

/// Whether `pid` was already observed in the previous refresh, so its
/// `disk_usage()` covers only the sampling window (see [`MetricsState`]). A
/// `start_time` mismatch means the PID was recycled: a different process.
pub(crate) fn has_disk_baseline(
    previous: &HashMap<sysinfo::Pid, u64>,
    pid: sysinfo::Pid,
    start_time: u64,
) -> bool {
    previous.get(&pid) == Some(&start_time)
}

/// Running total of one node's disk I/O across its process tree.
///
/// Only processes that already had a baseline may be summed (see
/// [`MetricsState`]); a newcomer would contribute total-since-start bytes and
/// spike the rate. Leaving one out under-reports that single window —
/// it is counted from the next refresh on — which beats both the spike and
/// blanking the row for a node that spawns a short-lived child every tick. When
/// nothing could be summed at all (the node's own first observation) there is no
/// measurement, and [`DiskDelta::window`] reports none.
#[derive(Default)]
pub(crate) struct DiskDelta {
    read: u64,
    written: u64,
    /// Whether any process contributed, i.e. whether these totals mean anything.
    any_baseline: bool,
}

impl DiskDelta {
    /// Add `process`'s bytes, if it was already observed in the previous refresh.
    fn add(
        &mut self,
        previous: &HashMap<sysinfo::Pid, u64>,
        pid: sysinfo::Pid,
        process: &sysinfo::Process,
    ) {
        if !has_disk_baseline(previous, pid, process.start_time()) {
            return;
        }
        let usage = process.disk_usage();
        self.read += usage.read_bytes;
        self.written += usage.written_bytes;
        self.any_baseline = true;
    }

    /// The window these bytes were measured over, or `None` if nothing was.
    fn window(&self, refresh_window: Option<Duration>) -> Option<Duration> {
        if self.any_baseline {
            refresh_window
        } else {
            None
        }
    }
}

/// Convert a byte delta observed over `window` into a bytes-per-second rate.
///
/// `window` is the wall-clock time since the previous successful refresh, or
/// `None` when the delta has no baseline to be a delta against (see
/// [`MetricsState`]). Returns `None` in that case, and also when the window
/// is so short that dividing by it would explode into a spurious spike.
pub(crate) fn disk_rate_bytes_per_sec(bytes: u64, window: Option<Duration>) -> Option<u64> {
    let secs = window?.as_secs_f64();
    if secs < MIN_METRICS_WINDOW_SECS {
        return None;
    }
    Some((bytes as f64 / secs) as u64)
}

/// Process attributes `dora top` samples for each node.
///
/// `without_tasks` keeps a node's threads out of the process list. On Linux,
/// sysinfo reports every thread as its own process parented to the node's pid
/// while `tasks` is enabled, and `ProcessRefreshKind::nothing()` leaves it
/// enabled — it resets every other attribute. The descendant walk would then
/// sum each thread's `memory()`, which reports the whole process's resident
/// set, so a node's reported memory grew with its thread count instead of its
/// RSS (#3546).
fn metrics_refresh_kind() -> sysinfo::ProcessRefreshKind {
    sysinfo::ProcessRefreshKind::nothing()
        .with_cpu()
        .with_memory()
        .with_disk_usage()
        .without_tasks()
}

/// Parent -> children edges for every process in `sys`.
fn child_process_map(sys: &sysinfo::System) -> HashMap<sysinfo::Pid, Vec<sysinfo::Pid>> {
    let mut children_map: HashMap<sysinfo::Pid, Vec<sysinfo::Pid>> = HashMap::new();
    for (pid, process) in sys.processes() {
        if let Some(parent) = process.parent() {
            children_map.entry(parent).or_default().push(*pid);
        }
    }
    children_map
}

/// CPU and memory of `pid` plus every descendant in `children_map`, together
/// with the disk deltas of the same set. Returns `None` when `sys` holds no
/// entry for `pid`.
///
/// CPU and memory are instantaneous readings, so every process contributes;
/// disk I/O is a delta, so [`DiskDelta`] decides which ones may.
fn aggregate_process_tree(
    sys: &sysinfo::System,
    children_map: &HashMap<sysinfo::Pid, Vec<sysinfo::Pid>>,
    previous_processes: &HashMap<sysinfo::Pid, u64>,
    pid: sysinfo::Pid,
) -> Option<(f32, u64, DiskDelta)> {
    let process = sys.process(pid)?;
    let mut cpu_usage = process.cpu_usage();
    let mut memory_bytes = process.memory();
    let mut disk = DiskDelta::default();
    disk.add(previous_processes, pid, process);

    // Recursively aggregate all descendants.
    let mut stack = vec![pid];
    while let Some(parent) = stack.pop() {
        if let Some(kids) = children_map.get(&parent) {
            for &child_pid in kids {
                if let Some(child) = sys.processes().get(&child_pid) {
                    cpu_usage += child.cpu_usage();
                    memory_bytes += child.memory();
                    disk.add(previous_processes, child_pid, child);
                }
                stack.push(child_pid);
            }
        }
    }

    Some((cpu_usage, memory_bytes, disk))
}

/// Collect and send metrics in the background. Errors are returned to the
/// caller (the spawned task logs them).
pub(crate) async fn collect_and_send_metrics_bg(
    dataflows: Vec<DataflowMetricsSnapshot>,
    metrics_state: Arc<std::sync::Mutex<MetricsState>>,
    sender: coordinator::CoordinatorSender,
    daemon_id: DaemonId,
    clock: Arc<uhlc::HLC>,
) -> eyre::Result<()> {
    use dora_message::daemon_to_coordinator::NodeMetrics;
    use sysinfo::{Pid, ProcessesToUpdate};

    let has_any_running = dataflows
        .iter()
        .any(|df| df.nodes.iter().any(|n| n.pid.is_some()));

    // Refresh sysinfo on a blocking thread if any nodes are running. `try_lock`
    // skips the cycle when a previous collection is still in progress.
    //
    // Taking the whole `MetricsState` (see its docs) keeps each delta together
    // with the window and the observed-process set that make it meaningful.
    // Dividing by that measured window rather than a constant also fixes a
    // catch-up cycle, which spans ~2 intervals but was divided by one.
    //
    // A cycle with no running nodes deliberately leaves the state untouched
    // rather than clearing it: nothing refreshed, so the stored baseline still
    // describes the stored `System`, and the next refresh's delta and window
    // both simply span the longer gap.
    let (refreshed_state, refresh_window, previous_processes) = if has_any_running {
        let MetricsState {
            system,
            last_refresh,
            processes: previous_processes,
        } = match metrics_state.try_lock() {
            Ok(mut guard) => std::mem::take(&mut *guard),
            Err(_) => {
                tracing::debug!("metrics: skipping, previous collection still running");
                return Ok(());
            }
        };
        let refresh_kind = metrics_refresh_kind();
        match tokio::task::spawn_blocking(move || {
            let mut system = system;
            system.refresh_processes_specifics(ProcessesToUpdate::All, true, refresh_kind);
            // Snapshot here rather than after the `await`: it is derived purely
            // from `system`, so it belongs on the blocking thread with the
            // refresh instead of on a reactor thread.
            let processes = snapshot_processes(&system);
            (system, processes)
        })
        .await
        {
            Ok((system, processes)) => {
                let now = Instant::now();
                let window = last_refresh.map(|prev| now.saturating_duration_since(prev));
                (
                    Some(MetricsState {
                        system,
                        last_refresh: Some(now),
                        processes,
                    }),
                    window,
                    previous_processes,
                )
            }
            Err(e) => {
                tracing::error!("sysinfo refresh panicked: {e}");
                // The `System` and every baseline it held are gone. The `take`
                // above already left an empty state behind, so the next refresh
                // starts over as a first sample; report nothing this cycle.
                (None, None, HashMap::new())
            }
        }
    } else {
        (None, None, HashMap::new())
    };

    for df in &dataflows {
        let mut metrics = BTreeMap::new();

        if let Some(state) = &refreshed_state {
            let sys = &state.system;
            // Pre-build parent->children map once per refresh.
            let children_map = child_process_map(sys);

            for node in &df.nodes {
                if let Some(pid_arc) = node.pid.as_ref() {
                    let pid = pid_arc.load(atomic::Ordering::Acquire);
                    let sys_pid = Pid::from_u32(pid);
                    if let Some((cpu_usage, memory_bytes, disk)) =
                        aggregate_process_tree(sys, &children_map, &previous_processes, sys_pid)
                    {
                        let disk_window = disk.window(refresh_window);

                        let restart_count = node.restart_count.load(atomic::Ordering::Acquire);
                        let status = if !node.broken_inputs.is_empty() {
                            dora_message::daemon_to_coordinator::NodeStatus::Degraded
                        } else {
                            dora_message::daemon_to_coordinator::NodeStatus::Running
                        };

                        metrics.insert(
                            node.node_id.clone(),
                            NodeMetrics {
                                pid,
                                cpu_usage,
                                memory_bytes,
                                disk_read_bytes: disk_rate_bytes_per_sec(disk.read, disk_window),
                                disk_write_bytes: disk_rate_bytes_per_sec(
                                    disk.written,
                                    disk_window,
                                ),
                                restart_count,
                                broken_inputs: node.broken_inputs.clone(),
                                status,
                                pending_messages: node.pending_messages,
                            },
                        );
                    }
                }
            }
        }

        // Second pass: report nodes without a running process.
        for node in &df.nodes {
            if !metrics.contains_key(&node.node_id) {
                let restart_count = node.restart_count.load(atomic::Ordering::Acquire);
                let status = if node.restarts_disabled {
                    dora_message::daemon_to_coordinator::NodeStatus::Failed
                } else if restart_count > 0 {
                    dora_message::daemon_to_coordinator::NodeStatus::Restarting
                } else {
                    continue;
                };
                metrics.insert(
                    node.node_id.clone(),
                    NodeMetrics {
                        pid: 0,
                        cpu_usage: 0.0,
                        memory_bytes: 0,
                        disk_read_bytes: None,
                        disk_write_bytes: None,
                        restart_count,
                        broken_inputs: Vec::new(),
                        status,
                        pending_messages: node.pending_messages,
                    },
                );
            }
        }

        if !metrics.is_empty() {
            let network = {
                let bs = df.net_bytes_sent.load(atomic::Ordering::Relaxed);
                let br = df.net_bytes_received.load(atomic::Ordering::Relaxed);
                let ms = df.net_messages_sent.load(atomic::Ordering::Relaxed);
                let mr = df.net_messages_received.load(atomic::Ordering::Relaxed);
                let pf = df.net_publish_failures.load(atomic::Ordering::Relaxed);
                if bs > 0 || br > 0 || ms > 0 || mr > 0 || pf > 0 {
                    Some(dora_message::daemon_to_coordinator::NetworkMetrics {
                        bytes_sent: bs,
                        bytes_received: br,
                        messages_sent: ms,
                        messages_received: mr,
                        publish_failures: pf,
                    })
                } else {
                    None
                }
            };
            let msg = match serde_json::to_vec(&Timestamped {
                inner: CoordinatorRequest::Event {
                    daemon_id: daemon_id.clone(),
                    event: DaemonEvent::NodeMetrics {
                        dataflow_id: df.dataflow_id,
                        metrics,
                        network,
                    },
                },
                timestamp: clock.new_timestamp(),
            }) {
                Ok(msg) => msg,
                // Skip this dataflow's batch rather than `?`-returning: an early
                // return here would bypass the `System` restore below, leaving
                // the shared metrics `System` (moved out via `mem::take`) empty
                // until the next successful collection repopulates it.
                // Serialization of this structure is effectively infallible, so
                // this is defense-in-depth — `continue` just keeps the cleanup
                // path unconditional against any future error here. Matches the
                // `send_event` failure handling just below.
                Err(e) => {
                    tracing::warn!("failed to serialize metrics for dataflow: {e}");
                    continue;
                }
            };
            if let Err(e) = sender.send_event(&msg).await {
                tracing::warn!("failed to send metrics for dataflow: {e}");
                continue;
            }
        }
    }

    // Return the refreshed System and its matching baseline to the shared mutex
    // for the next cycle.
    if let Some(state) = refreshed_state
        && let Ok(mut guard) = metrics_state.lock()
    {
        *guard = state;
    }

    Ok(())
}

impl Daemon {
    /// Snapshot running dataflow state and spawn metrics collection as a
    /// background task so it never blocks the event loop.
    pub(crate) fn spawn_metrics_collection(&self) {
        let sender = match &self.coordinator_sender {
            Some(s) => s.clone(),
            None => return,
        };

        // Snapshot per-dataflow data needed for metrics.
        let dataflow_snapshots: Vec<DataflowMetricsSnapshot> = self
            .running
            .iter()
            .map(|(id, df)| DataflowMetricsSnapshot {
                dataflow_id: *id,
                nodes: df
                    .running_nodes
                    .iter()
                    .map(|(nid, rn)| NodeSnapshot {
                        node_id: nid.clone(),
                        pid: rn.pid.clone(),
                        restart_count: rn.restart_count.clone(),
                        restarts_disabled: rn.restarts_disabled(),
                        broken_inputs: df
                            .broken_inputs
                            .keys()
                            .filter(|(n, _)| n == nid)
                            .map(|(_, d)| d.to_string())
                            .collect(),
                        pending_messages: df
                            .pending_messages
                            .get(nid)
                            .map(|c| c.load(atomic::Ordering::Relaxed))
                            .unwrap_or(0),
                    })
                    .collect(),
                net_bytes_sent: df.net_bytes_sent.clone(),
                net_bytes_received: df.net_bytes_received.clone(),
                net_messages_sent: df.net_messages_sent.clone(),
                net_messages_received: df.net_messages_received.clone(),
                net_publish_failures: df.net_publish_failures.clone(),
            })
            .collect();

        let metrics_state = self.metrics_state.clone();
        let daemon_id = self.daemon_id.clone();
        let clock = self.clock.clone();

        tokio::spawn(async move {
            if let Err(e) = collect_and_send_metrics_bg(
                dataflow_snapshots,
                metrics_state,
                sender,
                daemon_id,
                clock,
            )
            .await
            {
                tracing::warn!("metrics collection failed: {e}");
            }
        });
    }
}

#[cfg(test)]
mod disk_rate_tests {
    use super::*;

    #[test]
    fn no_baseline_reports_no_rate() {
        // First sample: no prior refresh, so no measurement window.
        assert_eq!(disk_rate_bytes_per_sec(10_000_000, None), None);
    }

    #[test]
    fn near_zero_window_reports_no_rate() {
        // A window below the floor would explode into a spurious spike.
        assert_eq!(
            disk_rate_bytes_per_sec(10_000_000, Some(Duration::from_millis(1))),
            None
        );
    }

    #[test]
    fn divides_by_the_actual_window_not_a_constant() {
        // 8 MB observed over a 4 s window (e.g. one skipped cycle) is 2 MB/s —
        // the old constant-2s divisor would have reported 4 MB/s.
        assert_eq!(
            disk_rate_bytes_per_sec(8_000_000, Some(Duration::from_secs(4))),
            Some(2_000_000)
        );
        // The steady-state 2 s window is unchanged.
        assert_eq!(
            disk_rate_bytes_per_sec(4_000_000, Some(Duration::from_secs(2))),
            Some(2_000_000)
        );
    }

    #[test]
    fn only_processes_seen_last_refresh_have_a_baseline() {
        let previous = HashMap::from([(sysinfo::Pid::from_u32(10), 1_700_000_000)]);

        // Same PID, same start time: `disk_usage()` is a real per-window delta.
        assert!(has_disk_baseline(
            &previous,
            sysinfo::Pid::from_u32(10),
            1_700_000_000
        ));
        // Never seen before — a node from a later dataflow, or a freshly spawned
        // descendant: `disk_usage()` is total-since-start.
        assert!(!has_disk_baseline(
            &previous,
            sysinfo::Pid::from_u32(11),
            1_700_000_000
        ));
        // Same PID, later start time: the PID was recycled, so no baseline.
        assert!(!has_disk_baseline(
            &previous,
            sysinfo::Pid::from_u32(10),
            1_700_000_500
        ));
    }

    #[test]
    fn a_node_with_nothing_summed_has_no_window() {
        let refresh_window = Some(Duration::from_secs(2));

        // Nothing could be summed — the node's own first observation. A valid
        // refresh window must not turn total-since-start bytes into a rate.
        assert_eq!(DiskDelta::default().window(refresh_window), None);

        // Something was summed, so the refresh window is the one it spans, even
        // if a newcomer's bytes were left out of it.
        let summed = DiskDelta {
            any_baseline: true,
            ..Default::default()
        };
        assert_eq!(summed.window(refresh_window), refresh_window);
    }

    #[test]
    fn a_refresh_records_a_baseline_a_previous_one_did_not_have() {
        use sysinfo::{ProcessRefreshKind, ProcessesToUpdate};

        // What `snapshot_processes` writes must be what `has_disk_baseline`
        // accepts, for a real process out of a real `System`.
        let pid = sysinfo::Pid::from_u32(std::process::id());
        let mut system = sysinfo::System::new();
        system.refresh_processes_specifics(
            ProcessesToUpdate::Some(&[pid]),
            true,
            ProcessRefreshKind::nothing().with_disk_usage(),
        );

        let start_time = system
            .process(pid)
            .expect("sysinfo sees the current process")
            .start_time();
        let snapshot = snapshot_processes(&system);

        // This refresh observed us for the first time, so the state it replaced
        // — an empty one, as after a panicked refresh — holds no baseline: the
        // `disk_usage()` it just reported is total-since-start, not a delta.
        assert!(!has_disk_baseline(&HashMap::new(), pid, start_time));
        // Having now been recorded, the *next* refresh's delta is a real one.
        assert!(has_disk_baseline(&snapshot, pid, start_time));
    }
}

/// Regression tests for #3546: on Linux, sysinfo reports every thread of a
/// process as its own process parented to it unless the refresh says
/// otherwise, so the descendant walk counted a node's RSS once per thread.
#[cfg(test)]
#[cfg(target_os = "linux")]
mod process_tree_tests {
    use super::*;
    use std::{
        process::{Command, Stdio},
        sync::{
            Arc,
            atomic::{AtomicBool, Ordering as AtomicOrdering},
        },
        thread,
        time::{Duration, Instant},
    };

    /// The Linux thread ids of the current process, from `/proc/self/task`.
    fn own_thread_ids() -> Vec<sysinfo::Pid> {
        std::fs::read_dir("/proc/self/task")
            .expect("read /proc/self/task")
            .map(|entry| {
                let tid = entry
                    .expect("read a thread entry")
                    .file_name()
                    .to_string_lossy()
                    .parse()
                    .expect("thread id is numeric");
                sysinfo::Pid::from_u32(tid)
            })
            .collect()
    }

    fn refresh_all() -> sysinfo::System {
        let mut system = sysinfo::System::new();
        system.refresh_processes_specifics(
            sysinfo::ProcessesToUpdate::All,
            true,
            metrics_refresh_kind(),
        );
        system
    }

    /// Run `body` while `count` extra threads of this process spin. Spawning
    /// them explicitly keeps the test independent of the harness's own thread
    /// count — `--test-threads=1` runs tests on the main thread alone — and
    /// gives the process a non-zero CPU delta between two refreshes.
    fn with_busy_threads<F: FnOnce()>(count: usize, body: F) {
        let stop = Arc::new(AtomicBool::new(false));
        let handles: Vec<_> = (0..count)
            .map(|_| {
                let stop = stop.clone();
                thread::spawn(move || {
                    let mut spin = 0u64;
                    while !stop.load(AtomicOrdering::Relaxed) {
                        spin = std::hint::black_box(spin.wrapping_add(1));
                    }
                })
            })
            .collect();

        // Wait for the threads to exist before refreshing, so the refresh
        // cannot race their creation.
        let deadline = Instant::now() + Duration::from_secs(10);
        while own_thread_ids().len() <= count {
            assert!(Instant::now() < deadline, "the extra threads did not start");
            thread::sleep(Duration::from_millis(5));
        }

        body();

        stop.store(true, AtomicOrdering::Relaxed);
        for handle in handles {
            handle.join().expect("busy thread panicked");
        }
    }

    #[test]
    fn a_node_is_counted_once_however_many_threads_it_runs() {
        with_busy_threads(3, || {
            let own_pid = sysinfo::Pid::from_u32(std::process::id());
            let thread_ids = own_thread_ids();
            assert!(thread_ids.len() > 1, "expected a multi-threaded process");

            // Two refreshes: `cpu_usage()` is a delta, so the first one has none.
            let mut system = refresh_all();
            thread::sleep(sysinfo::MINIMUM_CPU_UPDATE_INTERVAL);
            system.refresh_processes_specifics(
                sysinfo::ProcessesToUpdate::All,
                true,
                metrics_refresh_kind(),
            );

            // The node itself is still sampled: excluding threads must not take
            // CPU or memory with it.
            let process = system.process(own_pid).expect("the node is refreshed");
            assert!(process.memory() > 0, "the node's memory is still reported");
            assert!(
                process.cpu_usage() > 0.0,
                "the node's CPU is still reported"
            );

            // Every thread is a task of this process, not a process of its own,
            // so the descendant walk cannot add the node's RSS once per thread.
            let children = child_process_map(&system);
            let kids = children.get(&own_pid).cloned().unwrap_or_default();
            for tid in &thread_ids {
                assert!(
                    !kids.contains(tid),
                    "thread {tid:?} of {own_pid:?} is reported as its own process"
                );
            }
        });
    }

    #[test]
    fn a_single_threaded_node_is_aggregated_once() {
        let mut child = Command::new("sleep")
            .arg("300")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .expect("failed to spawn sleep");
        let pid = sysinfo::Pid::from_u32(child.id());

        // `sleep` is single-threaded and spawns nothing, so its process tree is
        // exactly one process and the aggregate is that process's own RSS.
        let deadline = Instant::now() + Duration::from_secs(10);
        let mut system = sysinfo::System::new();
        let measured = loop {
            system.refresh_processes_specifics(
                sysinfo::ProcessesToUpdate::All,
                true,
                metrics_refresh_kind(),
            );
            let children = child_process_map(&system);
            if let Some((cpu_usage, memory_bytes, _disk)) =
                aggregate_process_tree(&system, &children, &HashMap::new(), pid)
            {
                let own = system.process(pid).expect("the node is refreshed").memory();
                break (cpu_usage, memory_bytes, own);
            }
            assert!(
                Instant::now() < deadline,
                "sysinfo never saw the sleep process"
            );
            thread::sleep(Duration::from_millis(10));
        };

        let _ = child.kill();
        let _ = child.wait();

        let (_cpu_usage, memory_bytes, own_memory) = measured;
        assert!(own_memory > 0, "the node's memory is reported");
        assert_eq!(
            memory_bytes, own_memory,
            "a single-threaded node must be counted exactly once"
        );
    }
}
