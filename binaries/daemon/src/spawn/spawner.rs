use crate::{
    CoreNodeKindExt, Event,
    log::NodeLogger,
    node_communication::spawn_listener_loop,
    spawn::{command::path_spawn_command, prepared::PreparedNode},
};
use clonable_command::{Command, Stdio};
use crossbeam::queue::ArrayQueue;
use dora_core::{
    build::{managed_python_bin_dir, managed_python_interpreter},
    config::{Input, InputMapping, NodeId},
    descriptor::{
        CoreNodeKind, Descriptor, OperatorDefinition, OperatorSource, PythonSource, ResolvedNode,
    },
    get_python_path,
    topics::{DORA_ZENOH_CONNECT_ENV, DORA_ZENOH_LISTEN_ENV},
    uhlc::HLC,
};
use dora_message::{
    DataflowId,
    common::LogLevel,
    daemon_to_coordinator::Timestamped,
    daemon_to_node::{NodeConfig, RuntimeConfig},
    descriptor::EnvValue,
};
use eyre::{ContextCompat, WrapErr, bail};
use std::{
    collections::{BTreeMap, BTreeSet},
    ffi::OsString,
    future::Future,
    path::{Path, PathBuf},
    sync::{Arc, atomic::AtomicU64},
};
use tokio::sync::mpsc;

/// Environment variable names that must never be passed to spawned nodes.
const ENV_DENYLIST: &[&str] = &[
    "LD_PRELOAD",
    "LD_AUDIT",
    "DYLD_INSERT_LIBRARIES",
    "DYLD_LIBRARY_PATH",
    "LD_LIBRARY_PATH",
    "DORA_AUTH_TOKEN",
    "DORA_ALLOW_SHELL_NODES",
];

/// Returns true if the env var key is denied, logging a warning if so.
fn is_denied_env(key: &str) -> bool {
    if ENV_DENYLIST.contains(&key) {
        tracing::warn!(
            "skipping denied environment variable '{key}' (security: could inject shared libraries)"
        );
        true
    } else {
        false
    }
}

/// Strip all denied env vars from the inherited process environment.
/// This prevents the daemon's own env (e.g. `DORA_AUTH_TOKEN`) from leaking
/// to child nodes via `/proc/<pid>/environ`.
fn strip_denied_env(mut command: Command) -> Command {
    for key in ENV_DENYLIST {
        command = command.env_remove(key);
    }
    command
}

/// Point a spawned process at the managed Python env.
///
/// Sets `VIRTUAL_ENV` and prepends the env's `bin/` (or `Scripts/` on
/// Windows) to `PATH`. Without this, subprocesses, console scripts, and
/// `python -m pip` launched from inside the node still resolve from the
/// ambient environment — so the runtime is not actually hermetic even
/// though the top-level interpreter is the managed one.
///
/// The composed PATH puts the managed bin dir first, then the user-defined
/// `PATH` from `node_env` (if any), then the daemon's ambient `PATH`. This
/// preserves any custom PATH the node author set while still giving the
/// managed env priority for `python`, `pip`, and friends.
fn apply_managed_python_runtime_env(
    command: Command,
    python_env_dir: &Path,
    node_env: Option<&BTreeMap<String, EnvValue>>,
) -> eyre::Result<Command> {
    let bin_dir = managed_python_bin_dir(python_env_dir);

    let base_path = node_env
        .and_then(|envs| envs.get("PATH"))
        .map(|value| OsString::from(value.to_string()))
        .or_else(|| std::env::var_os("PATH"));

    let mut paths = vec![bin_dir];
    if let Some(base) = base_path {
        paths.extend(std::env::split_paths(&base));
    }

    let new_path = std::env::join_paths(paths)
        .wrap_err("failed to compose managed Python PATH for runtime spawn")?;

    Ok(command
        .env("VIRTUAL_ENV", python_env_dir)
        .env("PATH", new_path))
}

/// Where a single node's zenoh session should listen, and which peers it must
/// dial. See [`plan_zenoh_peering`].
#[derive(Clone, Debug)]
pub struct NodeZenohPeering {
    /// Loopback endpoint this node listens on, so its consumers can dial it.
    pub listen: String,
    /// Endpoints this node dials: the daemon, plus each node it consumes from.
    pub connect: Vec<String>,
}

/// Assign every node a loopback listener and dial-list so the links the dataflow
/// needs are established *by construction*.
///
/// Zenoh 1.9's `peer` hat hardcodes `full_linkstate: false` (its release notes
/// list "Disable `full_linkstate` in `peer::Hat::Network`" under Bug fixes),
/// where 1.8's `linkstate_peer` hat still honored `routing.peer.mode`. So peers
/// no longer relay for each other: a producer/consumer pair that never forms a
/// direct link cannot exchange data at all — there is no relay to fall back on
/// and no amount of waiting helps. Leaving those links to gossip's best-effort
/// autoconnect made them racy, measuring 5 failures in 20 runs of
/// `examples/rust-dataflow`; with this planning it is 0 in 20.
///
/// The nodes this daemon spawns are all on this machine, hence all
/// loopback-addressable, so we can just say who dials whom. Each node dials only
/// the nodes it *consumes from*: a zenoh transport is bidirectional, so the
/// consumer's dial is what carries the producer's data back. That is `|edges|`
/// links rather than `N^2`, and it is
/// exactly the set the dataflow requires.
///
/// Dynamic nodes are omitted: they are not in the descriptor's spawn set and join
/// at arbitrary times, so they keep the daemon-endpoint-only behavior.
///
/// # Caveat: per-node endpoints are advertised before they bind (#2762)
///
/// This reserves each local node's loopback port with
/// `dora_core::topics::reserve_loopback_zenoh_endpoint`
/// — which binds `127.0.0.1:0`, reads the port, and drops the socket — and commits
/// that port into every *consumer's* dial list here at plan time, i.e. **before the
/// producing node process has bound it**. Unlike the daemon's own listener, these
/// per-node endpoints are therefore *not* covered by the bind-verification in
/// `open_zenoh_session_with_listen` (#1858): the node opens with
/// `listen/exit_on_failure: false` and discards its own `effective_listen_endpoint`,
/// so a lost reserve→bind race (another process grabs the port in the window) leaves
/// the producer listener-less while its consumers hold a dead endpoint. Because those
/// consumers also have explicit `connect/endpoints`, multicast scouting is disabled
/// for them, so there is no fallback and that edge is silently partitioned. The
/// probability is low (the OS keeps handing out fresh ephemeral ports) but the impact
/// is silent data loss; closing the window fully requires holding each reservation
/// until the child binds, or verifying the per-node bind and re-planning on failure.
///
/// Only *local* nodes are planned. `nodes` spans the whole dataflow including
/// nodes on other daemons, whose endpoints are not on this host's loopback, so a
/// local consumer of a remote producer gets no dial for it and keeps the existing
/// cross-machine behavior. Multi-machine/NAT deployments supply their own zenoh
/// config via `ZENOH_CONFIG_PATH`, which bypasses this path entirely and can put
/// a real router (which *does* still relay) in between.
pub fn plan_zenoh_peering(
    nodes: &BTreeMap<NodeId, ResolvedNode>,
    local_nodes: &BTreeSet<NodeId>,
    daemon_endpoint: Option<&str>,
) -> BTreeMap<NodeId, NodeZenohPeering> {
    // Reserve a listener per node first, so the dial-lists below can reference
    // every node regardless of spawn order.
    //
    // Only nodes this daemon spawns get one: the endpoints are *loopback*, so
    // handing a local consumer the "endpoint" of a node running on another
    // machine would point it at this host's 127.0.0.1 — either a failed dial or,
    // worse, an unrelated local process. A local consumer of a remote producer
    // therefore gets no dial for it and keeps the existing cross-machine
    // behavior (see the `ZENOH_CONFIG_PATH` note above).
    let mut listeners: BTreeMap<NodeId, String> = BTreeMap::new();
    for (node_id, node) in nodes {
        if node.kind.dynamic() || !local_nodes.contains(node_id) {
            continue;
        }
        match dora_core::topics::reserve_loopback_zenoh_endpoint() {
            Ok(ep) => {
                listeners.insert(node_id.clone(), ep);
            }
            Err(err) => {
                // Fall back to gossip for this node rather than failing the
                // dataflow; it just loses the determinism guarantee.
                tracing::warn!(
                    node = %node_id,
                    "failed to reserve a zenoh listen endpoint ({err}); \
                     falling back to gossip discovery for this node"
                );
            }
        }
    }

    let mut plan = BTreeMap::new();
    for (node_id, node) in nodes {
        let Some(listen) = listeners.get(node_id) else {
            continue;
        };
        let mut connect: Vec<String> = Vec::new();
        if let Some(ep) = daemon_endpoint {
            connect.push(ep.to_string());
        }
        for source in input_sources(node) {
            // A node may consume from itself only via a timer/log mapping, which
            // `input_sources` already filters out.
            if &source == node_id {
                continue;
            }
            if let Some(ep) = listeners.get(&source) {
                connect.push(ep.clone());
            }
        }
        connect.dedup();
        plan.insert(
            node_id.clone(),
            NodeZenohPeering {
                listen: listen.clone(),
                connect,
            },
        );
    }
    plan
}

/// The nodes whose outputs `node` subscribes to (deduplicated).
fn input_sources(node: &ResolvedNode) -> BTreeSet<NodeId> {
    let inputs: Vec<&Input> = match &node.kind {
        CoreNodeKind::Custom(n) => n.run_config.inputs.values().collect(),
        CoreNodeKind::Runtime(n) => n
            .operators
            .iter()
            .flat_map(|op| op.config.inputs.values())
            .collect(),
    };
    inputs
        .iter()
        .filter_map(|input| match &input.mapping {
            InputMapping::User(mapping) => Some(mapping.source.clone()),
            _ => None,
        })
        .collect()
}

#[derive(Clone)]
pub struct Spawner {
    pub dataflow_id: DataflowId,
    pub daemon_tx: mpsc::Sender<Timestamped<Event>>,
    pub dataflow_descriptor: Descriptor,
    /// clock is required for generating timestamps when dropping messages early because queue is full
    pub clock: Arc<HLC>,
    pub uv: bool,
    pub ft_stats: Arc<crate::FaultToleranceStats>,
    /// Signals listener loops to shut down when the dataflow finishes.
    pub shutdown: tokio::sync::watch::Receiver<bool>,
    /// Endpoint of the daemon's zenoh listener. Forwarded to spawned nodes via
    /// `DORA_ZENOH_CONNECT` so the >=4 KiB zenoh data path works without
    /// multicast scouting (#1778).
    ///
    /// Loopback for a single-machine deployment, but possibly a routable address
    /// for a daemon in a cluster. Either way it is on the node's own host, so
    /// the node can reach it. Node listeners themselves stay on loopback: node
    /// output crosses machines by daemon-level forwarding, not node-to-node.
    pub zenoh_connect_endpoint: Option<String>,
    /// Per-node listener + dial-list, so the node↔node links the dataflow needs
    /// are established deterministically. See [`plan_zenoh_peering`].
    pub zenoh_peering: Arc<BTreeMap<NodeId, NodeZenohPeering>>,
}

impl Spawner {
    fn maybe_inject_zenoh_connect(&self, command: Command, node_id: &NodeId) -> Command {
        match self.zenoh_peering.get(node_id) {
            Some(peering) => {
                let command = command.env(DORA_ZENOH_LISTEN_ENV, &peering.listen);
                if peering.connect.is_empty() {
                    command
                } else {
                    command.env(DORA_ZENOH_CONNECT_ENV, peering.connect.join(","))
                }
            }
            // No plan for this node (dynamic node, or endpoint reservation
            // failed): fall back to dialing just the daemon and letting gossip
            // discover the rest.
            None => match &self.zenoh_connect_endpoint {
                Some(ep) => command.env(DORA_ZENOH_CONNECT_ENV, ep),
                None => command,
            },
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub async fn spawn_node(
        self,
        node: ResolvedNode,
        node_working_dir: PathBuf,
        python_env_dir: Option<PathBuf>,
        confined: bool,
        node_stderr_most_recent: Arc<ArrayQueue<String>>,
        write_events_to: Option<PathBuf>,
        logger: &mut NodeLogger<'_>,
    ) -> eyre::Result<impl Future<Output = eyre::Result<PreparedNode>> + use<>> {
        let dataflow_id = self.dataflow_id;
        let node_id = node.id.clone();
        logger
            .log(
                LogLevel::Debug,
                Some("daemon::spawner".into()),
                "spawning node",
            )
            .await;

        let last_activity = Arc::new(AtomicU64::new(crate::node_communication::current_millis()));
        let daemon_communication = spawn_listener_loop(
            &dataflow_id,
            &node_id,
            &self.daemon_tx,
            self.dataflow_descriptor.communication.local,
            self.clock.clone(),
            last_activity.clone(),
            self.shutdown.clone(),
        )
        .await?;

        let node_config = NodeConfig {
            dataflow_id,
            node_id: node_id.clone(),
            run_config: node.kind.run_config(),
            daemon_communication: Some(daemon_communication),
            dataflow_descriptor: serde_yaml::to_value(&self.dataflow_descriptor)
                .context("failed to serialize dataflow descriptor to YAML")?,
            dynamic: node.kind.dynamic(),
            write_events_to,
            restart_count: 0,
        };

        let mut logger = logger
            .try_clone()
            .await
            .wrap_err("failed to clone logger")?;
        let task = async move {
            self.prepare_node_inner(
                node,
                node_working_dir,
                python_env_dir,
                confined,
                &mut logger,
                dataflow_id,
                node_config,
                node_stderr_most_recent,
                last_activity,
            )
            .await
        };
        Ok(task)
    }

    #[allow(clippy::too_many_arguments)]
    async fn prepare_node_inner(
        self,
        node: ResolvedNode,
        node_working_dir: PathBuf,
        python_env_dir: Option<PathBuf>,
        confined: bool,
        logger: &mut NodeLogger<'_>,
        dataflow_id: uuid::Uuid,
        node_config: NodeConfig,
        node_stderr_most_recent: Arc<ArrayQueue<String>>,
        last_activity: Arc<AtomicU64>,
    ) -> eyre::Result<PreparedNode> {
        std::fs::create_dir_all(&node_working_dir)
            .context("failed to create node working directory")?;
        let (command, error_msg) = match &node.kind {
            dora_core::descriptor::CoreNodeKind::Custom(n) => {
                let command = path_spawn_command(
                    &node_working_dir,
                    self.uv,
                    python_env_dir.as_deref(),
                    confined,
                    logger,
                    n,
                    true,
                )
                .await?;

                let command = if let Some(mut command) = command {
                    command = command.current_dir(&node_working_dir);
                    command = command.stdin(Stdio::Null);
                    command = strip_denied_env(command);

                    command = command.env(
                        "DORA_NODE_CONFIG",
                        serde_yaml::to_string(&node_config.clone())
                            .wrap_err("failed to serialize node config")?,
                    );
                    command = self.maybe_inject_zenoh_connect(command, &node.id);
                    // Injecting the env variable defined in the `yaml` into
                    // the node runtime.
                    if let Some(envs) = &node.env {
                        for (key, value) in envs {
                            if !is_denied_env(key) {
                                command = command.env(key, value.to_string());
                            }
                        }
                    }
                    if let Some(envs) = &n.envs {
                        // node has some inner env variables -> add them too
                        for (key, value) in envs {
                            if !is_denied_env(key) {
                                command = command.env(key, value.to_string());
                            }
                        }
                    }

                    // For managed Python custom nodes, also set VIRTUAL_ENV and
                    // prepend the env's bin dir to PATH so subprocesses, console
                    // scripts, and `python -m pip` see the env. Mirrors the
                    // managed-interpreter selection in `path_spawn_command`.
                    if self.uv
                        && let Some(env_dir) = python_env_dir.as_deref()
                        && n.build.is_some()
                    {
                        command =
                            apply_managed_python_runtime_env(command, env_dir, node.env.as_ref())?;
                    }

                    command = command.env("PYTHONUNBUFFERED", "1");
                    command = command
                        .stdin(Stdio::Null)
                        .stdout(Stdio::Piped)
                        .stderr(Stdio::Piped);
                    Some(command)
                } else {
                    command
                };

                let error_msg = format!(
                    "failed to run `{}` with args `{}`",
                    n.path,
                    n.args.as_deref().unwrap_or_default(),
                );
                (command, error_msg)
            }
            dora_core::descriptor::CoreNodeKind::Runtime(n) => {
                let python_operators: Vec<&OperatorDefinition> = n
                    .operators
                    .iter()
                    .filter(|x| matches!(x.config.source, OperatorSource::Python { .. }))
                    .collect();

                let other_operators = n
                    .operators
                    .iter()
                    .any(|x| !matches!(x.config.source, OperatorSource::Python { .. }));

                let command = if !python_operators.is_empty() && !other_operators {
                    // Use python to spawn runtime if there is a python operator

                    // TODO: Handle multi-operator runtime once sub-interpreter is supported
                    if python_operators.len() > 1 {
                        eyre::bail!(
                            "Runtime currently only supports one Python Operator.
                     This is because PyO3 sub-interpreter is not yet available.
                     See: https://github.com/PyO3/pyo3/issues/576"
                        );
                    }

                    let python_operator = python_operators
                        .first()
                        .context("Runtime had no operators definition.")?;

                    if let OperatorSource::Python(PythonSource {
                        source: _,
                        conda_env: Some(conda_env),
                    }) = &python_operator.config.source
                    {
                        let conda = which::which("conda").context(
                        "failed to find `conda`, yet a `conda_env` was defined. Make sure that `conda` is available.",
                        )?;
                        let mut command = Command::new(conda);
                        command = command.args([
                            "run",
                            "-n",
                            conda_env,
                            "python",
                            "-uc",
                            format!("import dora; dora.start_runtime() # {}", node.id).as_str(),
                        ]);
                        Some(command)
                    } else {
                        let mut cmd = if self.uv {
                            if let Some(python_env_dir) = python_env_dir.as_deref() {
                                // Reuse the managed interpreter so Python operators run
                                // against the same environment Dora prepared during build.
                                let python = managed_python_interpreter(python_env_dir);
                                if !python.is_file() {
                                    eyre::bail!(
                                        "managed Python interpreter `{}` is missing",
                                        python.display()
                                    );
                                }
                                tracing::info!(
                                    "spawning managed Python {} -uc import dora; dora.start_runtime() # {}",
                                    python.display(),
                                    node.id
                                );
                                Command::new(python)
                            } else {
                                let mut cmd = Command::new("uv");
                                cmd = cmd.arg("run");
                                cmd = cmd.arg("python");
                                tracing::info!(
                                    "spawning: uv run python -uc import dora; dora.start_runtime() # {}",
                                    node.id
                                );
                                cmd
                            }
                        } else {
                            let python = get_python_path()
                                .wrap_err("Could not find python path when spawning custom node")?;
                            tracing::info!(
                                "spawning: python -uc import dora; dora.start_runtime() # {}",
                                node.id
                            );

                            Command::new(python)
                        };
                        // Force python to always flush stdout/stderr buffer
                        cmd = cmd.args([
                            "-uc",
                            format!("import dora; dora.start_runtime() # {}", node.id).as_str(),
                        ]);
                        Some(cmd)
                    }
                } else if python_operators.is_empty() && other_operators {
                    let current_exe = std::env::current_exe()
                        .wrap_err("failed to get current executable path")?;
                    let mut file_name = current_exe.clone();
                    file_name.set_extension("");
                    let file_name = file_name
                        .file_name()
                        .and_then(|s| s.to_str())
                        .context("failed to get file name from current executable")?;

                    // Check if the current executable is a python binary meaning that dora is installed within the python environment
                    if file_name.ends_with("python") || file_name.ends_with("python3") {
                        // Use the current executable to spawn runtime
                        let python = get_python_path()
                            .wrap_err("Could not find python path when spawning custom node")?;
                        let mut cmd = Command::new(python);

                        tracing::info!(
                            "spawning: python -uc import dora; dora.start_runtime() # {}",
                            node.id
                        );

                        cmd = cmd.args([
                            "-uc",
                            format!("import dora; dora.start_runtime() # {}", node.id).as_str(),
                        ]);
                        Some(cmd)
                    } else if file_name == "dora" {
                        // current_exe is the dora binary — use it so the
                        // spawned runtime always matches the daemon version.
                        // See #1797.
                        let mut cmd = Command::new(&current_exe);
                        cmd = cmd.arg("runtime");
                        Some(cmd)
                    } else {
                        // current_exe is something else, e.g. an embedded
                        // example runner that calls `dora_cli::run()` —
                        // see examples/c-dataflow/run.rs:21. Spawning
                        // current_exe with `runtime` would recurse into
                        // the example runner. Fall back to PATH lookup
                        // for the dora binary. See #1805.
                        let mut cmd = Command::new(
                            which::which("dora").wrap_err("failed to find dora binary on PATH")?,
                        );
                        cmd = cmd.arg("runtime");
                        Some(cmd)
                    }
                } else {
                    bail!(
                        "Cannot spawn runtime with both Python and non-Python operators. \
                        Please use a single operator or ensure that all operators are Python-based."
                    );
                };

                let runtime_config = RuntimeConfig {
                    node: node_config.clone(),
                    operators: n.operators.clone(),
                };

                let command = if let Some(mut command) = command {
                    command = command.current_dir(&node_working_dir);
                    command = strip_denied_env(command);

                    command = command.env(
                        "DORA_RUNTIME_CONFIG",
                        serde_yaml::to_string(&runtime_config)
                            .wrap_err("failed to serialize runtime config")?,
                    );
                    command = self.maybe_inject_zenoh_connect(command, &node.id);
                    // Injecting the env variable defined in the `yaml` into
                    // the node runtime.
                    if let Some(envs) = &node.env {
                        for (key, value) in envs {
                            if !is_denied_env(key) {
                                command = command.env(key, value.to_string());
                            }
                        }
                    }

                    // For managed Python runtime nodes (Python operator + uv on),
                    // set VIRTUAL_ENV and prepend the env's bin dir to PATH so
                    // anything the operator spawns sees the managed env.
                    if self.uv
                        && let Some(env_dir) = python_env_dir.as_deref()
                    {
                        command =
                            apply_managed_python_runtime_env(command, env_dir, node.env.as_ref())?;
                    }

                    command = command
                        .stdin(Stdio::Null)
                        .stdout(Stdio::Piped)
                        .stderr(Stdio::Piped);
                    Some(command)
                } else {
                    command
                };
                let error_msg = format!(
                    "failed to run runtime {}/{}",
                    runtime_config.node.dataflow_id, runtime_config.node.node_id
                );
                (command, error_msg)
            }
        };
        Ok(PreparedNode {
            command,
            spawn_error_msg: error_msg,
            node_working_dir,
            dataflow_id,
            node,
            node_config,
            clock: self.clock,
            daemon_tx: self.daemon_tx,
            node_stderr_most_recent,
            last_activity,
            ft_stats: self.ft_stats,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_core::descriptor::DescriptorExt;

    fn plan_for(yaml: &str, daemon: Option<&str>) -> BTreeMap<NodeId, NodeZenohPeering> {
        let descriptor: Descriptor = serde_yaml::from_str(yaml).expect("parse descriptor");
        let nodes = descriptor
            .resolve_aliases_and_set_defaults()
            .expect("resolve nodes");
        // Default: every node is local to this daemon.
        let local: BTreeSet<NodeId> = nodes.keys().cloned().collect();
        plan_zenoh_peering(&nodes, &local, daemon)
    }

    fn plan_for_local(
        yaml: &str,
        local: &[&str],
        daemon: Option<&str>,
    ) -> BTreeMap<NodeId, NodeZenohPeering> {
        let descriptor: Descriptor = serde_yaml::from_str(yaml).expect("parse descriptor");
        let nodes = descriptor
            .resolve_aliases_and_set_defaults()
            .expect("resolve nodes");
        let local: BTreeSet<NodeId> = local.iter().map(|id| node(id)).collect();
        plan_zenoh_peering(&nodes, &local, daemon)
    }

    fn node(id: &str) -> NodeId {
        NodeId::from(id.to_string())
    }

    /// Since zenoh 1.9 peers don't relay, a consumer that never dials its
    /// producer can never receive its data. So every consumer must dial exactly
    /// its producers — that link set is the whole point of the plan.
    #[test]
    fn each_consumer_dials_its_producers_and_nothing_else() {
        let plan = plan_for(
            r#"
nodes:
  - id: source
    path: source
    inputs:
      tick: dora/timer/millis/10
    outputs:
      - value
  - id: transform
    path: transform
    inputs:
      value: source/value
    outputs:
      - doubled
  - id: sink
    path: sink
    inputs:
      doubled: transform/doubled
"#,
            Some("tcp/127.0.0.1:1"),
        );

        let listen_of = |id: &str| plan[&node(id)].listen.clone();

        // A pure source has no `User` inputs, so it dials only the daemon: a
        // timer mapping is not a peer.
        assert_eq!(plan[&node("source")].connect, vec!["tcp/127.0.0.1:1"]);

        // Each consumer dials the daemon plus its own producer's listener.
        assert_eq!(
            plan[&node("transform")].connect,
            vec!["tcp/127.0.0.1:1".to_string(), listen_of("source")]
        );
        assert_eq!(
            plan[&node("sink")].connect,
            vec!["tcp/127.0.0.1:1".to_string(), listen_of("transform")]
        );

        // `sink` does not consume from `source`, so it must not dial it —
        // otherwise the plan degenerates toward N^2.
        assert!(!plan[&node("sink")].connect.contains(&listen_of("source")));

        // Listeners must be distinct, or two nodes would fight for a port.
        let listeners: BTreeSet<_> = plan.values().map(|p| p.listen.clone()).collect();
        assert_eq!(listeners.len(), 3, "each node needs its own listener");
    }

    /// A cycle must still produce a plan: every node is assigned a listener
    /// before any dial-list is built, so `a -> b -> a` resolves rather than
    /// leaving one side unable to reference the other.
    #[test]
    fn cyclic_dataflow_links_both_directions() {
        let plan = plan_for(
            r#"
nodes:
  - id: a
    path: a
    inputs:
      from_b: b/out_b
    outputs:
      - out_a
  - id: b
    path: b
    inputs:
      from_a: a/out_a
    outputs:
      - out_b
"#,
            None,
        );
        assert_eq!(
            plan[&node("a")].connect,
            vec![plan[&node("b")].listen.clone()]
        );
        assert_eq!(
            plan[&node("b")].connect,
            vec![plan[&node("a")].listen.clone()]
        );
    }

    /// `nodes` spans the whole dataflow, including nodes owned by other daemons.
    /// Their listeners would be on *this* host's loopback, so dialing one would hit
    /// nothing (or an unrelated local process). A local consumer of a remote
    /// producer must therefore get no dial for it.
    #[test]
    fn remote_nodes_are_never_dialled_on_local_loopback() {
        let yaml = r#"
nodes:
  - id: remote_source
    path: remote_source
    outputs:
      - value
  - id: local_sink
    path: local_sink
    inputs:
      v: remote_source/value
"#;
        // Only `local_sink` runs on this daemon.
        let plan = plan_for_local(yaml, &["local_sink"], Some("tcp/127.0.0.1:1"));

        assert!(
            !plan.contains_key(&node("remote_source")),
            "a node on another daemon must not be assigned a local loopback listener"
        );
        // The local consumer dials only the daemon: there is no local endpoint
        // for its remote producer, and inventing one would be worse than none.
        assert_eq!(plan[&node("local_sink")].connect, vec!["tcp/127.0.0.1:1"]);
    }

    /// Dynamic nodes join at arbitrary times and aren't part of the spawn set,
    /// so they get no plan (and fall back to daemon-only + gossip). A static
    /// consumer of a dynamic node must not end up with a dangling dial.
    #[test]
    fn dynamic_nodes_are_excluded_from_the_plan() {
        let plan = plan_for(
            r#"
nodes:
  - id: dyn_source
    path: dynamic
    outputs:
      - value
  - id: static_sink
    path: static_sink
    inputs:
      v: dyn_source/value
"#,
            Some("tcp/127.0.0.1:1"),
        );
        assert!(
            !plan.contains_key(&node("dyn_source")),
            "dynamic node must not be assigned a listener"
        );
        // The static consumer still gets a plan, but only dials the daemon:
        // there is no endpoint to dial for a node that hasn't joined yet.
        assert_eq!(plan[&node("static_sink")].connect, vec!["tcp/127.0.0.1:1"]);
    }
}
