use crate::{
    CoreNodeKindExt, Event,
    log::NodeLogger,
    node_communication::spawn_listener_loop,
    spawn::{command::path_spawn_command, prepared::PreparedNode},
};
use clonable_command::{Command, Stdio};
use crossbeam::queue::ArrayQueue;
use dora_core::{
    build::managed_python_bin_dir,
    config::{Input, InputMapping, NodeId},
    descriptor::{CoreNodeKind, Descriptor, ResolvedNode},
    topics::{
        DORA_RUN_PARENT_PID_ENV, DORA_ZENOH_CONNECT_ENV, DORA_ZENOH_LISTEN_ENV,
        DORA_ZENOH_MULTICAST_ENV, ZENOH_CONFIG_PATH_ENV,
    },
    uhlc::HLC,
};
use dora_message::{
    DataflowId,
    common::LogLevel,
    daemon_to_coordinator::Timestamped,
    daemon_to_node::{NodeConfig, OutputRouting, RuntimeConfig},
    descriptor::EnvValue,
    id::DataId,
};
use eyre::WrapErr;
use std::{
    collections::{BTreeMap, BTreeSet},
    ffi::OsString,
    future::Future,
    net::IpAddr,
    path::{Path, PathBuf},
    sync::{Arc, atomic::AtomicU64},
};
use tokio::sync::mpsc;

/// Environment variable names that must never be passed to spawned nodes:
/// loader-injection vectors and daemon-level secrets. Refused from a
/// descriptor *and* scrubbed from the environment nodes inherit.
const ENV_DENYLIST: &[&str] = &[
    "LD_PRELOAD",
    "LD_AUDIT",
    "DYLD_INSERT_LIBRARIES",
    "DORA_AUTH_TOKEN",
    "DORA_ALLOW_SHELL_NODES",
];

/// Library *search* paths. Refused from a descriptor like the hijack variables
/// above, but deliberately still inherited: a daemon started from a sourced
/// ROS or CUDA environment must keep passing these on, or a dynamically linked
/// node fails to start before it can report anything. A node that needs a
/// custom search path gets it from the environment the daemon runs in
/// (#2991 review).
const SEARCH_PATH_ENV: &[&str] = &["LD_LIBRARY_PATH", "DYLD_LIBRARY_PATH"];

/// Control-plane variables the daemon injects into every node it spawns.
/// Descriptor `env:` / `envs:` entries must not override them: they configure
/// how the node reaches its daemon and peers, and a forged value produces a
/// dataflow that starts cleanly but silently exchanges nothing (#2944).
///
/// `DORA_NODE_CONFIG` was previously safe only because `spawn_inner`
/// re-serializes it after the command is built (a restart-count fix, not a
/// security measure); listing it here makes that safety intentional.
const CONTROL_PLANE_ENV: &[&str] = &[
    "DORA_NODE_CONFIG",
    "DORA_RUNTIME_CONFIG",
    DORA_ZENOH_LISTEN_ENV,
    DORA_ZENOH_CONNECT_ENV,
    DORA_ZENOH_MULTICAST_ENV,
    // Names a pid the node will SIGKILL its own process group over once that
    // pid is gone. A descriptor-supplied value would be an arbitrary
    // self-destruct trigger, and an inherited one (from an outer `dora run`,
    // or a shell that happened to export it) would arm the guard against the
    // wrong process — so it is refused from both and set only here.
    DORA_RUN_PARENT_PID_ENV,
];

/// Whether `key` names the reserved variable `reserved`.
///
/// Exact on Unix, where `ld_preload` and `LD_PRELOAD` are genuinely different
/// variables and folding case would silently drop a legitimate descriptor
/// entry. On Windows the OS folds env keys with its uppercase mapping before
/// the child sees them, so a case (or Unicode) variant of a reserved name
/// collides with the real one and must be denied.
fn env_key_matches(key: &str, reserved: &str) -> bool {
    if cfg!(windows) {
        key.to_uppercase() == reserved.to_uppercase()
    } else {
        key == reserved
    }
}

/// Returns true if the env var key is denied, logging a warning if so.
fn is_denied_env(key: &str) -> bool {
    if ENV_DENYLIST.iter().any(|d| env_key_matches(key, d)) {
        tracing::warn!(
            "skipping denied environment variable {key:?} (security: could inject shared libraries)"
        );
        true
    } else if SEARCH_PATH_ENV.iter().any(|d| env_key_matches(key, d)) {
        tracing::warn!(
            "skipping {key:?} from the descriptor env: library search paths are \
             inherited from the environment the daemon runs in"
        );
        true
    } else if CONTROL_PLANE_ENV
        .iter()
        .chain(std::iter::once(&ZENOH_CONFIG_PATH_ENV))
        .any(|d| env_key_matches(key, d))
    {
        tracing::warn!(
            "ignoring {key:?} from the descriptor env: it is control-plane wiring \
             set by the daemon and cannot be overridden"
        );
        true
    } else if is_malformed_env_key(key) {
        tracing::warn!(
            "skipping malformed environment variable name {key:?}: names cannot be \
             empty or contain `=`, whitespace, or NUL"
        );
        true
    } else {
        false
    }
}

/// Whether the OS would refuse to treat `key` as one variable name.
///
/// A key containing `=` is handed to `execve` verbatim, so the entry
/// `LD_PRELOAD=/tmp/evil.so ` (denylist miss: the whole string is the *key*)
/// reaches the child as a real `LD_PRELOAD` assignment, since the loader
/// splits its value on whitespace. Rejecting the shape closes that off for
/// every reserved name at once.
fn is_malformed_env_key(key: &str) -> bool {
    key.is_empty()
        || key.contains('=')
        || key.contains('\0')
        || key.chars().any(char::is_whitespace)
}

/// Apply descriptor-provided env (`env:` / `envs:`) to the command, dropping
/// denied keys with a warning.
fn apply_descriptor_env(
    mut command: Command,
    envs: Option<&BTreeMap<String, EnvValue>>,
) -> Command {
    if let Some(envs) = envs {
        for (key, value) in envs {
            if !is_denied_env(key) {
                command = command.env(key, value.to_string());
            }
        }
    }
    command
}

/// Keep the daemon's own environment from reaching child nodes: loader-hijack
/// variables and `DORA_AUTH_TOKEN` (via `/proc/<pid>/environ`), plus the
/// control-plane wiring, so a stale `DORA_ZENOH_CONNECT` exported in the shell
/// that started the daemon cannot wrongly wire nodes the daemon deliberately
/// left unwired.
///
/// [`ZENOH_CONFIG_PATH_ENV`] and [`SEARCH_PATH_ENV`] are deliberately absent:
/// refused from a descriptor, but inheriting them is how a deployment points
/// every process at a custom zenoh config, and how a node linked against a
/// sourced ROS or CUDA install finds its libraries.
///
/// Inserts the `None` removal sentinel directly rather than calling
/// `Command::env_remove`, which `clonable_command` implements as a map
/// *removal* — that erases any explicit entry but leaves inheritance untouched,
/// so the whole scrub was a no-op (only `None` values become
/// `std::process::Command::env_remove` calls when the command is converted).
fn deny_inherited_env(mut command: Command) -> Command {
    for key in ENV_DENYLIST.iter().chain(CONTROL_PLANE_ENV) {
        command.environment.insert(OsString::from(*key), None);
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
/// dial. See [`reserve_node_listeners`] and [`build_peering_plan`].
#[derive(Clone, Debug)]
pub struct NodeZenohPeering {
    /// Endpoints this node listens on, so its consumers can dial it.
    ///
    /// Always a loopback endpoint for its same-machine consumers, plus — when
    /// the node has a consumer on another daemon — one on the address this
    /// host is reachable at. Listening on both keeps same-machine consumers on
    /// loopback, where the transport can carry shared memory, while remote ones
    /// get an endpoint they can actually dial.
    pub listen: Vec<String>,
    /// Endpoints this node dials: the daemon, plus each node it consumes from.
    pub connect: Vec<String>,
}

/// Reserve the listen endpoints for this daemon's local nodes, so the links the
/// dataflow needs can be established *by construction*.
///
/// One half of the peering plan; [`build_peering_plan`] is the other. They are
/// split because the daemon-to-daemon endpoint exchange runs between them: a
/// daemon must know its own nodes' endpoints before it can publish them, and
/// its peers' before it can build dial lists that include them.
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
/// Only *local* nodes are planned: `nodes` spans the whole dataflow, and this
/// daemon can only reserve ports on its own host. A local consumer of a remote
/// producer therefore gets no dial for it here — the remote producer's endpoint
/// arrives separately, from the daemon that owns it.
///
/// # Cross-machine consumers
///
/// A node whose consumers are all on this machine listens on loopback only,
/// which is both sufficient and the least exposed choice. A node with a
/// consumer on *another* daemon also listens on `routable_addr`, the address
/// this host is reachable at, so that consumer has something to dial. That
/// second listener is what lets cross-machine edges skip the daemon relay
/// entirely.
///
/// `routable_addr` is `None` for a single-machine daemon, and a loopback
/// address is refused for this purpose: advertising `127.0.0.1` to a remote
/// consumer points it at its *own* loopback, which is the silent-partition
/// failure this planning exists to prevent. Either way the edge falls back to
/// the daemon path, which is where it lives today.
pub fn reserve_node_listeners(
    nodes: &BTreeMap<NodeId, ResolvedNode>,
    local_nodes: &BTreeSet<NodeId>,
    routable_addr: Option<IpAddr>,
) -> BTreeMap<NodeId, NodeListeners> {
    let remotely_consumed = remotely_consumed_nodes(nodes, local_nodes);
    // Reserve listeners per node first, so the dial-lists below can reference
    // every node regardless of spawn order.
    //
    // Only nodes this daemon spawns get one: the loopback endpoint of a node
    // running on another machine would point a local consumer at this host's
    // 127.0.0.1 — either a failed dial or, worse, an unrelated local process.
    let mut listeners: BTreeMap<NodeId, NodeListeners> = BTreeMap::new();
    for (node_id, node) in nodes {
        if node.kind.dynamic() || !local_nodes.contains(node_id) {
            continue;
        }
        let loopback = match dora_core::topics::reserve_loopback_zenoh_endpoint() {
            Ok(ep) => ep,
            Err(err) => {
                // Fall back to gossip for this node rather than failing the
                // dataflow; it just loses the determinism guarantee.
                tracing::warn!(
                    node = %node_id,
                    "failed to reserve a zenoh listen endpoint ({err}); \
                     falling back to gossip discovery for this node"
                );
                continue;
            }
        };
        // Only pay for a network-reachable listener where a remote consumer
        // actually needs one.
        let routable = match routable_addr.filter(|_| remotely_consumed.contains(node_id)) {
            Some(addr) => match dora_core::topics::reserve_zenoh_endpoint(addr) {
                Ok(ep) => Some(ep),
                Err(err) => {
                    tracing::warn!(
                        node = %node_id,
                        "failed to reserve a routable zenoh listen endpoint on {addr} \
                         ({err}); consumers on other daemons will keep receiving this \
                         node's outputs over the daemon path"
                    );
                    None
                }
            },
            None => None,
        };
        listeners.insert(node_id.clone(), NodeListeners { loopback, routable });
    }
    listeners
}

/// Build each local node's dial list from the reserved local listeners plus
/// `remote_endpoints`, the endpoints other daemons reported for *their* nodes
/// (see `endpoint_exchange`).
///
/// A local consumer dials its local producers on loopback and its remote ones
/// on the endpoint their own daemon reserved. A remote producer that is absent
/// from `remote_endpoints` simply gets no dial, and that edge keeps the
/// daemon-forwarded path.
pub fn build_peering_plan(
    nodes: &BTreeMap<NodeId, ResolvedNode>,
    listeners: &BTreeMap<NodeId, NodeListeners>,
    daemon_endpoint: Option<&str>,
    remote_endpoints: &BTreeMap<NodeId, String>,
) -> BTreeMap<NodeId, NodeZenohPeering> {
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
            // A local producer is reachable on loopback, which is also the
            // transport that can carry shared memory; a remote one only at the
            // address its own daemon reserved.
            if let Some(ep) = listeners.get(&source) {
                connect.push(ep.loopback.clone());
            } else if let Some(ep) = remote_endpoints.get(&source) {
                connect.push(ep.clone());
            }
        }
        connect.dedup();
        plan.insert(
            node_id.clone(),
            NodeZenohPeering {
                listen: listen.endpoints(),
                connect,
            },
        );
    }
    plan
}

/// The endpoints reserved for one local node.
pub struct NodeListeners {
    /// Always present: this is how same-machine consumers reach the node, and
    /// a loopback transport is the one that can carry shared memory.
    loopback: String,
    /// Present only when a consumer on another daemon needs to dial in.
    routable: Option<String>,
}

impl NodeListeners {
    /// The endpoint a consumer on another machine dials, when there is one.
    pub fn routable(&self) -> Option<&str> {
        self.routable.as_deref()
    }

    fn endpoints(&self) -> Vec<String> {
        let mut endpoints = vec![self.loopback.clone()];
        endpoints.extend(self.routable.clone());
        endpoints
    }
}

/// Local nodes that at least one *other daemon's* static node consumes from.
///
/// Dynamic consumers are excluded: they are not in any spawn set, so nothing
/// can plan a dial for them, and their edges stay on the daemon path.
pub fn remotely_consumed_nodes(
    nodes: &BTreeMap<NodeId, ResolvedNode>,
    local_nodes: &BTreeSet<NodeId>,
) -> BTreeSet<NodeId> {
    nodes
        .iter()
        .filter(|(id, node)| !local_nodes.contains(*id) && !node.kind.dynamic())
        .flat_map(|(_, consumer)| input_sources(consumer))
        .filter(|source| local_nodes.contains(source))
        .collect()
}

/// Remote static nodes that this daemon's local nodes consume from.
///
/// The exact set of endpoints worth asking peers for: a remote node nobody here
/// subscribes to is never dialled, and a dynamic one cannot be planned.
pub fn remote_sources_of_local_nodes(
    nodes: &BTreeMap<NodeId, ResolvedNode>,
    local_nodes: &BTreeSet<NodeId>,
) -> BTreeSet<NodeId> {
    nodes
        .iter()
        .filter(|(id, node)| local_nodes.contains(*id) && !node.kind.dynamic())
        .flat_map(|(_, consumer)| input_sources(consumer))
        .filter(|source| {
            !local_nodes.contains(source) && nodes.get(source).is_some_and(|n| !n.kind.dynamic())
        })
        .collect()
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
    /// are established deterministically. See [`build_peering_plan`].
    pub zenoh_peering: Arc<BTreeMap<NodeId, NodeZenohPeering>>,
    /// Whether this daemon runs without multicast scouting, so spawned nodes
    /// should too (see [`DORA_ZENOH_MULTICAST_ENV`]). Mixing modes leaves a
    /// node scouting for a daemon that no longer answers.
    pub disable_multicast: bool,
    /// This machine's id (as registered with the coordinator), if any.
    /// Forwarded to spawned nodes via `DORA_MACHINE_ID` so the node API can
    /// derive the machine-qualified OS id of a mirrored cross-machine pool
    /// (see `create_cross_pool_shmem` in the daemon).
    pub machine_id: Option<String>,
    /// Whether this daemon runs *inside* the process that invoked it — the
    /// `Daemon::run_dataflow` case, where caller, coordinator and daemon are
    /// one process and the nodes are its children.
    ///
    /// Only then may a node treat its parent's death as the end of the
    /// dataflow; see [`DORA_RUN_PARENT_PID_ENV`] for why the `dora up` +
    /// `dora start` path must not.
    pub bind_nodes_to_parent: bool,
}

impl Spawner {
    fn maybe_inject_machine_id(&self, command: Command) -> Command {
        let command = match &self.machine_id {
            Some(machine_id) => command.env("DORA_MACHINE_ID", machine_id),
            None => command,
        };
        // Note: the direct-TCP auth token (`DORA_MEMORY_POOL_AUTH_TOKEN`)
        // is deliberately NOT injected into spawned nodes. The handshake
        // runs entirely inside the daemon (it reads its own env on
        // send/verify), and no node-side code ever reads the token —
        // injecting it would only leak the shared deployment secret into
        // every user node process (bot review 5301862843, 2026-08-15).
        command
    }

    fn maybe_inject_zenoh_connect(&self, command: Command, node_id: &NodeId) -> Command {
        let command = match self.zenoh_peering.get(node_id) {
            Some(peering) => {
                let command = command.env(DORA_ZENOH_LISTEN_ENV, peering.listen.join(","));
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
        };
        // Forward the daemon's multicast decision unconditionally: it is a
        // request, and `open_zenoh_session_with_listen` ignores it for a node
        // that ended up with neither an endpoint to dial nor a listener to be
        // dialled on. Re-deriving that condition here would duplicate the
        // #1856 guard against the code that owns it.
        if self.disable_multicast {
            command.env(DORA_ZENOH_MULTICAST_ENV, "off")
        } else {
            command
        }
    }

    /// Compose a spawned node's environment: scrub what must not be inherited,
    /// apply the descriptor's `env:` (and a custom node's inner `envs:`), then
    /// inject the daemon's control-plane wiring LAST.
    ///
    /// The order is the protection (#2944). A later `.env()` overwrites an
    /// earlier one, so injecting last means a descriptor entry cannot override
    /// the daemon's wiring even for a control-plane key [`is_denied_env`]
    /// misses. Both spawn paths compose through here so the two cannot drift.
    fn compose_node_env(
        &self,
        mut command: Command,
        node_id: &NodeId,
        descriptor_envs: &[Option<&BTreeMap<String, EnvValue>>],
        config_key: &str,
        config_yaml: String,
    ) -> Command {
        command = deny_inherited_env(command);
        for envs in descriptor_envs {
            command = apply_descriptor_env(command, *envs);
        }
        command = command.env(config_key, config_yaml);
        if self.bind_nodes_to_parent {
            command = command.env(DORA_RUN_PARENT_PID_ENV, std::process::id().to_string());
        }
        let command = self.maybe_inject_zenoh_connect(command, node_id);
        // The fork's machine-qualified pool naming needs the machine id on
        // the node (DORA_MACHINE_ID); compose_node_env covers the rest.
        self.maybe_inject_machine_id(command)
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
        output_routing: BTreeMap<DataId, OutputRouting>,
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
        // The incarnation identity for this spawn: assigned before the
        // listener is bound so every `Event::Node` from this process's
        // connection carries it (dora-rs/dora#2927).
        let generation = crate::running_dataflow::next_node_generation();
        let generation_counter = Arc::new(AtomicU64::new(generation));
        // Per-node listener lifetime: the sender ends up in the node's
        // RunningNode entry (and its restart loop), so retiring the node
        // closes the listener instead of leaking it until dataflow end
        // (dora-rs/dora#2988 review, finding 3).
        let (listener_shutdown, node_shutdown_rx) = tokio::sync::watch::channel(false);
        let daemon_communication = spawn_listener_loop(
            &dataflow_id,
            &node_id,
            generation_counter.clone(),
            &self.daemon_tx,
            self.clock.clone(),
            last_activity.clone(),
            self.shutdown.clone(),
            node_shutdown_rx,
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
            output_routing: Some(output_routing),
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
                generation,
                generation_counter,
                listener_shutdown,
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
        generation: u64,
        generation_counter: Arc<AtomicU64>,
        listener_shutdown: tokio::sync::watch::Sender<bool>,
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
                    command = self.compose_node_env(
                        command,
                        &node.id,
                        &[node.env.as_ref(), n.envs.as_ref()],
                        "DORA_NODE_CONFIG",
                        serde_yaml::to_string(&node_config)
                            .wrap_err("failed to serialize node config")?,
                    );

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
                let mut command = super::runtime_registry::runtime_command(
                    &node.id,
                    &n.operators,
                    self.uv,
                    python_env_dir.as_deref(),
                )?;

                let runtime_config = RuntimeConfig {
                    node: node_config.clone(),
                    operators: n.operators.clone(),
                };

                command = command.current_dir(&node_working_dir);
                command = self.compose_node_env(
                    command,
                    &node.id,
                    &[node.env.as_ref()],
                    "DORA_RUNTIME_CONFIG",
                    serde_yaml::to_string(&runtime_config)
                        .wrap_err("failed to serialize runtime config")?,
                );

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

                let error_msg = format!(
                    "failed to run runtime {}/{}",
                    runtime_config.node.dataflow_id, runtime_config.node.node_id
                );
                (Some(command), error_msg)
            }
        };
        Ok(PreparedNode {
            command,
            spawn_error_msg: error_msg,
            node_working_dir,
            dataflow_id,
            node,
            generation,
            generation_counter,
            listener_shutdown,
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
    use std::ffi::{OsStr, OsString};

    fn spawner_for(daemon_endpoint: Option<&str>, disable_multicast: bool) -> Spawner {
        let (daemon_tx, _rx) = mpsc::channel(1);
        let (_shutdown_tx, shutdown) = tokio::sync::watch::channel(false);
        Spawner {
            dataflow_id: uuid::Uuid::nil(),
            daemon_tx,
            dataflow_descriptor: serde_yaml::from_str("nodes: []").expect("parse descriptor"),
            clock: Arc::new(HLC::default()),
            uv: false,
            ft_stats: Arc::new(crate::FaultToleranceStats::default()),
            shutdown,
            zenoh_connect_endpoint: daemon_endpoint.map(String::from),
            // Empty: exercises the daemon-endpoint fallback arm, which is the
            // one a node without its own peering plan takes.
            zenoh_peering: Arc::new(BTreeMap::new()),
            disable_multicast,
            machine_id: None,
            // The `dora up` shape: a long-lived daemon whose nodes outlive it.
            // The `dora run` shape is covered by its own test below.
            bind_nodes_to_parent: false,
        }
    }

    fn injected_multicast(spawner: &Spawner) -> Option<OsString> {
        let command = spawner.maybe_inject_zenoh_connect(Command::new("true"), &node("sink"));
        command
            .environment
            .get(&OsString::from(DORA_ZENOH_MULTICAST_ENV))
            .cloned()
            .flatten()
    }

    /// A node must discover the same way the daemon does: a node left scouting
    /// for a daemon that has stopped scouting finds nothing. Whether the
    /// request is honored is `open_zenoh_session_with_listen`'s call (it keeps
    /// scouting for a node with no endpoint and no listener, per #1856), so the
    /// spawner forwards it and does not re-derive that condition.
    #[test]
    fn the_daemons_multicast_decision_is_forwarded_to_spawned_nodes() {
        assert_eq!(
            injected_multicast(&spawner_for(Some("tcp/127.0.0.1:7447"), true)).as_deref(),
            Some(OsStr::new("off")),
            "a daemon off multicast must tell its nodes"
        );
        assert_eq!(
            injected_multicast(&spawner_for(Some("tcp/127.0.0.1:7447"), false)),
            None,
            "a daemon still scouting must not disable it for its nodes"
        );
    }

    fn plan_for(yaml: &str, daemon: Option<&str>) -> BTreeMap<NodeId, NodeZenohPeering> {
        let descriptor: Descriptor = serde_yaml::from_str(yaml).expect("parse descriptor");
        let nodes = descriptor
            .resolve_aliases_and_set_defaults()
            .expect("resolve nodes");
        // Default: every node is local to this daemon, so nothing needs a
        // routable listener.
        let local: BTreeSet<NodeId> = nodes.keys().cloned().collect();
        let listeners = reserve_node_listeners(&nodes, &local, None);
        build_peering_plan(&nodes, &listeners, daemon, &BTreeMap::new())
    }

    fn plan_for_local(
        yaml: &str,
        local: &[&str],
        daemon: Option<&str>,
    ) -> BTreeMap<NodeId, NodeZenohPeering> {
        plan_for_local_with_routable(yaml, local, daemon, None)
    }

    fn plan_for_local_with_routable(
        yaml: &str,
        local: &[&str],
        daemon: Option<&str>,
        routable: Option<IpAddr>,
    ) -> BTreeMap<NodeId, NodeZenohPeering> {
        plan_with_remote_endpoints(yaml, local, daemon, routable, &BTreeMap::new())
    }

    /// The full pipeline a spawn runs: reserve local listeners, then build the
    /// dial lists with whatever the endpoint exchange returned.
    fn plan_with_remote_endpoints(
        yaml: &str,
        local: &[&str],
        daemon: Option<&str>,
        routable: Option<IpAddr>,
        remote_endpoints: &BTreeMap<NodeId, String>,
    ) -> BTreeMap<NodeId, NodeZenohPeering> {
        let descriptor: Descriptor = serde_yaml::from_str(yaml).expect("parse descriptor");
        let nodes = descriptor
            .resolve_aliases_and_set_defaults()
            .expect("resolve nodes");
        let local: BTreeSet<NodeId> = local.iter().map(|id| node(id)).collect();
        let listeners = reserve_node_listeners(&nodes, &local, routable);
        build_peering_plan(&nodes, &listeners, daemon, remote_endpoints)
    }

    /// The loopback listener, which every planned node has.
    fn loopback_of(plan: &BTreeMap<NodeId, NodeZenohPeering>, id: &str) -> String {
        plan[&node(id)].listen[0].clone()
    }

    fn node(id: &str) -> NodeId {
        NodeId::from(id.to_string())
    }

    /// #2944: the daemon's own wiring variables must be rejected from
    /// descriptor `env:` entries — they configure how the node reaches its
    /// daemon and peers, and a descriptor that sets them produces a dataflow
    /// that starts cleanly but silently exchanges nothing.
    #[test]
    fn reserved_control_plane_keys_are_denied() {
        for key in CONTROL_PLANE_ENV {
            assert!(
                is_denied_env(key),
                "`{key}` is daemon-managed control-plane wiring and must be denied"
            );
        }
        assert!(
            is_denied_env(ZENOH_CONFIG_PATH_ENV),
            "`{ZENOH_CONFIG_PATH_ENV}` builds the whole zenoh session from a file, \
             skipping every DORA_ZENOH_* variable — denying those and not this one \
             leaves the bypass open"
        );
        assert!(
            !is_denied_env("MY_APP_SETTING"),
            "ordinary variables must still pass through"
        );
        // Unix env vars are case-sensitive, so `ld_preload` is a different
        // (harmless) variable and must not be dropped. Windows folds keys, so
        // there the variant IS the reserved name.
        assert_eq!(
            is_denied_env("dora_zenoh_connect"),
            cfg!(windows),
            "case variants are only a bypass where the OS folds env keys"
        );
        assert_eq!(is_denied_env("ld_preload"), cfg!(windows));
    }

    /// A descriptor key the OS would not read as one variable name is a
    /// denylist bypass: `env: { "LD_PRELOAD=/tmp/evil.so ": "" }` reaches
    /// `execve` verbatim and the loader reads it as a real `LD_PRELOAD`.
    #[test]
    fn malformed_env_names_are_rejected() {
        for key in [
            "LD_PRELOAD=/tmp/evil.so ",
            "DORA_NODE_CONFIG=forged",
            "HAS SPACE",
            "",
        ] {
            assert!(is_denied_env(key), "{key:?} is not a usable variable name");
        }
    }

    fn env_of(command: &Command, key: &str) -> Option<Option<OsString>> {
        command.environment.get(&OsString::from(key)).cloned()
    }

    /// #2944 regression, driven through the same helper both spawn paths use.
    /// Pins the ordering defense independently of the denylist: a
    /// control-plane value already on the command (as if the denylist had
    /// missed it) must still lose to the daemon's inject-last wiring.
    #[test]
    fn descriptor_env_cannot_override_the_daemons_wiring() {
        let spawner = spawner_for(Some("tcp/127.0.0.1:7447"), true);
        let forged: BTreeMap<String, EnvValue> = CONTROL_PLANE_ENV
            .iter()
            .chain(std::iter::once(&ZENOH_CONFIG_PATH_ENV))
            .map(|key| (key.to_string(), EnvValue::String("FORGED".into())))
            .collect();
        let ordinary: BTreeMap<String, EnvValue> = [(
            "MY_APP_SETTING".to_string(),
            EnvValue::String("kept".into()),
        )]
        .into();

        let command = spawner.compose_node_env(
            // Pre-seeded as if the denylist had missed the key.
            Command::new("true").env("DORA_NODE_CONFIG", "FORGED"),
            &node("sink"),
            &[Some(&forged), Some(&ordinary)],
            "DORA_NODE_CONFIG",
            "real-config".into(),
        );

        let env = |key: &str| env_of(&command, key).flatten();
        assert_eq!(
            env("DORA_NODE_CONFIG").as_deref(),
            Some(OsStr::new("real-config")),
            "the node must receive the daemon's config, not the descriptor's"
        );
        assert_eq!(
            env(DORA_ZENOH_CONNECT_ENV).as_deref(),
            Some(OsStr::new("tcp/127.0.0.1:7447")),
            "the node must dial the daemon's endpoint, not the descriptor's"
        );
        assert_eq!(
            env(DORA_ZENOH_MULTICAST_ENV).as_deref(),
            Some(OsStr::new("off")),
            "the daemon's multicast decision must win"
        );
        assert_eq!(
            env(ZENOH_CONFIG_PATH_ENV),
            None,
            "a descriptor must not hand the node its own zenoh config file"
        );
        assert_eq!(
            env("MY_APP_SETTING").as_deref(),
            Some(OsStr::new("kept")),
            "ordinary descriptor env must still reach the node"
        );
    }

    /// The operator runtime composes its env through the same helper. It has
    /// no accidental backstop (`spawn_inner` re-serializes `DORA_NODE_CONFIG`
    /// only), so a forged `DORA_RUNTIME_CONFIG` would hand the runtime the
    /// wrong operator set.
    #[test]
    fn the_runtime_config_is_composed_like_a_node_config() {
        let spawner = spawner_for(Some("tcp/127.0.0.1:7447"), false);
        let forged: BTreeMap<String, EnvValue> = [(
            "DORA_RUNTIME_CONFIG".to_string(),
            EnvValue::String("FORGED".into()),
        )]
        .into();

        let command = spawner.compose_node_env(
            Command::new("true").env("DORA_RUNTIME_CONFIG", "FORGED"),
            &node("op"),
            &[Some(&forged)],
            "DORA_RUNTIME_CONFIG",
            "real-runtime-config".into(),
        );

        assert_eq!(
            env_of(&command, "DORA_RUNTIME_CONFIG").flatten().as_deref(),
            Some(OsStr::new("real-runtime-config")),
            "the runtime must receive the daemon's operator set"
        );
    }

    /// Nodes inherit the daemon's environment, so the scrub is the only thing
    /// keeping `DORA_AUTH_TOKEN` out of `/proc/<pid>/environ` and a stale
    /// `DORA_ZENOH_CONNECT` from the daemon's shell out of a node the daemon
    /// deliberately left unwired. Only a `None` entry becomes a real
    /// `Command::env_remove`; an absent key inherits.
    #[test]
    fn the_daemons_own_environment_does_not_leak_into_nodes() {
        let spawner = spawner_for(None, false);
        let command = spawner.compose_node_env(
            Command::new("true"),
            &node("sink"),
            &[None],
            "DORA_NODE_CONFIG",
            "real-config".into(),
        );

        for key in ["DORA_AUTH_TOKEN", "LD_PRELOAD", DORA_ZENOH_LISTEN_ENV] {
            assert_eq!(
                env_of(&command, key),
                Some(None),
                "`{key}` must be removed from the inherited environment, not merely unset"
            );
        }
        assert_eq!(
            env_of(&command, ZENOH_CONFIG_PATH_ENV),
            None,
            "a daemon-level zenoh config must still reach its nodes by inheritance"
        );
    }

    /// dora-rs/dora#2856: under `dora run` the daemon *is* the CLI process, so
    /// its death ends the dataflow — and `SIGKILL` leaves no code of ours to
    /// notice. The node is told which pid to watch so it can end itself.
    #[test]
    fn dora_run_tells_its_nodes_which_parent_to_outlive() {
        let mut spawner = spawner_for(None, false);
        spawner.bind_nodes_to_parent = true;
        let command = spawner.compose_node_env(
            Command::new("true"),
            &node("sink"),
            &[None],
            "DORA_NODE_CONFIG",
            "real-config".into(),
        );

        assert_eq!(
            env_of(&command, DORA_RUN_PARENT_PID_ENV).flatten(),
            Some(OsString::from(std::process::id().to_string())),
            "a `dora run` node must be pointed at the process it may not outlive"
        );
    }

    /// The `dora up` half of the same rule. A daemon-spawned node deliberately
    /// outlives coordinator drops, reconnects and watchdog disconnects while
    /// keeping its pid (#2029) — arming the guard there would kill nodes that
    /// are supposed to survive. The variable must be *removed*, never merely
    /// left unset, because nodes inherit the daemon's environment and a stale
    /// value from the shell that started it would arm them against a stranger.
    #[test]
    fn a_daemon_spawned_node_is_not_bound_to_the_daemons_lifetime() {
        let spawner = spawner_for(None, false);
        let command = spawner.compose_node_env(
            Command::new("true"),
            &node("sink"),
            &[None],
            "DORA_NODE_CONFIG",
            "real-config".into(),
        );

        assert_eq!(
            env_of(&command, DORA_RUN_PARENT_PID_ENV),
            Some(None),
            "`dora up` nodes must not be armed, by injection or by inheritance"
        );
    }

    /// The scrub must not strand a dynamically linked node: a daemon started
    /// from a sourced ROS or CUDA environment is the only source of these, and
    /// a descriptor cannot supply them, so scrubbing them means the node's
    /// loader fails before the node can report anything (#2991 review).
    #[test]
    fn library_search_paths_survive_the_scrub() {
        let spawner = spawner_for(None, false);
        let command = spawner.compose_node_env(
            Command::new("true"),
            &node("sink"),
            &[None],
            "DORA_NODE_CONFIG",
            "real-config".into(),
        );

        for key in SEARCH_PATH_ENV {
            assert_eq!(
                env_of(&command, key),
                None,
                "`{key}` must keep being inherited — a node linked against a \
                 sourced install has no other way to find its libraries"
            );
            assert!(
                is_denied_env(key),
                "`{key}` is still not something a descriptor gets to set"
            );
        }
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

        let listen_of = |id: &str| loopback_of(&plan, id);

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
        let listeners: BTreeSet<_> = plan.values().flat_map(|p| p.listen.clone()).collect();
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
        assert_eq!(plan[&node("a")].connect, vec![loopback_of(&plan, "b")]);
        assert_eq!(plan[&node("b")].connect, vec![loopback_of(&plan, "a")]);
    }

    /// `nodes` spans the whole dataflow, including nodes owned by other daemons.
    /// Their listeners would be on *this* host's loopback, so dialing one would hit
    /// nothing (or an unrelated local process). A local consumer of a remote
    /// producer must therefore get no dial for it here — the remote producer's
    /// real endpoint arrives from the daemon that owns it.
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
        assert_eq!(plan[&node("local_sink")].connect, vec!["tcp/127.0.0.1:1"]);
    }

    const CROSS_MACHINE_YAML: &str = r#"
nodes:
  - id: local_source
    path: local_source
    outputs:
      - value
  - id: local_sink
    path: local_sink
    inputs:
      v: local_source/value
  - id: remote_sink
    path: remote_sink
    inputs:
      v: local_source/value
"#;

    /// A node consumed from another machine needs an endpoint that machine can
    /// dial, or the edge can only ever be relayed by the two daemons. It keeps
    /// its loopback listener too: that is the one its same-machine consumers
    /// use, and the only one whose transport can carry shared memory.
    #[test]
    fn a_node_with_a_remote_consumer_also_listens_routably() {
        let routable: IpAddr = "127.0.0.2".parse().unwrap();
        let plan = plan_for_local_with_routable(
            CROSS_MACHINE_YAML,
            &["local_source", "local_sink"],
            Some("tcp/127.0.0.1:1"),
            Some(routable),
        );

        let source = &plan[&node("local_source")];
        assert_eq!(source.listen.len(), 2, "loopback plus routable: {source:?}");
        assert!(source.listen[0].starts_with("tcp/127.0.0.1:"));
        assert!(
            source.listen[1].starts_with("tcp/127.0.0.2:"),
            "the remote consumer needs a dialable endpoint, got {:?}",
            source.listen
        );

        // `local_sink` is consumed by nobody off-machine, so it stays loopback
        // only — exposure follows the dataflow, not the deployment.
        assert_eq!(plan[&node("local_sink")].listen.len(), 1);
    }

    /// The point of the whole exchange: once a remote producer's endpoint is
    /// known, its local consumer dials it directly instead of waiting on the
    /// daemon to relay every message.
    #[test]
    fn a_local_consumer_dials_a_remote_producer_it_has_an_endpoint_for() {
        let remote = BTreeMap::from([(node("remote_source"), "tcp/10.0.2.7:41000".to_string())]);
        let plan = plan_with_remote_endpoints(
            r#"
nodes:
  - id: remote_source
    path: remote_source
    outputs:
      - value
  - id: local_sink
    path: local_sink
    inputs:
      v: remote_source/value
"#,
            &["local_sink"],
            Some("tcp/127.0.0.1:1"),
            Some("127.0.0.2".parse().unwrap()),
            &remote,
        );

        assert_eq!(
            plan[&node("local_sink")].connect,
            vec![
                "tcp/127.0.0.1:1".to_string(),
                "tcp/10.0.2.7:41000".to_string()
            ],
            "the consumer must dial the daemon and its remote producer"
        );
        // Still no listener for a node this daemon does not run.
        assert!(!plan.contains_key(&node("remote_source")));
    }

    /// An endpoint the exchange did not return leaves that edge exactly where it
    /// is today — on the daemon path — rather than inventing a dial.
    #[test]
    fn a_remote_producer_without_an_endpoint_is_not_dialled() {
        let plan = plan_with_remote_endpoints(
            r#"
nodes:
  - id: remote_source
    path: remote_source
    outputs:
      - value
  - id: local_sink
    path: local_sink
    inputs:
      v: remote_source/value
"#,
            &["local_sink"],
            Some("tcp/127.0.0.1:1"),
            Some("127.0.0.2".parse().unwrap()),
            &BTreeMap::new(),
        );
        assert_eq!(plan[&node("local_sink")].connect, vec!["tcp/127.0.0.1:1"]);
    }

    /// A loopback "routable" address is worse than none: advertised to a remote
    /// consumer it points at *that* machine's loopback, which is the silent
    /// partition this planning exists to prevent. Same for a single-machine
    /// daemon, which has no routable address at all.
    #[test]
    fn a_loopback_or_absent_routable_address_yields_no_second_listener() {
        for routable in [None, Some("127.0.0.1".parse().unwrap())] {
            let plan = plan_for_local_with_routable(
                CROSS_MACHINE_YAML,
                &["local_source", "local_sink"],
                Some("tcp/127.0.0.1:1"),
                // A loopback address never reaches here in production —
                // `zenoh_routable_addr` filters it out — so passing `None` for
                // it is the honest simulation of that filter.
                routable.filter(|addr: &IpAddr| !addr.is_loopback()),
            );
            assert_eq!(
                plan[&node("local_source")].listen.len(),
                1,
                "no dialable address means the edge stays on the daemon path"
            );
        }
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
