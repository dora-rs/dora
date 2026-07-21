use dora_message::{
    config::{Input, InputMapping, NodeRunConfig},
    descriptor::{EnvValue, GitRepoRev, NodeSource},
    id::{DataId, NodeId, OperatorId},
};
use eyre::{Context, OptionExt, Result, bail};
use std::{
    collections::{BTreeMap, HashMap},
    env::consts::EXE_EXTENSION,
    path::{Path, PathBuf},
    process::Command,
};

// reexport for compatibility
pub use dora_message::descriptor::{
    CoreNodeKind, CustomNode, DYNAMIC_SOURCE, Descriptor, Node, OperatorConfig, OperatorDefinition,
    OperatorSource, PythonSource, ResolvedNode, Ros2BridgeConfig, Ros2Direction, Ros2QosConfig,
    Ros2TopicConfig, RuntimeNode, SHELL_SOURCE, SingleOperatorDefinition,
};
pub use validate::ResolvedNodeExt;
pub use visualize::collect_dora_timers;

mod expand;
pub mod validate;
mod visualize;

pub use expand::{
    ExpandedDescriptor, ModuleBoundaries, check_module_file, expand_modules,
    expand_modules_with_boundaries,
};

pub trait DescriptorExt {
    fn resolve_aliases_and_set_defaults(&self) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>>;
    /// Number of subscriber input-links for the output `source_node/source_output`
    /// across the whole (resolved) dataflow — i.e. how many `(node, input)` pairs
    /// map to it, counting each operator input of a runtime node separately.
    ///
    /// This is the count of zenoh data-plane subscribers a producer must see wired
    /// up before it is safe to switch that output from the reliable daemon path to
    /// the direct zenoh path without dropping startup messages (see
    /// [`crate::topics::zenoh_input_ready_liveliness_topic`]). It is a global
    /// topology count, so it correctly includes subscribers on other daemons.
    ///
    /// Provided in terms of [`resolve_aliases_and_set_defaults`](Self::resolve_aliases_and_set_defaults).
    fn output_subscriber_count(
        &self,
        source_node: &NodeId,
        source_output: &DataId,
    ) -> eyre::Result<usize> {
        let resolved = self.resolve_aliases_and_set_defaults()?;
        let mut count = 0;
        for node in resolved.values() {
            // Dynamic nodes connect at arbitrary times (or never), so the startup
            // barrier must not wait for them: they receive over zenoh via normal
            // publisher/subscriber matching once they connect, and excluding them
            // cannot lose a message to an already-connected static subscriber.
            if node_is_dynamic(node) {
                continue;
            }
            let inputs: Vec<&Input> = match &node.kind {
                CoreNodeKind::Custom(n) => n.run_config.inputs.values().collect(),
                CoreNodeKind::Runtime(n) => n
                    .operators
                    .iter()
                    .flat_map(|op| op.config.inputs.values())
                    .collect(),
            };
            for input in inputs {
                if let InputMapping::User(m) = &input.mapping
                    && &m.source == source_node
                    && &m.output == source_output
                {
                    count += 1;
                }
            }
        }
        Ok(count)
    }
    fn visualize_as_mermaid_with_boundaries(
        &self,
        boundaries: &ModuleBoundaries,
    ) -> eyre::Result<String>;
    fn blocking_read(path: &Path) -> eyre::Result<Descriptor>;
    fn parse(buf: Vec<u8>) -> eyre::Result<Descriptor>;
    fn check(&self, working_dir: &Path) -> eyre::Result<()>;
    /// Expand all module references into flat nodes.
    ///
    /// Module nodes are replaced by the inner nodes defined in their module
    /// file. Internal IDs are prefixed with `{module_id}.` and input/output
    /// wiring is rewritten accordingly.
    fn expand(&self, working_dir: &Path) -> eyre::Result<Descriptor>;
    /// Like [`expand`](Self::expand) but also returns module boundary metadata
    /// for visualization.
    fn expand_with_boundaries(
        &self,
        working_dir: &Path,
    ) -> eyre::Result<(Descriptor, ModuleBoundaries)>;
}

pub const SINGLE_OPERATOR_DEFAULT_ID: &str = "op";

/// Whether a resolved node is a dynamic node (spawned/connected at runtime rather
/// than at dataflow launch). Mirrors the daemon's classification.
fn node_is_dynamic(node: &ResolvedNode) -> bool {
    matches!(&node.kind, CoreNodeKind::Custom(n)
        if matches!(n.source, NodeSource::Local) && n.path == DYNAMIC_SOURCE)
}

impl DescriptorExt for Descriptor {
    fn resolve_aliases_and_set_defaults(&self) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>> {
        let default_op_id = OperatorId::from(SINGLE_OPERATOR_DEFAULT_ID.to_string());

        let single_operator_nodes: HashMap<_, _> = self
            .nodes
            .iter()
            .filter_map(|n| {
                n.operator
                    .as_ref()
                    .map(|op| (&n.id, op.id.as_ref().unwrap_or(&default_op_id)))
            })
            .collect();

        let mut resolved = BTreeMap::new();
        for mut node in self.nodes.clone() {
            // adjust ROS2 bridge input mappings early (before node_kind borrows node)
            if node.ros2.is_some() {
                for input in node.inputs.values_mut() {
                    if let InputMapping::User(m) = &mut input.mapping
                        && let Some(op_name) = single_operator_nodes.get(&m.source).copied()
                    {
                        m.output = DataId::from(format!("{op_name}/{}", m.output));
                    }
                }
            }

            // adjust input mappings
            let mut node_kind = node_kind_mut(&mut node)?;
            let input_mappings: Vec<_> = match &mut node_kind {
                NodeKindMut::Standard { inputs, .. } => inputs.values_mut().collect(),
                NodeKindMut::Runtime(node) => node
                    .operators
                    .iter_mut()
                    .flat_map(|op| op.config.inputs.values_mut())
                    .collect(),
                NodeKindMut::Custom(node) => node.run_config.inputs.values_mut().collect(),
                NodeKindMut::Operator(operator) => operator.config.inputs.values_mut().collect(),
                NodeKindMut::Ros2Bridge(_) => vec![],
            };
            for mapping in input_mappings
                .into_iter()
                .filter_map(|i| match &mut i.mapping {
                    InputMapping::Timer { .. } | InputMapping::Logs(_) => None,
                    InputMapping::User(m) => Some(m),
                })
            {
                if let Some(op_name) = single_operator_nodes.get(&mapping.source).copied() {
                    mapping.output = DataId::from(format!("{op_name}/{}", mapping.output));
                }
            }

            // resolve nodes
            let kind = match node_kind {
                NodeKindMut::Standard {
                    path,
                    source,
                    inputs: _,
                } => CoreNodeKind::Custom(CustomNode {
                    path: path.clone(),
                    source,
                    path_sha256: node.path_sha256,
                    args: node.args,
                    build: node.build,
                    send_stdout_as: node.send_stdout_as,
                    send_logs_as: node.send_logs_as,
                    min_log_level: node.min_log_level,
                    max_log_size: node.max_log_size,
                    max_rotated_files: node.max_rotated_files,
                    run_config: NodeRunConfig {
                        inputs: node.inputs,
                        outputs: node.outputs,
                        output_types: node.output_types,
                        output_framing: node.output_framing,
                        input_types: node.input_types,
                        shared_memory_pool_size: node.shared_memory_pool_size,
                    },
                    envs: None,
                    restart_policy: node.restart_policy,
                    max_restarts: node.max_restarts,
                    restart_delay: node.restart_delay,
                    max_restart_delay: node.max_restart_delay,
                    restart_window: node.restart_window,
                    health_check_timeout: node.health_check_timeout,
                    finish_grace_secs: node.finish_grace_secs,
                }),
                NodeKindMut::Custom(node) => CoreNodeKind::Custom(node.clone()),
                NodeKindMut::Runtime(node) => CoreNodeKind::Runtime(node.clone()),
                NodeKindMut::Operator(op) => CoreNodeKind::Runtime(RuntimeNode {
                    operators: vec![OperatorDefinition {
                        id: op.id.clone().unwrap_or_else(|| default_op_id.clone()),
                        config: op.config.clone(),
                    }],
                }),
                NodeKindMut::Ros2Bridge(config) => {
                    let bridge_config_json = serde_json::to_string(&config)
                        .context("failed to serialize ROS2 bridge config")?;

                    let mut envs = BTreeMap::new();
                    envs.insert(
                        "DORA_ROS2_BRIDGE_CONFIG".to_string(),
                        EnvValue::String(bridge_config_json),
                    );

                    CoreNodeKind::Custom(CustomNode {
                        path: "dora-ros2-bridge-node".to_string(),
                        source: NodeSource::Local,
                        path_sha256: None,
                        args: node.args,
                        build: None,
                        send_stdout_as: node.send_stdout_as,
                        send_logs_as: node.send_logs_as,
                        min_log_level: node.min_log_level,
                        max_log_size: node.max_log_size,
                        max_rotated_files: node.max_rotated_files,
                        run_config: NodeRunConfig {
                            inputs: node.inputs,
                            outputs: node.outputs,
                            output_types: node.output_types,
                            output_framing: node.output_framing,
                            input_types: node.input_types,
                            shared_memory_pool_size: node.shared_memory_pool_size,
                        },
                        envs: Some(envs),
                        restart_policy: node.restart_policy,
                        max_restarts: node.max_restarts,
                        restart_delay: node.restart_delay,
                        max_restart_delay: node.max_restart_delay,
                        restart_window: node.restart_window,
                        health_check_timeout: node.health_check_timeout,
                        finish_grace_secs: node.finish_grace_secs,
                    })
                }
            };

            if resolved.contains_key(&node.id) {
                eyre::bail!(
                    "duplicate node ID `{}` — each node must have a unique `id`",
                    node.id
                );
            }
            resolved.insert(
                node.id.clone(),
                ResolvedNode {
                    id: node.id,
                    name: node.name,
                    description: node.description,
                    // Merge the dataflow-level `env` into the per-node `env`.
                    // Per-node keys win on conflict so a node can override a
                    // shared default (e.g. global `RUST_LOG=info` with one
                    // verbose node setting `RUST_LOG=debug`).
                    env: merge_env(self.env.as_ref(), node.env),
                    cpu_affinity: node.cpu_affinity,
                    deploy: node.deploy,
                    kind,
                },
            );
        }

        Ok(resolved)
    }

    fn visualize_as_mermaid_with_boundaries(
        &self,
        boundaries: &ModuleBoundaries,
    ) -> eyre::Result<String> {
        let resolved = self.resolve_aliases_and_set_defaults()?;
        let flowchart = visualize::visualize_nodes_with_boundaries(&resolved, boundaries);
        Ok(flowchart)
    }

    fn blocking_read(path: &Path) -> eyre::Result<Descriptor> {
        let buf = std::fs::read(path).context("failed to open given file")?;
        Descriptor::parse(buf)
    }

    fn parse(buf: Vec<u8>) -> eyre::Result<Descriptor> {
        serde_yaml::from_slice(&buf).context("failed to parse given descriptor")
    }

    fn check(&self, working_dir: &Path) -> eyre::Result<()> {
        let expanded = self.expand(working_dir)?;
        validate::check_dataflow(&expanded, working_dir)
            .wrap_err("Dataflow could not be validated.")
    }

    fn expand(&self, working_dir: &Path) -> eyre::Result<Descriptor> {
        expand::expand_modules(self, working_dir)
    }

    fn expand_with_boundaries(
        &self,
        working_dir: &Path,
    ) -> eyre::Result<(Descriptor, ModuleBoundaries)> {
        let expanded = expand::expand_modules_with_boundaries(self, working_dir)?;
        Ok((expanded.descriptor, expanded.boundaries))
    }
}

/// Merge dataflow-level `env` into a node's `env`, with per-node keys winning
/// on conflict. Returns `None` when both inputs are empty so the resolved
/// node serializes cleanly when no env vars are set anywhere.
fn merge_env(
    global: Option<&BTreeMap<String, EnvValue>>,
    node: Option<BTreeMap<String, EnvValue>>,
) -> Option<BTreeMap<String, EnvValue>> {
    match (global, node) {
        (None, node) => node,
        (Some(global), None) if global.is_empty() => None,
        (Some(global), None) => Some(global.clone()),
        (Some(global), Some(node)) => {
            let mut merged = global.clone();
            // Per-node entries override global ones on key conflict.
            merged.extend(node);
            Some(merged)
        }
    }
}

pub async fn read_as_descriptor(path: &Path) -> eyre::Result<Descriptor> {
    let buf = tokio::fs::read(path)
        .await
        .context("failed to open given file")?;
    Descriptor::parse(buf)
}

fn node_kind_mut(node: &mut Node) -> eyre::Result<NodeKindMut<'_>> {
    match node.kind()? {
        NodeKind::Module(_) => {
            eyre::bail!(
                "module node `{}` must be expanded before resolution — \
                 call expand_modules() first",
                node.id
            )
        }
        NodeKind::Standard(_) => {
            let source = match (&node.git, &node.branch, &node.tag, &node.rev) {
                (None, None, None, None) => NodeSource::Local,
                (Some(repo), branch, tag, rev) => {
                    let rev = match (branch, tag, rev) {
                        (None, None, None) => None,
                        (Some(branch), None, None) => Some(GitRepoRev::Branch(branch.clone())),
                        (None, Some(tag), None) => Some(GitRepoRev::Tag(tag.clone())),
                        (None, None, Some(rev)) => Some(GitRepoRev::Rev(rev.clone())),
                        other @ (_, _, _) => {
                            eyre::bail!(
                                "only one of `branch`, `tag`, and `rev` are allowed (got {other:?})"
                            )
                        }
                    };
                    NodeSource::GitBranch {
                        repo: repo.clone(),
                        rev,
                    }
                }
                (None, _, _, _) => {
                    eyre::bail!("`git` source required when using branch, tag, or rev")
                }
            };

            Ok(NodeKindMut::Standard {
                path: node.path.as_ref().ok_or_eyre("missing `path` attribute")?,
                source,
                inputs: &mut node.inputs,
            })
        }
        NodeKind::Runtime(_) => node
            .operators
            .as_mut()
            .map(NodeKindMut::Runtime)
            .ok_or_eyre("no operators"),
        NodeKind::Custom(_) => node
            .custom
            .as_mut()
            .map(NodeKindMut::Custom)
            .ok_or_eyre("no custom"),
        NodeKind::Operator(_) => node
            .operator
            .as_mut()
            .map(NodeKindMut::Operator)
            .ok_or_eyre("no operator"),
        NodeKind::Ros2Bridge(_) => node
            .ros2
            .as_ref()
            .map(NodeKindMut::Ros2Bridge)
            .ok_or_eyre("no ros2"),
    }
}

/// Returns `true` if `source` is an `http://` or `https://` URL.
///
/// This is the trust boundary that decides whether a node `path` is fetched as
/// a remote download (and, for hub artifacts, checksum-verified) versus
/// resolved as a local filesystem path. The match is on the literal scheme
/// prefix and is **case-sensitive**: an upper-cased scheme such as `HTTPS://`
/// is treated as a path, not a URL. Schemes other than HTTP(S) (e.g. `ftp://`,
/// `s3://`, `file://`) are likewise not considered URLs here.
///
/// ```
/// use dora_core::descriptor::source_is_url;
///
/// assert!(source_is_url("https://example.com/node"));
/// assert!(source_is_url("http://example.com/node"));
///
/// assert!(!source_is_url("./build/my_node"));
/// assert!(!source_is_url("/usr/bin/my_node"));
/// assert!(!source_is_url("s3://bucket/key"));
/// assert!(!source_is_url("HTTPS://example.com/node")); // case-sensitive
/// ```
pub fn source_is_url(source: &str) -> bool {
    source.starts_with("https://") || source.starts_with("http://")
}

pub fn resolve_path(source: &str, working_dir: &Path) -> Result<PathBuf> {
    let path = Path::new(&source);
    let path = if path.extension().is_none() {
        path.with_extension(EXE_EXTENSION)
    } else {
        path.to_owned()
    };

    // Search path within current working directory
    if let Ok(abs_path) = working_dir.join(&path).canonicalize() {
        Ok(abs_path)
    // Otherwise resolve against the `uv`-managed environment first (when `uv`
    // is available), then fall back to the system `$PATH`.
    } else if which::which("uv").is_ok() {
        resolve_path_via_uv(&path)
    } else if let Ok(abs_path) = which::which(&path) {
        Ok(abs_path)
    } else {
        bail!("Could not find source path {}", path.display())
    }
}

/// Resolve a hub node's entrypoint under **confined** rules (spec §11): the
/// executable may only come from the node's own working directory
/// (`<clone>/<subdir>`) or its managed Python environment. There is no
/// ambient-`$PATH` fallback — a typo or missing console script fails loudly
/// instead of silently running a host binary — and the resolved path is
/// canonicalized and checked to stay inside those roots, so a symlink
/// escaping the working dir is rejected rather than followed.
pub fn resolve_path_confined(
    source: &str,
    working_dir: &Path,
    python_env_dir: Option<&Path>,
) -> Result<PathBuf> {
    let path = Path::new(&source);
    let path = if path.extension().is_none() {
        path.with_extension(EXE_EXTENSION)
    } else {
        path.to_owned()
    };

    // a console script installed into the node's managed env
    if let Some(env_dir) = python_env_dir {
        let bin_dir = env_dir.join(if cfg!(windows) { "Scripts" } else { "bin" });
        let candidate = bin_dir.join(&path);
        if candidate.is_file() {
            return confine(&candidate, &bin_dir);
        }
    }

    // a file within the node's working dir (e.g. target/release/<bin>)
    let candidate = working_dir.join(&path);
    if candidate.exists() {
        return confine(&candidate, working_dir);
    }

    bail!(
        "could not find `{}` in the node's working directory `{}`{} — \
         hub nodes resolve only within their own package (no $PATH fallback)",
        path.display(),
        working_dir.display(),
        python_env_dir
            .map(|env| format!(" or its managed environment `{}`", env.display()))
            .unwrap_or_default(),
    )
}

/// Canonicalize `candidate` and require it to stay under `root`.
fn confine(candidate: &Path, root: &Path) -> Result<PathBuf> {
    let resolved = candidate
        .canonicalize()
        .with_context(|| format!("failed to canonicalize `{}`", candidate.display()))?;
    let root = root
        .canonicalize()
        .with_context(|| format!("failed to canonicalize `{}`", root.display()))?;
    if !resolved.starts_with(&root) {
        bail!(
            "entrypoint `{}` resolves outside the node's directory `{}` \
             (symlink escape?) — refusing to run it",
            resolved.display(),
            root.display()
        );
    }
    Ok(resolved)
}

/// Resolve `path` against the `uv`-managed environment by running
/// `uv run which <path>`, returning an absolute path.
///
/// Unlike a fire-and-forget spawn, this waits for the child, checks its
/// exit status (so a missing binary surfaces as an error), and canonicalizes
/// the captured location so the result matches the absolute-path contract of
/// the other [`resolve_path`] branches.
fn resolve_path_via_uv(path: &Path) -> Result<PathBuf> {
    let which = if cfg!(windows) { "where" } else { "which" };
    let output = Command::new("uv")
        .arg("run")
        .arg(which)
        .arg(path)
        .output()
        .with_context(|| format!("failed to run `uv run {which}`"))?;
    if !output.status.success() {
        bail!("Could not find source path {} within uv", path.display());
    }
    // `which`/`where` may emit multiple matches; the first line is the
    // resolved binary.
    let stdout = String::from_utf8_lossy(&output.stdout);
    let resolved = stdout
        .lines()
        .map(str::trim)
        .find(|line| !line.is_empty())
        .ok_or_else(|| eyre::eyre!("`uv run {which} {}` produced no output", path.display()))?;
    PathBuf::from(resolved)
        .canonicalize()
        .with_context(|| format!("failed to canonicalize uv-resolved path {resolved}"))
}

pub trait NodeExt {
    fn kind(&self) -> eyre::Result<NodeKind<'_>>;
}

impl NodeExt for Node {
    fn kind(&self) -> eyre::Result<NodeKind<'_>> {
        if self.hub.is_some() && self.path.is_none() {
            // `hub:` is desugared into a concrete git node by `dora build` /
            // `dora run` / `dora validate` before any kind dispatch — an
            // unresolved reference reaching this point means a flow that
            // skipped resolution
            eyre::bail!(
                "node `{}` uses an unresolved `hub:` reference — run `dora build` \
                 first (`dora start` requires a prior build for hub nodes)",
                self.id
            );
        }
        match (
            &self.path,
            &self.operators,
            &self.custom,
            &self.operator,
            &self.ros2,
            &self.module,
        ) {
            (None, None, None, None, None, None) => {
                eyre::bail!(
                    "node `{}` requires a `path`, `custom`, `operators`, `ros2`, or `module` field",
                    self.id
                )
            }
            (None, None, None, Some(operator), None, None) => Ok(NodeKind::Operator(operator)),
            (None, None, Some(custom), None, None, None) => Ok(NodeKind::Custom(custom)),
            (None, Some(runtime), None, None, None, None) => Ok(NodeKind::Runtime(runtime)),
            (Some(path), None, None, None, None, None) => Ok(NodeKind::Standard(path)),
            (None, None, None, None, Some(ros2), None) => Ok(NodeKind::Ros2Bridge(ros2)),
            (None, None, None, None, None, Some(module)) => Ok(NodeKind::Module(module)),
            _ => {
                eyre::bail!(
                    "node `{}` has multiple exclusive fields set, only one of `path`, `custom`, `operators`, `operator`, `ros2`, and `module` is allowed",
                    self.id
                )
            }
        }
    }
}

#[derive(Debug)]
pub enum NodeKind<'a> {
    Standard(&'a String),
    /// Dora runtime node
    Runtime(&'a RuntimeNode),
    Custom(&'a CustomNode),
    Operator(&'a SingleOperatorDefinition),
    /// ROS2 bridge node
    Ros2Bridge(&'a Ros2BridgeConfig),
    /// Module (sub-dataflow) reference — must be expanded before resolution
    Module(&'a String),
}

#[derive(Debug)]
enum NodeKindMut<'a> {
    Standard {
        path: &'a String,
        source: NodeSource,
        inputs: &'a mut BTreeMap<DataId, Input>,
    },
    /// Dora runtime node
    Runtime(&'a mut RuntimeNode),
    Custom(&'a mut CustomNode),
    Operator(&'a mut SingleOperatorDefinition),
    /// ROS2 bridge node
    Ros2Bridge(&'a Ros2BridgeConfig),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn env(pairs: &[(&str, &str)]) -> BTreeMap<String, EnvValue> {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), EnvValue::String(v.to_string())))
            .collect()
    }

    #[test]
    fn merge_env_returns_none_when_both_absent() {
        assert!(merge_env(None, None).is_none());
    }

    #[test]
    fn merge_env_keeps_per_node_when_no_global() {
        let node_env = env(&[("A", "1")]);
        let merged = merge_env(None, Some(node_env.clone())).unwrap();
        assert_eq!(merged, node_env);
    }

    #[test]
    fn merge_env_keeps_global_when_no_per_node() {
        let global = env(&[("A", "1")]);
        let merged = merge_env(Some(&global), None).unwrap();
        assert_eq!(merged, global);
    }

    #[test]
    fn merge_env_per_node_overrides_global_on_conflict() {
        let global = env(&[("A", "global"), ("B", "global")]);
        let node_env = env(&[("A", "node"), ("C", "node")]);
        let merged = merge_env(Some(&global), Some(node_env)).unwrap();
        assert_eq!(merged.get("A"), Some(&EnvValue::String("node".into())));
        assert_eq!(merged.get("B"), Some(&EnvValue::String("global".into())));
        assert_eq!(merged.get("C"), Some(&EnvValue::String("node".into())));
    }

    #[test]
    fn output_subscriber_count_counts_fanout_and_ignores_non_subscribers() {
        // `source/value` fans out to `transform` and `recorder` (2); the
        // `transform/doubled` output feeds only `sink` (1). Timer inputs and
        // subscriptions to other outputs must not be counted. This count is what
        // a producer waits for before switching an output to the direct zenoh
        // data plane, so an undercount here would drop startup messages.
        let yaml = r#"
nodes:
  - id: source
    path: source
    inputs:
      tick: dora/timer/millis/50
    outputs:
      - value
  - id: transform
    path: transform
    inputs:
      value: source/value
    outputs:
      - doubled
  - id: recorder
    path: recorder
    inputs:
      rec_in: source/value
  - id: sink
    path: sink
    inputs:
      doubled: transform/doubled
"#;
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let source = NodeId::from("source".to_string());
        let transform = NodeId::from("transform".to_string());
        let sink = NodeId::from("sink".to_string());
        let value = DataId::from("value".to_string());
        let doubled = DataId::from("doubled".to_string());
        let missing = DataId::from("nonexistent".to_string());

        assert_eq!(desc.output_subscriber_count(&source, &value).unwrap(), 2);
        assert_eq!(
            desc.output_subscriber_count(&transform, &doubled).unwrap(),
            1
        );
        // Output nobody subscribes to.
        assert_eq!(desc.output_subscriber_count(&source, &missing).unwrap(), 0);
        // `sink` produces nothing.
        assert_eq!(desc.output_subscriber_count(&sink, &value).unwrap(), 0);
    }

    #[test]
    fn output_subscriber_count_excludes_dynamic_subscribers() {
        // A dynamic subscriber connects at runtime, so it must not be counted in
        // the startup barrier — otherwise the producer would stall on the daemon
        // path waiting for a node that may connect late or never.
        let yaml = r#"
nodes:
  - id: source
    path: source
    outputs:
      - value
  - id: static_sub
    path: static_sub
    inputs:
      v: source/value
  - id: dyn_sub
    path: dynamic
    inputs:
      v: source/value
"#;
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let source = NodeId::from("source".to_string());
        let value = DataId::from("value".to_string());
        assert_eq!(desc.output_subscriber_count(&source, &value).unwrap(), 1);
    }

    #[test]
    fn descriptor_global_env_parses_from_yaml() {
        // Verify the new top-level `env:` field parses and the resolver
        // hands merged envs to every node with per-node keys winning.
        let yaml = r#"
env:
  RUST_LOG: info
  OTEL_ENDPOINT: http://collector:4317
nodes:
  - id: a
    path: ./a
    env:
      RUST_LOG: debug
  - id: b
    path: ./b
"#;
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let resolved = desc.resolve_aliases_and_set_defaults().expect("resolve");

        let a = resolved.get(&NodeId::from("a".to_string())).unwrap();
        let a_env = a.env.as_ref().expect("node a inherits env");
        // Per-node RUST_LOG=debug wins over global RUST_LOG=info.
        assert_eq!(
            a_env.get("RUST_LOG"),
            Some(&EnvValue::String("debug".into()))
        );
        // Global key not overridden on node a is still visible.
        assert_eq!(
            a_env.get("OTEL_ENDPOINT"),
            Some(&EnvValue::String("http://collector:4317".into()))
        );

        let b = resolved.get(&NodeId::from("b".to_string())).unwrap();
        let b_env = b.env.as_ref().expect("node b inherits global env");
        assert_eq!(
            b_env.get("RUST_LOG"),
            Some(&EnvValue::String("info".into()))
        );
        assert_eq!(
            b_env.get("OTEL_ENDPOINT"),
            Some(&EnvValue::String("http://collector:4317".into()))
        );
    }

    #[test]
    fn resolve_path_errors_for_nonexistent_binary() {
        // Regression for #2016: the `uv` fallback previously spawned
        // `uv run which <path>` fire-and-forget and returned the original
        // (relative) path even when the binary did not exist. A missing
        // binary must surface as an `Err`, and any successful resolution
        // must be an absolute path.
        let working_dir = std::env::current_dir().expect("cwd");
        let result = resolve_path("dora_nonexistent_binary_2016_regression", &working_dir);
        assert!(
            result.is_err(),
            "expected Err for a binary that exists nowhere, got {result:?}"
        );
    }

    #[test]
    fn resolve_path_confined_has_no_path_fallback() {
        let tmp = tempfile::tempdir().expect("tempdir");
        // `sh` exists on $PATH everywhere on unix — confined resolution must
        // NOT find it (spec §11: a typo or missing console script fails, it
        // never silently runs a host binary)
        let result = resolve_path_confined("sh", tmp.path(), None);
        assert!(result.is_err(), "expected Err, got {result:?}");

        // a real file in the working dir resolves
        let exe = if cfg!(windows) {
            "node.exe"
        } else {
            "node.bin"
        };
        std::fs::write(tmp.path().join(exe), b"x").unwrap();
        let resolved = resolve_path_confined(exe, tmp.path(), None).unwrap();
        assert!(resolved.is_absolute());

        // a managed-env console script resolves through the env's bin dir
        let env_dir = tmp.path().join("env");
        let bin_dir = env_dir.join(if cfg!(windows) { "Scripts" } else { "bin" });
        std::fs::create_dir_all(&bin_dir).unwrap();
        std::fs::write(bin_dir.join(exe), b"x").unwrap();
        let resolved =
            resolve_path_confined(exe, tmp.path().join("empty").as_path(), Some(&env_dir));
        assert!(resolved.is_ok(), "{resolved:?}");
    }

    #[cfg(unix)]
    #[test]
    fn resolve_path_confined_rejects_symlink_escape() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let working_dir = tmp.path().join("work");
        std::fs::create_dir_all(&working_dir).unwrap();
        let outside = tmp.path().join("outside.bin");
        std::fs::write(&outside, b"x").unwrap();
        std::os::unix::fs::symlink(&outside, working_dir.join("escape.bin")).unwrap();
        let result = resolve_path_confined("escape.bin", &working_dir, None);
        assert!(
            result.is_err(),
            "a symlink pointing outside the working dir must be rejected, got {result:?}"
        );
        let msg = format!("{:#}", result.unwrap_err());
        assert!(msg.contains("outside"), "{msg}");
    }

    #[test]
    fn unresolved_hub_node_has_clear_kind_error() {
        let node: Node = serde_yaml::from_str("id: x\nhub: dora-yolo@^0.5\n").unwrap();
        let err = node.kind().unwrap_err();
        assert!(format!("{err}").contains("dora build"), "{err}");
    }

    #[test]
    fn resolve_path_via_uv_errors_for_nonexistent_binary() {
        // Pins the #2016 root cause directly on the `uv` branch. The buggy
        // code spawned `uv run which <path>` fire-and-forget and returned
        // `Ok(<relative path>)` regardless of the child's exit status, so a
        // missing binary was silently accepted. This branch only runs when
        // `uv` is installed (the only environment where the bug manifested),
        // so guard on its presence to keep the test meaningful where it can
        // actually discriminate the fix.
        if which::which("uv").is_err() {
            return;
        }
        let path = Path::new("dora_nonexistent_binary_2016_regression");
        let result = resolve_path_via_uv(path);
        assert!(
            result.is_err(),
            "expected Err from `uv run which` for a missing binary, got {result:?}"
        );
    }

    #[test]
    fn descriptor_without_global_env_preserves_per_node_env() {
        // Regression guard: no top-level `env:` must leave per-node env
        // semantics unchanged.
        let yaml = r#"
nodes:
  - id: a
    path: ./a
    env:
      FOO: bar
  - id: b
    path: ./b
"#;
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let resolved = desc.resolve_aliases_and_set_defaults().expect("resolve");
        let a = resolved.get(&NodeId::from("a".to_string())).unwrap();
        assert_eq!(
            a.env.as_ref().and_then(|e| e.get("FOO")),
            Some(&EnvValue::String("bar".into()))
        );
        let b = resolved.get(&NodeId::from("b".to_string())).unwrap();
        assert!(b.env.is_none(), "node b has no env anywhere");
    }

    #[test]
    fn duplicate_node_id_is_rejected() {
        // Regression for #2393: a plain dataflow with two nodes sharing the
        // same `id` must return an error instead of silently discarding one.
        let yaml = r#"
nodes:
  - id: my-node
    path: ./a
  - id: my-node
    path: ./b
"#;
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let err = desc
            .resolve_aliases_and_set_defaults()
            .expect_err("duplicate node ID must be rejected");
        let msg = format!("{err:#}");
        assert!(
            msg.contains("duplicate node ID") && msg.contains("my-node"),
            "unexpected error message: {msg}"
        );
    }
}
