use dora_message::{
    config::InputMapping,
    descriptor::EnvValue,
    id::{DataId, NodeId, OperatorId},
};
use eyre::{Context, OptionExt, Result, bail};
use std::{
    collections::{BTreeMap, HashMap},
    env::consts::EXE_EXTENSION,
    path::{Component, Path, PathBuf},
    process::Command,
};

// reexport for compatibility
pub use dora_message::descriptor::{
    CoreNodeKind, CustomNode, DYNAMIC_SOURCE, Descriptor, Node, OperatorConfig, OperatorDefinition,
    OperatorSource, PythonSource, RUNTIME_PYTHON, RUNTIME_SHARED_LIBRARY, RUNTIME_WASM,
    ResolvedNode, RmwZenohCompatibility, Ros2BridgeConfig, Ros2Direction, Ros2QosConfig,
    Ros2TopicConfig, Ros2TransportConfig, RuntimeNode, SHELL_SOURCE, SingleOperatorDefinition,
};
pub use validate::ResolvedNodeExt;
pub use visualize::collect_dora_timers;

mod classify;
/// Lexically normalize a path (collapse `.` and resolve `..`) without touching
/// the filesystem — the executable may not be built yet when this is called.
///
/// Used to sanitize untrusted node paths before a containment check, so the
/// two callers (module expansion and manifest injection) must collapse `..`
/// identically; keeping a single implementation prevents them from drifting.
pub(crate) fn normalize_path(path: &Path) -> PathBuf {
    let mut out = PathBuf::new();
    for component in path.components() {
        match component {
            Component::CurDir => {}
            Component::ParentDir => {
                out.pop();
            }
            other => out.push(other),
        }
    }
    out
}

mod expand;
pub mod validate;
mod visualize;

pub use expand::{
    ExpandedDescriptor, ModuleBoundaries, check_module_file, expand_modules,
    expand_modules_with_boundaries,
};

/// Operations on a parsed [`Descriptor`].
///
/// These live on an extension trait rather than inherent methods because
/// [`Descriptor`] is defined in `dora-message` (the wire-format crate), while
/// the parsing, validation, resolution, and visualization logic belongs to
/// `dora-core`. Import the trait to turn a descriptor into the resolved node
/// map dora runs, validate it, or render it as a mermaid graph.
///
/// # Example
///
/// ```
/// use dora_core::descriptor::{Descriptor, DescriptorExt};
///
/// let yaml = b"
/// nodes:
///   - id: source
///     path: ./source
///     outputs: [data]
///   - id: sink
///     path: ./sink
///     inputs:
///       value: source/data
/// ";
///
/// // Parse the YAML into a descriptor...
/// let descriptor = Descriptor::parse(yaml.to_vec())?;
/// assert_eq!(descriptor.nodes.len(), 2);
///
/// // ...then resolve aliases and fill in defaults to get the node map dora runs.
/// let resolved = descriptor.resolve_aliases_and_set_defaults()?;
/// assert_eq!(resolved.len(), 2);
/// # Ok::<(), eyre::Report>(())
/// ```
pub trait DescriptorExt {
    /// Resolve the descriptor into the map of nodes dora runs.
    ///
    /// Fills in defaults, rewrites single-operator input references to the
    /// operator-qualified output name, and returns the nodes keyed by id. The
    /// descriptor must already be module-expanded (see [`expand`](Self::expand))
    /// if it contains `module:` references.
    fn resolve_aliases_and_set_defaults(&self) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>>;
    /// Render the resolved dataflow as a mermaid `flowchart`, annotated with the
    /// given module `boundaries` (from [`expand_with_boundaries`](Self::expand_with_boundaries)).
    fn visualize_as_mermaid_with_boundaries(
        &self,
        boundaries: &ModuleBoundaries,
    ) -> eyre::Result<String>;
    /// Apply a command-line override to the dataflow's completion policy.
    ///
    /// `None` leaves the descriptor's own `exit_when_nodes_finish`
    /// alone, so a setting written in the YAML stands; `Some(v)`
    /// overrides it in either direction, so `--exit-when-nodes-finish`
    /// can force the policy on and `=false` can force it off for a
    /// descriptor that asks for it (dora-rs/dora#2920).
    ///
    /// Shared rather than spelled out at each call site: `dora run`,
    /// `dora start` and the daemon all have to agree, and three copies
    /// of the same three lines is how one of them ends up not being
    /// updated.
    fn apply_exit_when_nodes_finish(&mut self, over: Option<bool>);

    /// Read a descriptor from a YAML file at `path` and [`parse`](Self::parse) it.
    fn blocking_read(path: &Path) -> eyre::Result<Descriptor>;
    /// Parse a descriptor from raw YAML bytes.
    ///
    /// This only deserializes; it does not expand modules or validate the
    /// dataflow — use [`check`](Self::check) for that.
    fn parse(buf: Vec<u8>) -> eyre::Result<Descriptor>;
    /// Expand modules and validate the whole dataflow (node ids, input wiring,
    /// field combinations, …), resolving relative paths against `working_dir`.
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

/// Prefixes a single-operator node's output id with the operator id
/// (e.g. `result` -> `op/result`) so downstream references resolve to the
/// operator-qualified output.
///
/// Operator ids are *not* validated against the `DataId` character set when a
/// descriptor is parsed (`OperatorId` stores its string verbatim), so build the
/// qualified id fallibly: `DataId::from` panics on characters outside
/// `[a-zA-Z0-9_./-]`, which would abort `dora check`/`graph`/`build` on an
/// otherwise-parseable descriptor. Returning an `Err` surfaces it as a clean
/// descriptor error instead.
fn prefix_output_with_operator_id(op_name: &OperatorId, output: &DataId) -> eyre::Result<DataId> {
    format!("{op_name}/{output}")
        .parse::<DataId>()
        .map_err(|e| {
            eyre::eyre!(
                "operator id `{op_name}` produces an invalid output id `{op_name}/{output}`: {e}"
            )
        })
}

/// Like [`DescriptorExt::resolve_aliases_and_set_defaults`], but resolves
/// `desc` as a node (or nodes) being added to an already-running dataflow
/// whose current node set is `topology_nodes`.
///
/// Whole-descriptor resolution rewrites an input that references a
/// single-`operator:` producer from the bare output name to the
/// operator-qualified one (`result` -> `op/result`). The dynamic-topology
/// `AddNode` path resolves the new node in isolation, so that producer is not
/// present in the descriptor being resolved and the rewrite is skipped —
/// leaving the added node subscribed to an output name nobody publishes
/// (silent data loss, #2877). Supplying the surrounding `topology_nodes` makes
/// those producers visible so the prefixing is applied.
///
/// `topology_nodes` contribute *only* to the single-operator output-prefixing
/// lookup — they are never themselves resolved or emitted, and nothing else on
/// the surrounding descriptor (`env`, `deploy`, …) is consulted. Callers that
/// want the running dataflow's `env` merged in must set it on `desc` (the
/// `AddNode` handler does, see #2919). `desc`'s own nodes take precedence in
/// the lookup, so a node id present in both resolves against the copy being
/// added.
pub fn resolve_aliases_and_set_defaults_in_topology(
    desc: &Descriptor,
    topology_nodes: &[Node],
) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>> {
    let default_op_id = OperatorId::from(SINGLE_OPERATOR_DEFAULT_ID.to_string());

    let single_operator_nodes: HashMap<_, _> = topology_nodes
        .iter()
        .chain(desc.nodes.iter())
        .filter_map(|n| {
            n.operator
                .as_ref()
                .map(|op| (&n.id, op.id.as_ref().unwrap_or(&default_op_id)))
        })
        .collect();

    let mut resolved = BTreeMap::new();
    for mut node in desc.nodes.clone() {
        // classify node: determine kind + validate fields against whitelist
        let node_class = classify::classify(&node)?;

        // adjust ROS2 bridge input mappings early
        if node.ros2.is_some() {
            for input in node.inputs.values_mut() {
                if let InputMapping::User(m) = &mut input.mapping
                    && let Some(op_name) = single_operator_nodes.get(&m.source).copied()
                {
                    m.output = prefix_output_with_operator_id(op_name, &m.output)?;
                }
            }
        }

        // adjust input mappings
        let input_mappings: Vec<_> = match &node_class {
            classify::NodeClass::Standard { .. } => node.inputs.values_mut().collect(),
            classify::NodeClass::Runtime => node
                .operators
                .as_mut()
                .ok_or_eyre("no operators")?
                .operators
                .iter_mut()
                .flat_map(|op| op.config.inputs.values_mut())
                .collect(),
            classify::NodeClass::Operator => node
                .operator
                .as_mut()
                .ok_or_eyre("no operator")?
                .config
                .inputs
                .values_mut()
                .collect(),
            classify::NodeClass::Ros2Bridge => vec![],
        };
        for mapping in input_mappings
            .into_iter()
            .filter_map(|i| match &mut i.mapping {
                InputMapping::Timer { .. } | InputMapping::Logs(_) => None,
                InputMapping::User(m) => Some(m),
            })
        {
            if let Some(op_name) = single_operator_nodes.get(&mapping.source).copied() {
                mapping.output = prefix_output_with_operator_id(op_name, &mapping.output)?;
            }
        }

        // resolve nodes. The two custom-node arms drain the custom-node keys
        // out of `node` with `CustomNode::from_node`; the node-level keys stay
        // behind for `ResolvedNode::from_node` below.
        let kind = match node_class {
            classify::NodeClass::Standard { source } => {
                let path = node.path.take().ok_or_eyre("missing `path` attribute")?;
                let mut custom = CustomNode::from_node(&mut node, path);
                custom.source = source;
                CoreNodeKind::Custom(custom)
            }
            classify::NodeClass::Runtime => {
                let runtime = node.operators.as_ref().ok_or_eyre("no operators")?;
                CoreNodeKind::Runtime(runtime.clone())
            }
            classify::NodeClass::Operator => {
                let op = node.operator.as_ref().ok_or_eyre("no operator")?;
                CoreNodeKind::Runtime(RuntimeNode {
                    operators: vec![OperatorDefinition {
                        id: op.id.clone().unwrap_or_else(|| default_op_id.clone()),
                        config: op.config.clone(),
                    }],
                })
            }
            classify::NodeClass::Ros2Bridge => {
                let config = node.ros2.as_ref().ok_or_eyre("no ros2")?;
                let bridge_config_json = serde_json::to_string(&config)
                    .context("failed to serialize ROS2 bridge config")?;

                let mut envs = BTreeMap::new();
                envs.insert(
                    "DORA_ROS2_BRIDGE_CONFIG".to_string(),
                    EnvValue::String(bridge_config_json),
                );

                // The bridge binary is fixed, so `path` is a constant and
                // `source` stays at its default. `path_sha256` and `build` are
                // not accepted on a `ros2:` node (classification rejects them),
                // so `from_node` finds them unset.
                let mut custom =
                    CustomNode::from_node(&mut node, "dora-ros2-bridge-node".to_string());
                custom.envs = Some(envs);
                CoreNodeKind::Custom(custom)
            }
        };

        if resolved.contains_key(&node.id) {
            eyre::bail!(
                "duplicate node ID `{}` — each node must have a unique `id`",
                node.id
            );
        }
        let mut resolved_node = ResolvedNode::from_node(node, kind);
        // Merge the dataflow-level `env` into the per-node `env`. Per-node keys
        // win on conflict so a node can override a shared default (e.g. global
        // `RUST_LOG=info` with one verbose node setting `RUST_LOG=debug`).
        resolved_node.env = merge_env(desc.env.as_ref(), resolved_node.env.take());
        resolved.insert(resolved_node.id.clone(), resolved_node);
    }

    Ok(resolved)
}

impl DescriptorExt for Descriptor {
    fn resolve_aliases_and_set_defaults(&self) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>> {
        resolve_aliases_and_set_defaults_in_topology(self, &[])
    }

    fn visualize_as_mermaid_with_boundaries(
        &self,
        boundaries: &ModuleBoundaries,
    ) -> eyre::Result<String> {
        let resolved = self.resolve_aliases_and_set_defaults()?;
        let flowchart = visualize::visualize_nodes_with_boundaries(&resolved, boundaries);
        Ok(flowchart)
    }

    fn apply_exit_when_nodes_finish(&mut self, over: Option<bool>) {
        if let Some(over) = over {
            self.exit_when_nodes_finish = Some(over);
        }
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
/// on conflict. Returns `None` when the merged result is empty so the resolved
/// node serializes cleanly (no empty `env: {}` map) whenever no env vars are
/// effectively set — regardless of whether the emptiness comes from the global
/// map, the node map, or both (e.g. a node that declares `env: {}` with no
/// dataflow-level env).
fn merge_env(
    global: Option<&BTreeMap<String, EnvValue>>,
    node: Option<BTreeMap<String, EnvValue>>,
) -> Option<BTreeMap<String, EnvValue>> {
    let merged = match (global, node) {
        (None, None) => return None,
        (None, Some(node)) => node,
        (Some(global), None) => global.clone(),
        (Some(global), Some(node)) => {
            let mut merged = global.clone();
            // Per-node entries override global ones on key conflict.
            merged.extend(node);
            merged
        }
    };
    // Normalize an empty result to `None` regardless of which side was empty.
    (!merged.is_empty()).then_some(merged)
}

pub async fn read_as_descriptor(path: &Path) -> eyre::Result<Descriptor> {
    let buf = tokio::fs::read(path)
        .await
        .context("failed to open given file")?;
    Descriptor::parse(buf)
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

/// Resolve a node's executable `source` to an absolute path.
///
/// An extensionless `source` gets the platform executable extension. The path
/// is then searched, in order:
///
/// 1. under `working_dir` (the dataflow's directory),
/// 2. the `uv`-managed environment, when `uv` is on the host,
/// 3. the ambient system `$PATH`.
///
/// The resolved path is absolutized but symlinks are **not** resolved (the
/// binary is spawned at the path it was found). Unlike
/// [`resolve_path_confined`], this has an ambient-`$PATH` fallback and does not
/// confine the result to any root — it is the resolution used for local,
/// trusted dataflow files, whereas confined resolution is for untrusted `hub:`
/// nodes.
pub fn resolve_path(source: &str, working_dir: &Path) -> Result<PathBuf> {
    let path = Path::new(&source);
    let path = if path.extension().is_none() {
        path.with_extension(EXE_EXTENSION)
    } else {
        path.to_owned()
    };

    // Search path within current working directory.
    let joined = working_dir.join(&path);
    if joined.exists() {
        absolutize_preserving_symlinks(&joined)
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

/// Make an existing executable path absolute WITHOUT following symlinks.
///
/// Deliberately not `canonicalize()`: that resolves symlinks, and a
/// virtualenv's `bin/python` is a symlink whose *location* is what
/// CPython uses to discover `pyvenv.cfg`. Resolving it before exec runs
/// the base interpreter with no venv, so imports that work in a shell
/// fail under dora (dora-rs/dora#2918). `path::absolute` only prepends
/// the cwd and drops `.` components; symlinks and `..` are left for the
/// kernel to resolve at exec time, which matches shell behavior.
///
/// Errors if the path does not exist (`exists()` traverses symlinks, so
/// a dangling link counts as missing — same outcome canonicalize gave).
/// Shared by every [`resolve_path`] branch so the no-symlink-resolution
/// contract cannot regress in one branch while the tests exercise
/// another.
fn absolutize_preserving_symlinks(path: &Path) -> Result<PathBuf> {
    if !path.exists() {
        bail!("path {} does not exist", path.display());
    }
    std::path::absolute(path)
        .with_context(|| format!("failed to make path {} absolute", path.display()))
}

/// Resolve `path` against the `uv`-managed environment by running
/// `uv run which <path>`, returning an absolute path.
///
/// Unlike a fire-and-forget spawn, this waits for the child, checks its
/// exit status (so a missing binary surfaces as an error), and verifies
/// the captured location exists — without resolving symlinks, which
/// would reintroduce the venv bypass fixed for the working-dir branch
/// (dora-rs/dora#2918).
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
    absolutize_preserving_symlinks(Path::new(resolved))
        .with_context(|| format!("uv-resolved path {resolved} is not usable"))
}

/// Classification of a [`Node`] by which of its mutually exclusive
/// implementation fields is set.
pub trait NodeExt {
    /// Determine the node's [`NodeKind`].
    ///
    /// A node must set **exactly one** of `path`, `operators`, `operator`,
    /// `ros2`, or `module`; this returns an error if none or more than one is
    /// set. A node carrying an unresolved `hub:` reference is also rejected —
    /// `hub:` is desugared into a concrete node earlier in the build, so
    /// reaching `kind` with one still present means resolution was skipped.
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
            &self.operator,
            &self.ros2,
            &self.module,
        ) {
            (None, None, None, None, None) => {
                eyre::bail!(
                    "node `{}` requires a `path`, `operators`, `ros2`, or `module` field",
                    self.id
                )
            }
            (None, None, Some(operator), None, None) => Ok(NodeKind::Operator(operator)),
            (None, Some(runtime), None, None, None) => Ok(NodeKind::Runtime(runtime)),
            (Some(path), None, None, None, None) => Ok(NodeKind::Standard(path)),
            (None, None, None, Some(ros2), None) => Ok(NodeKind::Ros2Bridge(ros2)),
            (None, None, None, None, Some(module)) => Ok(NodeKind::Module(module)),
            _ => {
                eyre::bail!(
                    "node `{}` has multiple exclusive fields set, only one of `path`, `operators`, `operator`, `ros2`, and `module` is allowed",
                    self.id
                )
            }
        }
    }
}

/// The implementation kind of a [`Node`], returned by [`NodeExt::kind`].
///
/// Each variant borrows the one exclusive field that was set on the node.
#[derive(Debug)]
pub enum NodeKind<'a> {
    /// A custom node run from an executable `path`.
    Standard(&'a String),
    /// A dora runtime node hosting one or more in-process `operators`.
    Runtime(&'a RuntimeNode),
    /// A node defined by a single inline `operator`.
    Operator(&'a SingleOperatorDefinition),
    /// A ROS2 bridge node.
    Ros2Bridge(&'a Ros2BridgeConfig),
    /// A `module` (sub-dataflow) reference — must be expanded before resolution.
    Module(&'a String),
}

#[cfg(test)]
mod tests {
    /// dora-rs/dora#2920: the command-line flag beats the descriptor in
    /// BOTH directions, and its absence beats neither.
    ///
    /// The third state matters: if "flag omitted" meant `false`, a YAML
    /// `exit_when_nodes_finish: true` would be overridden on every
    /// invocation, and since `dora run` and `dora start` are the only
    /// ways to start a dataflow, the descriptor field could never take
    /// effect at all.
    #[test]
    fn exit_when_nodes_finish_override_semantics() {
        use super::DescriptorExt;

        let parse = |yaml: &str| -> Descriptor { serde_yaml::from_str(yaml).expect("parse") };
        let with_setting = "exit_when_nodes_finish: true\nnodes:\n  - id: a\n    path: ./a\n";
        let without = "nodes:\n  - id: a\n    path: ./a\n";

        // Omitted: the descriptor decides, either way.
        let mut d = parse(with_setting);
        d.apply_exit_when_nodes_finish(None);
        assert_eq!(
            d.exit_when_nodes_finish,
            Some(true),
            "omitting the flag must not silently disable a policy the \
             dataflow file asked for"
        );

        let mut d = parse(without);
        d.apply_exit_when_nodes_finish(None);
        assert_eq!(d.exit_when_nodes_finish, None, "and must not invent one");

        // Given: it wins, including against an opposite descriptor value.
        let mut d = parse(with_setting);
        d.apply_exit_when_nodes_finish(Some(false));
        assert_eq!(
            d.exit_when_nodes_finish,
            Some(false),
            "`--exit-when-nodes-finish=false` must be able to turn OFF a \
             policy the dataflow file turned on"
        );

        let mut d = parse(without);
        d.apply_exit_when_nodes_finish(Some(true));
        assert_eq!(d.exit_when_nodes_finish, Some(true));
    }

    use super::*;
    use dora_message::descriptor::{GitRepoRev, NodeSource};
    use std::collections::BTreeSet;

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
    fn merge_env_normalizes_empty_node_map_to_none() {
        // A node that declares `env: {}` with no dataflow-level env must
        // resolve to `None`, not `Some({})`, matching the documented contract
        // (and the `(Some(empty), None)` arm) so the resolved node serializes
        // without an empty `env:` map.
        assert!(merge_env(None, Some(env(&[]))).is_none());
    }

    #[test]
    fn merge_env_normalizes_empty_global_and_node_maps_to_none() {
        assert!(merge_env(Some(&env(&[])), Some(env(&[]))).is_none());
        assert!(merge_env(Some(&env(&[])), None).is_none());
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

    fn resolved_input_mapping<'a>(
        resolved: &'a BTreeMap<NodeId, ResolvedNode>,
        node: &str,
        input: &str,
    ) -> &'a InputMapping {
        let node = resolved
            .get(&NodeId::from(node.to_string()))
            .expect("node resolved");
        let inputs = match &node.kind {
            CoreNodeKind::Custom(n) => &n.run_config.inputs,
            CoreNodeKind::Runtime(_) => panic!("expected custom node"),
        };
        &inputs
            .get(&DataId::from(input.to_string()))
            .expect("input present")
            .mapping
    }

    #[test]
    fn add_node_prefixes_single_operator_producer_input_via_topology() {
        // A node added to a running dataflow via `AddNode` is resolved against a
        // single-node descriptor. If its input references an existing
        // single-`operator:` producer, the `op/` output prefix that
        // whole-descriptor resolution applies must still be added — sourced
        // from the surrounding topology — or the node subscribes to an output
        // name nobody publishes and silently receives no data (#2877).
        let topology: Descriptor = serde_yaml::from_str(
            "\
nodes:
  - id: producer
    operator:
      python: producer.py
      outputs:
        - result
",
        )
        .expect("parse topology");

        let added: Descriptor = serde_yaml::from_str(
            "\
nodes:
  - id: consumer
    path: consumer
    inputs:
      reading: producer/result
",
        )
        .expect("parse added node");

        // With the topology the input is prefixed to the operator-qualified
        // output name the runtime actually publishes under (`op/result`).
        let resolved = resolve_aliases_and_set_defaults_in_topology(&added, &topology.nodes)
            .expect("resolve in topology");
        match resolved_input_mapping(&resolved, "consumer", "reading") {
            InputMapping::User(m) => {
                assert_eq!(m.source, NodeId::from("producer".to_string()));
                assert_eq!(m.output, DataId::from("op/result".to_string()));
            }
            other => panic!("expected user mapping, got {other:?}"),
        }

        // Contrast: with no topology the producer is not in scope at all, so
        // there is nothing to key the rewrite off and the name stays bare.
        // That is the correct answer for the inputs given — which is exactly
        // why the `AddNode` path had to stop resolving in isolation.
        let resolved_isolated = added
            .resolve_aliases_and_set_defaults()
            .expect("resolve isolated");
        match resolved_input_mapping(&resolved_isolated, "consumer", "reading") {
            InputMapping::User(m) => {
                assert_eq!(m.output, DataId::from("result".to_string()));
            }
            other => panic!("expected user mapping, got {other:?}"),
        }
    }

    #[test]
    fn topology_lookup_prefers_the_node_being_added_over_a_same_id_topology_entry() {
        // The lookup chains topology nodes *before* `desc`'s own, so a node id
        // present in both resolves against the copy being added rather than a
        // stale topology entry. Unreachable through `AddNode` today (duplicate
        // ids are rejected up front and `RemoveNode` prunes the stored
        // descriptor), but the precedence should not depend on that.
        let topology: Descriptor = serde_yaml::from_str(
            "\
nodes:
  - id: producer
    operator:
      id: stale
      python: producer.py
      outputs:
        - result
",
        )
        .expect("parse topology");

        let added: Descriptor = serde_yaml::from_str(
            "\
nodes:
  - id: producer
    operator:
      id: fresh
      python: producer.py
      outputs:
        - result
  - id: consumer
    path: consumer
    inputs:
      reading: producer/result
",
        )
        .expect("parse added nodes");

        let resolved = resolve_aliases_and_set_defaults_in_topology(&added, &topology.nodes)
            .expect("resolve in topology");
        match resolved_input_mapping(&resolved, "consumer", "reading") {
            InputMapping::User(m) => {
                assert_eq!(m.output, DataId::from("fresh/result".to_string()));
            }
            other => panic!("expected user mapping, got {other:?}"),
        }
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
    fn invalid_operator_id_prefix_errors_instead_of_panicking() {
        // A single-operator node whose `operator.id` contains a character
        // outside the `DataId` set (here a space) is accepted at parse time
        // (`OperatorId` is unvalidated). When another node references its
        // output, the resolver prefixes the output with the operator id. That
        // used to build the qualified id via `DataId::from`, which panics on
        // invalid characters and aborted `dora check`/`graph`/`build`. It must
        // now surface as a clean `Err` instead.
        let yaml = r#"
nodes:
  - id: producer
    operator:
      id: "bad id"
      python: op.py
      outputs: [result]
  - id: consumer
    path: ./consumer
    inputs:
      x: producer/result
"#;
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let result = desc.resolve_aliases_and_set_defaults();
        assert!(result.is_err(), "expected a clean descriptor error, got Ok");
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

    /// dora-rs/dora#2918: resolving a node path must NOT follow symlinks.
    ///
    /// A virtualenv's `bin/python` is a symlink to the base interpreter,
    /// and CPython's venv discovery hinges on that: it looks for
    /// `pyvenv.cfg` relative to the path it was *invoked* as, not the
    /// symlink's target. Canonicalizing before exec therefore runs the
    /// base interpreter with no venv — imports that work in a shell
    /// (`.venv/bin/python -c "import numpy"`) fail under dora with
    /// `ModuleNotFoundError`.
    #[test]
    #[cfg(unix)]
    fn resolve_path_preserves_symlinks() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let target = tmp.path().join("base-interpreter.bin");
        std::fs::write(&target, b"x").unwrap();
        let link = tmp.path().join("venv-python.bin");
        std::os::unix::fs::symlink(&target, &link).unwrap();

        // relative source resolved against the working dir
        let resolved = resolve_path("venv-python.bin", tmp.path()).unwrap();
        assert!(resolved.is_absolute());
        assert!(
            resolved.ends_with("venv-python.bin"),
            "resolve_path followed the symlink: {} — venv discovery \
             (pyvenv.cfg) is keyed off the symlink location, so execing \
             the target bypasses the venv",
            resolved.display()
        );

        // absolute source (the shape from the issue: `path: /…/.venv/bin/python`)
        let resolved = resolve_path(link.to_str().unwrap(), Path::new("/")).unwrap();
        assert!(
            resolved.ends_with("venv-python.bin"),
            "absolute symlink path was canonicalized: {}",
            resolved.display()
        );

        // `..` components survive too: they are resolved by the kernel at
        // exec time, AFTER any symlinked directories — which is the
        // shell-matching semantic. A lexical "cleanup" that collapses
        // them would resolve differently through symlinked dirs.
        std::fs::create_dir(tmp.path().join("sub")).unwrap();
        let resolved = resolve_path("sub/../venv-python.bin", tmp.path()).unwrap();
        assert!(
            resolved.ends_with("sub/../venv-python.bin"),
            "`..` was normalized away: {}",
            resolved.display()
        );
    }

    /// A dangling symlink is "missing": `exists()` traverses the link, so
    /// resolution falls through to the uv/$PATH branches and ultimately
    /// errors — the same outcome the old `canonicalize()` failure gave.
    /// Pins the branch boundary so a switch to `symlink_metadata()`
    /// (which would treat the dangling link as present and exec a
    /// guaranteed-ENOENT path) doesn't slip in silently.
    #[test]
    #[cfg(unix)]
    fn resolve_path_treats_dangling_symlink_as_missing() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let link = tmp.path().join("dangling-2918-regression.bin");
        std::os::unix::fs::symlink(tmp.path().join("no-such-target"), &link).unwrap();

        let result = resolve_path("dangling-2918-regression.bin", tmp.path());
        assert!(
            result.is_err(),
            "a dangling symlink must not resolve (nothing on uv/$PATH matches \
             this name either), got {result:?}"
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

    /// Every `CustomNode` field name, taken from its JSON schema.
    ///
    /// A field marked `#[schemars(skip)]` would never appear in `properties`
    /// and would slip past the carried-through tests below. `Node::deploy` is
    /// exactly such a field, and the classify test compensates with a
    /// hardcoded insert — do the same here if `CustomNode` ever gains one.
    fn custom_node_field_names() -> BTreeSet<String> {
        let schema = schemars::schema_for!(dora_message::descriptor::CustomNode);
        let schema = serde_json::to_value(schema).expect("schema should serialize");
        schema
            .pointer("/$defs/CustomNode/properties")
            .or_else(|| schema.pointer("/definitions/CustomNode/properties"))
            .or_else(|| schema.pointer("/properties"))
            .and_then(serde_json::Value::as_object)
            .expect("CustomNode schema should expose properties")
            .keys()
            .cloned()
            .collect()
    }

    /// The per-node keys every custom-node kind resolves identically, each set
    /// to a value that differs from `CustomNode::new`'s default — a YAML
    /// fragment to append to a `- id:` entry.
    const SHARED_CUSTOM_NODE_KEYS: &str = r#"
    args: --verbose
    send_stdout_as: stdout-topic
    send_logs_as: logs-topic
    min_log_level: debug
    max_log_size: 4MB
    max_rotated_files: 3
    restart_policy: always
    max_restarts: 7
    restart_delay: 1.5
    max_restart_delay: 9.5
    restart_window: 60.0
    health_check_timeout: 2.5
    finish_grace_secs: 3.5
    shared_memory_pool_size: 8MB
    inputs:
      tick: dora/timer/millis/100
    outputs:
      - out
    output_types:
      out: arrow.int32
    output_framing:
      out: arrow-ipc
    input_types:
      tick: arrow.uint64
"#;

    /// Resolve the single node in `yaml` and assert that every `CustomNode`
    /// key outside `kind_specific` arrived carrying the value the YAML
    /// declared — and that the YAML did set it to something other than
    /// `CustomNode::new`'s default, so the comparison is never a trivial
    /// `None == None`. Returns the resolved node for the kind-specific checks.
    ///
    /// `CustomNode::from_node` is a struct literal inside `dora-message`, so a
    /// key added to `CustomNode` is already a compile error there. This is the
    /// value-level half: it catches a key wired to the wrong `Node` field, or
    /// left at its default. When it fails for a newly added key, carry the key
    /// in `from_node` and give it a non-default value in
    /// `SHARED_CUSTOM_NODE_KEYS`.
    fn resolve_and_check_carried_through(yaml: &str, kind_specific: &[&str]) -> CustomNode {
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let declared = serde_json::to_value(&desc.nodes[0]).expect("serialize declared");
        let resolved = desc.resolve_aliases_and_set_defaults().expect("resolve");
        let node = resolved.values().next().expect("one node");
        let CoreNodeKind::Custom(custom) = &node.kind else {
            panic!("expected a custom node, got {:?}", node.kind);
        };

        let actual = serde_json::to_value(custom).expect("serialize resolved");
        // A sentinel `path` that no YAML here uses, so `path` itself is checked
        // like every other field rather than comparing equal to it.
        let default =
            serde_json::to_value(CustomNode::new("<unset>".to_owned())).expect("serialize default");

        for field in custom_node_field_names() {
            if kind_specific.contains(&field.as_str()) {
                continue;
            }
            assert_ne!(
                actual.get(&field),
                default.get(&field),
                "`{field}` is still at its `CustomNode::new` default after \
                 resolution — either the YAML does not set it (add it to \
                 `SHARED_CUSTOM_NODE_KEYS`) or the key is parsed and then \
                 dropped (carry it in `CustomNode::from_node`)."
            );
            assert_eq!(
                actual.get(&field),
                declared.get(&field),
                "`{field}` resolved to a different value than the YAML declared \
                 — `CustomNode::from_node` copies it from the wrong `Node` field."
            );
        }
        custom.clone()
    }

    /// A `path:` node: every shared key and the standard-only `path_sha256` /
    /// `build` arrive. `source` resolves to `Local`, which is already the
    /// default, and `envs` is set only by the ROS2-bridge arm — the two tests
    /// below cover those.
    #[test]
    fn every_custom_node_field_is_carried_through() {
        let yaml = format!(
            "nodes:\n  - id: full\n    path: ./full-node\n    path_sha256: abc123\n    \
             build: cargo build{SHARED_CUSTOM_NODE_KEYS}"
        );
        let custom = resolve_and_check_carried_through(&yaml, &["source", "envs"]);
        assert!(
            matches!(custom.source, NodeSource::Local),
            "{:?}",
            custom.source
        );
        assert!(custom.envs.is_none(), "{:?}", custom.envs);
    }

    /// The `source` a `git:` node classifies to must reach the resolved node:
    /// it is the one assignment in the standard arm that `from_node` does not
    /// cover, and dropping it would turn every git node into a local one.
    #[test]
    fn git_source_is_carried_through() {
        let yaml = format!(
            "nodes:\n  - id: full\n    path: node\n    path_sha256: abc123\n    \
             build: cargo build\n    git: https://github.com/example/node.git\n    \
             branch: main{SHARED_CUSTOM_NODE_KEYS}"
        );
        let custom = resolve_and_check_carried_through(&yaml, &["source", "envs"]);
        assert!(
            matches!(
                &custom.source,
                NodeSource::GitBranch { repo, rev: Some(GitRepoRev::Branch(branch)) }
                    if repo == "https://github.com/example/node.git" && branch == "main"
            ),
            "{:?}",
            custom.source
        );
        assert!(custom.envs.is_none(), "{:?}", custom.envs);
    }

    /// The ROS2-bridge arm: `path` is the bridge binary, `envs` carries the
    /// bridge config the binary reads at startup, and every shared key still
    /// arrives. `path_sha256` and `build` are rejected on a `ros2:` node by
    /// classification, so they must stay unset.
    #[test]
    fn ros2_bridge_node_is_carried_through() {
        let yaml = format!(
            "nodes:\n  - id: bridge\n    ros2:\n      topic: /odom\n      \
             message_type: nav_msgs/msg/Odometry\n      direction: subscribe\
             {SHARED_CUSTOM_NODE_KEYS}"
        );
        let custom = resolve_and_check_carried_through(
            &yaml,
            &["path", "source", "path_sha256", "build", "envs"],
        );
        assert_eq!(custom.path, "dora-ros2-bridge-node");
        assert!(
            matches!(custom.source, NodeSource::Local),
            "{:?}",
            custom.source
        );
        assert!(custom.path_sha256.is_none(), "{:?}", custom.path_sha256);
        assert!(custom.build.is_none(), "{:?}", custom.build);
        let envs = custom
            .envs
            .expect("the bridge is configured through its environment");
        let Some(EnvValue::String(config)) = envs.get("DORA_ROS2_BRIDGE_CONFIG") else {
            panic!("DORA_ROS2_BRIDGE_CONFIG missing from {envs:?}");
        };
        assert!(config.contains("/odom"), "{config}");
    }

    /// The node-level keys — the ones `ResolvedNode::from_node` consumes once
    /// `CustomNode::from_node` has drained the rest — must reach the resolved
    /// node. `from_node` is a struct literal inside `dora-message`, so a new
    /// `ResolvedNode` field is a compile error there; this checks the values,
    /// including that the dataflow-level `env` is merged in with the per-node
    /// key winning.
    #[test]
    fn node_level_keys_are_carried_through() {
        let yaml = r#"
env:
  RUST_LOG: info
  SHARED: global
nodes:
  - id: full
    name: Full Node
    description: Sets every node-level key
    path: ./full-node
    env:
      SHARED: per-node
    cpu_affinity: [0, 1]
    deploy:
      machine: gpu-box
"#;
        let desc: Descriptor = serde_yaml::from_str(yaml).expect("parse");
        let resolved = desc.resolve_aliases_and_set_defaults().expect("resolve");
        let node = resolved.values().next().expect("one node");

        assert_eq!(node.id.to_string(), "full");
        assert_eq!(node.name.as_deref(), Some("Full Node"));
        assert_eq!(
            node.description.as_deref(),
            Some("Sets every node-level key")
        );
        assert_eq!(
            node.env,
            Some(env(&[("RUST_LOG", "info"), ("SHARED", "per-node")]))
        );
        assert_eq!(node.cpu_affinity, Some(vec![0, 1]));
        assert_eq!(
            node.deploy.as_ref().and_then(|d| d.machine.as_deref()),
            Some("gpu-box")
        );
    }
}
