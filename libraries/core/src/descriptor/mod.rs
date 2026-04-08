use adora_message::{
    config::{Input, InputMapping, NodeRunConfig},
    descriptor::{EnvValue, GitRepoRev, NodeSource},
    id::{DataId, NodeId, OperatorId},
};
use eyre::{Context, OptionExt, Result, bail};
use std::{
    collections::{BTreeMap, HashMap},
    env::consts::EXE_EXTENSION,
    path::{Path, PathBuf},
    process::Stdio,
};
use tokio::process::Command;

// reexport for compatibility
pub use adora_message::descriptor::{
    CoreNodeKind, CustomNode, DYNAMIC_SOURCE, Descriptor, Node, OperatorConfig, OperatorDefinition,
    OperatorSource, PythonSource, ResolvedNode, Ros2BridgeConfig, Ros2Direction, Ros2QosConfig,
    Ros2TopicConfig, RuntimeNode, SHELL_SOURCE, SingleOperatorDefinition,
};
pub use validate::ResolvedNodeExt;
pub use visualize::collect_adora_timers;

mod expand;
pub mod validate;
mod visualize;

pub use expand::{
    ExpandedDescriptor, ModuleBoundaries, check_module_file, expand_modules,
    expand_modules_with_boundaries,
};

pub trait DescriptorExt {
    fn resolve_aliases_and_set_defaults(&self) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>>;
    fn visualize_as_mermaid(&self) -> eyre::Result<String>;
    fn visualize_as_mermaid_with_boundaries(
        &self,
        boundaries: &ModuleBoundaries,
    ) -> eyre::Result<String>;
    fn blocking_read(path: &Path) -> eyre::Result<Descriptor>;
    fn parse(buf: Vec<u8>) -> eyre::Result<Descriptor>;
    fn check(&self, working_dir: &Path) -> eyre::Result<()>;
    fn check_in_daemon(&self, working_dir: &Path, coordinator_is_remote: bool) -> eyre::Result<()>;
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
                    },
                    envs: None,
                    restart_policy: node.restart_policy,
                    max_restarts: node.max_restarts,
                    restart_delay: node.restart_delay,
                    max_restart_delay: node.max_restart_delay,
                    restart_window: node.restart_window,
                    health_check_timeout: node.health_check_timeout,
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
                        "ADORA_ROS2_BRIDGE_CONFIG".to_string(),
                        EnvValue::String(bridge_config_json),
                    );

                    CoreNodeKind::Custom(CustomNode {
                        path: "adora-ros2-bridge-node".to_string(),
                        source: NodeSource::Local,
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
                        },
                        envs: Some(envs),
                        restart_policy: node.restart_policy,
                        max_restarts: node.max_restarts,
                        restart_delay: node.restart_delay,
                        max_restart_delay: node.max_restart_delay,
                        restart_window: node.restart_window,
                        health_check_timeout: node.health_check_timeout,
                    })
                }
            };

            resolved.insert(
                node.id.clone(),
                ResolvedNode {
                    id: node.id,
                    name: node.name,
                    description: node.description,
                    env: node.env,
                    cpu_affinity: node.cpu_affinity,
                    deploy: node.deploy,
                    kind,
                },
            );
        }

        Ok(resolved)
    }

    fn visualize_as_mermaid(&self) -> eyre::Result<String> {
        let resolved = self.resolve_aliases_and_set_defaults()?;
        let flowchart = visualize::visualize_nodes(&resolved);
        Ok(flowchart)
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
        validate::check_dataflow(&expanded, working_dir, None, false)
            .wrap_err("Dataflow could not be validated.")
    }

    fn check_in_daemon(&self, working_dir: &Path, coordinator_is_remote: bool) -> eyre::Result<()> {
        let expanded = self.expand(working_dir)?;
        validate::check_dataflow(&expanded, working_dir, None, coordinator_is_remote)
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
    // Search path within $PATH
    } else if which::which("uv").is_ok() {
        // spawn: uv run which <path>
        let which = if cfg!(windows) { "where" } else { "which" };
        let _output = Command::new("uv")
            .arg("run")
            .arg(which)
            .arg(&path)
            .stdout(Stdio::null())
            .spawn()
            .context("Could not find binary within uv")?;
        Ok(path)
    } else if let Ok(abs_path) = which::which(&path) {
        Ok(abs_path)
    } else {
        bail!("Could not find source path {}", path.display())
    }
}

pub trait NodeExt {
    fn kind(&self) -> eyre::Result<NodeKind<'_>>;
}

impl NodeExt for Node {
    fn kind(&self) -> eyre::Result<NodeKind<'_>> {
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
    /// Adora runtime node
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
    /// Adora runtime node
    Runtime(&'a mut RuntimeNode),
    Custom(&'a mut CustomNode),
    Operator(&'a mut SingleOperatorDefinition),
    /// ROS2 bridge node
    Ros2Bridge(&'a Ros2BridgeConfig),
}
