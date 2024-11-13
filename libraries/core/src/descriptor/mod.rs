use dora_message::{
    config::{Input, InputMapping, NodeRunConfig},
    id::{DataId, OperatorId},
};
use eyre::{bail, Context, OptionExt, Result};
use std::{
    collections::{BTreeMap, HashMap},
    env::consts::EXE_EXTENSION,
    path::{Path, PathBuf},
};

// reexport for compatibility
pub use dora_message::descriptor::{
    CoreNodeKind, CustomNode, Descriptor, Node, OperatorConfig, OperatorDefinition, OperatorSource,
    PythonSource, ResolvedDeploy, ResolvedNode, RuntimeNode, SingleOperatorDefinition,
    DYNAMIC_SOURCE, SHELL_SOURCE,
};
pub use validate::ResolvedNodeExt;
pub use visualize::collect_dora_timers;

mod validate;
mod visualize;

pub trait DescriptorExt {
    fn resolve_aliases_and_set_defaults(&self) -> eyre::Result<Vec<ResolvedNode>>;
    fn visualize_as_mermaid(&self) -> eyre::Result<String>;
    fn blocking_read(path: &Path) -> eyre::Result<Descriptor>;
    fn parse(buf: Vec<u8>) -> eyre::Result<Descriptor>;
    fn check(&self, working_dir: &Path) -> eyre::Result<()>;
    fn check_in_daemon(
        &self,
        working_dir: &Path,
        remote_machine_id: &[&str],
        coordinator_is_remote: bool,
    ) -> eyre::Result<()>;
}

pub const SINGLE_OPERATOR_DEFAULT_ID: &str = "op";

impl DescriptorExt for Descriptor {
    fn resolve_aliases_and_set_defaults(&self) -> eyre::Result<Vec<ResolvedNode>> {
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

        let mut resolved = vec![];
        for mut node in self.nodes.clone() {
            // adjust input mappings
            let mut node_kind = node_kind_mut(&mut node)?;
            let input_mappings: Vec<_> = match &mut node_kind {
                NodeKindMut::Standard { path: _, inputs } => inputs.values_mut().collect(),
                NodeKindMut::Runtime(node) => node
                    .operators
                    .iter_mut()
                    .flat_map(|op| op.config.inputs.values_mut())
                    .collect(),
                NodeKindMut::Custom(node) => node.run_config.inputs.values_mut().collect(),
                NodeKindMut::Operator(operator) => operator.config.inputs.values_mut().collect(),
            };
            for mapping in input_mappings
                .into_iter()
                .filter_map(|i| match &mut i.mapping {
                    InputMapping::Timer { .. } => None,
                    InputMapping::User(m) => Some(m),
                })
            {
                if let Some(op_name) = single_operator_nodes.get(&mapping.source).copied() {
                    mapping.output = DataId::from(format!("{op_name}/{}", mapping.output));
                }
            }

            // resolve nodes
            let kind = match node_kind {
                NodeKindMut::Standard { path, inputs: _ } => CoreNodeKind::Custom(CustomNode {
                    source: path.clone(),
                    args: node.args,
                    build: node.build,
                    send_stdout_as: node.send_stdout_as,
                    run_config: NodeRunConfig {
                        inputs: node.inputs,
                        outputs: node.outputs,
                    },
                    envs: None,
                }),
                NodeKindMut::Custom(node) => CoreNodeKind::Custom(node.clone()),
                NodeKindMut::Runtime(node) => CoreNodeKind::Runtime(node.clone()),
                NodeKindMut::Operator(op) => CoreNodeKind::Runtime(RuntimeNode {
                    operators: vec![OperatorDefinition {
                        id: op.id.clone().unwrap_or_else(|| default_op_id.clone()),
                        config: op.config.clone(),
                    }],
                }),
            };

            resolved.push(ResolvedNode {
                id: node.id,
                name: node.name,
                description: node.description,
                env: node.env,
                deploy: {
                    let default_machine = self.deploy.machine.as_deref().unwrap_or_default();
                    let machine = match node.deploy.machine {
                        Some(m) => m,
                        None => default_machine.to_owned(),
                    };
                    ResolvedDeploy { machine }
                },
                kind,
            });
        }

        Ok(resolved)
    }

    fn visualize_as_mermaid(&self) -> eyre::Result<String> {
        let resolved = self.resolve_aliases_and_set_defaults()?;
        let flowchart = visualize::visualize_nodes(&resolved);

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
        validate::check_dataflow(self, working_dir, None, false)
            .wrap_err("Dataflow could not be validated.")
    }

    fn check_in_daemon(
        &self,
        working_dir: &Path,
        remote_machine_id: &[&str],
        coordinator_is_remote: bool,
    ) -> eyre::Result<()> {
        validate::check_dataflow(
            self,
            working_dir,
            Some(remote_machine_id),
            coordinator_is_remote,
        )
        .wrap_err("Dataflow could not be validated.")
    }
}

pub async fn read_as_descriptor(path: &Path) -> eyre::Result<Descriptor> {
    let buf = tokio::fs::read(path)
        .await
        .context("failed to open given file")?;
    Descriptor::parse(buf)
}

fn node_kind_mut(node: &mut Node) -> eyre::Result<NodeKindMut> {
    match node.kind()? {
        NodeKind::Standard(_) => node
            .path
            .as_ref()
            .map(|path| NodeKindMut::Standard {
                path,
                inputs: &mut node.inputs,
            })
            .ok_or_eyre("no path"),
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
    }
}

pub fn source_is_url(source: &str) -> bool {
    source.contains("://")
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
    } else if let Ok(abs_path) = which::which(&path) {
        Ok(abs_path)
    } else {
        bail!("Could not find source path {}", path.display())
    }
}

pub trait NodeExt {
    fn kind(&self) -> eyre::Result<NodeKind>;
}

impl NodeExt for Node {
    fn kind(&self) -> eyre::Result<NodeKind> {
        match (&self.path, &self.operators, &self.custom, &self.operator) {
            (None, None, None, None) => {
                eyre::bail!(
                    "node `{}` requires a `path`, `custom`, or `operators` field",
                    self.id
                )
            }
            (None, None, None, Some(operator)) => Ok(NodeKind::Operator(operator)),
            (None, None, Some(custom), None) => Ok(NodeKind::Custom(custom)),
            (None, Some(runtime), None, None) => Ok(NodeKind::Runtime(runtime)),
            (Some(path), None, None, None) => Ok(NodeKind::Standard(path)),
            _ => {
                eyre::bail!(
                    "node `{}` has multiple exclusive fields set, only one of `path`, `custom`, `operators` and `operator` is allowed",
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
}

#[derive(Debug)]
enum NodeKindMut<'a> {
    Standard {
        path: &'a String,
        inputs: &'a mut BTreeMap<DataId, Input>,
    },
    /// Dora runtime node
    Runtime(&'a mut RuntimeNode),
    Custom(&'a mut CustomNode),
    Operator(&'a mut SingleOperatorDefinition),
}
