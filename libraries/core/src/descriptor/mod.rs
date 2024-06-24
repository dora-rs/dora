use crate::config::{
    CommunicationConfig, DataId, Input, InputMapping, NodeId, NodeRunConfig, OperatorId,
};
use eyre::{bail, eyre, Context, OptionExt, Result};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_with_expand_env::with_expand_envs;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    env::consts::EXE_EXTENSION,
    fmt,
    path::{Path, PathBuf},
};
use tracing::warn;
pub use visualize::collect_dora_timers;
mod validate;
mod visualize;
pub const SHELL_SOURCE: &str = "shell";
pub const DYNAMIC_SOURCE: &str = "dynamic";

/// Dataflow description
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
#[schemars(title = "dora-rs specification")]
pub struct Descriptor {
    #[schemars(skip)]
    #[serde(default)]
    pub communication: CommunicationConfig,
    #[schemars(skip)]
    #[serde(default, rename = "_unstable_deploy")]
    pub deploy: Deploy,
    pub nodes: Vec<Node>,
}

pub const SINGLE_OPERATOR_DEFAULT_ID: &str = "op";

impl Descriptor {
    pub fn resolve_aliases_and_set_defaults(&self) -> eyre::Result<Vec<ResolvedNode>> {
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
            let mut node_kind = node.kind_mut()?;
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
                deploy: ResolvedDeploy::new(node.deploy, self),
                kind,
            });
        }

        Ok(resolved)
    }

    pub fn visualize_as_mermaid(&self) -> eyre::Result<String> {
        let resolved = self.resolve_aliases_and_set_defaults()?;
        let flowchart = visualize::visualize_nodes(&resolved);

        Ok(flowchart)
    }

    pub async fn read(path: &Path) -> eyre::Result<Descriptor> {
        let buf = tokio::fs::read(path)
            .await
            .context("failed to open given file")?;
        Descriptor::parse(buf)
    }

    pub fn blocking_read(path: &Path) -> eyre::Result<Descriptor> {
        let buf = std::fs::read(path).context("failed to open given file")?;
        Descriptor::parse(buf)
    }

    pub fn parse(buf: Vec<u8>) -> eyre::Result<Descriptor> {
        serde_yaml::from_slice(&buf).context("failed to parse given descriptor")
    }

    pub fn check(&self, working_dir: &Path) -> eyre::Result<()> {
        validate::check_dataflow(self, working_dir, None, false)
            .wrap_err("Dataflow could not be validated.")
    }

    pub fn check_in_daemon(
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

#[derive(Debug, Clone, Default, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Deploy {
    pub machine: Option<String>,
}

/// Dora Node
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Node {
    /// Node identifier
    pub id: NodeId,
    /// Node name
    pub name: Option<String>,
    /// Description of the node
    pub description: Option<String>,
    /// Environment variables
    pub env: Option<BTreeMap<String, EnvValue>>,

    /// Unstable machine deployment configuration
    #[schemars(skip)]
    #[serde(default, rename = "_unstable_deploy")]
    pub deploy: Deploy,

    #[serde(default, skip_serializing_if = "Option::is_none")]
    operators: Option<RuntimeNode>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    custom: Option<CustomNode>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    operator: Option<SingleOperatorDefinition>,

    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub args: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub send_stdout_as: Option<String>,
    #[serde(default)]
    pub inputs: BTreeMap<DataId, Input>,
    #[serde(default)]
    pub outputs: BTreeSet<DataId>,
}

impl Node {
    pub fn kind(&self) -> eyre::Result<NodeKind> {
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

    fn kind_mut(&mut self) -> eyre::Result<NodeKindMut> {
        match self.kind()? {
            NodeKind::Standard(_) => self
                .path
                .as_ref()
                .map(|path| NodeKindMut::Standard {
                    path,
                    inputs: &mut self.inputs,
                })
                .ok_or_eyre("no path"),
            NodeKind::Runtime(_) => self
                .operators
                .as_mut()
                .map(NodeKindMut::Runtime)
                .ok_or_eyre("no operators"),
            NodeKind::Custom(_) => self
                .custom
                .as_mut()
                .map(NodeKindMut::Custom)
                .ok_or_eyre("no custom"),
            NodeKind::Operator(_) => self
                .operator
                .as_mut()
                .map(NodeKindMut::Operator)
                .ok_or_eyre("no operator"),
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResolvedNode {
    pub id: NodeId,
    pub name: Option<String>,
    pub description: Option<String>,
    pub env: Option<BTreeMap<String, EnvValue>>,

    #[serde(default)]
    pub deploy: ResolvedDeploy,

    #[serde(flatten)]
    pub kind: CoreNodeKind,
}

impl ResolvedNode {
    pub fn send_stdout_as(&self) -> Result<Option<String>> {
        match &self.kind {
            // TODO: Split stdout between operators
            CoreNodeKind::Runtime(n) => {
                let count = n
                    .operators
                    .iter()
                    .filter(|op| op.config.send_stdout_as.is_some())
                    .count();
                if count == 1 && n.operators.len() > 1 {
                    warn!("All stdout from all operators of a runtime are going to be sent in the selected `send_stdout_as` operator.")
                } else if count > 1 {
                    return Err(eyre!("More than one `send_stdout_as` entries for a runtime node. Please only use one `send_stdout_as` per runtime."));
                }
                Ok(n.operators.iter().find_map(|op| {
                    op.config
                        .send_stdout_as
                        .clone()
                        .map(|stdout| format!("{}/{}", op.id, stdout))
                }))
            }
            CoreNodeKind::Custom(n) => Ok(n.send_stdout_as.clone()),
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ResolvedDeploy {
    pub machine: String,
}
impl ResolvedDeploy {
    fn new(deploy: Deploy, descriptor: &Descriptor) -> Self {
        let default_machine = descriptor.deploy.machine.as_deref().unwrap_or_default();
        let machine = match deploy.machine {
            Some(m) => m,
            None => default_machine.to_owned(),
        };
        Self { machine }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CoreNodeKind {
    /// Dora runtime node
    #[serde(rename = "operators")]
    Runtime(RuntimeNode),
    Custom(CustomNode),
}

pub fn runtime_node_inputs(n: &RuntimeNode) -> BTreeMap<DataId, Input> {
    n.operators
        .iter()
        .flat_map(|operator| {
            operator.config.inputs.iter().map(|(input_id, mapping)| {
                (
                    DataId::from(format!("{}/{input_id}", operator.id)),
                    mapping.clone(),
                )
            })
        })
        .collect()
}

fn runtime_node_outputs(n: &RuntimeNode) -> BTreeSet<DataId> {
    n.operators
        .iter()
        .flat_map(|operator| {
            operator
                .config
                .outputs
                .iter()
                .map(|output_id| DataId::from(format!("{}/{output_id}", operator.id)))
        })
        .collect()
}

impl CoreNodeKind {
    pub fn run_config(&self) -> NodeRunConfig {
        match self {
            CoreNodeKind::Runtime(n) => NodeRunConfig {
                inputs: runtime_node_inputs(n),
                outputs: runtime_node_outputs(n),
            },
            CoreNodeKind::Custom(n) => n.run_config.clone(),
        }
    }

    pub fn dynamic(&self) -> bool {
        match self {
            CoreNodeKind::Runtime(_n) => false,
            CoreNodeKind::Custom(n) => n.source == DYNAMIC_SOURCE,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(transparent)]
pub struct RuntimeNode {
    pub operators: Vec<OperatorDefinition>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
pub struct OperatorDefinition {
    pub id: OperatorId,
    #[serde(flatten)]
    pub config: OperatorConfig,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
pub struct SingleOperatorDefinition {
    /// ID is optional if there is only a single operator.
    pub id: Option<OperatorId>,
    #[serde(flatten)]
    pub config: OperatorConfig,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
pub struct OperatorConfig {
    pub name: Option<String>,
    pub description: Option<String>,

    #[serde(default)]
    pub inputs: BTreeMap<DataId, Input>,
    #[serde(default)]
    pub outputs: BTreeSet<DataId>,

    #[serde(flatten)]
    pub source: OperatorSource,

    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub send_stdout_as: Option<String>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, Clone)]
#[serde(rename_all = "kebab-case")]
pub enum OperatorSource {
    SharedLibrary(String),
    Python(PythonSource),
    #[schemars(skip)]
    Wasm(String),
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(
    deny_unknown_fields,
    from = "PythonSourceDef",
    into = "PythonSourceDef"
)]
pub struct PythonSource {
    pub source: String,
    pub conda_env: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum PythonSourceDef {
    SourceOnly(String),
    WithOptions {
        source: String,
        conda_env: Option<String>,
    },
}

impl From<PythonSource> for PythonSourceDef {
    fn from(input: PythonSource) -> Self {
        match input {
            PythonSource {
                source,
                conda_env: None,
            } => Self::SourceOnly(source),
            PythonSource { source, conda_env } => Self::WithOptions { source, conda_env },
        }
    }
}

impl From<PythonSourceDef> for PythonSource {
    fn from(value: PythonSourceDef) -> Self {
        match value {
            PythonSourceDef::SourceOnly(source) => Self {
                source,
                conda_env: None,
            },
            PythonSourceDef::WithOptions { source, conda_env } => Self { source, conda_env },
        }
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

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct PythonOperatorConfig {
    pub path: PathBuf,
    #[serde(default)]
    pub inputs: BTreeMap<DataId, InputMapping>,
    #[serde(default)]
    pub outputs: BTreeSet<DataId>,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
pub struct CustomNode {
    /// Path of the source code
    ///
    /// If you want to use a specific `conda` environment.
    /// Provide the python path within the source.
    ///
    /// source: /home/peter/miniconda3/bin/python
    ///
    /// args: some_node.py
    ///
    /// Source can match any executable in PATH.
    pub source: String,
    /// Args for the executable.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub args: Option<String>,
    /// Environment variables for the custom nodes
    ///
    /// Deprecated, use outer-level `env` field instead.
    pub envs: Option<BTreeMap<String, EnvValue>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,
    /// Send stdout and stderr to another node
    #[serde(skip_serializing_if = "Option::is_none")]
    pub send_stdout_as: Option<String>,

    #[serde(flatten)]
    pub run_config: NodeRunConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum EnvValue {
    #[serde(deserialize_with = "with_expand_envs")]
    Bool(bool),
    #[serde(deserialize_with = "with_expand_envs")]
    Integer(u64),
    #[serde(deserialize_with = "with_expand_envs")]
    String(String),
}

impl fmt::Display for EnvValue {
    fn fmt(&self, fmt: &mut fmt::Formatter) -> fmt::Result {
        match self {
            EnvValue::Bool(bool) => fmt.write_str(&bool.to_string()),
            EnvValue::Integer(u64) => fmt.write_str(&u64.to_string()),
            EnvValue::String(str) => fmt.write_str(str),
        }
    }
}
