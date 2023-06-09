use crate::config::{
    CommunicationConfig, DataId, Input, InputMapping, NodeId, NodeRunConfig, OperatorId,
};
use eyre::{bail, Context, Result};
use serde::{Deserialize, Serialize};
use serde_with_expand_env::with_expand_envs;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    env::consts::EXE_EXTENSION,
    fmt,
    path::{Path, PathBuf},
};
pub use visualize::collect_dora_timers;

mod validate;
mod visualize;
pub const SHELL_SOURCE: &str = "shell";

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Descriptor {
    #[serde(default)]
    pub communication: CommunicationConfig,
    // deprecated
    pub daemon_config: Option<serde_yaml::Value>,
    #[serde(default, rename = "_unstable_deploy")]
    pub deploy: Deploy,
    pub nodes: Vec<Node>,
}

pub const SINGLE_OPERATOR_DEFAULT_ID: &str = "op";

impl Descriptor {
    pub fn resolve_aliases_and_set_defaults(&self) -> Vec<ResolvedNode> {
        let default_op_id = OperatorId::from(SINGLE_OPERATOR_DEFAULT_ID.to_string());

        let single_operator_nodes: HashMap<_, _> = self
            .nodes
            .iter()
            .filter_map(|n| match &n.kind {
                NodeKind::Operator(op) => Some((&n.id, op.id.as_ref().unwrap_or(&default_op_id))),
                _ => None,
            })
            .collect();

        let mut resolved = vec![];
        for mut node in self.nodes.clone() {
            // adjust input mappings
            let input_mappings: Vec<_> = match &mut node.kind {
                NodeKind::Runtime(node) => node
                    .operators
                    .iter_mut()
                    .flat_map(|op| op.config.inputs.values_mut())
                    .collect(),
                NodeKind::Custom(node) => node.run_config.inputs.values_mut().collect(),
                NodeKind::Operator(operator) => operator.config.inputs.values_mut().collect(),
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
            let kind = match node.kind {
                NodeKind::Custom(node) => CoreNodeKind::Custom(node),
                NodeKind::Runtime(node) => CoreNodeKind::Runtime(node),
                NodeKind::Operator(op) => CoreNodeKind::Runtime(RuntimeNode {
                    operators: vec![OperatorDefinition {
                        id: op.id.unwrap_or_else(|| default_op_id.clone()),
                        config: op.config,
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

        resolved
    }

    pub fn visualize_as_mermaid(&self) -> eyre::Result<String> {
        let resolved = self.resolve_aliases_and_set_defaults();
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
        validate::check_dataflow(self, working_dir).wrap_err("Dataflow could not be validated.")
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Deploy {
    pub machine: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Node {
    pub id: NodeId,
    pub name: Option<String>,
    pub description: Option<String>,
    pub env: Option<BTreeMap<String, EnvValue>>,

    #[serde(default, rename = "_unstable_deploy")]
    pub deploy: Deploy,

    #[serde(flatten)]
    pub kind: NodeKind,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum NodeKind {
    /// Dora runtime node
    #[serde(rename = "operators")]
    Runtime(RuntimeNode),
    Custom(CustomNode),
    Operator(SingleOperatorDefinition),
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

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(transparent)]
pub struct RuntimeNode {
    pub operators: Vec<OperatorDefinition>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct OperatorDefinition {
    pub id: OperatorId,
    #[serde(flatten)]
    pub config: OperatorConfig,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SingleOperatorDefinition {
    /// ID is optional if there is only a single operator.
    pub id: Option<OperatorId>,
    #[serde(flatten)]
    pub config: OperatorConfig,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
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
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(rename_all = "kebab-case")]
pub enum OperatorSource {
    SharedLibrary(String),
    Python(String),
    Wasm(String),
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CustomNode {
    pub source: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub args: Option<String>,
    pub envs: Option<BTreeMap<String, EnvValue>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub build: Option<String>,

    #[serde(flatten)]
    pub run_config: NodeRunConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
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
