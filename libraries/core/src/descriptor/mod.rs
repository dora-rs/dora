use dora_message::{
    config::{Input, InputMapping, NodeRunConfig},
    descriptor::{GitRepoRev, NodeSource},
    id::{DataId, NodeId, OperatorId},
};
use eyre::{Context, OptionExt, Result, bail};
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    env::consts::EXE_EXTENSION,
    path::{Path, PathBuf},
    process::Stdio,
};
use tokio::process::Command;

// reexport for compatibility
pub use dora_message::descriptor::{
    CoreNodeKind, CustomNode, DYNAMIC_SOURCE, Descriptor, GraphNode, InputDefinition, Node,
    NodeMetadata, NodeMetadataFile, OperatorConfig, OperatorDefinition, OperatorSource,
    OutputDefinition, PythonSource, ResolvedNode, RuntimeNode, SHELL_SOURCE,
    SingleOperatorDefinition,
};
pub use validate::ResolvedNodeExt;
pub use visualize::collect_dora_timers;

mod validate;
mod visualize;

#[cfg(test)]
mod tests;

pub trait DescriptorExt {
    fn resolve_aliases_and_set_defaults(&self) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>>;
    fn resolve_with_metadata(
        &self,
        metadata_file: Option<&NodeMetadataFile>,
    ) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>>;
    fn visualize_as_mermaid(&self) -> eyre::Result<String>;
    fn blocking_read(path: &Path) -> eyre::Result<Descriptor>;
    fn parse(buf: Vec<u8>) -> eyre::Result<Descriptor>;
    fn check(&self, working_dir: &Path) -> eyre::Result<()>;
    fn check_in_daemon(&self, working_dir: &Path, coordinator_is_remote: bool) -> eyre::Result<()>;
}

pub trait NodeMetadataFileExt {
    fn blocking_read(path: &Path) -> eyre::Result<NodeMetadataFile>;
    fn parse(buf: Vec<u8>) -> eyre::Result<NodeMetadataFile>;
}

pub const SINGLE_OPERATOR_DEFAULT_ID: &str = "op";

impl DescriptorExt for Descriptor {
    fn resolve_aliases_and_set_defaults(&self) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>> {
        // If graph is defined, resolve with metadata
        if !self.graph.is_empty() {
            // Try to load metadata file from the same directory
            // This is a simplified approach; in production, we might need more sophisticated path resolution
            return self.resolve_with_metadata(None);
        }

        // Legacy format resolution
        resolve_legacy_format(self)
    }

    fn resolve_with_metadata(
        &self,
        metadata_file: Option<&NodeMetadataFile>,
    ) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>> {
        if self.graph.is_empty() {
            // No graph defined, fall back to legacy format
            return resolve_legacy_format(self);
        }

        // Load metadata if not provided
        let metadata = if let Some(m) = metadata_file {
            m.clone()
        } else {
            // Try to load dora.yaml from current directory
            // In a real implementation, we would need to know the dataflow file's directory
            // For now, return an error if metadata is required but not provided
            bail!("Graph-based dataflow requires node metadata. Please provide dora.yaml or use resolve_with_metadata_from_path()");
        };

        // Build a map of node prototypes
        let mut prototypes: HashMap<String, &NodeMetadata> = HashMap::new();
        for node_meta in &metadata.nodes {
            prototypes.insert(node_meta.name.clone(), node_meta);
        }

        // Resolve graph nodes
        let mut resolved = BTreeMap::new();
        for graph_node in &self.graph {
            let proto = prototypes
                .get(&graph_node.proto)
                .ok_or_eyre(format!("Node prototype '{}' not found in dora.yaml", graph_node.proto))?;

            // Create a resolved node from the prototype and graph node
            let node = merge_graph_node_with_prototype(graph_node, proto)?;
            resolved.insert(graph_node.id.clone(), node);
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

    fn check_in_daemon(&self, working_dir: &Path, coordinator_is_remote: bool) -> eyre::Result<()> {
        validate::check_dataflow(self, working_dir, None, coordinator_is_remote)
            .wrap_err("Dataflow could not be validated.")
    }
}

/// Load descriptor and automatically load metadata if needed
pub fn load_descriptor_with_metadata(dataflow_path: &Path) -> eyre::Result<(Descriptor, Option<NodeMetadataFile>)> {
    let descriptor = Descriptor::blocking_read(dataflow_path)?;
    
    // If graph is defined, try to load metadata from the same directory
    if !descriptor.graph.is_empty() {
        let dataflow_dir = dataflow_path.parent().unwrap_or_else(|| Path::new("."));
        let metadata_path = dataflow_dir.join("dora.yaml");
        
        if metadata_path.exists() {
            let metadata = NodeMetadataFile::blocking_read(&metadata_path)
                .context("Failed to load dora.yaml metadata file")?;
            Ok((descriptor, Some(metadata)))
        } else {
            bail!("Graph-based dataflow requires a dora.yaml file in the same directory, but it was not found at: {}", metadata_path.display());
        }
    } else {
        // Legacy format, no metadata needed
        Ok((descriptor, None))
    }
}

/// Resolve descriptor with automatic metadata loading
pub fn resolve_descriptor_from_path(dataflow_path: &Path) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>> {
    let (descriptor, metadata) = load_descriptor_with_metadata(dataflow_path)?;
    
    if let Some(metadata) = metadata {
        descriptor.resolve_with_metadata(Some(&metadata))
    } else {
        descriptor.resolve_aliases_and_set_defaults()
    }
}

impl NodeMetadataFileExt for NodeMetadataFile {
    fn blocking_read(path: &Path) -> eyre::Result<NodeMetadataFile> {
        let buf = std::fs::read(path).context("failed to open node metadata file")?;
        Self::parse(buf)
    }

    fn parse(buf: Vec<u8>) -> eyre::Result<NodeMetadataFile> {
        serde_yaml::from_slice(&buf).context("failed to parse node metadata file")
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
        source: NodeSource,
        inputs: &'a mut BTreeMap<DataId, Input>,
    },
    /// Dora runtime node
    Runtime(&'a mut RuntimeNode),
    Custom(&'a mut CustomNode),
    Operator(&'a mut SingleOperatorDefinition),
}

/// Resolve nodes using the legacy format (nodes field)
fn resolve_legacy_format(descriptor: &Descriptor) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>> {
    let default_op_id = OperatorId::from(SINGLE_OPERATOR_DEFAULT_ID.to_string());

    let single_operator_nodes: HashMap<_, _> = descriptor
        .nodes
        .iter()
        .filter_map(|n| {
            n.operator
                .as_ref()
                .map(|op| (&n.id, op.id.as_ref().unwrap_or(&default_op_id)))
        })
        .collect();

    let mut resolved = BTreeMap::new();
    for mut node in descriptor.nodes.clone() {
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

        resolved.insert(
            node.id.clone(),
            ResolvedNode {
                id: node.id,
                name: node.name,
                description: node.description,
                env: node.env,
                deploy: node.deploy,
                kind,
            },
        );
    }

    Ok(resolved)
}

/// Merge a graph node with its prototype to create a resolved node
fn merge_graph_node_with_prototype(
    graph_node: &GraphNode,
    proto: &NodeMetadata,
) -> eyre::Result<ResolvedNode> {
    // Determine the node source
    let source = match (&proto.git, &proto.branch, &proto.tag, &proto.rev) {
        (None, None, None, None) => NodeSource::Local,
        (Some(repo), branch, tag, rev) => {
            let rev = match (branch, tag, rev) {
                (None, None, None) => None,
                (Some(branch), None, None) => Some(GitRepoRev::Branch(branch.clone())),
                (None, Some(tag), None) => Some(GitRepoRev::Tag(tag.clone())),
                (None, None, Some(rev)) => Some(GitRepoRev::Rev(rev.clone())),
                _ => bail!("only one of `branch`, `tag`, and `rev` are allowed"),
            };
            NodeSource::GitBranch {
                repo: repo.clone(),
                rev,
            }
        }
        (None, _, _, _) => {
            bail!("`git` source required when using branch, tag, or rev")
        }
    };

    // Convert outputs from OutputDefinition to DataId
    let outputs: BTreeSet<DataId> = proto
        .outputs
        .iter()
        .map(|o| DataId::from(o.name.clone()))
        .collect();

    // Build the node kind
    let kind = if let Some(operators) = &proto.operators {
        CoreNodeKind::Runtime(operators.clone())
    } else if let Some(operator) = &proto.operator {
        let default_op_id = OperatorId::from(SINGLE_OPERATOR_DEFAULT_ID.to_string());
        CoreNodeKind::Runtime(RuntimeNode {
            operators: vec![OperatorDefinition {
                id: operator.id.clone().unwrap_or_else(|| default_op_id.clone()),
                config: operator.config.clone(),
            }],
        })
    } else {
        // Standard custom node
        let path = proto
            .entry
            .as_ref()
            .ok_or_eyre("Node prototype must have an 'entry' field")?
            .clone();

        CoreNodeKind::Custom(CustomNode {
            path,
            source,
            args: proto.args.clone(),
            build: proto.build.clone(),
            send_stdout_as: None,
            run_config: NodeRunConfig {
                inputs: graph_node.inputs.clone(),
                outputs,
            },
            envs: None,
        })
    };

    // Merge environment variables (graph node overrides prototype)
    let env = match (&proto.env, &graph_node.env) {
        (None, None) => None,
        (Some(proto_env), None) => Some(proto_env.clone()),
        (None, Some(graph_env)) => Some(graph_env.clone()),
        (Some(proto_env), Some(graph_env)) => {
            let mut merged = proto_env.clone();
            merged.extend(graph_env.clone());
            Some(merged)
        }
    };

    Ok(ResolvedNode {
        id: graph_node.id.clone(),
        name: Some(proto.name.clone()),
        description: None,
        env,
        deploy: graph_node.deploy.clone(),
        kind,
    })
}
