use dora_message::{
    config::{Input, InputMapping, NodeRunConfig},
    descriptor::{GitRepoRev, NodeSource},
    id::{DataId, NodeId, OperatorId},
};
use eyre::{Context, OptionExt, Result, bail};
use std::{
    collections::{BTreeMap, BTreeSet, HashMap, HashSet},
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
    fn resolve_aliases_and_set_defaults_with_working_dir(
        &self,
        working_dir: &Path,
    ) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>>;
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
        // If graph is defined, this will fail with a clear error message
        if !self.graph.is_empty() {
            bail!(
                "Graph-based dataflow requires node metadata. Please provide dora.yaml or use resolve_with_metadata_from_path()"
            );
        }

        // Legacy format resolution
        resolve_legacy_format(self)
    }

    fn resolve_aliases_and_set_defaults_with_working_dir(
        &self,
        working_dir: &Path,
    ) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>> {
        // If graph is defined, try to load metadata from working directory
        if !self.graph.is_empty() {
            let metadata_path = working_dir.join("dora.yaml");

            if metadata_path.exists() {
                let metadata = NodeMetadataFile::blocking_read(&metadata_path)
                    .context("Failed to load dora.yaml metadata file")?;
                return resolve_with_metadata_and_base_dir(self, &metadata, working_dir);
            } else {
                if working_dir.join("dora.yml").exists() {
                    bail!("You need to rename dora.yml to dora.yaml");
                } else {
                    bail!(
                        "Graph-based dataflow requires a dora.yaml file in the working directory, but it was not found at: {}",
                        metadata_path.display()
                    );
                }
            }
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
            // In this path we don't know the dataflow file location, so we
            // fall back to the current working directory as base for
            // resolving dependencies.
            bail!(
                "Graph-based dataflow requires node metadata. Please provide dora.yaml or use resolve_descriptor_from_path()"
            );
        };

        let base_dir = std::env::current_dir().context("Failed to get current working directory")?;
        resolve_with_metadata_and_base_dir(self, &metadata, &base_dir)
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
pub fn load_descriptor_with_metadata(
    dataflow_path: &Path,
) -> eyre::Result<(Descriptor, Option<NodeMetadataFile>)> {
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
            bail!(
                "Graph-based dataflow requires a dora.yaml file in the same directory, but it was not found at: {}",
                metadata_path.display()
            );
        }
    } else {
        // Legacy format, no metadata needed
        Ok((descriptor, None))
    }
}

/// Resolve descriptor with automatic metadata loading
pub fn resolve_descriptor_from_path(
    dataflow_path: &Path,
) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>> {
    let (descriptor, metadata) = load_descriptor_with_metadata(dataflow_path)?;

    if let Some(metadata) = metadata {
        let dataflow_dir = dataflow_path
            .parent()
            .unwrap_or_else(|| Path::new("."));
        resolve_with_metadata_and_base_dir(&descriptor, &metadata, dataflow_dir)
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

/// Resolve a graph-based descriptor using a metadata file and a base directory
///
/// The base directory is used to resolve relative paths in metadata
/// dependencies ("herds").
fn resolve_with_metadata_and_base_dir(
    descriptor: &Descriptor,
    metadata: &NodeMetadataFile,
    base_dir: &Path,
) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>> {
    if descriptor.graph.is_empty() {
        return resolve_legacy_format(descriptor);
    }

    // Collect prototypes from the main metadata file
    // Store both the prototype metadata and the base directory where paths should be resolved
    let mut prototypes: HashMap<String, (NodeMetadata, PathBuf)> = HashMap::new();
    for node_meta in &metadata.nodes {
        prototypes.insert(node_meta.name.clone(), (node_meta.clone(), base_dir.to_path_buf()));
    }

    // Resolve dependency herds, if any
    // Dependencies is now a map where the key is the custom herd name (used in proto references)
    // and the value is the MetadataDependency with path and optional filtering
    for (custom_herd_name, dep) in &metadata.dependencies {
        let dep_path = base_dir.join(&dep.path);
        let dep_metadata_path = if dep_path.is_dir() {
            dep_path.join("dora.yaml")
        } else {
            dep_path.clone()
        };

        if !dep_metadata_path.exists() {
            bail!(
                "Metadata dependency '{}' not found at {}",
                custom_herd_name,
                dep_metadata_path.display()
            );
        }

        let dep_metadata = NodeMetadataFile::blocking_read(&dep_metadata_path).with_context(|| {
            format!(
                "Failed to load metadata for dependency '{}' from {}",
                custom_herd_name,
                dep_metadata_path.display()
            )
        })?;

        // Determine the herd base directory (where relative paths in herd nodes should be resolved)
        let herd_base_dir = if dep_path.is_dir() {
            dep_path
        } else {
            dep_path.parent().unwrap_or(&dep_path).to_path_buf()
        };

        let allowed: Option<HashSet<String>> = if dep.nodes.is_empty() {
            None
        } else {
            Some(dep.nodes.iter().cloned().collect())
        };

        for node_meta in &dep_metadata.nodes {
            // Only consider exported nodes
            if !node_meta.export {
                continue;
            }

            if let Some(ref allowed_set) = allowed {
                if !allowed_set.contains(&node_meta.name) {
                    continue;
                }
            }

            // Use the custom herd name (the map key) as prefix, not dep.package
            let qualified_name = format!("{}/{}", custom_herd_name, node_meta.name);
            prototypes.insert(qualified_name, (node_meta.clone(), herd_base_dir.clone()));
        }
    }

    // Resolve graph nodes
    let mut resolved = BTreeMap::new();
    for graph_node in &descriptor.graph {
        let (proto, proto_base_dir) = prototypes
            .get(&graph_node.proto)
            .ok_or_else(|| {
                eyre::eyre!(
                    "Node prototype '{}' not found in metadata (including dependencies)",
                    graph_node.proto
                )
            })?;

        // Check if this is a workflow-type node (entry points to a .yaml file)
        if let Some(entry) = &proto.entry {
            if entry.ends_with(".yaml") || entry.ends_with(".yml") {
                // Workflow-type node: load and resolve the sub-dataflow
                let workflow_nodes = resolve_workflow_node(graph_node, proto, proto_base_dir, &prototypes)?;
                for (id, node) in workflow_nodes {
                    resolved.insert(id, node);
                }
                continue;
            }
        }

        // Regular node
        let node = merge_graph_node_with_prototype(graph_node, proto, proto_base_dir)?;
        resolved.insert(graph_node.id.clone(), node);
    }

    Ok(resolved)
}

/// Resolve a workflow-type node (where entry points to a dataflow YAML)
///
/// This function loads the sub-dataflow, resolves it, and returns multiple
/// resolved nodes with prefixed IDs to avoid conflicts.
fn resolve_workflow_node(
    graph_node: &GraphNode,
    proto: &NodeMetadata,
    proto_base_dir: &Path,
    parent_prototypes: &HashMap<String, (NodeMetadata, PathBuf)>,
) -> eyre::Result<BTreeMap<NodeId, ResolvedNode>> {
    let entry_path = proto
        .entry
        .as_ref()
        .ok_or_eyre("Workflow node must have an 'entry' field")?;

    // Resolve the dataflow path relative to the prototype's base directory
    let dataflow_path = if Path::new(entry_path).is_relative() {
        proto_base_dir.join(entry_path)
    } else {
        PathBuf::from(entry_path)
    };

    if !dataflow_path.exists() {
        bail!(
            "Workflow dataflow file not found: {}",
            dataflow_path.display()
        );
    }

    // Load the sub-dataflow
    let sub_descriptor = Descriptor::blocking_read(&dataflow_path)
        .with_context(|| format!("Failed to load workflow dataflow from {}", dataflow_path.display()))?;

    if sub_descriptor.graph.is_empty() {
        bail!(
            "Workflow dataflow must use graph format: {}",
            dataflow_path.display()
        );
    }

    // Load metadata for the sub-dataflow (from same directory)
    let dataflow_dir = dataflow_path
        .parent()
        .ok_or_eyre("Dataflow path has no parent directory")?;
    let sub_metadata_path = dataflow_dir.join("dora.yaml");
    
    let sub_metadata = if sub_metadata_path.exists() {
        Some(NodeMetadataFile::blocking_read(&sub_metadata_path)
            .context("Failed to load sub-dataflow metadata")?)
    } else {
        None
    };

    // Build prototypes for the sub-dataflow (includes local nodes and parent's herds)
    let mut sub_prototypes: HashMap<String, (NodeMetadata, PathBuf)> = parent_prototypes.clone();
    
    if let Some(ref metadata) = sub_metadata {
        // Add local nodes from sub-dataflow's metadata
        for node_meta in &metadata.nodes {
            sub_prototypes.insert(node_meta.name.clone(), (node_meta.clone(), dataflow_dir.to_path_buf()));
        }
    }

    // Create a map of input targets from the workflow prototype
    // target format: "graph_node_id/input_id"
    let mut input_targets: HashMap<String, (NodeId, DataId)> = HashMap::new();
    for input_def in &proto.inputs {
        if let Some(ref target) = input_def.target {
            // Parse target: "graph_node_id/input_id"
            if let Some((node_id, input_id)) = target.split_once('/') {
                input_targets.insert(
                    input_def.name.clone(),
                    (NodeId::from(node_id.to_string()), DataId::from(input_id.to_string()))
                );
            }
        }
    }

    // Resolve sub-dataflow's graph nodes
    let mut resolved = BTreeMap::new();
    for sub_graph_node in &sub_descriptor.graph {
        let (sub_proto, sub_proto_base_dir) = sub_prototypes
            .get(&sub_graph_node.proto)
            .ok_or_else(|| {
                eyre::eyre!(
                    "Node prototype '{}' not found in sub-dataflow metadata",
                    sub_graph_node.proto
                )
            })?;

        // Create a modified graph node with prefixed ID and potentially overridden inputs
        let mut modified_graph_node = sub_graph_node.clone();
        
        // Prefix the node ID to avoid conflicts with parent dataflow
        let prefixed_id = NodeId::from(format!("{}_{}", graph_node.id, sub_graph_node.id));
        modified_graph_node.id = prefixed_id.clone();

        // Override inputs if they are targeted by the parent's inputs
        for (parent_input_name, parent_input_mapping) in &graph_node.inputs {
            if let Some((target_node_id, target_input_id)) = input_targets.get(parent_input_name.as_ref() as &str) {
                if target_node_id == &sub_graph_node.id {
                    // This graph node is the target for this input
                    // Override the specific input, but keep other inputs intact
                    modified_graph_node.inputs.insert(target_input_id.clone(), parent_input_mapping.clone());
                }
            }
        }

        // Update internal references to point to prefixed node IDs
        let mut updated_inputs = BTreeMap::new();
        for (input_id, input_mapping) in &modified_graph_node.inputs {
            let mut new_mapping = input_mapping.clone();
            if let Input { mapping: dora_message::config::InputMapping::User(ref mut user_mapping), .. } = new_mapping {
                // Check if the source is one of the sub-dataflow's graph nodes
                for sub_node in &sub_descriptor.graph {
                    if user_mapping.source == sub_node.id {
                        // Update to prefixed ID
                        user_mapping.source = NodeId::from(format!("{}_{}", graph_node.id, sub_node.id));
                        break;
                    }
                }
            }
            updated_inputs.insert(input_id.clone(), new_mapping);
        }
        modified_graph_node.inputs = updated_inputs;

        let node = merge_graph_node_with_prototype(&modified_graph_node, sub_proto, sub_proto_base_dir)?;
        resolved.insert(prefixed_id, node);
    }

    Ok(resolved)
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
///
/// The proto_base_dir is used to resolve relative paths in the prototype
/// (e.g., entry paths for herd nodes should be resolved relative to the herd directory)
fn merge_graph_node_with_prototype(
    graph_node: &GraphNode,
    proto: &NodeMetadata,
    proto_base_dir: &Path,
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
        let entry_path = proto
            .entry
            .as_ref()
            .ok_or_eyre("Node prototype must have an 'entry' field")?;
        
        // Resolve the entry path relative to the prototype's base directory
        let path = if Path::new(entry_path).is_relative() {
            proto_base_dir.join(entry_path)
                .to_str()
                .ok_or_eyre("Failed to convert path to string")?
                .to_string()
        } else {
            entry_path.clone()
        };

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
