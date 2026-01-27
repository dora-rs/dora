//! Hot-reload file watching module.
//!
//! This module provides file watching functionality for hot-reloading nodes.
//! It can be used by both `dora start` and `dora run`.

use dora_core::config::{NodeId, OperatorId};
use dora_core::descriptor::{CoreNodeKind, Descriptor, DescriptorExt, OperatorSource, ResolvedNode};
use eyre::{Context, Result};
use notify::{Config, Event as NotifyEvent, EventKind, RecommendedWatcher, RecursiveMode, Watcher};
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::sync::mpsc;
use std::time::Duration;
use tracing::info;
use uuid::Uuid;

/// A reload event indicating which node should be reloaded.
#[derive(Debug, Clone)]
pub struct ReloadEvent {
    pub dataflow_id: Uuid,
    pub node_id: NodeId,
    pub operator_id: Option<OperatorId>,
}

/// Events triggered by dataflow YAML changes.
#[derive(Debug, Clone)]
pub enum DataflowChangeEvent {
    /// A node was added to the dataflow
    NodeAdded {
        node_id: NodeId,
        node: ResolvedNode,
    },
    /// A node was removed from the dataflow
    NodeRemoved {
        node_id: NodeId,
    },
    /// A node's configuration changed (will trigger stop + respawn with new config)
    NodeChanged {
        node_id: NodeId,
        new_node: ResolvedNode,
    },
}

/// Combined hot-reload event that can be either a file change or dataflow change.
#[derive(Debug, Clone)]
pub enum HotReloadEvent {
    /// A node's binary/script file changed
    FileChanged(ReloadEvent),
    /// The dataflow YAML changed
    DataflowChanged {
        changes: Vec<DataflowChangeEvent>,
        new_descriptor: Descriptor,
        new_nodes: BTreeMap<NodeId, ResolvedNode>,
    },
}

/// Sets up file watching for hot-reload and returns a watcher and receiver for reload events.
///
/// The watcher must be kept alive for file watching to work.
/// When a watched file changes, a `ReloadEvent` is sent on the receiver.
pub fn setup_file_watcher(
    dataflow_id: Uuid,
    nodes: &BTreeMap<NodeId, ResolvedNode>,
    working_dir: &Path,
) -> Result<(RecommendedWatcher, mpsc::Receiver<ReloadEvent>)> {
    let (tx, rx) = mpsc::channel();

    // Build path lookup: path -> (dataflow_id, node_id, operator_id)
    let mut node_path_lookup: HashMap<PathBuf, (Uuid, NodeId, Option<OperatorId>)> = HashMap::new();

    for (node_id, node) in nodes.iter() {
        match &node.kind {
            CoreNodeKind::Custom(cn) => {
                let source = cn.path.as_str();
                // Try to resolve the path:
                // 1. Local file path (./foo or /path/to/foo)
                // 2. Command in PATH (pip-installed packages)
                let resolved_path = if source.starts_with("./") || source.starts_with("/") {
                    working_dir.join(source).canonicalize().ok()
                } else {
                    // Try to find in PATH using which (pure Rust, no process spawning)
                    which::which(source).ok()
                };
                if let Some(path) = resolved_path {
                    info!(
                        "Hot-reload: watching custom node '{}' at {:?}",
                        node_id, path
                    );
                    node_path_lookup.insert(path, (dataflow_id, node_id.clone(), None));
                } else {
                    info!(
                        "Hot-reload: could not resolve path for custom node '{}' (source: {})",
                        node_id, source
                    );
                }
            }
            CoreNodeKind::Runtime(rn) => {
                for op in rn.operators.iter() {
                    if let OperatorSource::Python(python_source) = &op.config.source {
                        if let Ok(path) = working_dir.join(&python_source.source).canonicalize() {
                            info!(
                                "Hot-reload: watching Python operator '{}' at {:?}",
                                op.id, path
                            );
                            node_path_lookup.insert(
                                path,
                                (dataflow_id, node_id.clone(), Some(op.id.clone())),
                            );
                        }
                    }
                }
            }
        }
    }

    info!(
        "Hot-reload: setting up file watcher for {} paths",
        node_path_lookup.len()
    );

    // Collect paths to watch before moving node_path_lookup into closure
    let paths_to_watch: Vec<PathBuf> = node_path_lookup.keys().cloned().collect();

    // Set up the notifier callback
    let notifier = move |event: Result<NotifyEvent, notify::Error>| {
        if let Ok(NotifyEvent { paths, kind, .. }) = event {
            // Handle Modify and Create events (pip install deletes and recreates files)
            let should_reload = matches!(kind, EventKind::Modify(_) | EventKind::Create(_));
            if should_reload {
                for path in paths {
                    let lookup_path = path.canonicalize().unwrap_or(path);
                    if let Some((dataflow_id, node_id, operator_id)) =
                        node_path_lookup.get(&lookup_path)
                    {
                        info!("Hot-reload: file changed, triggering reload for node {}", node_id);
                        let _ = tx.send(ReloadEvent {
                            dataflow_id: *dataflow_id,
                            node_id: node_id.clone(),
                            operator_id: operator_id.clone(),
                        });
                    }
                }
            }
        }
    };

    let mut watcher = RecommendedWatcher::new(
        notifier,
        Config::default().with_poll_interval(Duration::from_secs(1)),
    )
    .context("failed to create file watcher")?;

    // Watch parent directories instead of files directly
    // This is needed because pip install deletes and recreates files
    let mut watched_dirs: HashSet<PathBuf> = HashSet::new();
    for path in paths_to_watch {
        if let Some(parent) = path.parent() {
            if watched_dirs.insert(parent.to_path_buf()) {
                info!("Hot-reload: watching directory {:?}", parent);
                watcher
                    .watch(parent, RecursiveMode::NonRecursive)
                    .with_context(|| format!("failed to watch directory {:?}", parent))?;
            }
        }
    }

    Ok((watcher, rx))
}

/// Sets up comprehensive hot-reload watching for both node files and the dataflow YAML.
///
/// This function watches:
/// - Node binary/script files (triggers reload on change)
/// - The dataflow YAML file (triggers add/remove/change detection)
///
/// Returns a watcher and receiver for combined hot-reload events.
pub fn setup_comprehensive_watcher(
    dataflow_id: Uuid,
    dataflow_path: &Path,
    nodes: &BTreeMap<NodeId, ResolvedNode>,
    working_dir: &Path,
) -> Result<(RecommendedWatcher, mpsc::Receiver<HotReloadEvent>)> {
    let (tx, rx) = mpsc::channel();

    // Build path lookup for node files
    let mut node_path_lookup: HashMap<PathBuf, (Uuid, NodeId, Option<OperatorId>)> = HashMap::new();

    for (node_id, node) in nodes.iter() {
        match &node.kind {
            CoreNodeKind::Custom(cn) => {
                let source = cn.path.as_str();
                let resolved_path = if source.starts_with("./") || source.starts_with("/") {
                    working_dir.join(source).canonicalize().ok()
                } else {
                    which::which(source).ok()
                };
                if let Some(path) = resolved_path {
                    info!(
                        "Hot-reload: watching custom node '{}' at {:?}",
                        node_id, path
                    );
                    node_path_lookup.insert(path, (dataflow_id, node_id.clone(), None));
                } else {
                    info!(
                        "Hot-reload: could not resolve path for custom node '{}' (source: {})",
                        node_id, source
                    );
                }
            }
            CoreNodeKind::Runtime(rn) => {
                for op in rn.operators.iter() {
                    if let OperatorSource::Python(python_source) = &op.config.source {
                        if let Ok(path) = working_dir.join(&python_source.source).canonicalize() {
                            info!(
                                "Hot-reload: watching Python operator '{}' at {:?}",
                                op.id, path
                            );
                            node_path_lookup.insert(
                                path,
                                (dataflow_id, node_id.clone(), Some(op.id.clone())),
                            );
                        }
                    }
                }
            }
        }
    }

    // Store the dataflow path for YAML watching
    let dataflow_path_canonical = dataflow_path
        .canonicalize()
        .context("failed to canonicalize dataflow path")?;

    // Store current node IDs for diffing
    let current_node_ids: BTreeSet<NodeId> = nodes.keys().cloned().collect();
    let current_nodes = nodes.clone();

    info!(
        "Hot-reload: setting up comprehensive watcher for {} node paths + dataflow YAML",
        node_path_lookup.len()
    );

    // Collect paths to watch
    let paths_to_watch: Vec<PathBuf> = node_path_lookup.keys().cloned().collect();

    // Use interior mutability for tracking state in the callback
    let current_node_ids = std::sync::Arc::new(std::sync::RwLock::new(current_node_ids));
    let current_nodes = std::sync::Arc::new(std::sync::RwLock::new(current_nodes));
    let dataflow_path_for_closure = dataflow_path_canonical.clone();

    let notifier = move |event: Result<NotifyEvent, notify::Error>| {
        if let Ok(NotifyEvent { paths, kind, .. }) = event {
            let should_process = matches!(kind, EventKind::Modify(_) | EventKind::Create(_));
            if !should_process {
                return;
            }

            for path in paths {
                let lookup_path = path.canonicalize().unwrap_or(path.clone());

                // Check if it's a node file change
                if let Some((dataflow_id, node_id, operator_id)) =
                    node_path_lookup.get(&lookup_path)
                {
                    info!("Hot-reload: file changed, triggering reload for node {}", node_id);
                    let _ = tx.send(HotReloadEvent::FileChanged(ReloadEvent {
                        dataflow_id: *dataflow_id,
                        node_id: node_id.clone(),
                        operator_id: operator_id.clone(),
                    }));
                }

                // Check if it's the dataflow YAML change
                if lookup_path == dataflow_path_for_closure {
                    info!("Hot-reload: dataflow YAML changed, analyzing changes...");

                    // Re-read and parse the dataflow
                    match Descriptor::blocking_read(&dataflow_path_for_closure) {
                        Ok(new_descriptor) => {
                            match new_descriptor.resolve_aliases_and_set_defaults() {
                                Ok(new_nodes) => {
                                    let changes = detect_changes(
                                        &current_node_ids.read().unwrap(),
                                        &current_nodes.read().unwrap(),
                                        &new_nodes,
                                    );

                                    if !changes.is_empty() {
                                        info!(
                                            "Hot-reload: detected {} changes in dataflow YAML",
                                            changes.len()
                                        );

                                        // Update tracked state
                                        *current_node_ids.write().unwrap() =
                                            new_nodes.keys().cloned().collect();
                                        *current_nodes.write().unwrap() = new_nodes.clone();

                                        let _ = tx.send(HotReloadEvent::DataflowChanged {
                                            changes,
                                            new_descriptor,
                                            new_nodes,
                                        });
                                    }
                                }
                                Err(e) => {
                                    tracing::warn!(
                                        "Hot-reload: failed to resolve dataflow aliases: {}",
                                        e
                                    );
                                }
                            }
                        }
                        Err(e) => {
                            tracing::warn!("Hot-reload: failed to read dataflow YAML: {}", e);
                        }
                    }
                }
            }
        }
    };

    let mut watcher = RecommendedWatcher::new(
        notifier,
        Config::default().with_poll_interval(Duration::from_secs(1)),
    )
    .context("failed to create file watcher")?;

    // Watch parent directories for node files
    let mut watched_dirs: HashSet<PathBuf> = HashSet::new();
    for path in paths_to_watch {
        if let Some(parent) = path.parent() {
            if watched_dirs.insert(parent.to_path_buf()) {
                info!("Hot-reload: watching directory {:?}", parent);
                watcher
                    .watch(parent, RecursiveMode::NonRecursive)
                    .with_context(|| format!("failed to watch directory {:?}", parent))?;
            }
        }
    }

    // Watch the dataflow YAML file's parent directory
    if let Some(dataflow_parent) = dataflow_path_canonical.parent() {
        if watched_dirs.insert(dataflow_parent.to_path_buf()) {
            info!("Hot-reload: watching dataflow directory {:?}", dataflow_parent);
            watcher
                .watch(dataflow_parent, RecursiveMode::NonRecursive)
                .with_context(|| format!("failed to watch dataflow directory {:?}", dataflow_parent))?;
        }
    }

    Ok((watcher, rx))
}

/// Detect changes between old and new node configurations.
fn detect_changes(
    old_node_ids: &BTreeSet<NodeId>,
    old_nodes: &BTreeMap<NodeId, ResolvedNode>,
    new_nodes: &BTreeMap<NodeId, ResolvedNode>,
) -> Vec<DataflowChangeEvent> {
    let mut changes = Vec::new();

    let new_node_ids: BTreeSet<NodeId> = new_nodes.keys().cloned().collect();

    // Find added nodes
    for node_id in new_node_ids.difference(old_node_ids) {
        if let Some(node) = new_nodes.get(node_id) {
            info!("Hot-reload: detected added node '{}'", node_id);
            changes.push(DataflowChangeEvent::NodeAdded {
                node_id: node_id.clone(),
                node: node.clone(),
            });
        }
    }

    // Find removed nodes
    for node_id in old_node_ids.difference(&new_node_ids) {
        info!("Hot-reload: detected removed node '{}'", node_id);
        changes.push(DataflowChangeEvent::NodeRemoved {
            node_id: node_id.clone(),
        });
    }

    // Find changed nodes (nodes that exist in both but have different configurations)
    for node_id in old_node_ids.intersection(&new_node_ids) {
        let old_node = old_nodes.get(node_id);
        let new_node = new_nodes.get(node_id);

        if let (Some(old), Some(new)) = (old_node, new_node) {
            if node_config_changed(old, new) {
                info!("Hot-reload: detected changed node '{}'", node_id);
                changes.push(DataflowChangeEvent::NodeChanged {
                    node_id: node_id.clone(),
                    new_node: new.clone(),
                });
            }
        }
    }

    changes
}

/// Check if a node's configuration has changed.
fn node_config_changed(old: &ResolvedNode, new: &ResolvedNode) -> bool {
    // Compare the serialized forms for simplicity
    // This catches any changes in inputs, outputs, path, env, etc.
    let old_json = serde_json::to_string(old).unwrap_or_default();
    let new_json = serde_json::to_string(new).unwrap_or_default();
    old_json != new_json
}
