//! Hot-reload file watching module for CLI.
//!
//! This module provides YAML-only file watching for the CLI.
//! Binary/script file watching is handled by the daemon directly.

use dora_core::config::NodeId;
use dora_core::descriptor::{Descriptor, DescriptorExt, ResolvedNode};
use eyre::{Context, Result};
use notify::{Config, Event as NotifyEvent, EventKind, RecommendedWatcher, RecursiveMode, Watcher};
use std::collections::{BTreeMap, BTreeSet, HashSet};
use std::path::{Path, PathBuf};
use std::sync::mpsc;
use std::time::Duration;
use tracing::info;

/// A change detected in a node from the dataflow YAML.
#[derive(Debug, Clone)]
pub enum NodeChange {
    /// A node was added to the dataflow
    NodeAdded { node_id: NodeId },
    /// A node was removed from the dataflow
    NodeRemoved { node_id: NodeId },
    /// A node's configuration changed
    NodeChanged {
        node_id: NodeId,
        new_node: ResolvedNode,
    },
}

/// Event triggered when the dataflow YAML changes.
#[derive(Debug, Clone)]
pub struct DataflowChangeEvent {
    pub changes: Vec<NodeChange>,
    pub new_descriptor: Descriptor,
    pub new_nodes: BTreeMap<NodeId, ResolvedNode>,
}

/// Sets up YAML-only watching for the dataflow descriptor file.
///
/// Binary/script file watching is handled by the daemon.
/// This function only watches for changes to the dataflow YAML file.
pub fn setup_yaml_watcher(
    dataflow_path: &Path,
    nodes: &BTreeMap<NodeId, ResolvedNode>,
    _working_dir: &Path,
) -> Result<(RecommendedWatcher, mpsc::Receiver<DataflowChangeEvent>)> {
    let (tx, rx) = mpsc::channel();

    let dataflow_path_canonical = dataflow_path
        .canonicalize()
        .context("failed to canonicalize dataflow path")?;

    let current_node_ids = std::sync::Arc::new(std::sync::RwLock::new(
        nodes.keys().cloned().collect::<BTreeSet<NodeId>>(),
    ));
    let current_nodes = std::sync::Arc::new(std::sync::RwLock::new(nodes.clone()));

    info!("Hot-reload: setting up YAML-only watcher for dataflow file");

    let notifier = {
        let dataflow_path = dataflow_path_canonical.clone();
        move |event: Result<NotifyEvent, notify::Error>| {
            if let Ok(NotifyEvent { paths, kind, .. }) = event {
                if !matches!(kind, EventKind::Modify(_) | EventKind::Create(_)) {
                    return;
                }

                for path in paths {
                    let lookup_path = path.canonicalize().unwrap_or(path);

                    if lookup_path == dataflow_path {
                        info!("Hot-reload: dataflow YAML changed, analyzing changes...");

                        match Descriptor::blocking_read(&dataflow_path) {
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

                                            *current_node_ids.write().unwrap() =
                                                new_nodes.keys().cloned().collect();
                                            *current_nodes.write().unwrap() = new_nodes.clone();

                                            let _ = tx.send(DataflowChangeEvent {
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
        }
    };

    let mut watcher = RecommendedWatcher::new(
        notifier,
        Config::default().with_poll_interval(Duration::from_secs(1)),
    )
    .context("failed to create file watcher")?;

    let mut watched_dirs: HashSet<PathBuf> = HashSet::new();
    if let Some(dataflow_parent) = dataflow_path_canonical.parent() {
        if watched_dirs.insert(dataflow_parent.to_path_buf()) {
            info!(
                "Hot-reload: watching dataflow directory {:?}",
                dataflow_parent
            );
            watcher
                .watch(dataflow_parent, RecursiveMode::NonRecursive)
                .with_context(|| {
                    format!("failed to watch dataflow directory {:?}", dataflow_parent)
                })?;
        }
    }

    Ok((watcher, rx))
}

/// Detect changes between old and new node configurations.
fn detect_changes(
    old_node_ids: &BTreeSet<NodeId>,
    old_nodes: &BTreeMap<NodeId, ResolvedNode>,
    new_nodes: &BTreeMap<NodeId, ResolvedNode>,
) -> Vec<NodeChange> {
    let mut changes = Vec::new();
    let new_node_ids: BTreeSet<NodeId> = new_nodes.keys().cloned().collect();

    for node_id in new_node_ids.difference(old_node_ids) {
        info!("Hot-reload: detected added node '{}'", node_id);
        changes.push(NodeChange::NodeAdded {
            node_id: node_id.clone(),
        });
    }

    for node_id in old_node_ids.difference(&new_node_ids) {
        info!("Hot-reload: detected removed node '{}'", node_id);
        changes.push(NodeChange::NodeRemoved {
            node_id: node_id.clone(),
        });
    }

    for node_id in old_node_ids.intersection(&new_node_ids) {
        let old_node = old_nodes.get(node_id);
        let new_node = new_nodes.get(node_id);

        if let (Some(old), Some(new)) = (old_node, new_node) {
            if node_config_changed(old, new) {
                info!("Hot-reload: detected changed node '{}'", node_id);
                changes.push(NodeChange::NodeChanged {
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
    let old_json = serde_json::to_string(old).unwrap_or_default();
    let new_json = serde_json::to_string(new).unwrap_or_default();
    old_json != new_json
}
