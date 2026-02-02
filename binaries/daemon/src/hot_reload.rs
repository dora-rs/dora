use crate::DaemonHotReloadEvent;
use dora_core::config::{NodeId, OperatorId};
use dora_core::descriptor::{CoreNodeKind, Descriptor, DescriptorExt, OperatorSource, ResolvedNode};
use eyre::{Context, Result};
use notify::{Config, Event as NotifyEvent, EventKind, RecommendedWatcher, RecursiveMode, Watcher};
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::time::{Duration, Instant};
use tracing::info;

/// Sets up file watching in the daemon for hot-reload.
///
/// Watches both node binary/script files and the dataflow YAML file.
/// Sends `DaemonHotReloadEvent`s directly via a tokio channel.
///
/// The returned `RecommendedWatcher` must be kept alive for file watching to work.
pub fn setup_daemon_watcher(
    dataflow_path: Option<&Path>,
    nodes: &BTreeMap<NodeId, ResolvedNode>,
    working_dir: &Path,
    tx: tokio::sync::mpsc::Sender<DaemonHotReloadEvent>,
) -> Result<RecommendedWatcher> {
    // Build path lookup for node files
    let mut node_path_lookup: HashMap<PathBuf, (NodeId, Option<OperatorId>)> = HashMap::new();

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
                    node_path_lookup.insert(path, (node_id.clone(), None));
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
                            node_path_lookup
                                .insert(path, (node_id.clone(), Some(op.id.clone())));
                        }
                    }
                }
            }
        }
    }

    // Optionally track the dataflow YAML path
    let dataflow_path_canonical = dataflow_path
        .and_then(|p| p.canonicalize().ok());

    let current_node_ids = std::sync::Arc::new(std::sync::RwLock::new(
        nodes.keys().cloned().collect::<BTreeSet<NodeId>>(),
    ));
    let current_nodes = std::sync::Arc::new(std::sync::RwLock::new(nodes.clone()));

    info!(
        "Hot-reload: daemon watcher for {} node paths{}",
        node_path_lookup.len(),
        if dataflow_path_canonical.is_some() {
            " + dataflow YAML"
        } else {
            ""
        }
    );

    let paths_to_watch: Vec<PathBuf> = node_path_lookup.keys().cloned().collect();

    let last_event_times: Mutex<HashMap<PathBuf, Instant>> = Mutex::new(HashMap::new());
    let debounce_duration = Duration::from_secs(1);
    let dataflow_path_for_closure = dataflow_path_canonical.clone();

    let notifier = move |event: Result<NotifyEvent, notify::Error>| {
        if let Ok(NotifyEvent { paths, kind, .. }) = event {
            let should_process = matches!(kind, EventKind::Modify(_) | EventKind::Create(_));
            if !should_process {
                return;
            }

            for path in paths {
                let lookup_path = path.canonicalize().unwrap_or(path.clone());

                {
                    let mut times = last_event_times.lock().unwrap();
                    let now = Instant::now();
                    if let Some(last) = times.get(&lookup_path) {
                        if now.duration_since(*last) < debounce_duration {
                            continue;
                        }
                    }
                    times.insert(lookup_path.clone(), now);
                }

                if let Some((node_id, operator_id)) = node_path_lookup.get(&lookup_path) {
                    info!(
                        "Hot-reload: file changed, triggering reload for node {}",
                        node_id
                    );
                    let _ = tx.blocking_send(DaemonHotReloadEvent::Reload {
                        node_id: node_id.clone(),
                        operator_id: operator_id.clone(),
                    });
                }

                if let Some(ref dp) = dataflow_path_for_closure {
                    if &lookup_path == dp {
                        info!("Hot-reload: dataflow YAML changed, analyzing changes...");

                        match Descriptor::blocking_read(dp) {
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

                                            for change in changes {
                                                let event = match change {
                                                    DataflowChangeEvent::NodeAdded {
                                                        node_id,
                                                    } => {
                                                        if let Some(node) =
                                                            new_nodes.get(&node_id)
                                                        {
                                                            DaemonHotReloadEvent::SpawnNode {
                                                                node_id,
                                                                node: node.clone(),
                                                                new_descriptor: new_descriptor
                                                                    .clone(),
                                                            }
                                                        } else {
                                                            continue;
                                                        }
                                                    }
                                                    DataflowChangeEvent::NodeRemoved {
                                                        node_id,
                                                    } => {
                                                        DaemonHotReloadEvent::StopNode { node_id }
                                                    }
                                                    DataflowChangeEvent::NodeChanged {
                                                        node_id,
                                                        new_node,
                                                    } => DaemonHotReloadEvent::RestartNode {
                                                        node_id,
                                                        new_node,
                                                        new_descriptor: new_descriptor.clone(),
                                                    },
                                                };
                                                if tx.blocking_send(event).is_err() {
                                                    break;
                                                }
                                            }
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
                                tracing::warn!(
                                    "Hot-reload: failed to read dataflow YAML: {}",
                                    e
                                );
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
    if let Some(ref dp) = dataflow_path_canonical {
        if let Some(dataflow_parent) = dp.parent() {
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
    }

    Ok(watcher)
}

enum DataflowChangeEvent {
    NodeAdded { node_id: NodeId },
    NodeRemoved { node_id: NodeId },
    NodeChanged { node_id: NodeId, new_node: ResolvedNode },
}

fn detect_changes(
    old_node_ids: &BTreeSet<NodeId>,
    old_nodes: &BTreeMap<NodeId, ResolvedNode>,
    new_nodes: &BTreeMap<NodeId, ResolvedNode>,
) -> Vec<DataflowChangeEvent> {
    let mut changes = Vec::new();
    let new_node_ids: BTreeSet<NodeId> = new_nodes.keys().cloned().collect();

    for node_id in new_node_ids.difference(old_node_ids) {
        info!("Hot-reload: detected added node '{}'", node_id);
        changes.push(DataflowChangeEvent::NodeAdded {
            node_id: node_id.clone(),
        });
    }

    for node_id in old_node_ids.difference(&new_node_ids) {
        info!("Hot-reload: detected removed node '{}'", node_id);
        changes.push(DataflowChangeEvent::NodeRemoved {
            node_id: node_id.clone(),
        });
    }

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

fn node_config_changed(old: &ResolvedNode, new: &ResolvedNode) -> bool {
    let old_json = serde_json::to_string(old).unwrap_or_default();
    let new_json = serde_json::to_string(new).unwrap_or_default();
    old_json != new_json
}
