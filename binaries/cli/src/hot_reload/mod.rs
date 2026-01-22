//! Hot-reload file watching module.
//!
//! This module provides file watching functionality for hot-reloading nodes.
//! It can be used by both `dora start` and `dora run`.

use dora_core::config::{NodeId, OperatorId};
use dora_core::descriptor::{CoreNodeKind, OperatorSource, ResolvedNode};
use eyre::{Context, Result};
use notify::{Config, Event as NotifyEvent, EventKind, RecommendedWatcher, RecursiveMode, Watcher};
use std::collections::{BTreeMap, HashMap};
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
    let mut watched_dirs: std::collections::HashSet<PathBuf> = std::collections::HashSet::new();
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
