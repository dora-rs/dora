use dora_core::descriptor::{
    CoreNodeKind, Descriptor, DescriptorExt, OperatorSource, ResolvedNode, source_is_url,
};
use dora_message::cli_to_coordinator::ControlRequest;
use dora_message::common::LogMessage;
use dora_message::coordinator_to_cli::ControlRequestReply;
use dora_message::id::{NodeId, OperatorId};
use eyre::Context;
use notify::event::ModifyKind;
use notify::{Config, Event as NotifyEvent, EventKind, RecommendedWatcher, RecursiveMode, Watcher};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::{net::SocketAddr, sync::mpsc, time::Duration};
use tracing::{error, info, warn};
use uuid::Uuid;

use crate::common::{connect_to_coordinator, handle_dataflow_result};
use crate::output::{LogOutputConfig, print_log_message};
use crate::ws_client::WsSession;

pub fn attach_dataflow(
    dataflow: Descriptor,
    dataflow_path: PathBuf,
    dataflow_id: Uuid,
    session: &WsSession,
    coordinator_addr: SocketAddr,
    hot_reload: bool,
    log_level: log::LevelFilter,
) -> Result<(), eyre::ErrReport> {
    let (tx, rx) = mpsc::channel();

    // Same check as the detached path in `start/mod.rs`; no need to resolve
    // the descriptor for it.
    let print_daemon_name = dataflow.nodes.iter().any(|n| n.deploy.is_some());

    // Setup dataflow file watcher if reload option is set. Only
    // `--hot-reload` needs the operator sources on this machine: the dataflow
    // is already running at this point, so a watch problem must not detach
    // it from this CLI (no logs, no Ctrl-C) — it only costs hot-reload.
    let _watcher = if hot_reload {
        match hot_reload_lookup(&dataflow, &dataflow_path, dataflow_id) {
            Ok(node_path_lookup) => start_hot_reload_watcher(node_path_lookup, tx.clone()),
            Err(err) => {
                warn!("hot-reload disabled: {err:#}");
                None
            }
        }
    } else {
        None
    };

    // Setup Ctrlc Watcher to stop dataflow after ctrlc
    //
    // The first Ctrl-C is queued for the control loop below. The second one
    // must not be: the coordinator answers a graceful `Stop` only once the
    // dataflow has finished, so the control loop is blocked in
    // `session.request` for that whole grace period. The coordinator also
    // handles one request at a time per connection, so the force-stop goes
    // out on a connection of its own, like `dora stop --force` from another
    // terminal (#3969).
    let ctrlc_tx = tx.clone();
    let mut ctrlc_sent = false;
    ctrlc::set_handler(move || {
        if ctrlc_sent {
            eprintln!("received second Ctrl-C -> force-stopping dataflow {dataflow_id}");
            if let Err(err) = force_stop(coordinator_addr, dataflow_id) {
                eprintln!(
                    "could not confirm the force-stop: {err:#}\n  \
                     hint: check with `dora list`, and run `dora stop --force {dataflow_id}` \
                     if it is still running"
                );
            }
            std::process::exit(1);
        } else {
            let _ = ctrlc_tx.send(AttachEvent::Control(ControlRequest::Stop {
                dataflow_uuid: dataflow_id,
                grace_duration: None,
                force: false,
            }));
            ctrlc_sent = true;
        }
    })
    .wrap_err("failed to set ctrl-c handler")?;

    // subscribe to log messages
    let log_rx = session.subscribe_logs(
        &serde_json::to_vec(&ControlRequest::LogSubscribe {
            dataflow_id,
            level: log_level,
        })
        .wrap_err("failed to serialize message")?,
    )?;
    std::thread::spawn(move || {
        while let Ok(raw) = log_rx.recv() {
            let parsed: eyre::Result<LogMessage> = match raw {
                Ok(bytes) => serde_json::from_slice(&bytes).context("failed to parse log message"),
                Err(err) => Err(err),
            };
            if tx.send(AttachEvent::Log(parsed)).is_err() {
                break;
            }
        }
    });

    loop {
        let control_request = match rx.recv_timeout(Duration::from_secs(1)) {
            Err(_err) => ControlRequest::Check {
                dataflow_uuid: dataflow_id,
            },
            Ok(AttachEvent::Control(control_request)) => control_request,
            Ok(AttachEvent::Log(Ok(log_message))) => {
                let config = LogOutputConfig {
                    print_daemon_name,
                    ..LogOutputConfig::default()
                };
                print_log_message(log_message, &config);
                continue;
            }
            Ok(AttachEvent::Log(Err(err))) => {
                tracing::warn!("failed to parse log message: {:#?}", err);
                continue;
            }
        };

        let reply_raw = session
            .request(&serde_json::to_vec(&control_request)?)
            .wrap_err("failed to send request message to coordinator")?;
        let result: ControlRequestReply =
            serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
        match result {
            ControlRequestReply::DataflowSpawned { uuid: _ } => (),
            ControlRequestReply::DataflowStopped { uuid, result } => {
                info!("dataflow {uuid} stopped");
                break handle_dataflow_result(result, Some(uuid));
            }
            ControlRequestReply::DataflowReloaded { uuid } => {
                info!("dataflow {uuid} reloaded")
            }
            other => error!("Received unexpected Coordinator Reply: {:#?}", other),
        };
    }
}

/// How long the second Ctrl-C waits for the coordinator to answer the
/// force-stop. The reply comes once the killed nodes are gone, which is quick,
/// but a wedged coordinator must not keep the CLI from exiting.
const FORCE_STOP_REPLY_TIMEOUT: Duration = Duration::from_secs(5);

/// Send `Stop { force: true }` for `dataflow_id` on a new connection and wait
/// up to [`FORCE_STOP_REPLY_TIMEOUT`] for the coordinator's answer.
fn force_stop(coordinator_addr: SocketAddr, dataflow_id: Uuid) -> eyre::Result<()> {
    let (reply_tx, reply_rx) = mpsc::channel();
    std::thread::spawn(move || {
        let result = (|| {
            let session = connect_to_coordinator(coordinator_addr)?;
            let request = serde_json::to_vec(&ControlRequest::Stop {
                dataflow_uuid: dataflow_id,
                grace_duration: None,
                force: true,
            })?;
            let reply = session.request(&request)?;
            match serde_json::from_slice(&reply).wrap_err("failed to parse reply")? {
                ControlRequestReply::Error(err) => eyre::bail!("{err}"),
                _ => Ok(()),
            }
        })();
        let _ = reply_tx.send(result);
    });
    reply_rx
        .recv_timeout(FORCE_STOP_REPLY_TIMEOUT)
        .map_err(|_| {
            eyre::eyre!("no reply from the coordinator within {FORCE_STOP_REPLY_TIMEOUT:?}")
        })?
}

#[allow(clippy::large_enum_variant)]
enum AttachEvent {
    Control(ControlRequest),
    Log(eyre::Result<LogMessage>),
}

/// The watch map for `--hot-reload`: see [`hot_reload_watch_paths`].
fn hot_reload_lookup(
    dataflow: &Descriptor,
    dataflow_path: &Path,
    dataflow_id: Uuid,
) -> eyre::Result<HashMap<PathBuf, WatchedOperator>> {
    let nodes = dataflow.resolve_aliases_and_set_defaults()?;
    let working_dir = dataflow_path
        .canonicalize()
        .context("failed to canonicalize dataflow path")?
        .parent()
        .ok_or_else(|| eyre::eyre!("canonicalized dataflow path has no parent"))?
        .to_owned();
    Ok(hot_reload_watch_paths(
        nodes.into_values(),
        &working_dir,
        dataflow_id,
    ))
}

/// Watch every path in `node_path_lookup` and send a `Reload` for the
/// matching operator when one is modified. `None` if no watcher could be
/// created; a path that cannot be watched is skipped with a warning.
fn start_hot_reload_watcher(
    node_path_lookup: HashMap<PathBuf, WatchedOperator>,
    watcher_tx: mpsc::Sender<AttachEvent>,
) -> Option<RecommendedWatcher> {
    let paths: Vec<PathBuf> = node_path_lookup.keys().cloned().collect();
    let notifier = move |event| {
        if let Ok(NotifyEvent {
            paths,
            kind: EventKind::Modify(ModifyKind::Data(_data)),
            ..
        }) = event
        {
            for path in paths {
                if let Some((dataflow_id, node_id, operator_id)) = node_path_lookup.get(&path) {
                    // A filesystem event can arrive on the watcher thread
                    // after the control loop has stopped and dropped `rx`
                    // (the dataflow finished or errored out). A closed
                    // channel just means the CLI is shutting down, so drop
                    // the reload instead of panicking the watcher thread —
                    // matching the ctrl-c and log senders in this function.
                    let _ = watcher_tx.send(AttachEvent::Control(ControlRequest::Reload {
                        dataflow_id: *dataflow_id,
                        node_id: node_id.clone(),
                        operator_id: operator_id.clone(),
                    }));
                }
            }
            // TODO: Manage different file event
        }
    };

    match RecommendedWatcher::new(
        notifier,
        Config::default().with_poll_interval(Duration::from_secs(1)),
    ) {
        Ok(mut watcher) => {
            for path in &paths {
                if let Err(err) = watcher.watch(path, RecursiveMode::Recursive) {
                    warn!("hot-reload: cannot watch `{}`: {err}", path.display());
                }
            }
            Some(watcher)
        }
        Err(err) => {
            warn!("hot-reload disabled: cannot create a file watcher: {err}");
            None
        }
    }
}

type WatchedOperator = (Uuid, NodeId, Option<OperatorId>);

/// The local Python operator files `--hot-reload` watches, mapped to the
/// operator to reload when one changes. A source is looked up relative to
/// the node's working directory — the dataflow directory, or its
/// `deploy.working_dir` under it — which is the runtime's cwd (no `$PATH`
/// or uv lookup). One that cannot be watched — a URL, or a file that only
/// exists on the machine the node is deployed to — is skipped with a
/// warning, since the dataflow runs fine without it.
fn hot_reload_watch_paths(
    nodes: impl IntoIterator<Item = ResolvedNode>,
    working_dir: &Path,
    dataflow_id: Uuid,
) -> HashMap<PathBuf, WatchedOperator> {
    let mut lookup = HashMap::new();
    for node in nodes {
        // Reloading custom nodes and non-Python operators is not supported.
        // See: https://github.com/dora-rs/adora/pull/239#discussion_r1154313139
        let CoreNodeKind::Runtime(rn) = node.kind else {
            continue;
        };
        // The runtime runs in the node's working dir: the dataflow dir, or
        // `deploy.working_dir` under it (as the daemon resolves it).
        let node_working_dir = match node.deploy.as_ref().and_then(|d| d.working_dir.as_ref()) {
            Some(dir) => working_dir.join(dir),
            None => working_dir.to_owned(),
        };
        for op in rn.operators {
            let OperatorSource::Python(python_source) = &op.config.source else {
                continue;
            };
            let source = &python_source.source;
            if source_is_url(source) {
                warn!("not watching `{source}` for hot-reload: it is a URL");
                continue;
            }
            // Canonicalize: `notify` backends (FSEvents in particular) report
            // real paths, and the lookup is a plain `==` on the event path, so
            // a symlinked or `..`-containing source would otherwise never
            // match its own modification events.
            let Ok(path) = node_working_dir.join(source).canonicalize() else {
                warn!(
                    "not watching `{source}` for hot-reload: no such file in `{}`",
                    node_working_dir.display()
                );
                continue;
            };
            lookup.insert(path, (dataflow_id, node.id.clone(), Some(op.id)));
        }
    }
    lookup
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A URL operator source or a file that is not on this machine must not
    /// stop the CLI from attaching; only the local file is watched.
    #[test]
    fn hot_reload_watch_paths_skips_unwatchable_sources() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("local_op.py"), "").unwrap();
        let descriptor: Descriptor = serde_yaml::from_str(
            r#"
nodes:
  - id: local-node
    operators:
      - id: op
        python: local_op.py
  - id: remote-node
    operators:
      - id: op
        python: https://example.com/remote_op.py
  - id: elsewhere-node
    operators:
      - id: op
        python: only_on_the_robot.py
"#,
        )
        .unwrap();
        let nodes = descriptor.resolve_aliases_and_set_defaults().unwrap();
        let working_dir = dir.path().canonicalize().unwrap();
        let dataflow_id = Uuid::new_v4();

        let lookup = hot_reload_watch_paths(nodes.into_values(), &working_dir, dataflow_id);

        let watched: Vec<_> = lookup.values().collect();
        assert_eq!(
            watched,
            [&(
                dataflow_id,
                NodeId::from("local-node".to_string()),
                Some(OperatorId::from("op".to_string())),
            )],
        );
        assert!(lookup.contains_key(&working_dir.join("local_op.py")));
    }

    /// The runtime runs an operator in its node's `deploy.working_dir`, so
    /// that is where its source is watched.
    #[test]
    fn hot_reload_watch_paths_uses_the_deploy_working_dir() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::create_dir(dir.path().join("sub")).unwrap();
        std::fs::write(dir.path().join("sub/op.py"), "").unwrap();
        let descriptor: Descriptor = serde_yaml::from_str(
            r#"
nodes:
  - id: deployed
    deploy:
      working_dir: sub
    operators:
      - id: op
        python: op.py
"#,
        )
        .unwrap();
        let nodes = descriptor.resolve_aliases_and_set_defaults().unwrap();
        let working_dir = dir.path().canonicalize().unwrap();

        let lookup = hot_reload_watch_paths(nodes.into_values(), &working_dir, Uuid::new_v4());

        let watched: Vec<_> = lookup.keys().collect();
        assert_eq!(watched, [&working_dir.join("sub/op.py")]);
    }
}
