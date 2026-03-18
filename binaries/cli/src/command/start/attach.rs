use dora_core::descriptor::{CoreNodeKind, Descriptor, DescriptorExt, resolve_path};
use dora_core::topics::zenoh_log_subscribe_all_for_dataflow;
use dora_message::cli_to_coordinator::CoordinatorControlClient;
use dora_message::coordinator_to_cli::{CheckDataflowReply, DataflowResult, StopDataflowReply};
use dora_message::id::{NodeId, OperatorId};
use dora_message::tarpc;
use eyre::Context;
use notify::event::ModifyKind;
use notify::{Config, Event as NotifyEvent, EventKind, RecommendedWatcher, RecursiveMode, Watcher};
use std::time::Duration;
use std::{collections::HashMap, path::PathBuf};
use tracing::info;
use uuid::Uuid;

use crate::common::{handle_dataflow_result, long_context, rpc};
use crate::output::subscribe_and_print_logs;

pub async fn attach_dataflow(
    dataflow: Descriptor,
    dataflow_path: PathBuf,
    dataflow_id: Uuid,
    client: &CoordinatorControlClient,
    hot_reload: bool,
    zenoh_session: &zenoh::Session,
    log_level: log::LevelFilter,
) -> Result<(), eyre::ErrReport> {
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();

    // Generate path hashmap
    let mut node_path_lookup = HashMap::new();

    let nodes = dataflow.resolve_aliases_and_set_defaults()?;

    let print_daemon_name = nodes.values().any(|n| n.deploy.is_some());

    let working_dir = dataflow_path
        .canonicalize()
        .context("failed to canonicalize dataflow path")?
        .parent()
        .ok_or_else(|| eyre::eyre!("canonicalized dataflow path has no parent"))?
        .to_owned();

    for node in nodes.into_values() {
        match node.kind {
            // Reloading Custom Nodes is not supported. See: https://github.com/dora-rs/dora/pull/239#discussion_r1154313139
            CoreNodeKind::Custom(_cn) => (),
            CoreNodeKind::Runtime(rn) => {
                for op in rn.operators.iter() {
                    if let dora_core::descriptor::OperatorSource::Python(python_source) =
                        &op.config.source
                    {
                        let path = resolve_path(&python_source.source, &working_dir)
                            .wrap_err_with(|| {
                                format!("failed to resolve node source `{}`", python_source.source)
                            })?;
                        node_path_lookup
                            .insert(path, (dataflow_id, node.id.clone(), Some(op.id.clone())));
                    }
                    // Reloading non-python operator is not supported. See: https://github.com/dora-rs/dora/pull/239#discussion_r1154313139
                }
            }
        }
    }

    // Setup dataflow file watcher if reload option is set.
    let watcher_tx = tx.clone();
    let _watcher = if hot_reload {
        let hash = node_path_lookup.clone();
        let paths = hash.keys();
        let notifier = move |event| {
            if let Ok(NotifyEvent {
                paths,
                kind: EventKind::Modify(ModifyKind::Data(_data)),
                ..
            }) = event
            {
                for path in paths {
                    if let Some((dataflow_id, node_id, operator_id)) = node_path_lookup.get(&path) {
                        watcher_tx
                            .send(AttachEvent::Reload {
                                dataflow_id: *dataflow_id,
                                node_id: node_id.clone(),
                                operator_id: operator_id.clone(),
                            })
                            .unwrap_or_else(|_| {
                                panic!("Could not send reload request to the cli loop")
                            });
                    }
                }
                // TODO: Manage different file event
            }
        };

        let mut watcher = RecommendedWatcher::new(
            notifier,
            Config::default().with_poll_interval(Duration::from_secs(1)),
        )?;

        for path in paths {
            watcher.watch(path, RecursiveMode::Recursive)?;
        }
        Some(watcher)
    } else {
        None
    };

    // Setup Ctrlc Watcher to stop dataflow after ctrlc
    let ctrlc_tx = tx.clone();
    let mut ctrlc_sent = false;
    ctrlc::set_handler(move || {
        if ctrlc_sent {
            ctrlc_tx.send(AttachEvent::Stop { force: true }).ok();
            std::process::abort();
        } else {
            if ctrlc_tx.send(AttachEvent::Stop { force: false }).is_err() {
                // bail!("failed to report ctrl-c event to dora-daemon");
            }
            ctrlc_sent = true;
        }
    })
    .wrap_err("failed to set ctrl-c handler")?;

    // Subscribe to log messages via zenoh — prints directly without routing
    // through the event channel, since log display is fire-and-forget.
    // The zenoh session was opened by the caller *before* the start RPC
    // to avoid missing early log messages.
    let log_topic = zenoh_log_subscribe_all_for_dataflow(dataflow_id);
    let _log_task = subscribe_and_print_logs(
        zenoh_session,
        &log_topic,
        log_level,
        false,
        print_daemon_name,
    )
    .await?;

    loop {
        let event: AttachLoopEvent =
            match tokio::time::timeout(Duration::from_secs(1), rx.recv()).await {
                Err(_) => {
                    // timeout - check if dataflow is still running
                    let check_reply = rpc(
                        "check dataflow status",
                        client.check(tarpc::context::current(), dataflow_id),
                    )
                    .await?;
                    match check_reply {
                        CheckDataflowReply::Running { .. } => continue,
                        CheckDataflowReply::Stopped { uuid, result } => {
                            AttachLoopEvent::Stopped { uuid, result }
                        }
                    }
                }
                Ok(Some(AttachEvent::Reload {
                    dataflow_id,
                    node_id,
                    operator_id,
                })) => {
                    let uuid = rpc(
                        "reload operator",
                        client.reload(tarpc::context::current(), dataflow_id, node_id, operator_id),
                    )
                    .await?;
                    AttachLoopEvent::Reloaded { uuid }
                }
                Ok(Some(AttachEvent::Stop { force })) => {
                    let StopDataflowReply { uuid, result } = rpc(
                        "stop dataflow",
                        client.stop(long_context(), dataflow_id, None, force),
                    )
                    .await?;
                    AttachLoopEvent::Stopped { uuid, result }
                }
                Ok(None) => {
                    // all senders dropped, channel closed
                    break Ok(());
                }
            };

        match event {
            AttachLoopEvent::Stopped { uuid, result } => {
                info!("dataflow {uuid} stopped");
                break handle_dataflow_result(result, Some(uuid));
            }
            AttachLoopEvent::Reloaded { uuid } => {
                info!("dataflow {uuid} reloaded")
            }
        };
    }
}

enum AttachEvent {
    Reload {
        dataflow_id: Uuid,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    },
    Stop {
        force: bool,
    },
}

enum AttachLoopEvent {
    Stopped { uuid: Uuid, result: DataflowResult },
    Reloaded { uuid: Uuid },
}
