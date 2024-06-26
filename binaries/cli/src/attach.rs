use colored::Colorize;
use communication_layer_request_reply::{TcpConnection, TcpRequestReplyConnection};
use dora_core::{
    coordinator_messages::LogMessage,
    descriptor::{resolve_path, CoreNodeKind, Descriptor},
    topics::{ControlRequest, ControlRequestReply},
};
use eyre::Context;
use notify::event::ModifyKind;
use notify::{Config, Event as NotifyEvent, EventKind, RecommendedWatcher, RecursiveMode, Watcher};
use std::{
    collections::HashMap,
    net::{SocketAddr, TcpStream},
};
use std::{path::PathBuf, sync::mpsc, time::Duration};
use tracing::{error, info};
use uuid::Uuid;

use crate::handle_dataflow_result;

pub fn attach_dataflow(
    dataflow: Descriptor,
    dataflow_path: PathBuf,
    dataflow_id: Uuid,
    session: &mut TcpRequestReplyConnection,
    hot_reload: bool,
    coordinator_socket: SocketAddr,
    log_level: log::LevelFilter,
) -> Result<(), eyre::ErrReport> {
    let (tx, rx) = mpsc::sync_channel(2);

    // Generate path hashmap
    let mut node_path_lookup = HashMap::new();

    let nodes = dataflow.resolve_aliases_and_set_defaults()?;

    let working_dir = dataflow_path
        .canonicalize()
        .context("failed to canoncialize dataflow path")?
        .parent()
        .ok_or_else(|| eyre::eyre!("canonicalized dataflow path has no parent"))?
        .to_owned();

    for node in nodes {
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
                            .send(AttachEvent::Control(ControlRequest::Reload {
                                dataflow_id: *dataflow_id,
                                node_id: node_id.clone(),
                                operator_id: operator_id.clone(),
                            }))
                            .context("Could not send reload request to the cli loop")
                            .unwrap();
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
            std::process::abort();
        } else {
            if ctrlc_tx
                .send(AttachEvent::Control(ControlRequest::Stop {
                    dataflow_uuid: dataflow_id,
                    grace_duration: None,
                }))
                .is_err()
            {
                // bail!("failed to report ctrl-c event to dora-daemon");
            }
            ctrlc_sent = true;
        }
    })
    .wrap_err("failed to set ctrl-c handler")?;

    // subscribe to log messages
    let mut log_session = TcpConnection {
        stream: TcpStream::connect(coordinator_socket)
            .wrap_err("failed to connect to dora coordinator")?,
    };
    log_session
        .send(
            &serde_json::to_vec(&ControlRequest::LogSubscribe {
                dataflow_id,
                level: log_level,
            })
            .wrap_err("failed to serialize message")?,
        )
        .wrap_err("failed to send log subscribe request to coordinator")?;
    std::thread::spawn(move || {
        while let Ok(raw) = log_session.receive() {
            let parsed: eyre::Result<LogMessage> =
                serde_json::from_slice(&raw).context("failed to parse log message");
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
                let LogMessage {
                    dataflow_id: _,
                    node_id,
                    level,
                    target,
                    module_path: _,
                    file: _,
                    line: _,
                    message,
                } = log_message;
                let level = match level {
                    log::Level::Error => "ERROR".red(),
                    log::Level::Warn => "WARN ".yellow(),
                    log::Level::Info => "INFO ".green(),
                    other => format!("{other:5}").normal(),
                };
                let node = match node_id {
                    Some(node_id) => format!(" {node_id}").bold(),
                    None => "".normal(),
                };
                let target = match target {
                    Some(target) => format!(" {target}").dimmed(),
                    None => "".normal(),
                };

                println!("{level}{node}{target}: {message}");
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
            ControlRequestReply::DataflowStarted { uuid: _ } => (),
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

enum AttachEvent {
    Control(ControlRequest),
    Log(eyre::Result<LogMessage>),
}
