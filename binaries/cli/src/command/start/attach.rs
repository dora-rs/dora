use communication_layer_request_reply::{Transport, transport::FramedTransport};
use dora_core::descriptor::{CoreNodeKind, Descriptor, DescriptorExt, resolve_path};
use dora_message::cli_to_coordinator::{
    CheckResp, CliToCoordinatorClient, CliToCoordinatorEncoding, CliToCoordinatorRequest,
    DataflowStopped,
};
use dora_message::common::LogMessage;
use dora_message::id::{NodeId, OperatorId};
use eyre::Context;
use notify::event::ModifyKind;
use notify::{Config, Event as NotifyEvent, EventKind, RecommendedWatcher, RecursiveMode, Watcher};
use std::{
    collections::HashMap,
    net::{SocketAddr, TcpStream},
};
use std::{path::PathBuf, sync::mpsc, time::Duration};
use tracing::info;
use uuid::Uuid;

use crate::common::handle_dataflow_result;
use crate::output::print_log_message;

pub fn attach_dataflow(
    dataflow: Descriptor,
    dataflow_path: PathBuf,
    dataflow_id: Uuid,
    session: &mut CliToCoordinatorClient,
    hot_reload: bool,
    coordinator_socket: SocketAddr,
    log_level: log::LevelFilter,
) -> Result<(), eyre::ErrReport> {
    let (tx, rx) = mpsc::channel();

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
            ctrlc_tx
                .send(AttachEvent::Stop {
                    dataflow_uuid: dataflow_id,
                    grace_duration: None,
                    force: true,
                })
                .ok();
            std::process::abort();
        } else {
            if ctrlc_tx
                .send(AttachEvent::Stop {
                    dataflow_uuid: dataflow_id,
                    grace_duration: None,
                    force: false,
                })
                .is_err()
            {
                // bail!("failed to report ctrl-c event to dora-daemon");
            }
            ctrlc_sent = true;
        }
    })
    .wrap_err("failed to set ctrl-c handler")?;

    // subscribe to log messages
    let mut log_session = FramedTransport::new(
        TcpStream::connect(coordinator_socket).wrap_err("failed to connect to dora coordinator")?,
    )
    .with_encoding::<_, CliToCoordinatorRequest, LogMessage>(CliToCoordinatorEncoding);
    log_session
        .send(&CliToCoordinatorRequest::LogSubscribe {
            dataflow_id,
            level: log_level,
        })
        .wrap_err("failed to send log subscribe request to coordinator")?;
    std::thread::spawn(move || {
        while let Ok(Some(message)) = log_session.receive() {
            if tx.send(AttachEvent::Log(message)).is_err() {
                break;
            }
        }
    });

    loop {
        match rx.recv_timeout(Duration::from_secs(1)) {
            Err(_err) => {}
            Ok(AttachEvent::Stop {
                dataflow_uuid,
                grace_duration,
                force,
            }) => {
                let DataflowStopped { uuid, result } =
                    session.stop(dataflow_uuid, grace_duration, force)?;
                info!("dataflow {uuid} stopped");
                break handle_dataflow_result(result, Some(uuid));
            }
            Ok(AttachEvent::Reload {
                dataflow_id,
                node_id,
                operator_id,
            }) => {
                let uuid = session.reload(dataflow_id, node_id, operator_id)?;
                info!("dataflow {uuid} reloaded");
            }
            Ok(AttachEvent::Log(log_message)) => {
                print_log_message(log_message, false, print_daemon_name);
                continue;
            }
        }

        match session.check(dataflow_id)? {
            CheckResp::Spawned { uuid: _ } => (),
            CheckResp::Stopped { uuid, result } => {
                info!("dataflow {uuid} stopped");
                break handle_dataflow_result(result, Some(uuid));
            }
        }
    }
}

enum AttachEvent {
    Stop {
        dataflow_uuid: Uuid,
        grace_duration: Option<Duration>,
        force: bool,
    },
    Reload {
        dataflow_id: Uuid,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    },
    Log(LogMessage),
}
