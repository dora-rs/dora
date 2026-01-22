use communication_layer_request_reply::{TcpConnection, TcpRequestReplyConnection};
use dora_core::descriptor::{Descriptor, DescriptorExt};
use dora_message::cli_to_coordinator::ControlRequest;
use dora_message::common::LogMessage;
use dora_message::coordinator_to_cli::ControlRequestReply;
use eyre::Context;
use std::{
    net::{SocketAddr, TcpStream},
    sync::mpsc,
    time::Duration,
};
use std::path::PathBuf;
use tracing::{error, info};
use uuid::Uuid;

use crate::common::handle_dataflow_result;
use crate::hot_reload::{setup_file_watcher, ReloadEvent};
use crate::output::print_log_message;

pub fn attach_dataflow(
    dataflow: Descriptor,
    dataflow_path: PathBuf,
    dataflow_id: Uuid,
    session: &mut TcpRequestReplyConnection,
    hot_reload: bool,
    coordinator_socket: SocketAddr,
    log_level: log::LevelFilter,
) -> Result<(), eyre::ErrReport> {
    let (tx, rx) = mpsc::channel();

    let nodes = dataflow.resolve_aliases_and_set_defaults()?;

    let print_daemon_name = nodes.values().any(|n| n.deploy.is_some());

    let working_dir = dataflow_path
        .canonicalize()
        .context("failed to canonicalize dataflow path")?
        .parent()
        .ok_or_else(|| eyre::eyre!("canonicalized dataflow path has no parent"))?
        .to_owned();

    // Setup dataflow file watcher if reload option is set.
    let watcher_tx = tx.clone();
    let _watcher = if hot_reload {
        let (watcher, reload_rx) = setup_file_watcher(dataflow_id, &nodes, &working_dir)
            .context("failed to setup file watcher")?;

        // Spawn a thread to forward reload events to the main channel
        std::thread::spawn(move || {
            while let Ok(event) = reload_rx.recv() {
                let _ = watcher_tx.send(AttachEvent::Control(ControlRequest::Reload {
                    dataflow_id: event.dataflow_id,
                    node_id: event.node_id,
                    operator_id: event.operator_id,
                }));
            }
        });

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
                .send(AttachEvent::Control(ControlRequest::Stop {
                    dataflow_uuid: dataflow_id,
                    grace_duration: None,
                    force: true,
                }))
                .ok();
            std::process::abort();
        } else {
            if ctrlc_tx
                .send(AttachEvent::Control(ControlRequest::Stop {
                    dataflow_uuid: dataflow_id,
                    grace_duration: None,
                    force: false,
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
                print_log_message(log_message, false, print_daemon_name);
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

enum AttachEvent {
    Control(ControlRequest),
    Log(eyre::Result<LogMessage>),
}
