use communication_layer_request_reply::{TcpConnection, TcpRequestReplyConnection};
use dora_core::descriptor::Descriptor;
use dora_message::{
    BuildId,
    cli_to_coordinator::ControlRequest,
    common::{GitSource, LogMessage},
    coordinator_to_cli::ControlRequestReply,
    id::NodeId,
};
use eyre::{Context, bail};
use std::{
    collections::BTreeMap,
    net::{SocketAddr, TcpStream},
};

use crate::{output::print_log_message, progress::ProgressBar, session::DataflowSession};

pub fn build_distributed_dataflow(
    session: &mut TcpRequestReplyConnection,
    dataflow: Descriptor,
    git_sources: &BTreeMap<NodeId, GitSource>,
    dataflow_session: &DataflowSession,
    local_working_dir: Option<std::path::PathBuf>,
    uv: bool,
) -> eyre::Result<BuildId> {
    let pb = ProgressBar::new_spinner("Triggering distributed build...");

    let build_id = {
        let reply_raw = session
            .request(
                &serde_json::to_vec(&ControlRequest::Build {
                    session_id: dataflow_session.session_id,
                    dataflow,
                    git_sources: git_sources.clone(),
                    prev_git_sources: dataflow_session.git_sources.clone(),
                    local_working_dir,
                    uv,
                })
                .unwrap(),
            )
            .wrap_err("failed to send start dataflow message")?;

        let result: ControlRequestReply =
            serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
        match result {
            ControlRequestReply::DataflowBuildTriggered { build_id } => {
                pb.finish_with_message(format!("Dataflow build triggered: {}", build_id));
                build_id
            }
            ControlRequestReply::Error(err) => {
                pb.fail_with_message("Failed to trigger build");
                bail!("{err}")
            }
            other => {
                pb.fail_with_message("Unexpected response");
                bail!("unexpected start dataflow reply: {other:?}")
            }
        }
    };
    Ok(build_id)
}

pub fn wait_until_dataflow_built(
    build_id: BuildId,
    session: &mut TcpRequestReplyConnection,
    coordinator_socket: SocketAddr,
    log_level: log::LevelFilter,
) -> eyre::Result<BuildId> {
    let pb = ProgressBar::new_spinner("Building dataflow on remote machines...");

    // subscribe to log messages
    let mut log_session = TcpConnection {
        stream: TcpStream::connect(coordinator_socket)
            .wrap_err("failed to connect to dora coordinator")?,
    };
    log_session
        .send(
            &serde_json::to_vec(&ControlRequest::BuildLogSubscribe {
                build_id,
                level: log_level,
            })
            .wrap_err("failed to serialize message")?,
        )
        .wrap_err("failed to send build log subscribe request to coordinator")?;

    let pb_clone = ProgressBar::new_spinner("Processing build logs...");
    std::thread::spawn(move || {
        while let Ok(raw) = log_session.receive() {
            let parsed: eyre::Result<LogMessage> =
                serde_json::from_slice(&raw).context("failed to parse log message");
            match parsed {
                Ok(log_message) => {
                    // Update progress bar with log info
                    if let Some(node_id) = &log_message.node_id {
                        let msg = format!("{}: {}", node_id, log_message.message);
                        pb_clone.set_message(msg);
                    } else {
                        pb_clone.set_message(log_message.message.clone());
                    }
                    print_log_message(log_message, false, true);
                }
                Err(err) => {
                    tracing::warn!("failed to parse log message: {err:?}")
                }
            }
        }
        pb_clone.finish_and_clear();
    });

    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::WaitForBuild { build_id }).unwrap())
        .wrap_err("failed to send WaitForBuild message")?;

    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        ControlRequestReply::DataflowBuildFinished { build_id, result } => match result {
            Ok(()) => {
                pb.finish_with_message("Dataflow build finished successfully");
                Ok(build_id)
            }
            Err(err) => {
                pb.fail_with_message("Build failed");
                bail!("{err}")
            }
        },
        ControlRequestReply::Error(err) => {
            pb.fail_with_message("Build error");
            bail!("{err}")
        }
        other => {
            pb.fail_with_message("Unexpected response");
            bail!("unexpected start dataflow reply: {other:?}")
        }
    }
}
