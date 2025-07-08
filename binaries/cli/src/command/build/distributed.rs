use communication_layer_request_reply::{TcpConnection, TcpRequestReplyConnection};
use dora_core::descriptor::Descriptor;
use dora_message::{
    cli_to_coordinator::ControlRequest,
    common::{GitSource, LogMessage},
    coordinator_to_cli::ControlRequestReply,
    id::NodeId,
    BuildId,
};
use eyre::{bail, Context};
use std::{
    collections::BTreeMap,
    net::{SocketAddr, TcpStream},
};

use crate::{output::print_log_message, session::DataflowSession};

pub fn build_distributed_dataflow(
    session: &mut TcpRequestReplyConnection,
    dataflow: Descriptor,
    git_sources: &BTreeMap<NodeId, GitSource>,
    dataflow_session: &DataflowSession,
    local_working_dir: Option<std::path::PathBuf>,
    uv: bool,
) -> eyre::Result<BuildId> {
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
                eprintln!("dataflow build triggered: {build_id}");
                build_id
            }
            ControlRequestReply::Error(err) => bail!("{err}"),
            other => bail!("unexpected start dataflow reply: {other:?}"),
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
    std::thread::spawn(move || {
        while let Ok(raw) = log_session.receive() {
            let parsed: eyre::Result<LogMessage> =
                serde_json::from_slice(&raw).context("failed to parse log message");
            match parsed {
                Ok(log_message) => {
                    print_log_message(log_message, false, true);
                }
                Err(err) => {
                    tracing::warn!("failed to parse log message: {err:?}")
                }
            }
        }
    });

    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::WaitForBuild { build_id }).unwrap())
        .wrap_err("failed to send WaitForBuild message")?;

    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        ControlRequestReply::DataflowBuildFinished { build_id, result } => match result {
            Ok(()) => {
                eprintln!("dataflow build finished successfully");
                Ok(build_id)
            }
            Err(err) => bail!("{err}"),
        },
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected start dataflow reply: {other:?}"),
    }
}
