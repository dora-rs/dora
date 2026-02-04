use communication_layer_request_reply::{
    RequestReplyConnection, TcpConnection, TcpRequestReplyConnection, TypedRequestReplyConnection,
};
use dora_core::descriptor::Descriptor;
use dora_message::{
    BuildId,
    cli_to_coordinator::{
        BuildLogSubscribe, BuildRequest, ControlRequest, DataflowBuildTriggered, WaitForBuild,
    },
    common::{GitSource, LogMessage},
    coordinator_to_cli::{ControlRequestReply, DataflowBuildFinished},
    id::NodeId,
};
use eyre::{Context, bail};
use std::{
    collections::BTreeMap,
    net::{SocketAddr, TcpStream},
};

use crate::{output::print_log_message, session::DataflowSession};

pub fn build_distributed_dataflow(
    session: &mut impl TypedRequestReplyConnection<RequestEnum = ControlRequest, Error = std::io::Error>,
    dataflow: Descriptor,
    git_sources: &BTreeMap<NodeId, GitSource>,
    dataflow_session: &DataflowSession,
    local_working_dir: Option<std::path::PathBuf>,
    uv: bool,
) -> eyre::Result<BuildId> {
    let build_id = {
        let result = session
            .request(&BuildRequest {
                session_id: dataflow_session.session_id,
                dataflow,
                git_sources: git_sources.clone(),
                prev_git_sources: dataflow_session.git_sources.clone(),
                local_working_dir,
                uv,
            })
            .wrap_err("failed to send start dataflow message")?;

        match result {
            Ok(DataflowBuildTriggered { build_id }) => {
                eprintln!("dataflow build triggered: {build_id}");
                build_id
            }
            Err(err) => bail!("{err}"),
            other => bail!("unexpected start dataflow reply: {other:?}"),
        }
    };
    Ok(build_id)
}

pub fn wait_until_dataflow_built(
    build_id: BuildId,
    session: &mut impl TypedRequestReplyConnection<RequestEnum = ControlRequest, Error = std::io::Error>,
    coordinator_socket: SocketAddr,
    log_level: log::LevelFilter,
) -> eyre::Result<BuildId> {
    // subscribe to log messages
    let mut log_session = TcpConnection {
        stream: TcpStream::connect(coordinator_socket)
            .wrap_err("failed to connect to dora coordinator")?,
    };
    let log_session: impl TypedRequestReplyConnection<
        RequestEnum = ControlRequest,
        Error = std::io::Error,
    > = todo!();
    let reply = log_session
        .request(&BuildLogSubscribe {
            build_id,
            level: log_level,
        })
        .wrap_err("failed to send build log subscribe request to coordinator")?;
    match reply {
        Ok(()) => {}
        Err(err) => bail!("failed to subscribe to build log messages: {err}"),
    }
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

    let reply = session
        .request(&WaitForBuild { build_id })
        .wrap_err("failed to send WaitForBuild message")?;

    match reply {
        Ok(DataflowBuildFinished(DataflowBuildFinished { build_id, result })) => match result {
            Ok(()) => {
                eprintln!("dataflow build finished successfully");
                Ok(build_id)
            }
            Err(err) => bail!("{err}"),
        },
        Err(err) => bail!("{err}"),
        other => bail!("unexpected start dataflow reply: {other:?}"),
    }
}
