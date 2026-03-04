use crate::tcp::AsyncTcpConnection;
use dora_core::descriptor::Descriptor;
use dora_message::{
    BuildId,
    cli_to_coordinator::{BuildRequest, CoordinatorControlClient, LegacyControlRequest},
    common::{GitSource, LogMessage},
    id::NodeId,
    tarpc,
};
use eyre::Context;
use std::{collections::BTreeMap, net::SocketAddr};

use crate::common::{long_context, rpc};
use crate::output::print_log_message;
use crate::session::DataflowSession;

pub async fn build_distributed_dataflow(
    client: &CoordinatorControlClient,
    dataflow: Descriptor,
    git_sources: &BTreeMap<NodeId, GitSource>,
    dataflow_session: &DataflowSession,
    local_working_dir: Option<std::path::PathBuf>,
    uv: bool,
) -> eyre::Result<BuildId> {
    let build_id = rpc(
        "trigger build",
        client.build(
            tarpc::context::current(),
            BuildRequest {
                session_id: dataflow_session.session_id,
                dataflow,
                git_sources: git_sources.clone(),
                prev_git_sources: dataflow_session.git_sources.clone(),
                local_working_dir,
                uv,
            },
        ),
    )
    .await?;
    eprintln!("dataflow build triggered: {build_id}");
    Ok(build_id)
}

pub async fn wait_until_dataflow_built(
    build_id: BuildId,
    client: &CoordinatorControlClient,
    coordinator_socket: SocketAddr,
    log_level: log::LevelFilter,
) -> eyre::Result<BuildId> {
    // subscribe to build log messages (TCP streaming)
    let mut log_session = AsyncTcpConnection {
        stream: tokio::net::TcpStream::connect(coordinator_socket)
            .await
            .wrap_err("failed to connect to dora coordinator")?,
    };
    log_session
        .send(
            &serde_json::to_vec(&LegacyControlRequest::BuildLogSubscribe {
                build_id,
                level: log_level,
            })
            .wrap_err("failed to serialize message")?,
        )
        .await
        .wrap_err("failed to send build log subscribe request to coordinator")?;
    tokio::spawn(async move {
        while let Ok(raw) = log_session.receive().await {
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

    rpc(
        "wait for build",
        client.wait_for_build(long_context(), build_id),
    )
    .await?;
    eprintln!("dataflow build finished successfully");
    Ok(build_id)
}
