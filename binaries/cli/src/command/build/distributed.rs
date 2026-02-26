use dora_core::{
    descriptor::Descriptor,
    topics::zenoh_log_topic_for_build,
};
use dora_message::{
    BuildId,
    cli_to_coordinator::{BuildRequest, CoordinatorControlClient},
    common::{GitSource, LogMessage},
    id::NodeId,
    tarpc,
};
use eyre::Context;
use std::{collections::BTreeMap, net::IpAddr};

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
    coordinator_addr: IpAddr,
    log_level: log::LevelFilter,
) -> eyre::Result<BuildId> {
    // Subscribe to build log messages via zenoh
    let zenoh_session = dora_core::topics::open_zenoh_session(Some(coordinator_addr))
        .await
        .wrap_err("failed to open zenoh session for build log subscription")?;
    let log_topic = zenoh_log_topic_for_build(&build_id);
    let subscriber = zenoh_session
        .declare_subscriber(&log_topic)
        .await
        .map_err(|e| eyre::eyre!(e))
        .wrap_err("failed to subscribe to build log topic")?;
    tokio::spawn(async move {
        loop {
            match subscriber.recv_async().await {
                Ok(sample) => {
                    let payload = sample.payload().to_bytes();
                    let parsed: eyre::Result<LogMessage> = serde_json::from_slice(&payload)
                        .context("failed to parse log message from zenoh");
                    match parsed {
                        Ok(log_message) => {
                            if should_display(&log_message, log_level) {
                                print_log_message(log_message, false, true);
                            }
                        }
                        Err(err) => {
                            tracing::warn!("failed to parse log message: {err:?}")
                        }
                    }
                }
                Err(_) => break,
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

/// Check whether a log message should be displayed given the log level filter.
fn should_display(message: &LogMessage, filter: log::LevelFilter) -> bool {
    match &message.level {
        dora_core::build::LogLevelOrStdout::Stdout => true,
        dora_core::build::LogLevelOrStdout::LogLevel(level) => *level <= filter,
    }
}
