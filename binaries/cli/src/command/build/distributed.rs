use dora_core::descriptor::Descriptor;
use dora_message::{
    BuildId,
    cli_to_coordinator::{BuildRequest, CoordinatorControlClient},
    common::GitSource,
    id::NodeId,
    tarpc,
};
use eyre::Context;
use std::{collections::BTreeMap, net::IpAddr};

use crate::common::{long_context, rpc};
use crate::output::{abort_log_task_with_grace, subscribe_and_print_logs};
use crate::session::DataflowSession;

/// Trigger a distributed build and wait for it to complete.
///
/// The build ID is generated on the CLI side so we can subscribe to the
/// zenoh log topic *before* sending the build RPC, ensuring no early log
/// messages are missed.
pub async fn build_distributed_dataflow(
    client: &CoordinatorControlClient,
    dataflow: Descriptor,
    git_sources: &BTreeMap<NodeId, GitSource>,
    dataflow_session: &DataflowSession,
    local_working_dir: Option<std::path::PathBuf>,
    uv: bool,
    coordinator_addr: IpAddr,
    log_level: log::LevelFilter,
) -> eyre::Result<BuildId> {
    let build_id = BuildId::generate();

    // Subscribe to build log messages via zenoh *before* triggering the
    // build so we don't miss early messages.
    let zenoh_session = dora_core::topics::open_zenoh_session(Some(coordinator_addr))
        .await
        .wrap_err("failed to open zenoh session for build log subscription")?;
    let base_topic = dora_core::topics::zenoh_log_base_topic_for_build(&build_id);
    let log_task =
        subscribe_and_print_logs(&zenoh_session, &base_topic, log_level, false, true).await?;

    // Now trigger the build — the daemon may start publishing logs
    // immediately, but our subscriber is already active.
    rpc(
        "trigger build",
        client.build(
            tarpc::context::current(),
            BuildRequest {
                build_id: Some(build_id),
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

    // Wait for the build to finish, then clean up the log subscriber.
    let result = rpc(
        "wait for build",
        client.wait_for_build(long_context(), build_id),
    )
    .await;
    abort_log_task_with_grace(log_task).await;
    result?;
    eprintln!("dataflow build finished successfully");
    Ok(build_id)
}
