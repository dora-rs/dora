use communication_layer_request_reply::{
    Transport, encoding::JsonEncoding, transport::FramedTransport,
};
use dora_core::descriptor::Descriptor;
use dora_message::{
    BuildId,
    cli_to_coordinator::{CliToCoordinatorClient, CliToCoordinatorRequest},
    common::{GitSource, LogMessage},
    id::NodeId,
};
use eyre::{Context, bail};
use std::{
    collections::BTreeMap,
    net::{SocketAddr, TcpStream},
};

use crate::{output::print_log_message, session::DataflowSession};

pub fn build_distributed_dataflow(
    coordinator_client: &mut CliToCoordinatorClient,
    dataflow: Descriptor,
    git_sources: &BTreeMap<NodeId, GitSource>,
    dataflow_session: &DataflowSession,
    local_working_dir: Option<std::path::PathBuf>,
    uv: bool,
) -> eyre::Result<BuildId> {
    let build_id = coordinator_client.build(dora_message::cli_to_coordinator::BuildReq {
        session_id: dataflow_session.session_id,
        dataflow,
        git_sources: git_sources.clone(),
        prev_git_sources: dataflow_session.git_sources.clone(),
        local_working_dir,
        uv,
    })?;
    Ok(build_id)
}

pub fn wait_until_dataflow_built(
    build_id: BuildId,
    coordinator_client: &mut CliToCoordinatorClient,
    coordinator_socket: SocketAddr,
    log_level: log::LevelFilter,
) -> eyre::Result<BuildId> {
    // subscribe to log messages
    let mut log_session = FramedTransport::new(
        TcpStream::connect(coordinator_socket).wrap_err("failed to connect to dora coordinator")?,
    )
    .with_encoding::<_, CliToCoordinatorRequest, LogMessage>(JsonEncoding);
    log_session
        .send(&CliToCoordinatorRequest::BuildLogSubscribe {
            build_id,
            level: log_level,
        })
        .wrap_err("failed to send build log subscribe request to coordinator")?;
    std::thread::spawn(move || {
        while let Ok(Some(message)) = log_session.receive() {
            print_log_message(message, false, true);
        }
    });

    let resp = coordinator_client.wait_for_build(build_id)?;
    match resp.result {
        Ok(()) => {
            eprintln!("dataflow build finished successfully");
            Ok(resp.build_id)
        }
        Err(err) => bail!("{err}"),
    }
}
