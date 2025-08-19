use crate::formatting::FormatDataflowError;
use communication_layer_request_reply::{RequestReplyLayer, TcpLayer, TcpRequestReplyConnection};
use dora_core::descriptor::{Descriptor, source_is_url};
use dora_download::download_file;
use dora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{ControlRequestReply, DataflowList, DataflowResult},
};
use eyre::{Context, ContextCompat, bail};
use std::{
    env::current_dir,
    net::SocketAddr,
    path::{Path, PathBuf},
};
use tokio::runtime::Builder;
use uuid::Uuid;

pub(crate) fn handle_dataflow_result(
    result: DataflowResult,
    uuid: Option<Uuid>,
) -> Result<(), eyre::Error> {
    if result.is_ok() {
        Ok(())
    } else {
        Err(match uuid {
            Some(uuid) => {
                eyre::eyre!("Dataflow {uuid} failed:\n{}", FormatDataflowError(&result))
            }
            None => {
                eyre::eyre!("Dataflow failed:\n{}", FormatDataflowError(&result))
            }
        })
    }
}

pub(crate) fn query_running_dataflows(
    session: &mut TcpRequestReplyConnection,
) -> eyre::Result<DataflowList> {
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::List).unwrap())
        .wrap_err("failed to send list message")?;
    let reply: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    let ids = match reply {
        ControlRequestReply::DataflowList(list) => list,
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected list dataflow reply: {other:?}"),
    };

    Ok(ids)
}

pub(crate) fn connect_to_coordinator(
    coordinator_addr: SocketAddr,
) -> std::io::Result<Box<TcpRequestReplyConnection>> {
    TcpLayer::new().connect(coordinator_addr)
}

pub(crate) fn resolve_dataflow(dataflow: String) -> eyre::Result<PathBuf> {
    let dataflow = if source_is_url(&dataflow) {
        // try to download the shared library
        let target_path = current_dir().context("Could not access the current dir")?;
        let rt = Builder::new_current_thread()
            .enable_all()
            .build()
            .context("tokio runtime failed")?;
        rt.block_on(async { download_file(&dataflow, &target_path).await })
            .wrap_err("failed to download dataflow yaml file")?
    } else {
        PathBuf::from(dataflow)
    };
    Ok(dataflow)
}

pub(crate) fn local_working_dir(
    dataflow_path: &Path,
    dataflow_descriptor: &Descriptor,
    coordinator_session: &mut TcpRequestReplyConnection,
) -> eyre::Result<Option<PathBuf>> {
    Ok(
        if dataflow_descriptor
            .nodes
            .iter()
            .all(|n| n.deploy.as_ref().map(|d| d.machine.as_ref()).is_none())
            && cli_and_daemon_on_same_machine(coordinator_session)?
        {
            Some(
                dunce::canonicalize(dataflow_path)
                    .context("failed to canonicalize dataflow file path")?
                    .parent()
                    .context("dataflow path has no parent dir")?
                    .to_owned(),
            )
        } else {
            None
        },
    )
}

pub(crate) fn cli_and_daemon_on_same_machine(
    session: &mut TcpRequestReplyConnection,
) -> eyre::Result<bool> {
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::CliAndDefaultDaemonOnSameMachine).unwrap())
        .wrap_err("failed to send start dataflow message")?;

    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        ControlRequestReply::CliAndDefaultDaemonIps {
            default_daemon,
            cli,
        } => Ok(default_daemon.is_some() && default_daemon == cli),
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected start dataflow reply: {other:?}"),
    }
}
