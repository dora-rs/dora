pub use build::build;
pub use logs::logs;
pub use run::run;
pub use start::start;

use std::path::{Path, PathBuf};

use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_core::descriptor::Descriptor;
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use eyre::{bail, Context, ContextCompat};

mod build;
pub mod check;
mod logs;
mod run;
mod start;
pub mod up;

fn local_working_dir(
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

fn cli_and_daemon_on_same_machine(session: &mut TcpRequestReplyConnection) -> eyre::Result<bool> {
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
