use dora_daemon::Daemon;
use eyre::Context;
use tokio::runtime::Builder;

use crate::{handle_dataflow_result, resolve_dataflow, session::DataflowSession};

pub fn run(dataflow: String, uv: bool) -> Result<(), eyre::Error> {
    let dataflow_path = resolve_dataflow(dataflow).context("could not resolve dataflow")?;
    let dataflow_session =
        DataflowSession::read_session(&dataflow_path).context("failed to read DataflowSession")?;
    let rt = Builder::new_multi_thread()
        .enable_all()
        .build()
        .context("tokio runtime failed")?;
    let result = rt.block_on(Daemon::run_dataflow(
        &dataflow_path,
        dataflow_session.build_id,
        dataflow_session.session_id,
        uv,
    ))?;
    handle_dataflow_result(result, None)
}
