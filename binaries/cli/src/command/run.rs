use dora_daemon::{flume, Daemon, LogDestination};
use eyre::Context;
use tokio::runtime::Builder;

use crate::{
    handle_dataflow_result, output::print_log_message, resolve_dataflow, session::DataflowSession,
};

pub fn run(dataflow: String, uv: bool) -> Result<(), eyre::Error> {
    let dataflow_path = resolve_dataflow(dataflow).context("could not resolve dataflow")?;
    let dataflow_session =
        DataflowSession::read_session(&dataflow_path).context("failed to read DataflowSession")?;
    let rt = Builder::new_multi_thread()
        .enable_all()
        .build()
        .context("tokio runtime failed")?;

    let (log_tx, log_rx) = flume::bounded(100);
    std::thread::spawn(move || {
        for message in log_rx {
            print_log_message(message, false, false);
        }
    });

    let result = rt.block_on(Daemon::run_dataflow(
        &dataflow_path,
        dataflow_session.build_id,
        dataflow_session.local_build,
        dataflow_session.session_id,
        uv,
        LogDestination::Channel { sender: log_tx },
    ))?;
    handle_dataflow_result(result, None)
}
