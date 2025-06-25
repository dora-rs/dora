use super::Executable;
use crate::common::{handle_dataflow_result, resolve_dataflow};
use dora_daemon::Daemon;
use dora_tracing::TracingBuilder;
use eyre::Context;
use tokio::runtime::Builder;

#[derive(Debug, clap::Args)]
/// Run a dataflow locally.
///
/// Directly runs the given dataflow without connecting to a dora
/// coordinator or daemon. The dataflow is executed on the local machine.
pub struct Run {
    /// Path to the dataflow descriptor file
    #[clap(value_name = "PATH")]
    dataflow: String,
    // Use UV to run nodes.
    #[clap(long, action)]
    uv: bool,
}

impl Executable for Run {
    fn execute(self) -> eyre::Result<()> {
        #[cfg(feature = "tracing")]
        {
            let log_level = std::env::var("RUST_LOG").ok().unwrap_or("info".to_string());
            TracingBuilder::new("run")
                .with_stdout(log_level)
                .build()
                .wrap_err("failed to set up tracing subscriber")?;
        }

        let dataflow_path =
            resolve_dataflow(self.dataflow).context("could not resolve dataflow")?;
        let rt = Builder::new_multi_thread()
            .enable_all()
            .build()
            .context("tokio runtime failed")?;
        let result = rt.block_on(Daemon::run_dataflow(&dataflow_path, self.uv))?;
        handle_dataflow_result(result, None)
    }
}
