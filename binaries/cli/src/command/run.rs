//! The `dora run` command is a quick and easy way to run a dataflow locally.
//! It does not support distributed dataflows and will throw an error if there are any `deploy` keys in the YAML file.
//!
//! The `dora run` command does not interact with any `dora coordinator` or `dora daemon` instances, or with any other parallel `dora run` commands.
//!
//! Use `dora build --local` or manual build commands to build your nodes.

use super::Executable;
use crate::{
    common::{handle_dataflow_result, resolve_dataflow, write_events_to},
    output::print_log_message,
    session::DataflowSession,
};
use dora_daemon::{Daemon, LogDestination, flume};
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

#[deprecated(note = "use `run` instead")]
pub fn run_func(dataflow: String, uv: bool) -> eyre::Result<()> {
    run(dataflow, uv)
}

pub fn run(dataflow: String, uv: bool) -> eyre::Result<()> {
    let rt = Builder::new_multi_thread()
        .enable_all()
        .build()
        .context("tokio runtime failed")?;
    #[cfg(feature = "tracing")]
    {
        use std::sync::{Arc, Mutex};

        use dora_tracing::OtelGuard;

        let guard: Arc<Mutex<Option<OtelGuard>>> = Arc::new(Mutex::new(None));
        let clone = guard.clone();
        rt.spawn(async move {
            let mut builder = TracingBuilder::new("dora-run");
            if std::env::var("DORA_OTLP_ENDPOINT").is_ok()
                || std::env::var("DORA_JAEGER_TRACING").is_ok()
            {
                builder = builder
                    .with_otlp_tracing()
                    .context("failed to set up OTLP tracing")
                    .unwrap();
                *clone.lock().unwrap() = builder.guard.take();
            } else {
                let env_log = std::env::var("RUST_LOG").unwrap_or("info".to_string());
                builder = builder.with_stdout(env_log, false);
            }
            builder
                .build()
                .wrap_err("failed to set up tracing subscriber")
                .unwrap();
            loop {
                tokio::time::sleep(std::time::Duration::from_secs(1)).await;
            }
        });
    }

    let dataflow_path = resolve_dataflow(dataflow).context("could not resolve dataflow")?;
    let dataflow_session =
        DataflowSession::read_session(&dataflow_path).context("failed to read DataflowSession")?;

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
        write_events_to(),
    ))?;
    handle_dataflow_result(result, None)
}

impl Executable for Run {
    fn execute(self) -> eyre::Result<()> {
        run(self.dataflow, self.uv)
    }
}
