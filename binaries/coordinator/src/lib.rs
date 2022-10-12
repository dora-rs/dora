use eyre::WrapErr;
use futures::StreamExt;
use futures_concurrency::stream::Merge;
use std::path::{Path, PathBuf};
use tokio_stream::wrappers::ReceiverStream;

mod control;
mod run;

#[derive(Debug, Clone, clap::Parser)]
#[clap(about = "Dora coordinator")]
pub struct Args {
    #[clap(long)]
    pub runtime: Option<PathBuf>,
    #[clap(long)]
    pub run_dataflow: Option<PathBuf>,
}

pub async fn run(args: Args) -> eyre::Result<()> {
    let Args {
        runtime,
        run_dataflow,
    } = args;

    let runtime_path = runtime.unwrap_or_else(|| {
        std::env::args()
            .next()
            .map(PathBuf::from)
            .unwrap_or_default()
            .with_file_name("dora-runtime")
    });

    match run_dataflow {
        Some(path) => {
            // start the given dataflow directly
            run::run_dataflow(path.clone(), &runtime_path)
                .await
                .wrap_err_with(|| format!("failed to run dataflow at {}", path.display()))?;
        }
        None => {
            // start in daemon mode
            start(&runtime_path).await?;
        }
    }

    Ok(())
}

async fn start(runtime_path: &Path) -> eyre::Result<()> {
    let (dataflow_errors_tx, dataflow_errors) = tokio::sync::mpsc::channel(2);
    let mut dataflow_errors_tx = Some(dataflow_errors_tx);
    let dataflow_error_events = ReceiverStream::new(dataflow_errors).map(Event::DataflowError);

    let control_events = control::control_events();

    let mut events = (dataflow_error_events, control_events).merge();

    while let Some(event) = events.next().await {
        let mut stop = false;
        match event {
            Event::DataflowError(err) => {
                tracing::error!("{err:?}");
            }
            Event::ParseError(err) => {
                let err = err.wrap_err("failed to parse message");
                tracing::error!("{err:?}");
            }
            Event::ControlChannelError(err) => {
                let err = err.wrap_err("Stopping because of fatal control channel error");
                tracing::error!("{err:?}");
                stop = true;
            }
            Event::StartDataflow { path } => {
                let runtime_path = runtime_path.to_owned();
                let dataflow_errors_tx = match &dataflow_errors_tx {
                    Some(channel) => channel.clone(),
                    None => {
                        tracing::error!("cannot start new dataflow after receiving stop command");
                        continue;
                    }
                };
                let task = async move {
                    let result = run::run_dataflow(path.clone(), &runtime_path)
                        .await
                        .wrap_err_with(|| format!("failed to run dataflow at {}", path.display()));
                    match result {
                        Ok(()) => {}
                        Err(err) => {
                            let _ = dataflow_errors_tx.send(err).await;
                        }
                    }
                };
                tokio::spawn(task);
            }
            Event::Stop => {
                tracing::info!("Received stop command");
                stop = true;
            }
        }

        if stop {
            tracing::info!("stopping...");

            // ensure that no new dataflows can be started
            dataflow_errors_tx = None;
        }
    }

    tracing::info!("stopped");

    Ok(())
}

enum Event {
    DataflowError(eyre::Report),
    ControlChannelError(eyre::Report),
    StartDataflow { path: PathBuf },
    ParseError(eyre::Report),
    Stop,
}
