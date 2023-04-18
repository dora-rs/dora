use dora_core::topics::DORA_COORDINATOR_PORT_DEFAULT;
use dora_daemon::{Daemon, Event};
#[cfg(feature = "tracing")]
use dora_tracing::set_up_tracing;
#[cfg(feature = "tracing")]
use eyre::Context;
use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;

use std::{net::Ipv4Addr, path::PathBuf};

#[derive(Debug, Clone, clap::Parser)]
#[clap(about = "Dora daemon")]
pub struct Args {
    #[clap(long)]
    pub run_dataflow: Option<PathBuf>,

    #[clap(long)]
    pub dora_runtime_path: Option<PathBuf>,
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    // the tokio::main proc macro confuses some tools such as rust-analyzer, so
    // directly invoke a "normal" async function
    run().await
}

async fn run() -> eyre::Result<()> {
    #[cfg(feature = "tracing")]
    set_up_tracing("dora-daemon").wrap_err("failed to set up tracing subscriber")?;

    let Args {
        run_dataflow,
        dora_runtime_path,
    } = clap::Parser::parse();

    let ctrl_c_events = {
        let (ctrl_c_tx, ctrl_c_rx) = mpsc::channel(1);
        let mut ctrlc_sent = false;
        ctrlc::set_handler(move || {
            if ctrlc_sent {
                tracing::warn!("received second ctrlc signal -> aborting immediately");
                std::process::abort();
            } else {
                tracing::info!("received ctrlc signal");
                if ctrl_c_tx.blocking_send(Event::CtrlC).is_err() {
                    tracing::error!("failed to report ctrl-c event to dora-daemon");
                }
                ctrlc_sent = true;
            }
        })
        .wrap_err("failed to set ctrl-c handler")?;
        ReceiverStream::new(ctrl_c_rx)
    };

    match run_dataflow {
        Some(dataflow_path) => {
            tracing::info!("Starting dataflow `{}`", dataflow_path.display());

            Daemon::run_dataflow(&dataflow_path, dora_runtime_path).await
        }
        None => {
            tracing::info!("Starting in local mode");
            let localhost = Ipv4Addr::new(127, 0, 0, 1);
            let coordinator_socket = (localhost, DORA_COORDINATOR_PORT_DEFAULT);

            let machine_id = String::new(); // TODO

            Daemon::run(
                coordinator_socket.into(),
                machine_id,
                dora_runtime_path,
                ctrl_c_events,
            )
            .await
        }
    }
}
