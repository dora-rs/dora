use dora_core::{
    daemon_messages::Timestamped, message::uhlc::HLC, topics::DORA_COORDINATOR_PORT_DEFAULT,
};
use dora_daemon::{Daemon, Event};
#[cfg(feature = "tracing")]
use dora_tracing::set_up_tracing;
#[cfg(feature = "tracing")]
use eyre::Context;
use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;

use std::{
    net::{Ipv4Addr, SocketAddr},
    path::PathBuf,
};

#[derive(Debug, Clone, clap::Parser)]
#[clap(about = "Dora daemon")]
pub struct Args {
    #[clap(long)]
    pub machine_id: Option<String>,

    #[clap(long)]
    pub coordinator_addr: Option<SocketAddr>,

    #[clap(long)]
    pub run_dataflow: Option<PathBuf>,

    #[clap(long)]
    pub run_dora_runtime: bool,
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    // the tokio::main proc macro confuses some tools such as rust-analyzer, so
    // directly invoke a "normal" async function
    run().await
}

async fn run() -> eyre::Result<()> {
    let Args {
        run_dataflow,
        machine_id,
        coordinator_addr,
        run_dora_runtime,
    } = clap::Parser::parse();

    if run_dora_runtime {
        return tokio::task::block_in_place(dora_daemon::run_dora_runtime);
    }

    #[cfg(feature = "tracing")]
    set_up_tracing("dora-daemon").wrap_err("failed to set up tracing subscriber")?;

    let ctrl_c_events = {
        let (ctrl_c_tx, ctrl_c_rx) = mpsc::channel(1);
        let mut ctrlc_sent = false;
        ctrlc::set_handler(move || {
            let clock = HLC::default();
            if ctrlc_sent {
                tracing::warn!("received second ctrlc signal -> aborting immediately");
                std::process::abort();
            } else {
                tracing::info!("received ctrlc signal");
                let event = Timestamped {
                    inner: Event::CtrlC,
                    timestamp: clock.new_timestamp(),
                };
                if ctrl_c_tx.blocking_send(event).is_err() {
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

            Daemon::run_dataflow(&dataflow_path).await
        }
        None => {
            Daemon::run(
                coordinator_addr.unwrap_or_else(|| {
                    tracing::info!("Starting in local mode");
                    let localhost = Ipv4Addr::new(127, 0, 0, 1);
                    (localhost, DORA_COORDINATOR_PORT_DEFAULT).into()
                }),
                machine_id.unwrap_or_default(),
                ctrl_c_events,
            )
            .await
        }
    }
}
