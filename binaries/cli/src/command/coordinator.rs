use super::Executable;
use crate::LISTEN_DEFAULT;
use adora_coordinator::Event;
use adora_core::topics::ADORA_COORDINATOR_PORT_WS_DEFAULT;

#[cfg(feature = "tracing")]
use adora_tracing::TracingBuilder;

use eyre::Context;
use std::net::{IpAddr, SocketAddr};
use tokio::runtime::Builder;
use tracing::level_filters::LevelFilter;

#[derive(Debug, clap::Args)]
/// Run coordinator
pub struct Coordinator {
    /// Network interface to bind to
    #[clap(long, default_value_t = LISTEN_DEFAULT)]
    interface: IpAddr,
    /// Port number to bind to
    #[clap(long, default_value_t = ADORA_COORDINATOR_PORT_WS_DEFAULT)]
    port: u16,
    /// Suppresses all log output to stdout.
    #[clap(long)]
    quiet: bool,
}

impl Executable for Coordinator {
    fn execute(self) -> eyre::Result<()> {
        #[cfg(feature = "tracing")]
        {
            let name = "adora-coordinator";
            let mut builder = TracingBuilder::new(name);
            if !self.quiet {
                builder = builder.with_stdout("info", false);
            }
            builder = builder.with_file(name, LevelFilter::INFO)?;
            builder
                .build()
                .wrap_err("failed to set up tracing subscriber")?;
        }

        let rt = Builder::new_multi_thread()
            .enable_all()
            .build()
            .context("tokio runtime failed")?;
        rt.block_on(async {
            let bind = SocketAddr::new(self.interface, self.port);
            let (port, task) =
                adora_coordinator::start(bind, futures::stream::empty::<Event>()).await?;
            if !self.quiet {
                println!("Listening on port {port}");
            }
            task.await
        })
        .context("failed to run adora-coordinator")
    }
}
