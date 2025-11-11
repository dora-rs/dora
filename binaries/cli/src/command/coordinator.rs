use super::Executable;
use crate::LISTEN_WILDCARD;
use dora_coordinator::Event;
use dora_core::topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, DORA_COORDINATOR_PORT_DEFAULT};

#[cfg(feature = "tracing")]
use dora_tracing::TracingBuilder;

use eyre::Context;
use std::net::{IpAddr, SocketAddr};
use tokio::runtime::Builder;
use tracing::level_filters::LevelFilter;

#[derive(Debug, clap::Args)]
/// Run coordinator
pub struct Coordinator {
    /// Network interface to bind to for daemon communication
    #[clap(long, default_value_t = LISTEN_WILDCARD)]
    interface: IpAddr,
    /// Port number to bind to for daemon communication
    #[clap(long, default_value_t = DORA_COORDINATOR_PORT_DEFAULT)]
    port: u16,
    /// Network interface to bind to for control communication
    #[clap(long, default_value_t = LISTEN_WILDCARD)]
    control_interface: IpAddr,
    /// Port number to bind to for control communication
    #[clap(long, default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    control_port: u16,
    /// Suppresses all log output to stdout.
    #[clap(long)]
    quiet: bool,
}

impl Executable for Coordinator {
    fn execute(self) -> eyre::Result<()> {
        #[cfg(feature = "tracing")]
        {
            let name = "dora-coordinator";
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
            let bind_control = SocketAddr::new(self.control_interface, self.control_port);
            let (port, task) =
                dora_coordinator::start(bind, bind_control, futures::stream::empty::<Event>())
                    .await?;
            if !self.quiet {
                println!("Listening for incoming daemon connection on {port}");
            }
            task.await
        })
        .context("failed to run dora-coordinator")
    }
}
