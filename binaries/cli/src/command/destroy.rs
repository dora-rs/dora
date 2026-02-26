use super::{Executable, default_tracing, up};
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use std::net::IpAddr;
use std::path::PathBuf;

#[derive(Debug, clap::Args)]
/// Destroy running coordinator and daemon. If some dataflows are still running, they will be stopped first.
pub struct Destroy {
    /// Use a custom configuration
    #[clap(long, hide = true)]
    config: Option<PathBuf>,
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP")]
    coordinator_addr: Option<IpAddr>,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT")]
    coordinator_port: Option<u16>,
}

impl Executable for Destroy {
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        use crate::common::resolve_coordinator_addr;
        let (addr, port) = resolve_coordinator_addr(
            self.coordinator_addr,
            self.coordinator_port,
            DORA_COORDINATOR_PORT_CONTROL_DEFAULT,
        );

        up::destroy(self.config.as_deref(), (addr, port).into()).await
    }
}
