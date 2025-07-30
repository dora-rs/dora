use super::{Executable, default_tracing, up};
use dora_core::topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST};
use std::net::IpAddr;
use std::path::PathBuf;

#[derive(Debug, clap::Args)]
/// Destroy running coordinator and daemon. If some dataflows are still running, they will be stopped first.
pub struct Destroy {
    /// Use a custom configuration
    #[clap(long, hide = true)]
    config: Option<PathBuf>,
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    coordinator_addr: IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    coordinator_port: u16,
}

impl Executable for Destroy {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        up::destroy(
            self.config.as_deref(),
            (self.coordinator_addr, self.coordinator_port).into(),
        )
    }
}
