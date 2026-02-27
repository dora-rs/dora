use super::{Executable, default_tracing, up};
use adora_core::topics::{ADORA_COORDINATOR_PORT_WS_DEFAULT, LOCALHOST};
use std::net::IpAddr;
use std::path::PathBuf;

#[derive(Debug, clap::Args)]
/// Tear down coordinator and daemon. Stops any running dataflows first.
pub struct Destroy {
    /// Use a custom configuration
    #[clap(long, hide = true)]
    config: Option<PathBuf>,
    /// Address of the adora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST, env = "ADORA_COORDINATOR_ADDR")]
    coordinator_addr: IpAddr,
    /// Port number of the coordinator
    #[clap(long, value_name = "PORT", default_value_t = ADORA_COORDINATOR_PORT_WS_DEFAULT, env = "ADORA_COORDINATOR_PORT")]
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
