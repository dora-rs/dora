use super::{Executable, default_tracing, up};
use crate::common::CoordinatorOptions;
use std::path::PathBuf;

#[derive(Debug, clap::Args)]
/// Tear down coordinator and daemon. Stops any running dataflows first.
pub struct Destroy {
    /// Use a custom configuration
    #[clap(long, hide = true)]
    config: Option<PathBuf>,
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Destroy {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        up::destroy(self.config.as_deref(), self.coordinator.socket_addr())
    }
}
