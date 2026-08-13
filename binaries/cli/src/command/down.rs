use super::{Executable, default_tracing, up};
use crate::common::CoordinatorOptions;
use std::path::PathBuf;

#[derive(Debug, clap::Args)]
/// Tear down coordinator and daemon. Stops any running dataflows first.
pub struct Down {
    /// Use a custom configuration
    #[clap(long, hide = true)]
    config: Option<PathBuf>,
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
    /// Destroy even if the coordinator still has running dataflows,
    /// terminating them immediately first.
    ///
    /// Without this, `dora down` refuses when the target coordinator
    /// reports running dataflows. Lifecycle commands reach whatever
    /// coordinator owns the port, so the instance you hit may belong to
    /// another checkout or project on the same machine
    /// (dora-rs/dora#2924). Give each instance its own
    /// `DORA_COORDINATOR_PORT` to keep them apart.
    ///
    /// This stops each dataflow before tearing the daemons down. `Destroy`
    /// on its own does not wait for the stop to take effect, so a wedged
    /// node would outlive its daemon and be orphaned.
    #[clap(long, action)]
    force: bool,
}

impl Executable for Down {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        up::down(
            self.config.as_deref(),
            self.coordinator.socket_addr(),
            self.force,
        )
    }
}
