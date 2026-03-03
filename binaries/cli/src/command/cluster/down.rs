use clap::Args;

use crate::{
    command::{Executable, default_tracing, up},
    common::CoordinatorOptions,
};

/// Tear down the cluster (coordinator and all daemons).
///
/// Examples:
///
///   adora cluster down
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Down {
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Down {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        up::down(None, self.coordinator.socket_addr())
    }
}
