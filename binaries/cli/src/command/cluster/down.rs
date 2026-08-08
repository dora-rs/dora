use clap::Args;

use crate::{
    command::{Executable, default_tracing, up},
    common::CoordinatorOptions,
};

/// Tear down the cluster (coordinator and all daemons).
///
/// Examples:
///
///   dora cluster down
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Down {
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Down {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        // `dora cluster down` keeps its existing unconditional behavior: it
        // is an explicit cluster teardown, and the #2924 report is about
        // `dora down`/`destroy` hitting an unrelated instance on the shared
        // default port. Revisit if the same footgun shows up here.
        up::down(None, self.coordinator.socket_addr(), true)
    }
}
