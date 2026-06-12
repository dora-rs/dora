//! The `dora hub` subcommand — node packaging, discovery, and distribution
//! (docs/plan-node-hub.md). Phase 1 surface: `init` scaffolds a node
//! manifest; `install` is a deliberate error stub explaining the
//! per-dataflow model (spec §9).

use crate::command::Executable;

mod init;
mod install;

/// Package, discover, and use dora nodes (unstable)
#[derive(Debug, clap::Subcommand)]
pub enum Hub {
    Init(init::Init),
    Install(install::Install),
}

impl Executable for Hub {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Hub::Init(cmd) => cmd.execute(),
            Hub::Install(cmd) => cmd.execute(),
        }
    }
}
