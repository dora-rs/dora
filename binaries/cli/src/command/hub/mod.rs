//! The `dora hub` subcommand — node packaging, discovery, and distribution
//! (docs/plan-node-hub.md). Phase 1 surface: `init` scaffolds a node
//! manifest; `install` is a deliberate error stub explaining the
//! per-dataflow model (spec §9).

use crate::command::Executable;

mod common;
mod fetch;
mod info;
mod init;
mod install;
mod list;
mod outdated;
mod publish;
mod search;
mod update;
mod yank;

/// Package, discover, and use dora nodes (unstable)
#[derive(Debug, clap::Subcommand)]
pub enum Hub {
    Init(init::Init),
    Search(search::Search),
    Info(info::Info),
    List(list::List),
    Fetch(fetch::Fetch),
    Install(install::Install),
    Publish(publish::Publish),
    Yank(yank::Yank),
    Outdated(outdated::Outdated),
    Update(update::Update),
}

impl Executable for Hub {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Hub::Init(cmd) => cmd.execute(),
            Hub::Search(cmd) => cmd.execute(),
            Hub::Info(cmd) => cmd.execute(),
            Hub::List(cmd) => cmd.execute(),
            Hub::Fetch(cmd) => cmd.execute(),
            Hub::Install(cmd) => cmd.execute(),
            Hub::Publish(cmd) => cmd.execute(),
            Hub::Yank(cmd) => cmd.execute(),
            Hub::Outdated(cmd) => cmd.execute(),
            Hub::Update(cmd) => cmd.execute(),
        }
    }
}
