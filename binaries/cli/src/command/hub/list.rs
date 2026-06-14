//! `dora hub list` — hub packages pinned in a dataflow's lockfile.

use std::path::PathBuf;

use super::common::sanitize;
use crate::command::{Executable, build::lockfile::BuildLockfile};

/// List the hub packages used by a dataflow
#[derive(Debug, clap::Args)]
pub struct List {
    /// Dataflow whose lockfile to read (defaults to the lockfile path rules)
    #[clap(value_name = "DATAFLOW")]
    dataflow: PathBuf,
}

impl Executable for List {
    fn execute(self) -> eyre::Result<()> {
        let lockfile_path = BuildLockfile::path_for_dataflow(&self.dataflow, None);
        if !lockfile_path.exists() {
            eyre::bail!(
                "no lockfile at `{}` — run `dora build --write-lockfile {}` first",
                lockfile_path.display(),
                self.dataflow.display()
            );
        }
        let lockfile = BuildLockfile::read_from(&lockfile_path)?;
        let mut found = false;
        for (node_id, source) in &lockfile.git_sources {
            let Some(hub) = &source.hub else {
                continue;
            };
            found = true;
            // commit_hash comes from the lockfile, which `read_from` does not
            // charset-validate — sanitize before it reaches the terminal
            let short: String = source.commit_hash.chars().take(12).collect();
            println!(
                "{node_id}: {} {} ({})",
                sanitize(&hub.name),
                sanitize(&hub.version),
                sanitize(&short)
            );
        }
        if !found {
            println!("No hub packages pinned in {}.", lockfile_path.display());
        }
        Ok(())
    }
}
