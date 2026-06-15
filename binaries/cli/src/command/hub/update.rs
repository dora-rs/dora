//! `dora hub update` (spec §9, P3.2): re-resolve a dataflow's hub pins to the
//! latest versions matching the descriptor's `hub:` ranges and rewrite the
//! lockfile — without building.
//!
//! It runs `dora build`'s full resolve / contract / type-check / lockfile
//! pipeline with `lockfile_only`, so the lockfile is identical to a real
//! `dora build --write-lockfile` (non-hub git refs re-resolved, contracts and
//! types validated), just without compiling any node. Implementing it as a
//! partial re-resolution instead silently diverged from build (stale non-hub
//! pins, skipped type checks), so it delegates to the real thing.

use std::path::PathBuf;

use crate::command::{
    Executable,
    build::{BuildConfig, build, lockfile::BuildLockfile},
};

/// Re-resolve hub pins to the latest matching versions and rewrite the lockfile.
#[derive(Debug, clap::Args)]
pub struct Update {
    /// Dataflow whose lockfile to update.
    #[clap(value_name = "DATAFLOW")]
    dataflow: PathBuf,
    /// Do not refresh the index over the network; use only the cached copy.
    #[clap(long, action)]
    offline: bool,
    /// Resolve and report what would change, without writing the lockfile.
    /// (Resolution may still refresh the index cache unless `--offline`.)
    #[clap(long, action)]
    dry_run: bool,
}

impl Executable for Update {
    fn execute(self) -> eyre::Result<()> {
        // update refreshes an EXISTING lockfile; if there's none, point the user
        // at the command that creates one rather than silently creating it here.
        let lockfile_path = BuildLockfile::path_for_dataflow(&self.dataflow, None);
        if !lockfile_path.exists() {
            eyre::bail!(
                "no lockfile at `{}` — run `dora build --write-lockfile {}` first",
                lockfile_path.display(),
                self.dataflow.display()
            );
        }

        // Re-resolve through the real build pipeline (fresh hub + git
        // resolution, contract + type checks, fingerprint) and stop before
        // building. `--dry-run` resolves and reports but doesn't write.
        build(BuildConfig {
            dataflow: self.dataflow.to_string_lossy().into_owned(),
            offline: self.offline,
            write_lockfile: !self.dry_run,
            lockfile_only: true,
            ..Default::default()
        })?;

        if self.dry_run {
            println!("dry run — lockfile not written");
        } else {
            println!("updated {}", lockfile_path.display());
        }
        Ok(())
    }
}
