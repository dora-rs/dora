//! `dora hub outdated` (spec §9): compare a dataflow's locked hub pins against
//! the latest non-yanked version available in the index.

use std::path::PathBuf;

use dora_hub_client::{reference::PackageRef, semver::VersionReq};

use super::common::{HubContext, sanitize};
use crate::command::{Executable, build::lockfile::BuildLockfile};

/// Show hub packages whose lockfile pin is behind the index.
#[derive(Debug, clap::Args)]
pub struct Outdated {
    /// Dataflow whose lockfile to check.
    #[clap(value_name = "DATAFLOW")]
    dataflow: PathBuf,
    /// Do not refresh the index over the network; use only the cached copy.
    #[clap(long, action)]
    offline: bool,
}

impl Executable for Outdated {
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
        let mut ctx = HubContext::load(self.offline)?;

        let mut checked = 0usize;
        let mut outdated = 0usize;
        let mut errored = 0usize;
        for (node_id, source) in &lockfile.git_sources {
            let Some(hub) = &source.hub else {
                continue;
            };
            checked += 1;
            // The node id, hub name, and version all come from the lockfile,
            // and resolve errors embed the lockfile-derived package key — none
            // are charset-validated, so sanitize every one before echoing (a
            // hostile entry could otherwise inject terminal escapes). Treat a
            // malformed pin as "can't check".
            let node = sanitize(node_id.as_ref());
            let Some((namespace, name)) = hub.name.split_once('/') else {
                println!("{node}: malformed hub name `{}`", sanitize(&hub.name));
                errored += 1;
                continue;
            };
            let pinned = match hub.version.parse::<dora_hub_client::semver::Version>() {
                Ok(v) => v,
                Err(_) => {
                    println!(
                        "{node}: malformed pinned version `{}`",
                        sanitize(&hub.version)
                    );
                    errored += 1;
                    continue;
                }
            };

            let catalog = match ctx.catalog_for_namespace(namespace) {
                Ok(catalog) => catalog,
                Err(err) => {
                    println!(
                        "{node}: {}/{} — could not read index ({})",
                        sanitize(namespace),
                        sanitize(name),
                        sanitize(&format!("{err:#}"))
                    );
                    errored += 1;
                    continue;
                }
            };
            // the highest *non-yanked* version (ignoring the dataflow's range,
            // so a major bump still shows as available to upgrade to).
            let reference = PackageRef {
                namespace: namespace.to_string(),
                name: name.to_string(),
                requirement: VersionReq::STAR,
            };
            match catalog.resolve(&reference) {
                Ok(latest) if latest.version > pinned => {
                    outdated += 1;
                    println!(
                        "{node}: {}/{} {pinned} -> {} (newer available)",
                        sanitize(namespace),
                        sanitize(name),
                        latest.version
                    );
                }
                Ok(_) => {} // up to date
                Err(err) => {
                    println!(
                        "{node}: {}/{} {pinned} — {}",
                        sanitize(namespace),
                        sanitize(name),
                        sanitize(&format!("{err:#}"))
                    );
                    errored += 1;
                }
            }
        }
        ctx.drain_warnings();

        if checked == 0 {
            println!("No hub packages pinned in {}.", lockfile_path.display());
        } else if outdated > 0 {
            println!(
                "{outdated} of {checked} hub package(s) are outdated — \
                 update the `hub:` range and rebuild to upgrade."
            );
        } else if errored == 0 {
            println!("All {checked} hub package(s) are up to date.");
        }
        // A check that couldn't complete must not read as "up to date": report
        // it and exit non-zero so a human or CI can tell the report is partial.
        if errored > 0 {
            eyre::bail!("{errored} of {checked} hub package(s) could not be checked (see above)");
        }
        Ok(())
    }
}
