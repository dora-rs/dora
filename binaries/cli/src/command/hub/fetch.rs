//! `dora hub fetch` — mirror pinned node sources into a portable directory
//! (UC7) for an air-gapped or CI rebuild.
//!
//! A *dataflow* target mirrors exactly what `dora build --locked` would clone:
//! the pins are read straight from the dataflow's lockfile, so the mirror can
//! never drift from the locked build (resolving the index live could pick a
//! newer version than the lockfile pins). A bare `name@version` target has no
//! lockfile, so it resolves live against the index.
//!
//! Note: `dora build` clones git sources through its own (libgit2) build
//! cache, so it does not yet re-use this mirror for the *source* fetch — full
//! air-gapped source reuse is a follow-up (tracked under #2097). What this
//! command guarantees today is a verifiable local copy of each pinned source.

use std::path::PathBuf;

use super::common::HubContext;
use crate::command::{Executable, build::lockfile::BuildLockfile};
use dora_hub_client::reference::PackageRef;
use dora_message::common::GitSource;
use eyre::Context;

/// Warm the index cache and mirror hub package sources
#[derive(Debug, clap::Args)]
pub struct Fetch {
    /// A dataflow YAML, or a single `name@version` package reference
    #[clap(value_name = "TARGET")]
    target: String,
    /// Directory to clone sources into (default: ./hub-cache)
    #[clap(long, value_name = "DIR", default_value = "hub-cache")]
    target_dir: PathBuf,
}

impl Executable for Fetch {
    fn execute(self) -> eyre::Result<()> {
        let sources = if self.target.ends_with(".yml") || self.target.ends_with(".yaml") {
            // a dataflow mirrors exactly what `--locked` builds — read the
            // lockfile, no index needed
            resolve_dataflow_pins(&self.target)?
        } else {
            // a bare reference resolves live against the index
            let mut ctx = HubContext::load(false)?;
            let source = resolve_single(&self.target, &mut ctx)?;
            ctx.drain_warnings();
            vec![source]
        };

        if sources.is_empty() {
            println!("Nothing to fetch (no hub packages found).");
            return Ok(());
        }
        std::fs::create_dir_all(&self.target_dir)
            .with_context(|| format!("failed to create `{}`", self.target_dir.display()))?;
        for source in &sources {
            clone_pinned(source, &self.target_dir)?;
        }
        println!(
            "Fetched {} package source(s) into {}.",
            sources.len(),
            self.target_dir.display()
        );
        Ok(())
    }
}

/// The pinned git sources of a dataflow's `hub:` nodes, read from its lockfile.
///
/// The lockfile is authoritative: `dora build --locked` rebuilds from these
/// exact commits, so the mirror must match them. To honor that contract, the
/// *current* dataflow is validated against the lockfile first (like `--locked`)
/// — a stale lockfile must not mirror sources the YAML no longer references, or
/// at a version it no longer requests. Requires a lockfile.
fn resolve_dataflow_pins(dataflow: &str) -> eyre::Result<Vec<GitSource>> {
    use dora_core::descriptor::{Descriptor, DescriptorExt};

    let path = std::path::Path::new(dataflow);
    let lockfile_path = BuildLockfile::path_for_dataflow(path, None);
    if !lockfile_path.exists() {
        eyre::bail!(
            "no lockfile at `{}` — run `dora build --write-lockfile {dataflow}` first",
            lockfile_path.display()
        );
    }
    let lockfile = BuildLockfile::read_from(&lockfile_path)?;

    // hub nodes that resolved to a prebuilt binary — `dora hub fetch` clones git
    // sources but does not pre-download binary artifacts yet (they are fetched +
    // verified by the daemon at spawn time).
    let binary_pinned: std::collections::BTreeSet<_> =
        lockfile.binary_sources.keys().cloned().collect();

    // pinned hub sources from the lockfile, keyed by node id
    let pinned: std::collections::BTreeMap<_, _> = lockfile
        .git_sources
        .into_iter()
        .filter(|(_, s)| s.hub.is_some())
        .collect();

    // the current dataflow's `hub:` nodes
    let working_dir = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| std::path::Path::new("."));
    let descriptor = Descriptor::blocking_read(path)?
        .expand(working_dir)
        .context("failed to expand modules")?;

    let regen = format!("regenerate with `dora build --write-lockfile {dataflow}`");
    let mut yaml_hub_nodes = std::collections::BTreeSet::new();
    for node in &descriptor.nodes {
        let Some(raw) = &node.hub else { continue };
        yaml_hub_nodes.insert(node.id.clone());
        let reference = PackageRef::parse(raw)
            .with_context(|| format!("node `{}`: invalid `hub:` reference", node.id))?;
        let source = match pinned.get(&node.id) {
            Some(source) => source,
            None if binary_pinned.contains(&node.id) => eyre::bail!(
                "node `{}`: resolves to a prebuilt binary artifact — `dora hub fetch` does not \
                 pre-download binary artifacts yet (the daemon fetches and verifies them at \
                 spawn time); offline (UC7) binary prefetch is a follow-up",
                node.id
            ),
            None => eyre::bail!(
                "node `{}`: `hub:` reference is not in the lockfile — {regen}",
                node.id
            ),
        };
        let prov = source.hub.as_ref().expect("filtered to hub sources");
        let version = dora_hub_client::semver::Version::parse(&prov.version)
            .map_err(|_| eyre::eyre!("node `{}`: invalid version in lockfile", node.id))?;
        if prov.name != reference.key() || !reference.requirement.matches(&version) {
            eyre::bail!(
                "node `{}`: the dataflow's `hub:` reference changed since the lockfile \
                 was written — {regen}",
                node.id
            );
        }
    }
    // the lockfile must not pin a hub node the dataflow no longer has
    if let Some(id) = pinned
        .keys()
        .chain(binary_pinned.iter())
        .find(|id| !yaml_hub_nodes.contains(*id))
    {
        eyre::bail!(
            "node `{id}`: the lockfile pins a `hub:` node the dataflow no longer has — {regen}"
        );
    }

    Ok(pinned.into_values().collect())
}

fn resolve_single(reference: &str, ctx: &mut HubContext) -> eyre::Result<GitSource> {
    let reference = PackageRef::parse(reference)?;
    let catalog = ctx.catalog_for_namespace(&reference.namespace)?;
    let resolved = catalog.resolve(&reference)?;
    let (git, rev) = resolved.entry.source.git_pin()?;
    Ok(GitSource {
        repo: git.to_string(),
        commit_hash: rev.to_string(),
        subdir: resolved.entry.source.subdir.clone(),
        hub: None,
    })
}

/// Blobless-clone the pinned commit into `<target_dir>/<commit>` (the commit
/// hash is the integrity guarantee). A `<target_dir>/.<commit>.complete`
/// marker — kept outside the clone so the source repo can't forge it — lets a
/// re-run skip a finished clone while re-doing an interrupted one.
fn clone_pinned(source: &GitSource, target_dir: &std::path::Path) -> eyre::Result<()> {
    // re-validate the hash at the API boundary — `GitSource.commit_hash` is a
    // plain string and a future caller might not have run it through git_pin
    let valid_hash = matches!(source.commit_hash.len(), 40 | 64)
        && source.commit_hash.chars().all(|c| c.is_ascii_hexdigit());
    if !valid_hash {
        eyre::bail!("invalid commit hash for `{}`", source.repo);
    }
    // re-validate the URL here too: the dataflow path reads `repo` straight
    // from the lockfile, which is otherwise passed to `git clone` without the
    // scheme/transport allowlist the single-package path gets via `git_pin`.
    dora_hub_client::validate_git_url_untrusted(&source.repo)
        .with_context(|| format!("refusing to clone `{}`", source.repo))?;
    let dest = target_dir.join(&source.commit_hash);
    // the completion marker lives *outside* the clone working tree — a source
    // repo could otherwise ship a file literally named `.dora-clone-complete`
    // and make an interrupted checkout look finished
    let marker = target_dir.join(format!(".{}.complete", source.commit_hash));
    if marker.exists() {
        // trust the marker only if the checkout is actually present at the pin
        // — a deleted or corrupted clone dir with a leftover marker must not
        // report success without a source. Otherwise drop the stale marker and
        // fall through to a clean re-clone.
        if dest.is_dir()
            && run_git_in(
                &dest,
                &[
                    "cat-file",
                    "-e",
                    &format!("{}^{{commit}}", source.commit_hash),
                ],
            )
            .is_ok()
        {
            return Ok(());
        }
        let _ = std::fs::remove_file(&marker);
    }
    if dest.exists() {
        // a prior interrupted clone — start clean
        std::fs::remove_dir_all(&dest)
            .with_context(|| format!("failed to clear `{}`", dest.display()))?;
    }
    let dest_str = dest
        .to_str()
        .ok_or_else(|| eyre::eyre!("cache path is not valid UTF-8"))?;
    run_git(&[
        "clone",
        "--filter=blob:none",
        "--quiet",
        "--",
        &source.repo,
        dest_str,
    ])
    .with_context(|| format!("failed to clone `{}`", source.repo))?;
    run_git_in(&dest, &["checkout", "--quiet", &source.commit_hash])
        .with_context(|| format!("failed to checkout `{}`", source.commit_hash))?;
    let _ = std::fs::write(&marker, b"");
    println!("  {} @ {}", source.repo, source.commit_hash);
    Ok(())
}

fn run_git(args: &[&str]) -> eyre::Result<()> {
    run_git_impl(None, args)
}
fn run_git_in(dir: &std::path::Path, args: &[&str]) -> eyre::Result<()> {
    run_git_impl(Some(dir), args)
}
fn run_git_impl(dir: Option<&std::path::Path>, args: &[&str]) -> eyre::Result<()> {
    let mut cmd = std::process::Command::new("git");
    cmd.env("GIT_TERMINAL_PROMPT", "0");
    if let Some(dir) = dir {
        cmd.current_dir(dir);
    }
    let status = cmd.args(args).status().context("failed to run `git`")?;
    if !status.success() {
        eyre::bail!("`git {}` failed", args.join(" "));
    }
    Ok(())
}
