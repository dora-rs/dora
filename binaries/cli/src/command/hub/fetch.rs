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
/// exact commits, so the mirror must match them rather than re-resolving the
/// index (which could pick a newer version). Requires a lockfile — the same
/// contract as `dora hub list`.
fn resolve_dataflow_pins(dataflow: &str) -> eyre::Result<Vec<GitSource>> {
    let lockfile_path = BuildLockfile::path_for_dataflow(std::path::Path::new(dataflow), None);
    if !lockfile_path.exists() {
        eyre::bail!(
            "no lockfile at `{}` — run `dora build --write-lockfile {dataflow}` first",
            lockfile_path.display()
        );
    }
    let lockfile = BuildLockfile::read_from(&lockfile_path)?;
    let sources = lockfile
        .git_sources
        .into_values()
        .filter(|source| source.hub.is_some())
        .collect();
    Ok(sources)
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
/// hash is the integrity guarantee). A `.dora-clone-complete` marker lets a
/// re-run skip a finished clone while re-doing an interrupted one.
fn clone_pinned(source: &GitSource, target_dir: &std::path::Path) -> eyre::Result<()> {
    // re-validate the hash at the API boundary — `GitSource.commit_hash` is a
    // plain string and a future caller might not have run it through git_pin
    let valid_hash = (7..=64).contains(&source.commit_hash.len())
        && source.commit_hash.chars().all(|c| c.is_ascii_hexdigit());
    if !valid_hash {
        eyre::bail!("invalid commit hash for `{}`", source.repo);
    }
    let dest = target_dir.join(&source.commit_hash);
    let marker = dest.join(".dora-clone-complete");
    if marker.exists() {
        return Ok(());
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
    println!("  {} @ {}", source.repo, &source.commit_hash);
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
