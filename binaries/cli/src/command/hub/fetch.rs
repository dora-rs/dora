//! `dora hub fetch` — warm the index cache and mirror pinned node sources
//! (UC7). Resolving every reference populates the on-disk index cache, so a
//! later `dora build --offline` can resolve `hub:` references without the
//! network; the sources are additionally cloned into a portable directory.
//!
//! Note: `dora build` clones git sources through its own (libgit2) build
//! cache, so it does not yet re-use this mirror for the *source* fetch — full
//! air-gapped source reuse is a follow-up (tracked under #2097). What this
//! command guarantees today is offline *resolution* plus a verifiable local
//! copy of each pinned source.

use std::path::PathBuf;

use super::common::HubContext;
use crate::command::Executable;
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
        let mut ctx = HubContext::load(false)?;
        let sources = if self.target.ends_with(".yml") || self.target.ends_with(".yaml") {
            resolve_dataflow_pins(&self.target, &mut ctx)?
        } else {
            vec![resolve_single(&self.target, &mut ctx)?]
        };
        ctx.drain_warnings();

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

/// Resolve every `hub:` node of a dataflow to its pinned git source.
fn resolve_dataflow_pins(dataflow: &str, ctx: &mut HubContext) -> eyre::Result<Vec<GitSource>> {
    use dora_core::descriptor::{Descriptor, DescriptorExt};
    let path = std::path::Path::new(dataflow);
    let working_dir = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| std::path::Path::new("."));
    let descriptor = Descriptor::blocking_read(path)?
        .expand(working_dir)
        .context("failed to expand modules")?;

    let mut sources = Vec::new();
    for node in &descriptor.nodes {
        let Some(raw) = &node.hub else { continue };
        let reference = PackageRef::parse(raw)
            .with_context(|| format!("node `{}`: invalid `hub:` reference", node.id))?;
        let catalog = ctx.catalog_for_namespace(&reference.namespace)?;
        let resolved = catalog
            .resolve(&reference)
            .with_context(|| format!("node `{}`: failed to resolve `{raw}`", node.id))?;
        let (git, rev) = resolved.entry.source.git_pin()?;
        sources.push(GitSource {
            repo: git.to_string(),
            commit_hash: rev.to_string(),
            subdir: resolved.entry.source.subdir.clone(),
            hub: None,
        });
    }
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
