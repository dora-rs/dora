//! `dora hub update` (spec §9, P3.2): re-resolve a dataflow's hub pins to the
//! latest versions matching the descriptor's `hub:` ranges and rewrite the
//! lockfile — without building. This is the resolve-and-lock half of
//! `dora build --write-lockfile` (which resolves hub nodes fresh whenever it
//! isn't run `--locked`), for refreshing pins without a full rebuild.

use std::{collections::BTreeMap, path::PathBuf};

use dora_core::{
    descriptor::{CoreNodeKind, CustomNode, Descriptor, DescriptorExt},
    types::TypeRegistry,
};
use dora_message::{common::GitSource, descriptor::NodeSource, id::NodeId};
use eyre::{Context, ContextCompat, bail};

use super::common::sanitize;
use crate::{
    command::{
        Executable,
        build::{hub::resolve_hub_nodes, lockfile::BuildLockfile},
    },
    common::working_dir_or_parent,
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
        let lockfile_path = BuildLockfile::path_for_dataflow(&self.dataflow, None);
        if !lockfile_path.exists() {
            bail!(
                "no lockfile at `{}` — run `dora build --write-lockfile {}` first",
                lockfile_path.display(),
                self.dataflow.display()
            );
        }
        let lockfile = BuildLockfile::read_from(&lockfile_path)?;
        let old_sources = &lockfile.git_sources;

        // Re-resolve every hub node fresh (no pins) — exactly what
        // `dora build --write-lockfile` does without `--locked`, so the lockfile
        // we write is byte-compatible with a later `dora build --locked`.
        let working_dir = working_dir_or_parent(None, &self.dataflow);
        let mut descriptor = Descriptor::blocking_read(&self.dataflow)
            .with_context(|| format!("failed to read dataflow at `{}`", self.dataflow.display()))?
            .expand(working_dir)
            .context("failed to expand modules in dataflow descriptor")?;
        let mut registry = TypeRegistry::new();
        // Load the dataflow's own `types/` before resolving, exactly as `dora
        // build` does — a hub package contract may reference a user-defined type
        // URN, so without these the contract check could spuriously fail.
        let types_dir = working_dir.join("types");
        if types_dir.is_dir() {
            registry
                .load_from_dir(&types_dir)
                .map_err(|e| eyre::eyre!("failed to load user types from `types/`: {e}"))?;
        }
        let resolution = resolve_hub_nodes(
            &mut descriptor,
            &mut registry,
            self.offline,
            None,
            &BTreeMap::new(),
        )?;
        for note in &resolution.notes {
            println!("  {note}");
        }
        for warning in &resolution.warnings {
            eprintln!("  warning: {warning}");
        }
        if resolution.is_empty() {
            println!(
                "No hub packages in {}; nothing to update.",
                self.dataflow.display()
            );
            return Ok(());
        }

        // Reassemble the lockfile's git_sources: freshly-resolved hub nodes, and
        // the existing pin preserved for every non-hub git node (update only
        // touches hub pins). The fingerprint is derived from the descriptor's
        // declared git sources, so carrying a non-hub pin over stays consistent.
        let resolved_nodes = descriptor
            .resolve_aliases_and_set_defaults()
            .context("failed to resolve nodes")?;
        let mut descriptor_git_sources = BTreeMap::new();
        let mut new_sources: BTreeMap<NodeId, GitSource> = BTreeMap::new();
        for (node_id, node) in &resolved_nodes {
            let CoreNodeKind::Custom(CustomNode {
                source: NodeSource::GitBranch { repo, rev },
                ..
            }) = &node.kind
            else {
                continue;
            };
            descriptor_git_sources.insert(
                node_id.clone(),
                NodeSource::GitBranch {
                    repo: repo.clone(),
                    rev: rev.clone(),
                },
            );
            let source = match resolution.sources.get(node_id) {
                Some(source) => source.clone(),
                None => old_sources.get(node_id).cloned().with_context(|| {
                    format!(
                        "node `{node_id}` is a git source with no lockfile entry — \
                         run `dora build --write-lockfile {}` to (re)generate the lockfile",
                        self.dataflow.display()
                    )
                })?,
            };
            new_sources.insert(node_id.clone(), source);
        }
        let fingerprint =
            BuildLockfile::fingerprint_descriptor_git_sources(&descriptor_git_sources);

        // Report each hub pin that moved (or is newly pinned).
        for (node_id, source) in &new_sources {
            let Some(hub) = &source.hub else { continue };
            let old = old_sources
                .get(node_id)
                .and_then(|s| s.hub.as_ref())
                .map(|h| h.version.as_str());
            match old {
                Some(old) if old != hub.version => println!(
                    "{node_id}: {} {} -> {}",
                    sanitize(&hub.name),
                    sanitize(old),
                    sanitize(&hub.version)
                ),
                None => println!(
                    "{node_id}: {} -> {} (newly pinned)",
                    sanitize(&hub.name),
                    sanitize(&hub.version)
                ),
                Some(_) => {} // unchanged
            }
        }

        let changed =
            new_sources != *old_sources || lockfile.descriptor_fingerprint() != Some(&fingerprint);
        if !changed {
            println!("All hub pins are already up to date.");
            return Ok(());
        }
        if self.dry_run {
            println!("(dry run — lockfile not written)");
            return Ok(());
        }
        BuildLockfile::write_git_sources(&lockfile_path, &new_sources, &fingerprint)
            .with_context(|| format!("failed to write lockfile `{}`", lockfile_path.display()))?;
        println!("updated {}", lockfile_path.display());
        Ok(())
    }
}
