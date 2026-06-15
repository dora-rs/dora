//! `dora hub yank` (spec §9, UC10).
//!
//! Flipping the `yanked` flag is the one mutation an existing index version
//! file is allowed (spec §7.5). New resolutions skip a yanked version; an
//! existing `--locked` pin keeps working but `dora build` warns. `--undo`
//! restores it.
//!
//! For a local (`path =`) index the flag is flipped in place; for a git-backed
//! index this is a flag-flip PR, so the command prints what to change.

use std::path::Path;

use dora_hub_client::{config::ResolvedConfig, index::IndexEntry, reference::PackageRef, semver};
use eyre::{Context, ContextCompat, bail};

use crate::command::Executable;

#[derive(Debug, clap::Args)]
/// Yank (or restore) a published version.
pub struct Yank {
    /// Package and version: `[<namespace>/]<name>@<version>`.
    #[clap(value_name = "PKG@VERSION")]
    package: String,
    /// Reason recorded with the yank (shown to consumers who pinned it).
    #[clap(long, value_name = "REASON")]
    reason: Option<String>,
    /// Restore a previously yanked version instead of yanking it.
    #[clap(long, action)]
    undo: bool,
}

impl Executable for Yank {
    fn execute(self) -> eyre::Result<()> {
        // Parse `[ns/]name@version`.
        let (pkg, version_str) = self.package.rsplit_once('@').with_context(|| {
            format!(
                "invalid package `{}`: expected `[<namespace>/]<name>@<version>`",
                self.package
            )
        })?;
        let version = semver::Version::parse(version_str.trim())
            .with_context(|| format!("version `{version_str}` is not valid semver"))?;
        let reference =
            PackageRef::parse(pkg).with_context(|| format!("invalid package reference `{pkg}`"))?;
        let (namespace, name) = (&reference.namespace, &reference.name);

        let config = ResolvedConfig::load_default().context("failed to load hub configuration")?;
        let index = config.index_for_namespace(namespace);
        let rel_path = format!("{namespace}/{name}/{version}.yml");
        let action = if self.undo { "restore" } else { "yank" };

        if !index.is_local() {
            println!(
                "the `{}` index is git-backed ({}). {action} is a flag-flip PR: in `{rel_path}` set \
                 `yanked: {}`{} and open a PR against the index repo.",
                index.alias,
                index.git.as_deref().unwrap_or("?"),
                !self.undo,
                self.reason
                    .as_deref()
                    .map(|r| format!(" with `yank_reason: {r}`"))
                    .unwrap_or_default(),
            );
            return Ok(());
        }

        let catalog = index.local_catalog_dir(&config.config_dir)?;
        let entry_path = catalog.join(&rel_path);
        if !entry_path.is_file() {
            bail!(
                "no published version {version} of `{namespace}/{name}` at `{}`",
                entry_path.display()
            );
        }
        let raw = std::fs::read_to_string(&entry_path)
            .with_context(|| format!("failed to read `{}`", entry_path.display()))?;
        let mut entry: IndexEntry = serde_yaml::from_str(&raw)
            .with_context(|| format!("`{}` is not a valid index entry", entry_path.display()))?;

        let target = !self.undo;
        if entry.yanked == target {
            println!(
                "`{namespace}/{name}@{version}` is already {}",
                if target { "yanked" } else { "not yanked" }
            );
            return Ok(());
        }
        entry.yanked = target;
        entry.yank_reason = target.then(|| self.reason.clone()).flatten();

        let updated = serde_yaml::to_string(&entry).context("failed to serialize index entry")?;
        write_atomic(&entry_path, &updated)?;
        if target {
            println!("yanked `{namespace}/{name}@{version}`");
        } else {
            println!("restored `{namespace}/{name}@{version}`");
        }
        Ok(())
    }
}

/// Overwrite `path` by writing a temp file in the same directory and renaming
/// it into place, so a yank flip can't leave a half-written entry.
fn write_atomic(path: &Path, contents: &str) -> eyre::Result<()> {
    let dir = path.parent().unwrap_or(Path::new("."));
    let tmp = path.with_extension("yml.tmp");
    std::fs::write(&tmp, contents)
        .with_context(|| format!("failed to write `{}`", tmp.display()))?;
    std::fs::rename(&tmp, path).with_context(|| {
        format!(
            "failed to replace `{}` (dir `{}`)",
            path.display(),
            dir.display()
        )
    })?;
    Ok(())
}
