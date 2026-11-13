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

use dora_hub_client::{
    config::ResolvedConfig,
    index::IndexEntry,
    reference::PackageRef,
    registry::{REGISTRY_URL_ENV, RegistryClient, read_saved_token_for, registry_url},
    semver,
};
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
    /// Registry URL. Setting this forces the registry path. Without it,
    /// `DORA_HUB_REGISTRY_URL` opts an *unbound* namespace into the registry;
    /// an explicit `hub.toml` binding still wins.
    #[clap(long, value_name = "URL")]
    registry: Option<String>,
    /// Registry API token (default: the saved token; `DORA_HUB_TOKEN` is
    /// honored for the official registry only).
    #[clap(long, value_name = "TOKEN")]
    token: Option<String>,
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
        // We build a catalog path from these and *write* to it, so apply the
        // same key-part guard the read paths use — `PackageRef::parse` does not
        // reject `..`, and a write must never escape the catalog root.
        if !dora_hub_client::index::is_valid_key_part(namespace)
            || !dora_hub_client::index::is_valid_key_part(name)
        {
            bail!(
                "invalid package `{namespace}/{name}`: namespace and name must be `[A-Za-z0-9._-]` and not start with `.`"
            );
        }

        let config = ResolvedConfig::load_default().context("failed to load hub configuration")?;
        let action = if self.undo { "restore" } else { "yank" };
        let via_registry = self.registry.is_some()
            || (!config.is_bound(namespace)
                && !config.official_explicit()
                && std::env::var_os(REGISTRY_URL_ENV).is_some());
        if via_registry {
            if self.reason.is_some() {
                eprintln!(
                    "  warning: the registry yank API does not accept a reason; \
                     `--reason` is only recorded for local/git indexes"
                );
            }
            let registry_url = self.registry.unwrap_or_else(registry_url);
            let client = RegistryClient::with_url(&registry_url, false)
                .context("invalid registry configuration")?;
            let token = match self.token {
                Some(token) if !token.trim().is_empty() => token,
                _ => read_saved_token_for(&registry_url)?
                    .or_else(|| dora_hub_client::registry::read_env_token_for(&registry_url))
                    .context(
                        "no registry token: run `dora hub login`, pass `--token`, or set \
                         DORA_HUB_TOKEN for the official registry",
                    )?,
            };
            let response = client
                .set_yanked(&token, namespace, name, &version.to_string(), !self.undo)
                .with_context(|| {
                    format!("registry {action} failed for `{namespace}/{name}@{version}`")
                })?;
            println!(
                "{action} `{namespace}/{name}@{version}` on {registry_url}
{}",
                serde_json::to_string_pretty(&response)?
            );
            return Ok(());
        }
        let index = config.index_for_namespace(namespace);
        let rel_path = format!("{namespace}/{name}/{version}.yml");

        if !index.is_local() {
            // Mirror the local path: yanking records the reason, restoring
            // clears it. Don't tell a `--undo` author to keep a stale reason.
            let reason_hint = if self.undo {
                " and remove any `yank_reason`".to_string()
            } else {
                self.reason
                    .as_deref()
                    .map(|r| format!(" with `yank_reason: {r}`"))
                    .unwrap_or_default()
            };
            println!(
                "the `{}` index is git-backed ({}). {action} is a flag-flip PR: in `{rel_path}` set \
                 `yanked: {}`{reason_hint} and open a PR against the index repo.",
                index.alias,
                index.git.as_deref().unwrap_or("?"),
                !self.undo,
            );
            return Ok(());
        }

        // Open the catalog so the write path is confined under the canonical
        // root — the same symlink-escape guard the read paths use. The `..`
        // key-part check above blocks lexical traversal; this blocks a
        // symlinked namespace/package dir that resolves outside the root.
        let catalog_dir = index.local_catalog_dir(&config.config_dir)?;
        let catalog = dora_hub_client::index::IndexCatalog::open(&catalog_dir)
            .with_context(|| format!("failed to open index catalog `{}`", catalog_dir.display()))?;
        let entry_path = catalog.version_entry_path(namespace, name, &version)?;
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
        let new_reason = target.then(|| self.reason.clone()).flatten();
        // No-op only when nothing would change — re-yanking with a *different*
        // reason must update it, not silently discard the new one.
        if entry.yanked == target && entry.yank_reason == new_reason {
            println!(
                "`{namespace}/{name}@{version}` is already {}",
                if target { "yanked" } else { "not yanked" }
            );
            return Ok(());
        }
        entry.yanked = target;
        entry.yank_reason = new_reason;

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
