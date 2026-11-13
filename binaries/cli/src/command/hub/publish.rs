//! `dora hub publish` (spec §9.1, P3.1).
//!
//! Publishing is just *adding an index entry that points at your source* —
//! there is no upload, because the bytes already live in a git repo. This
//! command validates the node manifest, resolves the source pin (git repo +
//! commit + subdir) and the package version, and produces the index version
//! file `<namespace>/<name>/<version>.yml`.
//!
//! With `--dry-run` it validates and prints the entry. Otherwise it writes the
//! entry into a **local** index (the `path =` form — enterprise/private indexes
//! and the integration fixtures, UC8/UC11). For a git-backed index it prints
//! the entry and the path to add, since automated PR-opening against the
//! official `node-index` lands with the index bootstrap (P2.3).

use std::{
    io::{self, BufRead, IsTerminal, Write},
    path::{Path, PathBuf},
};

use flate2::{Compression, write::GzEncoder};

use dora_core::{
    manifest::{MANIFEST_FILENAME, NodeManifest},
    types::TypeRegistry,
};
use dora_hub_client::{
    config::ResolvedConfig,
    index::{IndexEntry, SourceSpec},
    registry::{self as hub_registry, PublishMetadata, RegistryClient},
    semver,
};
use eyre::{Context, ContextCompat, bail};

use crate::command::Executable;

#[derive(Debug, clap::Args)]
/// Publish a node: validate its manifest and add a pinned index entry.
pub struct Publish {
    /// Directory holding the node's `dora-node.yml` (and `Cargo.toml` /
    /// `pyproject.toml`). Defaults to the current directory.
    #[clap(value_name = "PATH", default_value = ".")]
    path: PathBuf,
    /// Validate and print the index entry without writing it or opening a PR.
    #[clap(long, action)]
    dry_run: bool,
    /// Skip the confirmation prompt when publishing to the public dora.pub registry.
    #[clap(long, action)]
    yes: bool,
    /// Git ref (tag, branch, or commit) to pin as the source. Defaults to the
    /// current `HEAD`.
    #[clap(long, value_name = "REF")]
    rev: Option<String>,
    /// Source git repository URL. Defaults to the `origin` remote of the
    /// node's directory; SSH remotes of known public hosts (atomgit.com,
    /// github.com, …) are rewritten to their HTTPS URL so consumers can clone
    /// without an SSH key.
    #[clap(long, value_name = "URL")]
    repo: Option<String>,
    /// Package version (semver). Defaults to `[package].version` (Cargo) or
    /// `[project].version` (pyproject).
    #[clap(long, value_name = "SEMVER")]
    version: Option<String>,
    /// Index to publish into (by `hub.toml` alias). Defaults to the index
    /// bound to the manifest's namespace.
    #[clap(long, value_name = "ALIAS")]
    index: Option<String>,
    /// HTTP registry URL. Defaults to `DORA_HUB_REGISTRY_URL` or
    /// <https://dora.pub>; setting this forces a registry upload.
    #[clap(long, env = "DORA_HUB_REGISTRY_URL", value_name = "URL")]
    registry: Option<String>,
    /// Registry API token. Defaults to the saved token from `dora hub login`
    /// (or `DORA_HUB_TOKEN` for the official registry only).
    #[clap(long, value_name = "TOKEN")]
    token: Option<String>,
    /// One-line description stored in the registry (defaults to the manifest).
    #[clap(long, value_name = "TEXT")]
    description: Option<String>,
    /// SPDX license identifier stored in the registry.
    #[clap(long, value_name = "SPDX")]
    license: Option<String>,
    /// Source repository URL stored in the registry.
    #[clap(long, value_name = "URL")]
    repository: Option<String>,
}

impl Executable for Publish {
    fn execute(self) -> eyre::Result<()> {
        let dir = self.path.as_path();
        // git runs *in* this directory (`super::common::node_repo_git`), so a
        // missing one would fail at spawn with a bare "No such file or
        // directory" that reads as if git were not installed.
        if !dir.is_dir() {
            bail!(
                "node directory `{}` does not exist or is not a directory",
                dir.display()
            );
        }

        // 1. Resolve the source pin (spec §9.1 step 2): repo + commit + subdir.
        // The manifest and version are then read from that *commit*, not the
        // working tree — a published entry is immutable, so it must never
        // describe files that aren't committed at `source.rev` (a dirty tree
        // would otherwise append an entry whose manifest/version don't exist at
        // the pin, and consumers would clone the old commit while trusting it).
        let repo = match &self.repo {
            Some(r) => r.trim().to_string(),
            None => {
                let origin = git_in(dir, &["remote", "get-url", "origin"]).context(
                    "no `--repo` given and the directory has no `origin` remote — \
                     pass `--repo <url>`",
                )?;
                // An SSH origin is a valid clone URL for the publisher, but an
                // anonymous consumer cannot use it. Record the public HTTPS
                // form for known hosting platforms; anything else passes through.
                dora_hub_client::normalize_public_repo_url(&origin).unwrap_or(origin)
            }
        };
        // The repo URL is consumed by others as an untrusted index source, and
        // the resolver only accepts https/ssh/file/scp-style/abs-path forms.
        // Validate it now so a cleartext-http or otherwise-unresolvable `origin`
        // fails on the publisher, not silently on every later consumer.
        dora_hub_client::validate_git_url_untrusted(&repo).with_context(|| {
            format!("source repo `{repo}` is not a usable index source — pass a valid `--repo`")
        })?;
        let rev_arg = self.rev.as_deref().unwrap_or("HEAD");
        let commit = git_in(dir, &["rev-parse", &format!("{rev_arg}^{{commit}}")])
            .with_context(|| format!("failed to resolve git ref `{rev_arg}` to a commit"))?;
        // Accept SHA-1 (40) or SHA-256 (64) hashes — the index resolver
        // (`git_pin`) takes both, so publish must not be stricter.
        if !matches!(commit.len(), 40 | 64) || !commit.chars().all(|c| c.is_ascii_hexdigit()) {
            bail!("resolved commit `{commit}` is not a full git hash (40 or 64 hex chars)");
        }
        // `--show-prefix` is the node directory's repo-relative path (forward
        // slashes, trailing slash, empty at the repo root). It both locates the
        // committed files (`<commit>:<prefix><file>`) and becomes `source.subdir`.
        let prefix = git_in(dir, &["rev-parse", "--show-prefix"])
            .context("`dora hub publish` must be run inside the node's git repository")?;
        let subdir = Some(prefix.trim_end_matches('/').to_string()).filter(|p| !p.is_empty());

        // 2. Read + validate the manifest as committed at `commit`.
        let manifest_yaml = git_in(
            dir,
            &["show", &format!("{commit}:{prefix}{MANIFEST_FILENAME}")],
        )
        .with_context(|| {
            format!(
                "{MANIFEST_FILENAME} is not committed at {} — commit it (and any \
                     version bump) before publishing, or pass `--rev <commit>` that \
                     contains it",
                &commit[..12]
            )
        })?;
        let manifest = NodeManifest::parse(&manifest_yaml)
            .with_context(|| format!("invalid {MANIFEST_FILENAME} at commit {}", &commit[..12]))?;
        manifest
            .validate_strict(&TypeRegistry::new())
            .context("the node manifest is invalid — fix it before publishing")?;
        // `validate_strict` guarantees `name`/`namespace` are present and valid.
        let name = manifest
            .name
            .clone()
            .context("manifest is missing `name`")?;
        let namespace = manifest.namespace.clone();

        // 3. Version: `--version`, else read from the native manifest at `commit`.
        let version_str = match &self.version {
            Some(v) => v.clone(),
            None => read_committed_version(dir, &commit, &prefix).context(
                "could not determine the package version — pass `--version`, or commit \
                 `[package].version` (Cargo) / `[project].version` (pyproject)",
            )?,
        };
        let version = semver::Version::parse(version_str.trim())
            .with_context(|| format!("version `{version_str}` is not valid semver"))?;

        // Registry publishing is opt-in: explicit `--registry`, or
        // `DORA_HUB_REGISTRY_URL` for an unbound namespace. Without one of
        // those, keep the legacy local/git index publish path; flipping the
        // default destination to the public registry is a separate decision.
        let config = ResolvedConfig::load_default().context("failed to load hub configuration")?;
        let registry_opt_in = self.registry.is_some()
            || (std::env::var_os(hub_registry::REGISTRY_URL_ENV).is_some()
                && !config.is_bound(&namespace));
        let use_registry = self.index.is_none() && registry_opt_in;
        if use_registry {
            return publish_to_registry(
                dir,
                &commit,
                &prefix,
                &namespace,
                &name,
                &version,
                &manifest,
                self.registry.as_deref(),
                self.token.as_deref(),
                &repo,
                subdir.clone(),
                self.description,
                self.license,
                self.repository,
                self.dry_run,
                self.yes,
            );
        }

        // 4. Construct the index entry.
        let entry = IndexEntry {
            manifest: manifest.clone(),
            source: SourceSpec {
                git: Some(repo.clone()),
                rev: Some(commit.clone()),
                subdir: subdir.clone(),
                binary: Vec::new(),
                fallback_git: None,
            },
            published: Some(chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Secs, true)),
            yanked: false,
            yank_reason: None,
            archive_sha256: None,
        };
        let entry_yaml =
            serde_yaml::to_string(&entry).context("failed to serialize index entry")?;
        let rel_path = format!("{namespace}/{name}/{version}.yml");

        // 5. Resolve the target index up-front so `--dry-run` validates the same
        // `--index`/namespace binding a real publish would, and previews where
        // the entry lands. `load_default` falls back to the official index when
        // no hub.toml is present, so dry-run still works without a config.
        let index = match &self.index {
            Some(alias) => {
                let target = config
                    .indexes
                    .iter()
                    .find(|i| i.alias.eq_ignore_ascii_case(alias))
                    .with_context(|| format!("no index with alias `{alias}` in hub.toml"))?;
                // A namespace resolves against exactly one index (spec §7.3).
                // Refuse to seed a namespace into an index it isn't bound to —
                // otherwise `--index` would let a `dora-rs/*` entry be written
                // into any local index, defeating namespace binding.
                let bound = config.index_for_namespace(&namespace);
                if !bound.alias.eq_ignore_ascii_case(&target.alias) {
                    bail!(
                        "namespace `{namespace}` is bound to index `{}`, not `{}` — publish \
                         into `{}`, or bind the namespace to `{}` in hub.toml",
                        bound.alias,
                        target.alias,
                        bound.alias,
                        target.alias
                    );
                }
                target
            }
            None => config.index_for_namespace(&namespace),
        };

        // 5a. Dry run: validate + preview only.
        if self.dry_run {
            println!("# {rel_path} (index: {})", index.alias);
            print!("{entry_yaml}");
            println!(
                "\nvalidated `{namespace}/{name}` v{version} @ {} (subdir: {})",
                &commit[..12],
                subdir.as_deref().unwrap_or(".")
            );
            return Ok(());
        }

        // 5b. Publish into the resolved index.
        if index.is_local() {
            let catalog = index.local_catalog_dir(&config.config_dir)?;
            let dest = catalog.join(&rel_path);
            if let Some(parent) = dest.parent() {
                std::fs::create_dir_all(parent)
                    .with_context(|| format!("failed to create `{}`", parent.display()))?;
            }
            // append-only: create exclusively (O_EXCL) so a published version is
            // never overwritten, even under a concurrent publish (spec §7.5).
            let mut file = match std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&dest)
            {
                Ok(file) => file,
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => bail!(
                    "version {version} of `{namespace}/{name}` already exists at `{}` — \
                     the index is append-only; bump the version to publish a new one",
                    dest.display()
                ),
                Err(e) => {
                    return Err(e).with_context(|| format!("failed to write `{}`", dest.display()));
                }
            };
            std::io::Write::write_all(&mut file, entry_yaml.as_bytes())
                .with_context(|| format!("failed to write `{}`", dest.display()))?;
            println!(
                "published `{namespace}/{name}` v{version} -> {}",
                dest.display()
            );
        } else {
            // git-backed index: PR-opening needs the bootstrapped node-index
            // repo + CI (P2.3). Print the entry and where it goes.
            println!("# add this as `{rel_path}` in the index repo:");
            println!("{entry_yaml}");
            println!(
                "the `{}` index is git-backed ({}). Commit the entry above at `{rel_path}` and \
                 open a PR. Automated `dora hub publish` PR-opening lands with the node-index \
                 bootstrap (P2.3).",
                index.alias,
                index.git.as_deref().unwrap_or("?")
            );
        }
        Ok(())
    }
}

#[allow(clippy::too_many_arguments)]
fn publish_to_registry(
    dir: &Path,
    commit: &str,
    prefix: &str,
    namespace: &str,
    name: &str,
    version: &semver::Version,
    manifest: &NodeManifest,
    registry_override: Option<&str>,
    token_override: Option<&str>,
    repo: &str,
    subdir: Option<String>,
    description: Option<String>,
    license: Option<String>,
    repository: Option<String>,
    dry_run: bool,
    yes: bool,
) -> eyre::Result<()> {
    let registry_url = registry_override
        .map(str::to_string)
        .unwrap_or_else(hub_registry::registry_url);
    let client = RegistryClient::with_url(&registry_url, false)
        .context("invalid hub registry configuration")?;
    let metadata = PublishMetadata {
        namespace: namespace.to_string(),
        name: name.to_string(),
        version: version.to_string(),
        description: description.or_else(|| manifest.description.clone()),
        license,
        repository,
        source: Some(hub_registry::PublishSource {
            git: repo.to_string(),
            rev: commit.to_string(),
            subdir: subdir.clone(),
        }),
    };

    let treeish = if prefix.is_empty() {
        commit.to_string()
    } else {
        format!("{commit}:{prefix}")
    };
    let tar = git_archive(dir, &treeish)
        .with_context(|| format!("failed to package committed source at {treeish}"))?;
    let mut gz = GzEncoder::new(Vec::new(), Compression::default());
    gz.write_all(&tar)
        .context("failed to compress package archive")?;
    let archive = gz.finish().context("failed to compress package archive")?;
    if archive.len() > 50 * 1024 * 1024 {
        bail!(
            "package archive is {} MiB (limit 50 MiB) — trim data files before publishing",
            archive.len() / (1024 * 1024)
        );
    }

    if dry_run {
        println!(
            "would publish `{namespace}/{name}` v{version} to {registry_url}\n\
             archive: {} bytes from commit {}\n{}",
            archive.len(),
            &commit[..12],
            serde_json::to_string_pretty(&metadata)?
        );
        return Ok(());
    }

    if !yes && registry_url.trim_end_matches('/') == hub_registry::DEFAULT_REGISTRY_URL {
        confirm_public_publish(
            &registry_url,
            namespace,
            name,
            &version.to_string(),
            archive.len(),
            &commit[..12],
        )?;
    }

    let token = match token_override {
        Some(token) if !token.trim().is_empty() => token.trim().to_string(),
        _ => hub_registry::read_saved_token_for(&registry_url)?
            .or_else(|| hub_registry::read_env_token_for(&registry_url))
            .context(
                "no registry token: run `dora hub login`, pass `--token`, or set \
                 DORA_HUB_TOKEN for the official registry",
            )?,
    };
    let response = client
        .publish(&token, &metadata, &archive)
        .with_context(|| format!("publish to {registry_url} failed"))?;
    println!(
        "published `{namespace}/{name}` v{version} to {registry_url}\n{}",
        serde_json::to_string_pretty(&response)?
    );
    Ok(())
}

fn confirm_public_publish(
    registry_url: &str,
    namespace: &str,
    name: &str,
    version: &str,
    archive_len: usize,
    commit: &str,
) -> eyre::Result<()> {
    eprintln!(
        "about to publish `{namespace}/{name}` v{version} to the PUBLIC registry \
         {registry_url}\n  archive: {archive_len} bytes from commit {commit}\n  \
         anyone will be able to download the committed source tree"
    );
    if !io::stdin().is_terminal() {
        eyre::bail!(
            "refusing to publish to the public registry without confirmation\n  \
             pass `--yes` in non-interactive runs, or `--dry-run` to preview"
        );
    }
    eprint!("publish to {registry_url}? [y/N] ");
    io::stderr().flush().context("failed to flush prompt")?;
    let mut answer = String::new();
    io::stdin()
        .lock()
        .read_line(&mut answer)
        .context("failed to read confirmation")?;
    let answer = answer.trim().to_ascii_lowercase();
    if !matches!(answer.as_str(), "y" | "yes") {
        eyre::bail!("publish cancelled");
    }
    Ok(())
}

fn git_archive(dir: &Path, treeish: &str) -> eyre::Result<Vec<u8>> {
    // `git archive <commit>:<subdir>` only works from the repository root;
    // from a tracked subdirectory git reports "current working directory is
    // not tracked". Find the root first, then archive there.
    let root = super::common::node_repo_git(dir)
        .args(["rev-parse", "--show-toplevel"])
        .output()
        .context("failed to locate the git repository root")?;
    if !root.status.success() {
        bail!(
            "failed to locate the git repository root: {}",
            String::from_utf8_lossy(&root.stderr).trim()
        );
    }
    let root = String::from_utf8_lossy(&root.stdout).trim().to_string();
    let output = super::common::node_repo_git(Path::new(&root))
        .args(["archive", "--format=tar", treeish])
        .output()
        .context("failed to run git archive")?;
    if !output.status.success() {
        bail!(
            "git archive {treeish} failed: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        );
    }
    Ok(output.stdout)
}

/// Run `git <args>` in `dir`, returning trimmed stdout or an error on failure.
/// See [`super::common::node_repo_git`] for why it is not a plain `git -C`.
fn git_in(dir: &Path, args: &[&str]) -> eyre::Result<String> {
    let output = super::common::node_repo_git(dir)
        .args(args)
        .output()
        .context("failed to run git")?;
    if !output.status.success() {
        bail!(
            "git {args:?} failed: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        );
    }
    Ok(String::from_utf8_lossy(&output.stdout).trim().to_string())
}

/// Read the package version from `Cargo.toml` (`[package].version`) or
/// `pyproject.toml` (`[project].version`) **as committed at `commit`**,
/// returning the first one found. `prefix` is the node directory's
/// repo-relative path (forward slashes, trailing slash, empty at the root).
fn read_committed_version(dir: &Path, commit: &str, prefix: &str) -> Option<String> {
    let read = |file: &str| git_in(dir, &["show", &format!("{commit}:{prefix}{file}")]).ok();
    if let Some(raw) = read("Cargo.toml")
        && let Ok(t) = raw.parse::<toml::Table>()
        && let Some(v) = t
            .get("package")
            .and_then(|p| p.get("version"))
            .and_then(|v| v.as_str())
    {
        return Some(v.to_string());
    }
    if let Some(raw) = read("pyproject.toml")
        && let Ok(t) = raw.parse::<toml::Table>()
        && let Some(v) = t
            .get("project")
            .and_then(|p| p.get("version"))
            .and_then(|v| v.as_str())
    {
        return Some(v.to_string());
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A mistyped node path must say so, not look like a missing `git`.
    #[test]
    fn a_missing_node_directory_is_reported_as_such() {
        let tmp = tempfile::tempdir().unwrap();
        let publish = Publish {
            path: tmp.path().join("does-not-exist"),
            dry_run: true,
            yes: false,
            rev: None,
            repo: Some("https://example.com/node.git".to_string()),
            version: None,
            index: None,
            registry: None,
            token: None,
            description: None,
            license: None,
            repository: None,
        };
        let msg = format!("{:#}", publish.execute().unwrap_err());
        assert!(
            msg.contains("does not exist or is not a directory"),
            "{msg}"
        );
        assert!(!msg.contains("failed to run git"), "{msg}");
    }
}
