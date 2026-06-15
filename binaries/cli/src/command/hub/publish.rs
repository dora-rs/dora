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

use std::path::{Path, PathBuf};

use dora_core::{
    manifest::{MANIFEST_FILENAME, NodeManifest},
    types::TypeRegistry,
};
use dora_hub_client::{
    config::ResolvedConfig,
    index::{IndexEntry, SourceSpec},
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
    /// Git ref (tag, branch, or commit) to pin as the source. Defaults to the
    /// current `HEAD`.
    #[clap(long, value_name = "REF")]
    rev: Option<String>,
    /// Source git repository URL. Defaults to the `origin` remote of the
    /// node's directory.
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
}

impl Executable for Publish {
    fn execute(self) -> eyre::Result<()> {
        let dir = self.path.as_path();

        // 1. Resolve the source pin (spec §9.1 step 2): repo + commit + subdir.
        // The manifest and version are then read from that *commit*, not the
        // working tree — a published entry is immutable, so it must never
        // describe files that aren't committed at `source.rev` (a dirty tree
        // would otherwise append an entry whose manifest/version don't exist at
        // the pin, and consumers would clone the old commit while trusting it).
        let repo = match &self.repo {
            Some(r) => r.trim().to_string(),
            None => git_in(dir, &["remote", "get-url", "origin"]).context(
                "no `--repo` given and the directory has no `origin` remote — \
                 pass `--repo <url>`",
            )?,
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
        };
        let entry_yaml =
            serde_yaml::to_string(&entry).context("failed to serialize index entry")?;
        let rel_path = format!("{namespace}/{name}/{version}.yml");

        // 5. Resolve the target index up-front so `--dry-run` validates the same
        // `--index`/namespace binding a real publish would, and previews where
        // the entry lands. `load_default` falls back to the official index when
        // no hub.toml is present, so dry-run still works without a config.
        let config = ResolvedConfig::load_default().context("failed to load hub configuration")?;
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

/// Run `git <args>` in `dir`, returning trimmed stdout or an error on failure.
fn git_in(dir: &Path, args: &[&str]) -> eyre::Result<String> {
    let output = std::process::Command::new("git")
        .arg("-C")
        .arg(dir)
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
