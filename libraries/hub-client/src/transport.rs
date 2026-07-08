//! Index transport and cache (spec §7.3, P2.2).
//!
//! Remote indexes are cloned blobless + sparse (scoped to the catalog
//! directory) into `~/.cache/dora/index/<alias>/`, refreshed at most once per
//! invocation, with a fast-forward check that warns loudly on history
//! rewrites or rollback (a stale or replayed index could otherwise hide
//! yanks — the warning is informational; the new state is still used). The
//! persistent clone's `HEAD` is the cross-invocation record of the last
//! trusted index state. `--offline` skips all network access and fails
//! loudly on a cache miss. Local `path =` indexes are read in place with no
//! transport.
//!
//! Transport shells out to the `git` CLI rather than reusing dora-core's
//! libgit2-based `GitManager`: libgit2 has no partial-clone or
//! sparse-checkout support, which this fetch path depends on to avoid
//! pulling the node source trees living in the same repository.

use std::{
    collections::BTreeSet,
    path::{Path, PathBuf},
    process::Command,
};

use eyre::{Context, eyre};

use crate::{OFFICIAL_INDEX_PATH, config::IndexConfig};

/// Marker written into a clone once `clone_index` completes. Its absence means
/// the clone was interrupted (e.g. a SIGKILL mid-checkout) and should be
/// re-cloned; its presence means the clone is sound even if it happens not to
/// contain the catalog subpath.
const CLONE_COMPLETE_MARKER: &str = ".dora-clone-complete";

/// Fetches and caches index catalogs for one CLI invocation.
#[derive(Debug)]
pub struct IndexFetcher {
    cache_root: PathBuf,
    offline: bool,
    refreshed: BTreeSet<String>,
    /// Loud-but-nonfatal problems (e.g. non-fast-forward index updates);
    /// drain and surface to the user.
    pub warnings: Vec<String>,
}

impl IndexFetcher {
    /// Cache under the user cache dir (`~/.cache/dora/index`).
    pub fn new(offline: bool) -> eyre::Result<Self> {
        let cache_root = dirs::cache_dir()
            .ok_or_else(|| eyre!("could not determine the user cache directory"))?
            .join("dora")
            .join("index");
        Ok(Self::with_cache_root(cache_root, offline))
    }

    /// Cache under an explicit root (tests, custom layouts).
    pub fn with_cache_root(cache_root: PathBuf, offline: bool) -> Self {
        Self {
            cache_root,
            offline,
            refreshed: BTreeSet::new(),
            warnings: Vec::new(),
        }
    }

    /// Make the catalog of `index` locally available and return its
    /// directory. Remote indexes are cloned/refreshed into the cache;
    /// local indexes resolve relative to `config_dir`.
    pub fn catalog_dir(&mut self, index: &IndexConfig, config_dir: &Path) -> eyre::Result<PathBuf> {
        if index.is_local() {
            let dir = index.local_catalog_dir(config_dir)?;
            if !dir.is_dir() {
                eyre::bail!(
                    "local index `{}` does not exist at `{}`",
                    index.alias,
                    dir.display()
                );
            }
            return Ok(dir);
        }

        // belt-and-suspenders: configs loaded via ResolvedConfig are already
        // validated, but IndexConfig can be constructed directly — every
        // field below feeds a path join or a git argument
        crate::config::validate_index_entry(index)?;
        let git_url = index.git.as_deref().expect("checked by is_local");
        let clone_dir = self.cache_root.join(&index.alias);
        let catalog_subpath = index.path.as_deref().unwrap_or(OFFICIAL_INDEX_PATH);

        if !clone_dir.join(".git").exists() {
            if self.offline {
                eyre::bail!(
                    "index `{}` is not cached and `--offline` is set\n  \
                     hint: run once with network access to populate the cache",
                    index.alias
                );
            }
            self.clone_index(git_url, &clone_dir, catalog_subpath)
                .with_context(|| format!("failed to clone index `{}`", index.alias))?;
            self.refreshed.insert(index.alias.clone());
        } else if cached_source_differs(&clone_dir, git_url, catalog_subpath) {
            // the cache is keyed by alias, but `hub.toml` may have re-pointed
            // this alias at a different URL/subpath (including an `official`
            // mirror override) — fetching the old clone's `origin` would
            // silently keep serving the previous source, so re-clone instead
            if self.offline {
                eyre::bail!(
                    "cached index `{}` is for a different source than configured \
                     and `--offline` is set\n  \
                     hint: run once with network access to refresh the cache",
                    index.alias
                );
            }
            self.reclone(index, git_url, &clone_dir, catalog_subpath)?;
        } else if !self.offline
            && self.refreshed.insert(index.alias.clone())
            && let Err(refresh_err) = self.refresh_index(&index.alias, &clone_dir)
        {
            // self-heal a corrupt cache (e.g. a SIGKILL mid-clone left a
            // broken .git); genuine network errors propagate so a flaky
            // connection cannot wipe a cache that offline runs rely on
            if git(Some(&clone_dir), &["rev-parse", "HEAD"]).is_err() {
                self.reclone(index, git_url, &clone_dir, catalog_subpath)?;
            } else {
                return Err(refresh_err)
                    .with_context(|| format!("failed to refresh index `{}`", index.alias));
            }
        }

        let catalog = clone_dir.join(catalog_subpath);
        if !catalog.is_dir() {
            // Distinguish an interrupted clone (no completion marker → re-clone
            // to self-heal) from a sound clone whose repository simply does not
            // contain the catalog subpath (a real configuration/bootstrap error
            // — must NOT be re-cloned on every invocation).
            let mut incomplete = !clone_dir.join(CLONE_COMPLETE_MARKER).exists();
            if incomplete && !self.offline {
                self.reclone(index, git_url, &clone_dir, catalog_subpath)?;
                incomplete = false; // reclone() writes CLONE_COMPLETE_MARKER on success
            }
            if !catalog.is_dir() {
                if incomplete {
                    eyre::bail!(
                        "cached index `{}` is incomplete (interrupted clone) and \
                         `--offline` is set — re-run once with network access to repair it",
                        index.alias
                    );
                }
                eyre::bail!(
                    "index `{}` has no `{catalog_subpath}` directory — \
                     check the index's `path` setting",
                    index.alias
                );
            }
        }
        // the catalog subpath comes from the index repo, which could ship it
        // (or a parent) as a symlink pointing outside the clone — confine it
        let real_catalog = catalog
            .canonicalize()
            .with_context(|| format!("failed to resolve catalog `{}`", catalog.display()))?;
        let real_clone = clone_dir
            .canonicalize()
            .with_context(|| format!("failed to resolve clone `{}`", clone_dir.display()))?;
        if !real_catalog.starts_with(&real_clone) {
            eyre::bail!(
                "index `{}` catalog `{catalog_subpath}` escapes the clone (symlink?)",
                index.alias
            );
        }
        Ok(catalog)
    }

    /// Replace a corrupt or incomplete cached clone with a fresh one.
    fn reclone(
        &mut self,
        index: &IndexConfig,
        git_url: &str,
        clone_dir: &Path,
        catalog_subpath: &str,
    ) -> eyre::Result<()> {
        self.warnings.push(format!(
            "cached index `{}` was incomplete — re-cloning",
            index.alias
        ));
        std::fs::remove_dir_all(clone_dir)
            .with_context(|| format!("failed to remove corrupt cache `{}`", clone_dir.display()))?;
        self.clone_index(git_url, clone_dir, catalog_subpath)
            .with_context(|| format!("failed to re-clone index `{}`", index.alias))?;
        self.refreshed.insert(index.alias.clone());
        Ok(())
    }

    fn clone_index(
        &mut self,
        git_url: &str,
        clone_dir: &Path,
        catalog_subpath: &str,
    ) -> eyre::Result<()> {
        if let Some(parent) = clone_dir.parent() {
            std::fs::create_dir_all(parent)
                .with_context(|| format!("failed to create `{}`", parent.display()))?;
        }
        let clone_dir_str = clone_dir
            .to_str()
            .ok_or_else(|| eyre!("cache path is not valid UTF-8"))?;
        // blobless + sparse keeps the fetch to the catalog tree, not the
        // node sources living in the same repo; not every server supports
        // partial clone, so degrade to a plain clone when refused
        // `--` keeps git from ever parsing the URL or target as a flag
        let sparse = git(
            None,
            &[
                "clone",
                "--filter=blob:none",
                "--no-checkout",
                "--quiet",
                "--",
                git_url,
                clone_dir_str,
            ],
        );
        if sparse.is_err() {
            let _ = std::fs::remove_dir_all(clone_dir);
            git(
                None,
                &[
                    "clone",
                    "--no-checkout",
                    "--quiet",
                    "--",
                    git_url,
                    clone_dir_str,
                ],
            )
            .context("git clone failed")?;
        }
        // cone-mode sparse checkout of the catalog dir only; best-effort —
        // a full checkout is correct too, just bigger
        let _ = git(
            Some(clone_dir),
            &["sparse-checkout", "set", catalog_subpath],
        );
        git(Some(clone_dir), &["checkout", "--quiet"]).context("git checkout failed")?;
        // record what this cache is a clone of, so a later config change that
        // re-points the alias forces a re-clone instead of fetching the stale
        // origin (see `cached_source_differs`)
        let _ = std::fs::write(
            clone_dir.join(SOURCE_MARKER),
            format!("{git_url}\n{catalog_subpath}"),
        );
        // mark the clone sound so a later missing catalog subpath is treated
        // as a config error, not an interrupted clone to re-fetch
        let _ = std::fs::write(clone_dir.join(CLONE_COMPLETE_MARKER), b"");
        Ok(())
    }

    fn refresh_index(&mut self, alias: &str, clone_dir: &Path) -> eyre::Result<()> {
        let local = git(Some(clone_dir), &["rev-parse", "HEAD"])?;
        git(Some(clone_dir), &["fetch", "--quiet", "origin"]).context("git fetch failed")?;
        // `git fetch` never updates an existing refs/remotes/origin/HEAD —
        // without this, an index repo renaming its default branch would
        // freeze every cache on the old branch tip silently
        let _ = git(Some(clone_dir), &["remote", "set-head", "origin", "--auto"]);
        let remote = remote_tip(clone_dir)?;
        if remote == local {
            return Ok(());
        }
        let fast_forward = git(
            Some(clone_dir),
            &["merge-base", "--is-ancestor", &local, &remote],
        )
        .is_ok();
        if !fast_forward {
            self.warnings.push(format!(
                "index `{alias}` history was rewritten or rolled back \
                 ({} -> {}) — a replayed index can hide yanked versions; \
                 verify the index source if this is unexpected",
                short(&local),
                short(&remote)
            ));
        }
        git(Some(clone_dir), &["reset", "--hard", "--quiet", &remote])
            .context("git reset failed")?;
        Ok(())
    }
}

/// The commit the remote's default branch points at after a fetch.
///
/// Resolves strictly through `refs/remotes/origin/HEAD` (populated by clone
/// and refreshed by `git remote set-head origin --auto` in `refresh_index`).
/// Guessing `origin/main`/`origin/master` was removed deliberately: for an
/// index whose default branch is e.g. `develop`, a stale or coincidental
/// `origin/main` ref would resolve against the wrong branch and produce
/// spurious "history was rolled back" warnings or a stale tip. Better to fail
/// loudly and have the caller re-clone than to silently track the wrong head.
fn remote_tip(clone_dir: &Path) -> eyre::Result<String> {
    git(Some(clone_dir), &["rev-parse", "refs/remotes/origin/HEAD"]).map_err(|_| {
        eyre::eyre!(
            "could not determine the index's default branch \
             (refs/remotes/origin/HEAD is unset) — the cache may be corrupt; \
             remove it and re-fetch"
        )
    })
}

fn short(commit: &str) -> &str {
    &commit[..commit.len().min(12)]
}

/// Marker file recording the `git_url` + catalog subpath a cached clone was
/// made from, so a config change that re-points the alias is detected.
const SOURCE_MARKER: &str = ".dora-index-source";

/// Whether the cached clone was made from a different source than now
/// configured. A missing/unreadable marker counts as "differs" so an old
/// cache (or a tampered one) is re-cloned rather than trusted.
fn cached_source_differs(clone_dir: &Path, git_url: &str, catalog_subpath: &str) -> bool {
    match std::fs::read_to_string(clone_dir.join(SOURCE_MARKER)) {
        Ok(recorded) => recorded != format!("{git_url}\n{catalog_subpath}"),
        Err(_) => true,
    }
}

/// Run a git command, returning trimmed stdout.
fn git(dir: Option<&Path>, args: &[&str]) -> eyre::Result<String> {
    let mut command = Command::new("git");
    if let Some(dir) = dir {
        command.current_dir(dir);
    }
    // never hang on a credential prompt (e.g. a private index URL)
    command.env("GIT_TERMINAL_PROMPT", "0");
    let output = command
        .args(args)
        .output()
        .context("failed to run `git` — is git installed?")?;
    if !output.status.success() {
        eyre::bail!(
            "`git {}` failed: {}",
            args.join(" "),
            String::from_utf8_lossy(&output.stderr).trim()
        );
    }
    Ok(String::from_utf8_lossy(&output.stdout).trim().to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::index::IndexCatalog;
    use crate::reference::PackageRef;

    const ENTRY: &str = r#"
manifest:
  apiVersion: 1
  name: dora-yolo
  namespace: dora-rs
  runtime: python
  entrypoint: dora-yolo
source:
  git: https://github.com/dora-rs/dora-hub
  rev: aaa111
  subdir: node-hub/dora-yolo
"#;

    /// Create a git "remote" containing a catalog at node-index/.
    fn make_remote(dir: &Path) {
        std::fs::create_dir_all(dir.join("node-index/dora-rs/dora-yolo")).unwrap();
        std::fs::write(dir.join("node-index/dora-rs/dora-yolo/0.5.1.yml"), ENTRY).unwrap();
        run_git(dir, &["init", "--quiet", "-b", "main"]);
        commit_all(dir, "initial");
        // allow partial clone over the local transport
        run_git(dir, &["config", "uploadpack.allowFilter", "true"]);
    }

    fn run_git(dir: &Path, args: &[&str]) {
        let status = Command::new("git")
            .current_dir(dir)
            .args([
                "-c",
                "user.email=test@dora.rs",
                "-c",
                "user.name=test",
                "-c",
                "commit.gpgsign=false",
            ])
            .args(args)
            .status()
            .unwrap();
        assert!(status.success(), "git {args:?} failed");
    }

    fn commit_all(dir: &Path, message: &str) {
        run_git(dir, &["add", "."]);
        run_git(dir, &["commit", "--quiet", "-m", message]);
    }

    fn remote_index(remote: &Path) -> IndexConfig {
        IndexConfig {
            alias: "test".into(),
            git: Some(remote.to_str().unwrap().to_string()),
            path: Some("node-index".into()),
            namespaces: vec![],
        }
    }

    #[test]
    fn clones_resolves_and_refreshes() {
        let remote_dir = tempfile::tempdir().unwrap();
        let cache_dir = tempfile::tempdir().unwrap();
        make_remote(remote_dir.path());
        let index = remote_index(remote_dir.path());

        let mut fetcher = IndexFetcher::with_cache_root(cache_dir.path().into(), false);
        let catalog_dir = fetcher.catalog_dir(&index, Path::new(".")).unwrap();
        let catalog = IndexCatalog::open(&catalog_dir).unwrap();
        let resolved = catalog
            .resolve(&PackageRef::parse("dora-yolo@^0.5").unwrap())
            .unwrap();
        assert_eq!(resolved.version.to_string(), "0.5.1");

        // publish 0.5.2 upstream; a NEW fetcher (new invocation) sees it
        std::fs::write(
            remote_dir
                .path()
                .join("node-index/dora-rs/dora-yolo/0.5.2.yml"),
            ENTRY,
        )
        .unwrap();
        commit_all(remote_dir.path(), "publish 0.5.2");

        // same fetcher: refresh-once-per-invocation means no new fetch
        let dir_again = fetcher.catalog_dir(&index, Path::new(".")).unwrap();
        let resolved = IndexCatalog::open(&dir_again)
            .unwrap()
            .resolve(&PackageRef::parse("dora-yolo@^0.5").unwrap())
            .unwrap();
        assert_eq!(
            resolved.version.to_string(),
            "0.5.1",
            "no mid-invocation refresh"
        );

        let mut second = IndexFetcher::with_cache_root(cache_dir.path().into(), false);
        let catalog_dir = second.catalog_dir(&index, Path::new(".")).unwrap();
        let resolved = IndexCatalog::open(&catalog_dir)
            .unwrap()
            .resolve(&PackageRef::parse("dora-yolo@^0.5").unwrap())
            .unwrap();
        assert_eq!(resolved.version.to_string(), "0.5.2");
        assert!(second.warnings.is_empty(), "{:?}", second.warnings);
    }

    #[test]
    fn offline_uses_cache_and_fails_on_miss() {
        let remote_dir = tempfile::tempdir().unwrap();
        let cache_dir = tempfile::tempdir().unwrap();
        make_remote(remote_dir.path());
        let index = remote_index(remote_dir.path());

        // miss: nothing cached yet
        let mut offline = IndexFetcher::with_cache_root(cache_dir.path().into(), true);
        let err = offline.catalog_dir(&index, Path::new(".")).unwrap_err();
        assert!(format!("{err:#}").contains("--offline"), "{err:#}");

        // populate, then offline works without touching the remote
        let mut online = IndexFetcher::with_cache_root(cache_dir.path().into(), false);
        online.catalog_dir(&index, Path::new(".")).unwrap();
        std::fs::remove_dir_all(remote_dir.path()).unwrap();
        let mut offline = IndexFetcher::with_cache_root(cache_dir.path().into(), true);
        let catalog_dir = offline.catalog_dir(&index, Path::new(".")).unwrap();
        assert!(catalog_dir.join("dora-rs/dora-yolo/0.5.1.yml").is_file());
    }

    #[test]
    fn non_fast_forward_refresh_warns() {
        let remote_dir = tempfile::tempdir().unwrap();
        let cache_dir = tempfile::tempdir().unwrap();
        make_remote(remote_dir.path());
        let index = remote_index(remote_dir.path());

        let mut fetcher = IndexFetcher::with_cache_root(cache_dir.path().into(), false);
        fetcher.catalog_dir(&index, Path::new(".")).unwrap();

        // rewrite remote history (amend) — a rollback/replay signature
        std::fs::write(
            remote_dir
                .path()
                .join("node-index/dora-rs/dora-yolo/0.5.1.yml"),
            ENTRY.replace("aaa111", "evil00"),
        )
        .unwrap();
        run_git(remote_dir.path(), &["add", "."]);
        run_git(
            remote_dir.path(),
            &["commit", "--quiet", "--amend", "-m", "rewritten"],
        );

        let mut second = IndexFetcher::with_cache_root(cache_dir.path().into(), false);
        second.catalog_dir(&index, Path::new(".")).unwrap();
        assert_eq!(second.warnings.len(), 1, "{:?}", second.warnings);
        assert!(second.warnings[0].contains("rewritten or rolled back"));
    }

    #[test]
    fn wedged_cache_self_heals() {
        let remote_dir = tempfile::tempdir().unwrap();
        let cache_dir = tempfile::tempdir().unwrap();
        make_remote(remote_dir.path());
        let index = remote_index(remote_dir.path());

        let mut fetcher = IndexFetcher::with_cache_root(cache_dir.path().into(), false);
        fetcher.catalog_dir(&index, Path::new(".")).unwrap();

        // simulate an interrupted clone: worktree gone, completion marker
        // never written
        let clone_dir = cache_dir.path().join("test");
        std::fs::remove_dir_all(clone_dir.join("node-index")).unwrap();
        let _ = std::fs::remove_file(clone_dir.join(".dora-clone-complete"));

        // offline cannot repair — fails with a clear "incomplete cache" error
        let mut offline = IndexFetcher::with_cache_root(cache_dir.path().into(), true);
        let err = offline.catalog_dir(&index, Path::new(".")).unwrap_err();
        assert!(format!("{err:#}").contains("incomplete"), "{err:#}");

        // online self-heals instead of failing forever
        let mut second = IndexFetcher::with_cache_root(cache_dir.path().into(), false);
        let catalog = second.catalog_dir(&index, Path::new(".")).unwrap();
        assert!(catalog.join("dora-rs/dora-yolo/0.5.1.yml").is_file());
        assert!(
            second.warnings.iter().any(|w| w.contains("re-cloning")),
            "{:?}",
            second.warnings
        );
    }

    #[test]
    fn valid_clone_missing_catalog_subpath_is_not_recloned() {
        // a healthy clone whose repo simply lacks the catalog subpath must
        // error clearly, NOT re-clone on every invocation
        let remote_dir = tempfile::tempdir().unwrap();
        let cache_dir = tempfile::tempdir().unwrap();
        // remote has content, but under a different dir than the index `path`
        std::fs::create_dir_all(remote_dir.path().join("node-hub")).unwrap();
        std::fs::write(remote_dir.path().join("node-hub/readme"), "x").unwrap();
        run_git(remote_dir.path(), &["init", "--quiet", "-b", "main"]);
        commit_all(remote_dir.path(), "init");
        run_git(
            remote_dir.path(),
            &["config", "uploadpack.allowFilter", "true"],
        );
        let index = remote_index(remote_dir.path()); // path = node-index (absent)

        let mut fetcher = IndexFetcher::with_cache_root(cache_dir.path().into(), false);
        let err = fetcher.catalog_dir(&index, Path::new(".")).unwrap_err();
        assert!(format!("{err:#}").contains("node-index"), "{err:#}");
        assert!(
            !fetcher.warnings.iter().any(|w| w.contains("re-cloning")),
            "a valid clone must not be re-cloned: {:?}",
            fetcher.warnings
        );
    }

    #[test]
    fn stale_incomplete_flag_not_reused_after_successful_reclone() {
        // an "incomplete" clone (no completion marker) whose repo genuinely
        // lacks the catalog subpath must, after a successful *online*
        // reclone, report the config error — not the stale "interrupted
        // clone / --offline" message (which implies offline is set and the
        // clone is still broken, neither of which is true here).
        let remote_dir = tempfile::tempdir().unwrap();
        let cache_dir = tempfile::tempdir().unwrap();
        // remote has content, but under a different dir than the index `path`
        std::fs::create_dir_all(remote_dir.path().join("node-hub")).unwrap();
        std::fs::write(remote_dir.path().join("node-hub/readme"), "x").unwrap();
        run_git(remote_dir.path(), &["init", "--quiet", "-b", "main"]);
        commit_all(remote_dir.path(), "init");
        run_git(
            remote_dir.path(),
            &["config", "uploadpack.allowFilter", "true"],
        );
        let index = remote_index(remote_dir.path()); // path = node-index (absent upstream)

        // manually seed a clone that mimics an interrupted first run: cloned
        // and its source recorded, but the completion marker never written
        let clone_dir = cache_dir.path().join(&index.alias);
        git(
            None,
            &[
                "clone",
                "--quiet",
                "--",
                remote_dir.path().to_str().unwrap(),
                clone_dir.to_str().unwrap(),
            ],
        )
        .unwrap();
        std::fs::write(
            clone_dir.join(SOURCE_MARKER),
            format!("{}\nnode-index", remote_dir.path().to_str().unwrap()),
        )
        .unwrap();

        let mut fetcher = IndexFetcher::with_cache_root(cache_dir.path().into(), false);
        let err = fetcher.catalog_dir(&index, Path::new(".")).unwrap_err();
        let msg = format!("{err:#}");
        assert!(msg.contains("node-index"), "{msg}");
        assert!(
            !msg.contains("interrupted clone") && !msg.contains("--offline"),
            "stale `incomplete` flag misreported a real config error as an \
             interrupted-clone/offline issue: {msg}"
        );
    }

    #[test]
    fn local_index_needs_no_transport() {
        let tmp = tempfile::tempdir().unwrap();
        let catalog = tmp.path().join("index");
        std::fs::create_dir_all(&catalog).unwrap();
        let index = IndexConfig {
            alias: "fixtures".into(),
            git: None,
            path: Some("index".into()),
            namespaces: vec!["test".into()],
        };
        // offline does not matter for local indexes
        let mut fetcher = IndexFetcher::with_cache_root(tmp.path().join("cache"), true);
        let dir = fetcher.catalog_dir(&index, tmp.path()).unwrap();
        assert_eq!(dir, catalog);
    }
}
