use crate::build::{BuildLogger, PrevGitSource};
use dora_message::{DataflowId, SessionId, common::LogLevel};
use eyre::{ContextCompat, WrapErr, bail};
use git2::FetchOptions;
use itertools::Itertools;
use std::{
    collections::{BTreeMap, BTreeSet},
    path::{Path, PathBuf},
};
use url::Url;

#[derive(Default)]
pub struct GitManager {
    /// Directories that are currently in use by running dataflows.
    pub clones_in_use: BTreeMap<PathBuf, BTreeSet<DataflowId>>,
    /// Builds that are prepared, but not done yet.
    prepared_builds: BTreeMap<SessionId, PreparedBuild>,
    // reuse_for: BTreeMap<PathBuf, PathBuf>,
}

#[derive(Default)]
struct PreparedBuild {
    /// Clone dirs that will be created during the build process.
    ///
    /// This allows subsequent nodes to reuse the dirs.
    planned_clone_dirs: BTreeSet<PathBuf>,
}

impl GitManager {
    pub fn choose_clone_dir(
        &mut self,
        session_id: SessionId,
        repo: String,
        commit_hash: String,
        prev_git: Option<PrevGitSource>,
        target_dir: &Path,
    ) -> eyre::Result<GitFolder> {
        let repo_url = Url::parse(&repo).context("failed to parse git repository URL")?;
        let clone_dir = Self::clone_dir_path(target_dir, &repo_url, &commit_hash)?;

        let prev_commit_hash = prev_git
            .as_ref()
            .filter(|p| p.git_source.repo == repo)
            .map(|p| &p.git_source.commit_hash);

        if let Some(using) = self.clones_in_use.get(&clone_dir)
            && !using.is_empty()
        {
            // The directory is currently in use by another dataflow. Rebuilding
            // while a dataflow is running could lead to unintended behavior.
            eyre::bail!(
                "the build directory is still in use by the following \
                    dataflows, please stop them before rebuilding: {}",
                using.iter().join(", ")
            )
        }

        let reuse = if self.clone_dir_ready(session_id, &clone_dir) {
            // The directory already contains a checkout of the commit we're interested in.
            // So we can simply reuse the directory without doing any additional git
            // operations.
            //
            // Only hand down a commit to verify (see the Reuse arm) when the dir was
            // left by a prior, finished build: it's on disk but this session didn't
            // plan it. A dir this session *did* plan is about to be filled in by a
            // sibling node's NewClone, which may be running right now on another
            // thread (parallel builds share one JoinSet with no per-dir lock). If I
            // checked HEAD on that I could delete a clone that's still being written,
            // so I skip verification and just reuse it, same as before this change.
            let planned_this_session = self
                .prepared_builds
                .get(&session_id)
                .map(|p| p.planned_clone_dirs.contains(&clone_dir))
                .unwrap_or(false);
            ReuseOptions::Reuse {
                dir: clone_dir.clone(),
                verify_commit: (!planned_this_session).then_some(commit_hash),
            }
        } else if let Some(previous_commit_hash) = prev_commit_hash {
            // we might be able to update a previous clone
            let prev_clone_dir = Self::clone_dir_path(target_dir, &repo_url, previous_commit_hash)?;

            if prev_clone_dir.exists() {
                let still_needed = prev_git
                    .map(|g| g.still_needed_for_this_build)
                    .unwrap_or(false);
                let used_by_others = self
                    .clones_in_use
                    .get(&prev_clone_dir)
                    .map(|ids| !ids.is_empty())
                    .unwrap_or(false);
                if still_needed || used_by_others {
                    // previous clone is still in use -> we cannot rename it, but we can copy it
                    ReuseOptions::CopyAndFetch {
                        from: prev_clone_dir,
                        target_dir: clone_dir.clone(),
                        commit_hash,
                    }
                } else {
                    // there is an unused previous clone that is no longer needed -> rename it
                    ReuseOptions::RenameAndFetch {
                        from: prev_clone_dir,
                        target_dir: clone_dir.clone(),
                        commit_hash,
                    }
                }
            } else {
                // no existing clone associated with previous build id
                ReuseOptions::NewClone {
                    target_dir: clone_dir.clone(),
                    repo_url,
                    commit_hash,
                }
            }
        } else {
            // no previous build that we can reuse
            ReuseOptions::NewClone {
                target_dir: clone_dir.clone(),
                repo_url,
                commit_hash,
            }
        };
        self.register_ready_clone_dir(session_id, clone_dir);

        Ok(GitFolder { reuse })
    }

    pub fn clone_dir_ready(&self, session_id: SessionId, dir: &Path) -> bool {
        self.prepared_builds
            .get(&session_id)
            .map(|p| p.planned_clone_dirs.contains(dir))
            .unwrap_or(false)
            || dir.exists()
    }

    pub fn register_ready_clone_dir(&mut self, session_id: SessionId, dir: PathBuf) -> bool {
        self.prepared_builds
            .entry(session_id)
            .or_default()
            .planned_clone_dirs
            .insert(dir)
    }

    fn clone_dir_path(
        base_dir: &Path,
        repo_url: &Url,
        commit_hash: &String,
    ) -> eyre::Result<PathBuf> {
        // file:// URLs (local mirrors, air-gapped setups, tests) have no
        // hostname — group their clones under a "localhost" directory
        let host = repo_url.host_str().unwrap_or("localhost");
        let mut path = base_dir.join(host);
        path.extend(repo_url.path_segments().context("no path in git URL")?);
        let path = path.join(commit_hash);
        Ok(dunce::simplified(&path).to_owned())
    }

    pub fn clear_planned_builds(&mut self, session_id: SessionId) {
        self.prepared_builds.remove(&session_id);
    }
}

pub struct GitFolder {
    /// Specifies whether an existing repo should be reused.
    reuse: ReuseOptions,
}

impl GitFolder {
    pub async fn prepare(self, logger: &mut impl BuildLogger) -> eyre::Result<PathBuf> {
        let GitFolder { reuse } = self;

        tracing::info!("reuse: {reuse:?}");
        let clone_dir = match reuse {
            ReuseOptions::NewClone {
                target_dir,
                repo_url,
                commit_hash,
            } => {
                logger
                    .log_message(
                        LogLevel::Info,
                        format!(
                            "cloning {repo_url}#{commit_hash} into {}",
                            target_dir.display()
                        ),
                    )
                    .await;
                let clone_target = target_dir.clone();
                let checkout_result = tokio::task::spawn_blocking(move || {
                    let repository = clone_into(repo_url.clone(), &clone_target)
                        .with_context(|| format!("failed to clone git repo from `{repo_url}`"))?;
                    checkout_tree(&repository, &commit_hash)
                        .with_context(|| format!("failed to checkout commit `{commit_hash}`"))
                })
                .await
                .unwrap();

                match checkout_result {
                    Ok(()) => target_dir,
                    Err(err) => {
                        logger
                            .log_message(LogLevel::Error, format!("{err:?}"))
                            .await;
                        cleanup_failed_clone(logger, &target_dir).await;
                        bail!(err)
                    }
                }
            }
            ReuseOptions::CopyAndFetch {
                from,
                target_dir,
                commit_hash,
            } => {
                let from_clone = from.clone();
                let to = target_dir.clone();

                // I want the whole copy + fetch + checkout to be all-or-nothing.
                // If any step dies partway we're left with a half-copied dir on
                // disk, and I don't want the next build mistaking it for a good
                // clone and building the wrong commit (#2480).
                let result: eyre::Result<()> = async {
                    tokio::task::spawn_blocking(move || {
                        std::fs::create_dir_all(&to)
                            .context("failed to create directory for copying git repo")?;
                        fs_extra::dir::copy(
                            &from_clone,
                            &to,
                            &fs_extra::dir::CopyOptions::new().content_only(true),
                        )
                        .with_context(|| {
                            format!(
                                "failed to copy repo clone from `{}` to `{}`",
                                from_clone.display(),
                                to.display()
                            )
                        })
                    })
                    .await??;

                    logger
                        .log_message(
                            LogLevel::Info,
                            format!("fetching changes after copying {}", from.display()),
                        )
                        .await;

                    let repository = fetch_changes(&target_dir, None).await?;
                    checkout_tree(&repository, &commit_hash)?;
                    Ok(())
                }
                .await;

                match result {
                    Ok(()) => target_dir,
                    Err(err) => {
                        cleanup_failed_clone(logger, &target_dir).await;
                        bail!(err)
                    }
                }
            }
            ReuseOptions::RenameAndFetch {
                from,
                target_dir,
                commit_hash,
            } => {
                tokio::fs::rename(&from, &target_dir)
                    .await
                    .context("failed to rename repo clone")?;

                logger
                    .log_message(
                        LogLevel::Info,
                        format!("fetching changes after renaming {}", from.display()),
                    )
                    .await;

                // The old clone now lives at target_dir. If the fetch or checkout
                // fails from here, that dir is left sitting at the old commit, so
                // I clean it up instead of letting a later build reuse it (#2480).
                let result: eyre::Result<()> = async {
                    let repository = fetch_changes(&target_dir, None).await?;
                    checkout_tree(&repository, &commit_hash)?;
                    Ok(())
                }
                .await;

                match result {
                    Ok(()) => target_dir,
                    Err(err) => {
                        cleanup_failed_clone(logger, &target_dir).await;
                        bail!(err)
                    }
                }
            }
            ReuseOptions::Reuse { dir, verify_commit } => {
                // Belt and braces for #2480: even with the cleanup above, a stale
                // dir could still slip through if remove_dir_all itself failed or
                // we got killed mid-checkout. So for a clone left by a prior build
                // (verify_commit is Some) that's pinned to a full commit hash, I
                // check that HEAD actually points there. verify_commit is None when
                // a sibling node in this build owns the dir and may still be cloning
                // into it, so I leave those completely untouched. Branch and tag
                // pins I can't verify offline, so those pass through too.
                if let Some(commit_hash) = verify_commit
                    && dir.exists()
                    && is_full_commit_hash(&commit_hash)
                {
                    let repo_dir = dir.clone();
                    let head = tokio::task::spawn_blocking(move || -> eyre::Result<String> {
                        let repo =
                            git2::Repository::open(&repo_dir).context("failed to open git repo")?;
                        let id = repo
                            .head()
                            .context("failed to read HEAD")?
                            .peel_to_commit()
                            .context("failed to resolve HEAD commit")?
                            .id()
                            .to_string();
                        Ok(id)
                    })
                    .await
                    .context("HEAD read task panicked")?;

                    let on_right_commit =
                        matches!(&head, Ok(h) if h.eq_ignore_ascii_case(&commit_hash));
                    if !on_right_commit {
                        // Drop the stale clone so the next build re-clones from
                        // scratch, then fail loudly instead of quietly building the
                        // old source.
                        cleanup_failed_clone(logger, &dir).await;
                        bail!(
                            "clone dir {} is not on the requested commit {commit_hash} \
                             (found {head:?}); I removed it, please rebuild",
                            dir.display()
                        );
                    }
                }

                logger
                    .log_message(
                        LogLevel::Info,
                        format!("reusing up-to-date {}", dir.display()),
                    )
                    .await;
                dir
            }
        };
        Ok(clone_dir)
    }
}

#[derive(Debug)]
enum ReuseOptions {
    /// Create a new clone of the repository.
    NewClone {
        target_dir: PathBuf,
        repo_url: Url,
        commit_hash: String,
    },
    /// Reuse an existing up-to-date clone of the repository.
    ///
    /// `verify_commit` is `Some(hash)` only for a clone left by a prior build,
    /// where it's safe to check HEAD against `hash`. It's `None` when a sibling
    /// node in this same build owns the dir (it may still be cloning into it),
    /// in which case the reuse is a plain read with no HEAD check.
    Reuse {
        dir: PathBuf,
        verify_commit: Option<String>,
    },
    /// Copy an older clone of the repository and fetch changes, then reuse it.
    CopyAndFetch {
        from: PathBuf,
        target_dir: PathBuf,
        commit_hash: String,
    },
    /// Rename an older clone of the repository and fetch changes, then reuse it.
    RenameAndFetch {
        from: PathBuf,
        target_dir: PathBuf,
        commit_hash: String,
    },
}

/// Wipe a half-prepared clone dir after a failed clone/fetch/checkout so a
/// later build doesn't pick it up and treat it as a good clone of the commit
/// it was meant to become (#2480). A dir that was never created is fine, I only
/// grumble if a real directory refuses to go away.
async fn cleanup_failed_clone(logger: &mut impl BuildLogger, dir: &Path) {
    match tokio::fs::remove_dir_all(dir).await {
        Ok(()) => {}
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => {}
        Err(err) => {
            logger
                .log_message(
                    LogLevel::Error,
                    format!(
                        "couldn't remove clone dir after a failed build: {}",
                        err.kind()
                    ),
                )
                .await;
        }
    }
}

/// True for a full-length hex commit id (40 chars for SHA-1, 64 for SHA-256).
/// Branch and tag names are shorter or non-hex, and I can't resolve those to a
/// commit without hitting the network, so those pins skip the HEAD check.
fn is_full_commit_hash(s: &str) -> bool {
    matches!(s.len(), 40 | 64) && s.bytes().all(|b| b.is_ascii_hexdigit())
}

fn clone_into(repo_addr: Url, clone_dir: &Path) -> eyre::Result<git2::Repository> {
    if let Some(parent) = clone_dir.parent() {
        std::fs::create_dir_all(parent)
            .context("failed to create parent directory for git clone")?;
    }

    let clone_dir = clone_dir.to_owned();

    let mut builder = git2::build::RepoBuilder::new();
    let mut fetch_options = git2::FetchOptions::new();
    fetch_options.download_tags(git2::AutotagOption::All);
    builder.fetch_options(fetch_options);
    builder
        .clone(repo_addr.as_str(), &clone_dir)
        .context("failed to clone repo")
}

async fn fetch_changes(
    repo_dir: &Path,
    refname: Option<String>,
) -> Result<git2::Repository, eyre::Error> {
    let repo_dir = repo_dir.to_owned();
    let fetch_changes = tokio::task::spawn_blocking(move || {
        let repository = git2::Repository::open(&repo_dir).context("failed to open git repo")?;

        {
            let mut remote = repository
                .find_remote("origin")
                .context("failed to find remote `origin` in repo")?;
            remote
                .connect(git2::Direction::Fetch)
                .context("failed to connect to remote")?;
            let default_branch = remote
                .default_branch()
                .context("failed to get default branch for remote")?;
            let fetch = match &refname {
                Some(refname) => refname,
                None => default_branch
                    .as_str()
                    .context("failed to read default branch as string")?,
            };
            let mut fetch_options = FetchOptions::new();
            fetch_options.download_tags(git2::AutotagOption::All);
            remote
                .fetch(&[&fetch], Some(&mut fetch_options), None)
                .context("failed to fetch from git repo")?;
        }
        Result::<_, eyre::Error>::Ok(repository)
    });
    let repository = fetch_changes.await??;
    Ok(repository)
}

fn checkout_tree(repository: &git2::Repository, commit_hash: &str) -> eyre::Result<()> {
    // Reject arbitrary rev-spec expressions; only allow hex commit hashes and branch/tag names.
    // This must stay a denylist of every rev-spec operator `revparse_ext` understands, not just
    // the ones above: `~`/`@` enable ancestor/reflog/upstream navigation (e.g. `HEAD~3`,
    // `main@{upstream}`) just as much as `..`, `:`, and `^` do. We reject `@`/`{`/`}` wholesale
    // rather than only the `@{…}` sequence; that also turns away the rare valid ref name that
    // embeds one of these characters, but a hard error is the safe failure mode for this guard.
    if commit_hash.contains("..")
        || commit_hash.contains(':')
        || commit_hash.contains('^')
        || commit_hash.contains('~')
        || commit_hash.contains('@')
        || commit_hash.contains('{')
        || commit_hash.contains('}')
    {
        eyre::bail!(
            "invalid commit reference '{commit_hash}': rev-spec expressions are not allowed"
        );
    }
    let (object, reference) = repository
        .revparse_ext(commit_hash)
        .context("failed to parse ref")?;
    repository
        .checkout_tree(&object, None)
        .context("failed to checkout ref")?;
    match reference {
        Some(reference) => repository
            .set_head(reference.name().context("failed to get reference_name")?)
            .context("failed to set head")?,
        None => repository
            .set_head_detached(object.id())
            .context("failed to set detached head")?,
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_message::common::LogLevelOrStdout;

    // A logger that throws everything away. I don't care what gets logged in
    // these tests, only what happens to the files on disk.
    struct TestLogger;
    impl BuildLogger for TestLogger {
        type Clone = TestLogger;
        async fn log_message(
            &mut self,
            _level: impl Into<LogLevelOrStdout> + Send,
            _message: impl Into<String> + Send,
        ) {
        }
        async fn try_clone(&self) -> eyre::Result<Self::Clone> {
            Ok(TestLogger)
        }
    }

    // Spin up a real local git repo with a single commit and hand back the
    // commit id. There's deliberately no `origin` remote, so any fetch against
    // it fails right away and I can exercise the failure path without a network.
    fn init_repo_with_commit(path: &Path) -> String {
        let repo = git2::Repository::init(path).unwrap();
        std::fs::write(path.join("file.txt"), b"A").unwrap();
        let mut index = repo.index().unwrap();
        index.add_path(Path::new("file.txt")).unwrap();
        index.write().unwrap();
        let tree_id = index.write_tree().unwrap();
        let tree = repo.find_tree(tree_id).unwrap();
        let sig = git2::Signature::now("t", "t@t").unwrap();
        repo.commit(Some("HEAD"), &sig, &sig, "A", &tree, &[])
            .unwrap()
            .to_string()
    }

    #[tokio::test]
    async fn rename_and_fetch_removes_dir_when_fetch_fails() {
        let base = tempfile::tempdir().unwrap();
        let from = base.path().join("from");
        let target = base.path().join("target");
        init_repo_with_commit(&from);

        let folder = GitFolder {
            reuse: ReuseOptions::RenameAndFetch {
                from,
                target_dir: target.clone(),
                // 40 hex chars, but we never get that far: the fetch blows up first.
                commit_hash: "deadbeef".repeat(5),
            },
        };
        assert!(folder.prepare(&mut TestLogger).await.is_err());
        assert!(
            !target.exists(),
            "a failed rename+fetch must not leave the dir behind"
        );
    }

    #[tokio::test]
    async fn copy_and_fetch_removes_dir_when_fetch_fails() {
        let base = tempfile::tempdir().unwrap();
        let from = base.path().join("from");
        let target = base.path().join("target");
        init_repo_with_commit(&from);

        let folder = GitFolder {
            reuse: ReuseOptions::CopyAndFetch {
                from,
                target_dir: target.clone(),
                commit_hash: "deadbeef".repeat(5),
            },
        };
        assert!(folder.prepare(&mut TestLogger).await.is_err());
        assert!(
            !target.exists(),
            "a failed copy+fetch must not leave the dir behind"
        );
    }

    #[tokio::test]
    async fn reuse_bails_and_removes_dir_on_head_mismatch() {
        let base = tempfile::tempdir().unwrap();
        let dir = base.path().join("clone");
        init_repo_with_commit(&dir); // HEAD sits at commit A

        let folder = GitFolder {
            reuse: ReuseOptions::Reuse {
                dir: dir.clone(),
                // Well-formed full hash that is definitely not commit A.
                verify_commit: Some("0".repeat(40)),
            },
        };
        assert!(folder.prepare(&mut TestLogger).await.is_err());
        assert!(
            !dir.exists(),
            "a clone on the wrong commit must be removed so the next build re-clones"
        );
    }

    #[tokio::test]
    async fn reuse_ok_when_head_matches() {
        let base = tempfile::tempdir().unwrap();
        let dir = base.path().join("clone");
        let oid = init_repo_with_commit(&dir);

        let folder = GitFolder {
            reuse: ReuseOptions::Reuse {
                dir: dir.clone(),
                verify_commit: Some(oid),
            },
        };
        assert_eq!(folder.prepare(&mut TestLogger).await.unwrap(), dir);
        assert!(dir.exists());
    }

    #[tokio::test]
    async fn reuse_skips_verification_for_branch_ref() {
        let base = tempfile::tempdir().unwrap();
        let dir = base.path().join("clone");
        init_repo_with_commit(&dir);

        // "main" isn't a full hash, so I skip the HEAD check and just reuse.
        let folder = GitFolder {
            reuse: ReuseOptions::Reuse {
                dir: dir.clone(),
                verify_commit: Some("main".into()),
            },
        };
        assert!(folder.prepare(&mut TestLogger).await.is_ok());
    }

    #[tokio::test]
    async fn reuse_without_verify_leaves_a_sibling_clone_alone() {
        // verify_commit = None models the case where a sibling node in this same
        // build owns the dir and might still be cloning into it. Even though HEAD
        // sits at commit A and not whatever the pin is, I must not read or delete
        // it — deleting a clone another thread is writing is exactly the race the
        // dir.exists() guard alone didn't cover.
        let base = tempfile::tempdir().unwrap();
        let dir = base.path().join("clone");
        init_repo_with_commit(&dir);

        let folder = GitFolder {
            reuse: ReuseOptions::Reuse {
                dir: dir.clone(),
                verify_commit: None,
            },
        };
        assert_eq!(folder.prepare(&mut TestLogger).await.unwrap(), dir);
        assert!(dir.exists(), "a sibling-owned clone must never be deleted");
    }

    /// Repo with two commits, so `HEAD~1` / `HEAD^` resolve to a real (but
    /// forbidden) ancestor commit.
    fn repo_with_two_commits() -> (tempfile::TempDir, git2::Repository) {
        let dir = tempfile::tempdir().unwrap();
        let repository = git2::Repository::init(dir.path()).unwrap();
        let signature = git2::Signature::now("test", "test@dora.rs").unwrap();
        {
            let tree_id = repository.index().unwrap().write_tree().unwrap();
            let tree = repository.find_tree(tree_id).unwrap();
            let first = repository
                .commit(Some("HEAD"), &signature, &signature, "first", &tree, &[])
                .unwrap();
            let first_commit = repository.find_commit(first).unwrap();
            repository
                .commit(
                    Some("HEAD"),
                    &signature,
                    &signature,
                    "second",
                    &tree,
                    &[&first_commit],
                )
                .unwrap();
        }
        (dir, repository)
    }

    #[test]
    fn rejects_rev_spec_navigation_operators() {
        let (_dir, repository) = repo_with_two_commits();

        // These are genuine, resolvable rev-specs, not typos: without the guard
        // `revparse_ext` would happily walk them to the first commit. The guard
        // must reject them anyway.
        for resolvable in ["HEAD~1", "HEAD^"] {
            assert!(
                repository.revparse_ext(resolvable).is_ok(),
                "test setup: `{resolvable}` should resolve in a two-commit repo"
            );
        }

        for commit_hash in [
            "HEAD~1",
            "main~1",
            "HEAD@{1}",
            "main@{upstream}",
            "@",
            "HEAD^",
            "main..HEAD",
            "HEAD:foo",
        ] {
            let err = checkout_tree(&repository, commit_hash).unwrap_err();
            assert!(
                format!("{err:#}").contains("rev-spec expressions are not allowed"),
                "expected `{commit_hash}` to be rejected as a rev-spec, got: {err:#}"
            );
        }
    }

    #[test]
    fn accepts_branch_name_and_commit_hash() {
        let (_dir, repository) = repo_with_two_commits();
        let head = repository.head().unwrap();
        let branch_name = head.shorthand().unwrap().to_string();
        let head_commit = head.peel_to_commit().unwrap().id();
        checkout_tree(&repository, &branch_name).unwrap();
        checkout_tree(&repository, &head_commit.to_string()).unwrap();
    }
}
