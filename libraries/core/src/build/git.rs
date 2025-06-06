use crate::build::BuildLogger;
use dora_message::{common::LogLevel, descriptor::GitRepoRev, DataflowId, SessionId};
use eyre::{ContextCompat, WrapErr};
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
    reuse_for: BTreeMap<PathBuf, PathBuf>,
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
        repo_url: Url,
        commit_hash: String,
        prev_commit_hash: Option<String>,
        target_dir: &Path,
    ) -> eyre::Result<GitFolder> {
        let clone_dir = Self::clone_dir_path(&target_dir, &repo_url, &commit_hash)?;

        if let Some(using) = self.clones_in_use.get(&clone_dir) {
            if !using.is_empty() {
                // The directory is currently in use by another dataflow. Rebuilding
                // while a dataflow is running could lead to unintended behavior.
                eyre::bail!(
                    "the build directory is still in use by the following \
                    dataflows, please stop them before rebuilding: {}",
                    using.iter().join(", ")
                )
            }
        }

        let reuse = if self.clone_dir_ready(session_id, &clone_dir) {
            // The directory already contains a checkout of the commit we're interested in.
            // So we can simply reuse the directory without doing any additional git
            // operations.
            ReuseOptions::Reuse {
                dir: clone_dir.clone(),
            }
        } else if let Some(previous_commit_hash) = prev_commit_hash {
            // we might be able to update a previous clone
            let prev_clone_dir =
                Self::clone_dir_path(&target_dir, &repo_url, &previous_commit_hash)?;

            if self
                .clones_in_use
                .get(&prev_clone_dir)
                .map(|ids| !ids.is_empty())
                .unwrap_or(false)
            {
                // previous clone is still in use -> we cannot rename it, but we can copy it
                ReuseOptions::CopyAndFetch {
                    from: prev_clone_dir,
                    target_dir: clone_dir.clone(),
                    commit_hash,
                }
            } else if prev_clone_dir.exists() {
                // there is an unused previous clone that is not in use -> rename it
                ReuseOptions::RenameAndFetch {
                    from: prev_clone_dir,
                    target_dir: clone_dir.clone(),
                    commit_hash,
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

    pub fn in_use(&self, dir: &Path) -> bool {
        self.clones_in_use
            .get(dir)
            .map(|ids| !ids.is_empty())
            .unwrap_or(false)
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
        let mut path = base_dir.join(repo_url.host_str().context("git URL has no hostname")?);
        path.extend(repo_url.path_segments().context("no path in git URL")?);
        let path = path.join(commit_hash);
        Ok(dunce::simplified(&path).to_owned())
    }
}

pub struct GitFolder {
    /// Specifies whether an existing repo should be reused.
    reuse: ReuseOptions,
}

impl GitFolder {
    pub async fn prepare(self, logger: &mut impl BuildLogger) -> eyre::Result<PathBuf> {
        let GitFolder { reuse } = self;

        let clone_dir = match reuse {
            ReuseOptions::NewClone {
                target_dir,
                repo_url,
                commit_hash,
            } => {
                let repository = clone_into(repo_url, &target_dir, logger).await?;
                checkout_tree(&repository, &commit_hash)?;
                target_dir
            }
            ReuseOptions::CopyAndFetch {
                from,
                target_dir,
                commit_hash,
            } => {
                tokio::fs::copy(&from, &target_dir)
                    .await
                    .context("failed to copy repo clone")?;

                logger
                    .log_message(
                        LogLevel::Info,
                        format!("fetching changes after copying {}", from.display()),
                    )
                    .await;

                let repository = fetch_changes(&target_dir, None).await?;
                checkout_tree(&repository, &commit_hash)?;
                target_dir
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

                let repository = fetch_changes(&target_dir, None).await?;
                checkout_tree(&repository, &commit_hash)?;
                target_dir
            }
            ReuseOptions::Reuse { dir } => {
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

enum ReuseOptions {
    /// Create a new clone of the repository.
    NewClone {
        target_dir: PathBuf,
        repo_url: Url,
        commit_hash: String,
    },
    /// Reuse an existing up-to-date clone of the repository.
    Reuse { dir: PathBuf },
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

fn rev_str(rev: &Option<GitRepoRev>) -> String {
    match rev {
        Some(GitRepoRev::Branch(branch)) => format!(" (branch {branch})"),
        Some(GitRepoRev::Tag(tag)) => format!(" (tag {tag})"),
        Some(GitRepoRev::Rev(rev)) => format!(" (rev {rev})"),
        None => String::new(),
    }
}

async fn clone_into(
    repo_addr: Url,
    clone_dir: &Path,
    logger: &mut impl BuildLogger,
) -> eyre::Result<git2::Repository> {
    if let Some(parent) = clone_dir.parent() {
        tokio::fs::create_dir_all(parent)
            .await
            .context("failed to create parent directory for git clone")?;
    }

    logger
        .log_message(
            LogLevel::Info,
            format!("cloning {repo_addr} into {}", clone_dir.display()),
        )
        .await;
    let clone_dir = clone_dir.to_owned();
    let task = tokio::task::spawn_blocking(move || {
        let mut builder = git2::build::RepoBuilder::new();
        let mut fetch_options = git2::FetchOptions::new();
        fetch_options.download_tags(git2::AutotagOption::All);
        builder.fetch_options(fetch_options);
        builder
            .clone(repo_addr.as_str(), &clone_dir)
            .context("failed to clone repo")
    });
    let repo = task.await??;
    Ok(repo)
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
