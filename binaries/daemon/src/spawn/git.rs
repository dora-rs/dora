use crate::log::NodeLogger;
use dora_message::{common::LogLevel, descriptor::GitRepoRev, DataflowId};
use eyre::{ContextCompat, WrapErr};
use git2::FetchOptions;
use std::{
    collections::{BTreeMap, BTreeSet},
    path::{Path, PathBuf},
};
use url::Url;
use uuid::Uuid;

pub struct GitFolder {
    /// The URL of the git repository.
    repo_addr: String,
    /// The branch, tag, or git revision to checkout.
    rev: Option<GitRepoRev>,
    /// The directory that should contain the checked-out repository.
    clone_dir: PathBuf,
    /// Specifies whether an existing repo should be reused.
    reuse: ReuseOptions,
}

impl GitFolder {
    pub fn choose_clone_dir(
        dataflow_id: uuid::Uuid,
        repo_addr: String,
        rev: Option<GitRepoRev>,
        target_dir: &Path,
        repos_in_use: &mut BTreeMap<PathBuf, BTreeSet<DataflowId>>,
    ) -> eyre::Result<Self> {
        let repo_url = Url::parse(&repo_addr).context("failed to parse git repository URL")?;

        let base_dir = {
            let base = {
                let mut path =
                    target_dir.join(repo_url.host_str().context("git URL has no hostname")?);

                path.extend(repo_url.path_segments().context("no path in git URL")?);
                path
            };
            match &rev {
                None => base,
                Some(rev) => match rev {
                    GitRepoRev::Branch(branch) => base.join("branch").join(branch),
                    GitRepoRev::Tag(tag) => base.join("tag").join(tag),
                    GitRepoRev::Rev(rev) => base.join("rev").join(rev),
                },
            }
        };
        let clone_dir = if clone_dir_exists(&base_dir, repos_in_use) {
            let used_by_other = used_by_other_dataflow(dataflow_id, &base_dir, repos_in_use);
            if used_by_other {
                // don't reuse, choose new directory
                // (TODO reuse if still up to date)

                let dir_name = base_dir.file_name().unwrap().to_str().unwrap();
                let mut i = 1;
                loop {
                    let new_path = base_dir.with_file_name(format!("{dir_name}-{i}"));
                    if clone_dir_exists(&new_path, repos_in_use)
                        && used_by_other_dataflow(dataflow_id, &new_path, repos_in_use)
                    {
                        i += 1;
                    } else {
                        break new_path;
                    }
                }
            } else {
                base_dir
            }
        } else {
            base_dir
        };
        let clone_dir = dunce::simplified(&clone_dir).to_owned();

        let reuse = if clone_dir_exists(&clone_dir, repos_in_use) {
            let empty = BTreeSet::new();
            let in_use = repos_in_use.get(&clone_dir).unwrap_or(&empty);
            let used_by_other_dataflow = in_use.iter().any(|&id| id != dataflow_id);
            if used_by_other_dataflow {
                // The directory is currently in use by another dataflow. We currently don't
                // support reusing the same clone across multiple dataflow runs. Above, we
                // choose a new directory if we detect such a case. So this `if` branch
                // should never be reached.
                eyre::bail!("clone_dir is already in use by other dataflow")
            } else if in_use.is_empty() {
                // The cloned repo is not used by any dataflow, so we can safely reuse it. However,
                // the clone might be still on an older commit, so we need to do a `git fetch`
                // before we reuse it.
                ReuseOptions::ReuseAfterFetch
            } else {
                // This clone is already used for another node of this dataflow. We will do a
                // `git fetch` operation for the first node of this dataflow, so we don't need
                // to do it again for other nodes of the dataflow. So we can simply reuse the
                // directory without doing any additional git operations.
                ReuseOptions::Reuse
            }
        } else {
            ReuseOptions::NewClone
        };
        repos_in_use
            .entry(clone_dir.clone())
            .or_default()
            .insert(dataflow_id);

        Ok(GitFolder {
            clone_dir,
            reuse,
            repo_addr,
            rev,
        })
    }

    pub async fn prepare(self, logger: &mut NodeLogger<'_>) -> eyre::Result<PathBuf> {
        let GitFolder {
            clone_dir,
            reuse,
            repo_addr,
            rev,
        } = self;

        let rev_str = rev_str(&rev);
        let refname = rev.clone().map(|rev| match rev {
            GitRepoRev::Branch(branch) => format!("refs/remotes/origin/{branch}"),
            GitRepoRev::Tag(tag) => format!("refs/tags/{tag}"),
            GitRepoRev::Rev(rev) => rev,
        });

        match reuse {
            ReuseOptions::NewClone => {
                let repository = clone_into(&repo_addr, &rev, &clone_dir, logger).await?;
                checkout_tree(&repository, refname)?;
            }
            ReuseOptions::ReuseAfterFetch => {
                logger
                    .log(
                        LogLevel::Info,
                        None,
                        format!("fetching changes and reusing {repo_addr}{rev_str}"),
                    )
                    .await;
                let refname_cloned = refname.clone();
                let clone_dir = clone_dir.clone();
                let repository = fetch_changes(clone_dir, refname_cloned).await?;
                checkout_tree(&repository, refname)?;
            }
            ReuseOptions::Reuse => {
                logger
                    .log(
                        LogLevel::Info,
                        None,
                        format!("reusing up-to-date {repo_addr}{rev_str}"),
                    )
                    .await;
            }
        };
        Ok(clone_dir)
    }
}

fn used_by_other_dataflow(
    dataflow_id: uuid::Uuid,
    clone_dir_base: &PathBuf,
    repos_in_use: &mut BTreeMap<PathBuf, BTreeSet<DataflowId>>,
) -> bool {
    let empty = BTreeSet::new();
    let in_use = repos_in_use.get(clone_dir_base).unwrap_or(&empty);
    let used_by_other_dataflow = in_use.iter().any(|&id| id != dataflow_id);
    used_by_other_dataflow
}

enum ReuseOptions {
    /// Create a new clone of the repository.
    NewClone,
    /// Reuse an existing up-to-date clone of the repository.
    Reuse,
    /// Update an older clone of the repository, then reuse it.
    ReuseAfterFetch,
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
    repo_addr: &String,
    rev: &Option<GitRepoRev>,
    clone_dir: &Path,
    logger: &mut NodeLogger<'_>,
) -> eyre::Result<git2::Repository> {
    if let Some(parent) = clone_dir.parent() {
        tokio::fs::create_dir_all(parent)
            .await
            .context("failed to create parent directory for git clone")?;
    }

    let rev_str = rev_str(rev);
    logger
        .log(
            LogLevel::Info,
            None,
            format!("cloning {repo_addr}{rev_str} into {}", clone_dir.display()),
        )
        .await;
    let rev: Option<GitRepoRev> = rev.clone();
    let clone_into = clone_dir.to_owned();
    let repo_addr = repo_addr.clone();
    let task = tokio::task::spawn_blocking(move || {
        let mut builder = git2::build::RepoBuilder::new();
        let mut fetch_options = git2::FetchOptions::new();
        fetch_options.download_tags(git2::AutotagOption::All);
        builder.fetch_options(fetch_options);
        if let Some(GitRepoRev::Branch(branch)) = &rev {
            builder.branch(branch);
        }
        builder
            .clone(&repo_addr, &clone_into)
            .context("failed to clone repo")
    });
    let repo = task.await??;
    Ok(repo)
}

async fn fetch_changes(
    repo_dir: PathBuf,
    refname: Option<String>,
) -> Result<git2::Repository, eyre::Error> {
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

fn checkout_tree(repository: &git2::Repository, refname: Option<String>) -> eyre::Result<()> {
    if let Some(refname) = refname {
        let (object, reference) = repository
            .revparse_ext(&refname)
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
    }
    Ok(())
}

fn clone_dir_exists(dir: &PathBuf, repos_in_use: &BTreeMap<PathBuf, BTreeSet<Uuid>>) -> bool {
    repos_in_use.contains_key(dir) || dir.exists()
}
