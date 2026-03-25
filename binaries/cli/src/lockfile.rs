use crate::session::DataflowSession;
use dora_core::descriptor::{CoreNodeKind, CustomNode, Descriptor, DescriptorExt};
use dora_message::{descriptor::GitRepoRev, id::NodeId};
use eyre::{Context, bail};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

const LOCKFILE_VERSION: u32 = 1;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DoraLockfile {
    pub version: u32,
    #[serde(default)]
    pub git_sources: BTreeMap<NodeId, LockedGitSource>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct LockedGitSource {
    pub repo: String,
    pub requested_rev: Option<GitRepoRev>,
    pub commit_hash: String,
}

pub fn validate_locked_dataflow(
    dataflow_path: &Path,
    descriptor: &Descriptor,
    dataflow_session: &DataflowSession,
) -> eyre::Result<()> {
    let lockfile = read_lockfile(dataflow_path)?;
    if lockfile.version != LOCKFILE_VERSION {
        bail!(
            "unsupported dora.lock version `{}` (expected `{}`)",
            lockfile.version,
            LOCKFILE_VERSION
        );
    }

    let expected = collect_git_dependencies(descriptor)?;
    let mut remaining = lockfile.git_sources;

    for (node_id, expected_source) in expected {
        let Some(locked_source) = remaining.remove(&node_id) else {
            bail!(
                "missing lock entry for git node `{node_id}` in `{}`",
                lockfile_path(dataflow_path).display()
            );
        };

        if locked_source.repo != expected_source.repo {
            bail!(
                "locked repository mismatch for node `{node_id}`: descriptor=`{}` lock=`{}`",
                expected_source.repo,
                locked_source.repo
            );
        }

        if !same_git_rev(
            expected_source.requested_rev.as_ref(),
            locked_source.requested_rev.as_ref(),
        ) {
            bail!(
                "locked revision selector mismatch for node `{node_id}` (descriptor and dora.lock differ)"
            );
        }

        let Some(session_git_source) = dataflow_session.git_sources.get(&node_id) else {
            bail!(
                "missing built git source for node `{node_id}` in session state; run `dora build` before using `--locked`"
            );
        };

        if session_git_source.repo != locked_source.repo
            || session_git_source.commit_hash != locked_source.commit_hash
        {
            bail!(
                "session git source mismatch for node `{node_id}`; locked commit is `{}`, session commit is `{}`. Re-run `dora build` to rebuild with the lockfile state",
                locked_source.commit_hash,
                session_git_source.commit_hash
            );
        }
    }

    if !remaining.is_empty() {
        let stale_entries = remaining
            .keys()
            .map(ToString::to_string)
            .collect::<Vec<_>>()
            .join(", ");
        bail!("dora.lock contains stale git entries not present in descriptor: {stale_entries}");
    }

    Ok(())
}

fn read_lockfile(dataflow_path: &Path) -> eyre::Result<DoraLockfile> {
    let path = lockfile_path(dataflow_path);
    let file = std::fs::read_to_string(&path)
        .with_context(|| format!("failed to read dora lockfile `{}`", path.display()))?;
    serde_yaml::from_str(&file)
        .with_context(|| format!("failed to parse dora lockfile `{}`", path.display()))
}

fn lockfile_path(dataflow_path: &Path) -> PathBuf {
    dataflow_path.with_file_name("dora.lock")
}

fn collect_git_dependencies(
    descriptor: &Descriptor,
) -> eyre::Result<BTreeMap<NodeId, GitDependency>> {
    let resolved_nodes = descriptor
        .resolve_aliases_and_set_defaults()
        .context("failed to resolve nodes while validating lockfile")?;
    let mut dependencies = BTreeMap::new();
    for (node_id, node) in resolved_nodes {
        if let CoreNodeKind::Custom(CustomNode {
            source: dora_message::descriptor::NodeSource::GitBranch { repo, rev },
            ..
        }) = node.kind
        {
            dependencies.insert(
                node_id,
                GitDependency {
                    repo,
                    requested_rev: rev,
                },
            );
        }
    }
    Ok(dependencies)
}

fn same_git_rev(left: Option<&GitRepoRev>, right: Option<&GitRepoRev>) -> bool {
    match (left, right) {
        (None, None) => true,
        (Some(GitRepoRev::Branch(a)), Some(GitRepoRev::Branch(b))) => a == b,
        (Some(GitRepoRev::Tag(a)), Some(GitRepoRev::Tag(b))) => a == b,
        (Some(GitRepoRev::Rev(a)), Some(GitRepoRev::Rev(b))) => a == b,
        _ => false,
    }
}

struct GitDependency {
    repo: String,
    requested_rev: Option<GitRepoRev>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_core::build::BuildInfo;
    use dora_message::{BuildId, SessionId, common::GitSource};

    #[test]
    fn locked_validation_accepts_matching_descriptor_lock_and_session() {
        let dataflow = dataflow_descriptor("https://example.com/repo.git", Some("main"));
        let temp = tmp_dir();
        let dataflow_path = temp.join("dataflow.yml");
        std::fs::write(&dataflow_path, "nodes: []").unwrap();

        write_lock(
            &dataflow_path,
            "https://example.com/repo.git",
            Some(GitRepoRev::Branch("main".to_owned())),
            "abc123",
        );

        let session = session_with_commit("https://example.com/repo.git", "abc123");
        validate_locked_dataflow(&dataflow_path, &dataflow, &session).unwrap();
    }

    #[test]
    fn locked_validation_fails_when_session_commit_differs() {
        let dataflow = dataflow_descriptor("https://example.com/repo.git", Some("main"));
        let temp = tmp_dir();
        let dataflow_path = temp.join("dataflow.yml");
        std::fs::write(&dataflow_path, "nodes: []").unwrap();

        write_lock(
            &dataflow_path,
            "https://example.com/repo.git",
            Some(GitRepoRev::Branch("main".to_owned())),
            "abc123",
        );
        let session = session_with_commit("https://example.com/repo.git", "def456");

        let err = validate_locked_dataflow(&dataflow_path, &dataflow, &session)
            .unwrap_err()
            .to_string();
        assert!(err.contains("session git source mismatch"));
    }

    #[test]
    fn locked_validation_fails_when_descriptor_rev_differs() {
        let dataflow = dataflow_descriptor("https://example.com/repo.git", Some("release"));
        let temp = tmp_dir();
        let dataflow_path = temp.join("dataflow.yml");
        std::fs::write(&dataflow_path, "nodes: []").unwrap();

        write_lock(
            &dataflow_path,
            "https://example.com/repo.git",
            Some(GitRepoRev::Branch("main".to_owned())),
            "abc123",
        );
        let session = session_with_commit("https://example.com/repo.git", "abc123");

        let err = validate_locked_dataflow(&dataflow_path, &dataflow, &session)
            .unwrap_err()
            .to_string();
        assert!(err.contains("revision selector mismatch"));
    }

    fn dataflow_descriptor(repo: &str, branch: Option<&str>) -> Descriptor {
        let mut rev = String::new();
        if let Some(branch) = branch {
            rev = format!("    branch: {branch}\n");
        }
        let yaml = format!(
            "nodes:\n  - id: node-a\n    path: ./bin/node-a\n    git: {repo}\n{rev}    outputs: [out]\n"
        );
        serde_yaml::from_str::<Descriptor>(&yaml).unwrap()
    }

    fn session_with_commit(repo: &str, commit: &str) -> DataflowSession {
        DataflowSession {
            build_id: Some(BuildId::generate()),
            session_id: SessionId::generate(),
            git_sources: BTreeMap::from([(
                NodeId::from("node-a".to_owned()),
                GitSource {
                    repo: repo.to_owned(),
                    commit_hash: commit.to_owned(),
                },
            )]),
            local_build: Some(BuildInfo {
                node_working_dirs: BTreeMap::new(),
            }),
        }
    }

    fn write_lock(dataflow_path: &Path, repo: &str, rev: Option<GitRepoRev>, commit_hash: &str) {
        let lock = DoraLockfile {
            version: LOCKFILE_VERSION,
            git_sources: BTreeMap::from([(
                NodeId::from("node-a".to_owned()),
                LockedGitSource {
                    repo: repo.to_owned(),
                    requested_rev: rev,
                    commit_hash: commit_hash.to_owned(),
                },
            )]),
        };
        std::fs::write(
            lockfile_path(dataflow_path),
            serde_yaml::to_string(&lock).unwrap(),
        )
        .unwrap();
    }

    fn tmp_dir() -> PathBuf {
        let dir = std::env::temp_dir().join(format!("dora-cli-lock-test-{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }
}
