use dora_message::{common::GitSource, descriptor::GitRepoRev, id::NodeId};
use eyre::Context;
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

const LOCKFILE_EXTENSION: &str = "dora.lock";
const LOCKFILE_VERSION: u32 = 1;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DoraLock {
    pub version: u32,
    #[serde(default)]
    pub git_sources: BTreeMap<NodeId, LockedGitSource>,
}

impl DoraLock {
    pub fn new() -> Self {
        Self {
            version: LOCKFILE_VERSION,
            git_sources: BTreeMap::new(),
        }
    }

    pub fn read_for_dataflow(dataflow_path: &Path) -> eyre::Result<Self> {
        let lockfile_path = lockfile_path(dataflow_path)?;
        if !lockfile_path.exists() {
            return Ok(Self::new());
        }

        let contents = std::fs::read_to_string(&lockfile_path)
            .with_context(|| format!("failed to read lockfile `{}`", lockfile_path.display()))?;
        let lockfile: DoraLock = serde_yaml::from_str(&contents).with_context(|| {
            format!(
                "failed to parse lockfile `{}` as YAML",
                lockfile_path.display()
            )
        })?;

        if lockfile.version != LOCKFILE_VERSION {
            eyre::bail!(
                "unsupported lockfile version {} in `{}` (expected {})",
                lockfile.version,
                lockfile_path.display(),
                LOCKFILE_VERSION
            );
        }

        Ok(lockfile)
    }

    pub fn write_for_dataflow(&self, dataflow_path: &Path) -> eyre::Result<()> {
        let lockfile_path = lockfile_path(dataflow_path)?;
        let serialized = serde_yaml::to_string(self).context("failed to serialize lockfile")?;
        std::fs::write(&lockfile_path, serialized)
            .with_context(|| format!("failed to write lockfile `{}`", lockfile_path.display()))?;
        Ok(())
    }

    pub fn resolve_git_source(
        &self,
        node_id: &NodeId,
        repo: &str,
        requested_rev: Option<&GitRepoRev>,
    ) -> Option<GitSource> {
        let entry = self.git_sources.get(node_id)?;
        if entry.repo != repo {
            return None;
        }
        if entry.requested_rev != requested_rev.map(LockedGitRev::from_repo_rev) {
            return None;
        }

        Some(GitSource {
            repo: entry.repo.clone(),
            commit_hash: entry.commit_hash.clone(),
        })
    }

    pub fn set_git_source(
        &mut self,
        node_id: NodeId,
        repo: String,
        requested_rev: Option<GitRepoRev>,
        commit_hash: String,
    ) {
        self.git_sources.insert(
            node_id,
            LockedGitSource {
                repo,
                requested_rev: requested_rev.as_ref().map(LockedGitRev::from_repo_rev),
                commit_hash,
            },
        );
    }
}

impl Default for DoraLock {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LockedGitSource {
    pub repo: String,
    #[serde(default)]
    pub requested_rev: Option<LockedGitRev>,
    pub commit_hash: String,
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields, rename_all = "snake_case")]
pub enum LockedGitRev {
    Branch(String),
    Tag(String),
    Rev(String),
}

impl LockedGitRev {
    fn from_repo_rev(rev: &GitRepoRev) -> Self {
        match rev {
            GitRepoRev::Branch(branch) => Self::Branch(branch.clone()),
            GitRepoRev::Tag(tag) => Self::Tag(tag.clone()),
            GitRepoRev::Rev(rev) => Self::Rev(rev.clone()),
        }
    }
}

fn lockfile_path(dataflow_path: &Path) -> eyre::Result<PathBuf> {
    let dir = dataflow_path
        .parent()
        .ok_or_else(|| eyre::eyre!("dataflow path has no parent directory"))?;
    let stem = dataflow_path
        .file_stem()
        .ok_or_else(|| eyre::eyre!("dataflow path has no file stem"))?
        .to_string_lossy();

    Ok(dir.join(format!("{stem}.{LOCKFILE_EXTENSION}")))
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_message::descriptor::GitRepoRev;
    use uuid::Uuid;

    #[test]
    fn resolve_git_source_matches_repo_and_rev() {
        let node_id = NodeId::from("camera".to_owned());
        let mut lock = DoraLock::new();
        lock.set_git_source(
            node_id.clone(),
            "https://github.com/dora-rs/dora.git".to_owned(),
            Some(GitRepoRev::Branch("main".to_owned())),
            "abc123".to_owned(),
        );

        let resolved = lock.resolve_git_source(
            &node_id,
            "https://github.com/dora-rs/dora.git",
            Some(&GitRepoRev::Branch("main".to_owned())),
        );

        assert!(resolved.is_some());
        assert_eq!(resolved.unwrap().commit_hash, "abc123");
    }

    #[test]
    fn resolve_git_source_rejects_mismatched_rev() {
        let node_id = NodeId::from("camera".to_owned());
        let mut lock = DoraLock::new();
        lock.set_git_source(
            node_id.clone(),
            "https://github.com/dora-rs/dora.git".to_owned(),
            Some(GitRepoRev::Branch("main".to_owned())),
            "abc123".to_owned(),
        );

        let resolved = lock.resolve_git_source(
            &node_id,
            "https://github.com/dora-rs/dora.git",
            Some(&GitRepoRev::Tag("v0.1.0".to_owned())),
        );

        assert!(resolved.is_none());
    }

    #[test]
    fn lockfile_roundtrip() {
        let dir = std::env::temp_dir().join(format!("dora-lock-test-{}", Uuid::new_v4()));
        std::fs::create_dir_all(&dir).unwrap();
        let dataflow_path = dir.join("dataflow.yml");
        std::fs::write(&dataflow_path, "nodes: []\n").unwrap();

        let mut lock = DoraLock::new();
        lock.set_git_source(
            NodeId::from("node-a".to_owned()),
            "https://github.com/dora-rs/dora.git".to_owned(),
            None,
            "deadbeef".to_owned(),
        );
        lock.write_for_dataflow(&dataflow_path).unwrap();

        let loaded = DoraLock::read_for_dataflow(&dataflow_path).unwrap();
        assert_eq!(loaded.version, LOCKFILE_VERSION);
        assert_eq!(loaded.git_sources.len(), 1);

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn lockfile_isolated_per_dataflow_in_same_directory() {
        let dir = std::env::temp_dir().join(format!("dora-lock-test-{}", Uuid::new_v4()));
        std::fs::create_dir_all(&dir).unwrap();
        let df_a = dir.join("a.yml");
        let df_b = dir.join("b.yaml");
        std::fs::write(&df_a, "nodes: []\n").unwrap();
        std::fs::write(&df_b, "nodes: []\n").unwrap();

        let mut lock_a = DoraLock::new();
        lock_a.set_git_source(
            NodeId::from("node-a".to_owned()),
            "https://github.com/dora-rs/dora.git".to_owned(),
            None,
            "aaaa".to_owned(),
        );
        lock_a.write_for_dataflow(&df_a).unwrap();

        let mut lock_b = DoraLock::new();
        lock_b.set_git_source(
            NodeId::from("node-b".to_owned()),
            "https://github.com/dora-rs/dora.git".to_owned(),
            None,
            "bbbb".to_owned(),
        );
        lock_b.write_for_dataflow(&df_b).unwrap();

        let loaded_a = DoraLock::read_for_dataflow(&df_a).unwrap();
        let loaded_b = DoraLock::read_for_dataflow(&df_b).unwrap();

        assert_eq!(loaded_a.git_sources.len(), 1);
        assert!(
            loaded_a
                .git_sources
                .contains_key(&NodeId::from("node-a".to_owned()))
        );
        assert!(
            !loaded_a
                .git_sources
                .contains_key(&NodeId::from("node-b".to_owned()))
        );

        assert_eq!(loaded_b.git_sources.len(), 1);
        assert!(
            loaded_b
                .git_sources
                .contains_key(&NodeId::from("node-b".to_owned()))
        );
        assert!(
            !loaded_b
                .git_sources
                .contains_key(&NodeId::from("node-a".to_owned()))
        );

        let _ = std::fs::remove_dir_all(&dir);
    }
}
