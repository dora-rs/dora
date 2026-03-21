use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

use adora_message::{common::GitSource, id::NodeId};
use eyre::{Context, ContextCompat};

const LOCKFILE_VERSION: u32 = 1;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct BuildLockfile {
    version: u32,
    pub git_sources: BTreeMap<NodeId, GitSource>,
}

impl BuildLockfile {
    pub fn from_git_sources(git_sources: BTreeMap<NodeId, GitSource>) -> Self {
        Self {
            version: LOCKFILE_VERSION,
            git_sources,
        }
    }

    pub fn path_for_dataflow(dataflow_path: &Path, override_path: Option<PathBuf>) -> PathBuf {
        override_path.unwrap_or_else(|| {
            let file_stem = dataflow_path
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("dataflow");
            dataflow_path.with_file_name(format!("{file_stem}.adora-lock.yaml"))
        })
    }

    pub fn read_from(path: &Path) -> eyre::Result<Self> {
        let contents = std::fs::read_to_string(path).context("failed to read lockfile")?;
        let lockfile: BuildLockfile =
            serde_yaml::from_str(&contents).context("failed to parse lockfile")?;
        if lockfile.version != LOCKFILE_VERSION {
            eyre::bail!(
                "unsupported lockfile version {}, expected {}",
                lockfile.version,
                LOCKFILE_VERSION
            );
        }
        Ok(lockfile)
    }

    pub fn write_to(&self, path: &Path) -> eyre::Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).context("failed to create lockfile directory")?;
        }
        let serialized = serde_yaml::to_string(self).context("failed to serialize lockfile")?;
        std::fs::write(path, serialized).context("failed to write lockfile")?;
        Ok(())
    }

    pub fn get_source(&self, node_id: &NodeId, repo: &str) -> eyre::Result<GitSource> {
        let source = self
            .git_sources
            .get(node_id)
            .cloned()
            .with_context(|| format!("no lockfile entry found for node `{node_id}`"))?;
        if source.repo != repo {
            eyre::bail!(
                "lockfile repo mismatch for node `{node_id}`: expected `{repo}`, found `{}`",
                source.repo
            );
        }
        Ok(source)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn source(repo: &str, commit_hash: &str) -> GitSource {
        GitSource {
            repo: repo.to_owned(),
            commit_hash: commit_hash.to_owned(),
        }
    }

    #[test]
    fn default_lockfile_path_uses_dataflow_stem() {
        let path = BuildLockfile::path_for_dataflow(Path::new("examples/demo.yml"), None);
        assert_eq!(path, PathBuf::from("examples/demo.adora-lock.yaml"));
    }

    #[test]
    fn read_write_roundtrip() {
        let tmp = tempfile::tempdir().unwrap();
        let lockfile_path = tmp.path().join("demo.adora-lock.yaml");
        let mut git_sources = BTreeMap::new();
        git_sources.insert(
            "node-a".parse().unwrap(),
            source("https://example.com/repo", "abc123"),
        );

        let lockfile = BuildLockfile::from_git_sources(git_sources.clone());
        lockfile.write_to(&lockfile_path).unwrap();
        let loaded = BuildLockfile::read_from(&lockfile_path).unwrap();

        assert_eq!(loaded.version, LOCKFILE_VERSION);
        assert_eq!(loaded.git_sources, git_sources);
    }

    #[test]
    fn get_source_rejects_repo_mismatch() {
        let mut git_sources = BTreeMap::new();
        git_sources.insert(
            "node-a".parse().unwrap(),
            source("https://example.com/repo", "abc123"),
        );
        let lockfile = BuildLockfile::from_git_sources(git_sources);

        let err = lockfile
            .get_source(&"node-a".parse().unwrap(), "https://example.com/other")
            .unwrap_err();
        assert!(err.to_string().contains("lockfile repo mismatch"));
    }
}
