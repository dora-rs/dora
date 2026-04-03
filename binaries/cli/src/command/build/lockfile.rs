use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

use adora_message::{
    common::GitSource,
    descriptor::{GitRepoRev, NodeSource},
    id::NodeId,
};
use eyre::{Context, ContextCompat};

const LOCKFILE_VERSION: u32 = 2;
const MIN_SUPPORTED_LOCKFILE_VERSION: u32 = 1;

#[derive(serde::Serialize)]
struct BuildLockfileView<'a> {
    version: u32,
    descriptor_fingerprint: &'a str,
    git_sources: &'a BTreeMap<NodeId, GitSource>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct BuildLockfile {
    version: u32,
    #[serde(default)]
    descriptor_fingerprint: Option<String>,
    pub git_sources: BTreeMap<NodeId, GitSource>,
}

impl BuildLockfile {
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
        if !(MIN_SUPPORTED_LOCKFILE_VERSION..=LOCKFILE_VERSION).contains(&lockfile.version) {
            eyre::bail!(
                "unsupported lockfile version {}, expected {}..={}",
                lockfile.version,
                MIN_SUPPORTED_LOCKFILE_VERSION,
                LOCKFILE_VERSION
            );
        }
        Ok(lockfile)
    }

    pub(crate) fn fingerprint_descriptor_git_sources(
        descriptor_git_sources: &BTreeMap<NodeId, NodeSource>,
    ) -> String {
        let mut canonical = String::new();
        // Intentional: include schema version in canonical content so lockfile format
        // bumps force regeneration even if descriptor sources are unchanged.
        canonical.push_str(&format!("adora-lockfile-v{LOCKFILE_VERSION}\n"));

        for (node_id, source) in descriptor_git_sources {
            let NodeSource::GitBranch { repo, rev } = source else {
                continue;
            };
            canonical.push_str(node_id.as_ref());
            canonical.push('\n');
            canonical.push_str(repo);
            canonical.push('\n');
            let (kind, value) = match rev {
                Some(GitRepoRev::Branch(v)) => ("branch", v.as_str()),
                Some(GitRepoRev::Tag(v)) => ("tag", v.as_str()),
                Some(GitRepoRev::Rev(v)) => ("rev", v.as_str()),
                None => ("head", ""),
            };
            canonical.push_str(kind);
            canonical.push(':');
            canonical.push_str(value);
            canonical.push('\n');
        }

        // Deterministic compact digest (FNV-1a 64-bit). This is for reproducibility
        // drift detection, not cryptographic integrity.
        const FNV_OFFSET_BASIS: u64 = 0xcbf29ce484222325;
        const FNV_PRIME: u64 = 0x100000001b3;
        let mut hash = FNV_OFFSET_BASIS;
        for byte in canonical.as_bytes() {
            hash ^= *byte as u64;
            hash = hash.wrapping_mul(FNV_PRIME);
        }
        format!("{hash:016x}")
    }

    pub(crate) fn ensure_descriptor_fingerprint_matches(&self, expected: &str) -> eyre::Result<()> {
        let Some(actual) = self.descriptor_fingerprint.as_deref() else {
            eyre::bail!(
                "lockfile is missing `descriptor_fingerprint` (v1 format). \
                 regenerate with `adora build --write-lockfile` before using `--locked`"
            );
        };
        if actual != expected {
            eyre::bail!(
                "lockfile descriptor/source fingerprint mismatch. \
                 the dataflow git source graph changed since lockfile generation; \
                 regenerate with `adora build --write-lockfile`"
            );
        }
        Ok(())
    }

    pub fn write_git_sources(
        path: &Path,
        git_sources: &BTreeMap<NodeId, GitSource>,
        descriptor_fingerprint: &str,
    ) -> eyre::Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).context("failed to create lockfile directory")?;
        }
        let view = BuildLockfileView {
            version: LOCKFILE_VERSION,
            descriptor_fingerprint,
            git_sources,
        };
        let serialized = serde_yaml::to_string(&view).context("failed to serialize lockfile")?;
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
        let mut descriptor_git_sources = BTreeMap::new();
        git_sources.insert(
            "node-a".parse().unwrap(),
            source("https://example.com/repo", "abc123"),
        );
        descriptor_git_sources.insert(
            "node-a".parse().unwrap(),
            NodeSource::GitBranch {
                repo: "https://example.com/repo".to_string(),
                rev: Some(GitRepoRev::Branch("main".to_string())),
            },
        );
        let fingerprint =
            BuildLockfile::fingerprint_descriptor_git_sources(&descriptor_git_sources);

        BuildLockfile::write_git_sources(&lockfile_path, &git_sources, &fingerprint).unwrap();
        let loaded = BuildLockfile::read_from(&lockfile_path).unwrap();

        assert_eq!(loaded.version, LOCKFILE_VERSION);
        assert_eq!(loaded.git_sources, git_sources);
        loaded
            .ensure_descriptor_fingerprint_matches(&fingerprint)
            .unwrap();
    }

    #[test]
    fn get_source_rejects_repo_mismatch() {
        let mut git_sources = BTreeMap::new();
        git_sources.insert(
            "node-a".parse().unwrap(),
            source("https://example.com/repo", "abc123"),
        );
        let lockfile = BuildLockfile {
            version: LOCKFILE_VERSION,
            descriptor_fingerprint: Some("abc".into()),
            git_sources,
        };

        let err = lockfile
            .get_source(&"node-a".parse().unwrap(), "https://example.com/other")
            .unwrap_err();
        assert!(err.to_string().contains("lockfile repo mismatch"));
    }

    #[test]
    fn fingerprint_detects_source_graph_drift() {
        let mut before = BTreeMap::new();
        before.insert(
            "node-a".parse().unwrap(),
            NodeSource::GitBranch {
                repo: "https://example.com/repo".to_string(),
                rev: Some(GitRepoRev::Branch("main".to_string())),
            },
        );
        let mut after = before.clone();
        after.insert(
            "node-b".parse().unwrap(),
            NodeSource::GitBranch {
                repo: "https://example.com/repo2".to_string(),
                rev: Some(GitRepoRev::Tag("v1.2.3".to_string())),
            },
        );

        let before_fp = BuildLockfile::fingerprint_descriptor_git_sources(&before);
        let after_fp = BuildLockfile::fingerprint_descriptor_git_sources(&after);
        assert_ne!(before_fp, after_fp);
    }

    #[test]
    fn rejects_v1_lockfile_without_fingerprint_in_locked_mode() {
        let lockfile = BuildLockfile {
            version: 1,
            descriptor_fingerprint: None,
            git_sources: BTreeMap::new(),
        };
        let err = lockfile
            .ensure_descriptor_fingerprint_matches("expected")
            .unwrap_err();
        assert!(err.to_string().contains("missing `descriptor_fingerprint`"));
    }
}
