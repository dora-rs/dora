use eyre::Context;
use std::path::PathBuf;
use uuid::Uuid;

/// Simple file-backed artifact store for HTTP binary distribution.
///
/// Daemons pull binaries from the coordinator via HTTP before spawning.
pub(crate) struct ArtifactStore {
    base_dir: PathBuf,
}

impl ArtifactStore {
    /// Create a new artifact store in a unique temp directory.
    pub fn new() -> eyre::Result<Self> {
        let base_dir = std::env::temp_dir().join(format!("dora-artifacts-{}", Uuid::new_v4()));
        std::fs::create_dir_all(&base_dir)
            .with_context(|| format!("failed to create artifact dir {}", base_dir.display()))?;
        Ok(Self { base_dir })
    }

    /// Construct the path to a stored artifact (without checking existence).
    /// Returns `None` if the node_id fails sanitization.
    pub fn artifact_path(&self, build_id: &Uuid, node_id: &str) -> Option<PathBuf> {
        sanitize_node_id(node_id).map(|s| self.base_dir.join(build_id.to_string()).join(s))
    }

    /// Remove all stored artifacts.
    pub fn cleanup(&self) {
        if self.base_dir.exists() {
            if let Err(err) = std::fs::remove_dir_all(&self.base_dir) {
                tracing::warn!(
                    "failed to clean up artifact store at {}: {err}",
                    self.base_dir.display()
                );
            } else {
                tracing::debug!("cleaned up artifact store at {}", self.base_dir.display());
            }
        }
    }
}

impl Drop for ArtifactStore {
    fn drop(&mut self) {
        self.cleanup();
    }
}

/// Sanitize a node_id for safe use as a filename.
/// Returns None if the id is empty or contains path traversal sequences.
fn sanitize_node_id(node_id: &str) -> Option<String> {
    if node_id.is_empty()
        || node_id.contains("..")
        || node_id.contains('/')
        || node_id.contains('\\')
    {
        return None;
    }
    // Replace any non-alphanumeric/dash/underscore chars with underscore
    let sanitized: String = node_id
        .chars()
        .map(|c| {
            if c.is_alphanumeric() || c == '-' || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect();
    Some(sanitized)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn artifact_path_constructs_correct_path() {
        let store = ArtifactStore::new().unwrap();
        let build_id = Uuid::new_v4();
        let path = store.artifact_path(&build_id, "my-node").unwrap();
        assert!(path.ends_with("my-node"));
        assert!(path.to_str().unwrap().contains(&build_id.to_string()));
    }

    #[test]
    fn artifact_path_rejects_traversal() {
        let store = ArtifactStore::new().unwrap();
        let build_id = Uuid::new_v4();
        assert!(store.artifact_path(&build_id, "../etc/passwd").is_none());
        assert!(store.artifact_path(&build_id, "foo/bar").is_none());
        assert!(store.artifact_path(&build_id, "").is_none());
    }

    #[test]
    fn sanitizes_special_chars() {
        assert_eq!(sanitize_node_id("my node!"), Some("my_node_".into()));
        assert_eq!(sanitize_node_id("ok-node_1"), Some("ok-node_1".into()));
    }

    #[test]
    fn drop_cleans_up_temp_dir() {
        // Use a unique directory to avoid races with parallel tests that
        // also create/drop ArtifactStore (they share the same base path).
        let dir = {
            let store = ArtifactStore {
                base_dir: std::env::temp_dir()
                    .join(format!("dora-artifacts-test-{}", Uuid::new_v4())),
            };
            std::fs::create_dir_all(&store.base_dir).unwrap();
            let dir = store.base_dir.clone();
            // Write a file so the dir is non-empty
            let build_dir = dir.join("test-build");
            std::fs::create_dir_all(&build_dir).unwrap();
            std::fs::write(build_dir.join("artifact.bin"), b"data").unwrap();
            assert!(dir.exists());
            dir
            // store dropped here
        };
        assert!(!dir.exists(), "artifact dir should be removed on drop");
    }

    #[test]
    fn artifact_path_rejects_backslash() {
        let store = ArtifactStore::new().unwrap();
        let build_id = Uuid::new_v4();
        assert!(store.artifact_path(&build_id, r"foo\..\bar").is_none());
        assert!(store.artifact_path(&build_id, r"foo\bar").is_none());
    }
}
