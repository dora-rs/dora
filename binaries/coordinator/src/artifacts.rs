use dora_message::id::NodeId;
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
    ///
    /// Returns `None` if `node_id` is not a valid [`NodeId`]. A valid id is
    /// used unchanged as the file name: `NodeId` only allows `[a-zA-Z0-9_.-]`
    /// and forbids a leading `.`, so it can contain no path separator and
    /// cannot be a `.`/`..` segment. Rewriting it instead would map distinct
    /// ids (`cam.left`, `cam_left`) to the same file.
    pub fn artifact_path(&self, build_id: &Uuid, node_id: &str) -> Option<PathBuf> {
        let node_id: NodeId = node_id.parse().ok()?;
        Some(
            self.base_dir
                .join(build_id.to_string())
                .join(node_id.as_ref()),
        )
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
    fn artifact_path_rejects_invalid_node_ids() {
        let store = ArtifactStore::new().unwrap();
        let build_id = Uuid::new_v4();
        for id in ["my node!", ".hidden", "..", ".", "dora", "n\u{e9}"] {
            assert!(
                store.artifact_path(&build_id, id).is_none(),
                "{id:?} must be rejected"
            );
        }
    }

    #[test]
    fn distinct_node_ids_get_distinct_paths() {
        // `.` is valid in a NodeId; it must not be rewritten to `_`, or
        // `cam.left` and `cam_left` would share one artifact file (#4186).
        let store = ArtifactStore::new().unwrap();
        let build_id = Uuid::new_v4();
        let dotted = store.artifact_path(&build_id, "cam.left").unwrap();
        let underscored = store.artifact_path(&build_id, "cam_left").unwrap();
        assert_ne!(dotted, underscored);
        assert!(dotted.ends_with("cam.left"));
    }

    #[test]
    fn artifact_path_accepts_inner_double_dot() {
        // Only a leading `.` is forbidden; `a..b` is a valid NodeId and a
        // plain file name, not a traversal (#4186).
        let store = ArtifactStore::new().unwrap();
        let build_id = Uuid::new_v4();
        let path = store.artifact_path(&build_id, "a..b").unwrap();
        assert_eq!(path, store.base_dir.join(build_id.to_string()).join("a..b"));
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
