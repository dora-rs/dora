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
    /// The node id is used unchanged as the file name. That is safe because
    /// `NodeId` only allows `[a-zA-Z0-9_.-]` and forbids a leading `.`, so it
    /// can contain no path separator and cannot be a `.`/`..` segment.
    /// Rewriting it instead would map distinct ids (`cam.left`, `cam_left`)
    /// to the same file (#4186).
    pub fn artifact_path(&self, build_id: &Uuid, node_id: &NodeId) -> PathBuf {
        self.base_dir
            .join(build_id.to_string())
            .join(node_id.as_ref())
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
    fn artifact_path_uses_node_id_unchanged() {
        // `cam.left`/`cam_left` must not collide, and `a..b` is a valid id,
        // not a traversal (#4186).
        let store = ArtifactStore::new().unwrap();
        let build_id = Uuid::new_v4();
        for id in ["my-node", "cam.left", "cam_left", "a..b"] {
            let node_id: NodeId = id.parse().unwrap();
            assert_eq!(
                store.artifact_path(&build_id, &node_id),
                store.base_dir.join(build_id.to_string()).join(id)
            );
        }
    }

    #[test]
    fn unsafe_file_names_are_not_node_ids() {
        // `artifact_path` relies on `NodeId` validation to keep the file
        // name inside the build directory.
        for id in [
            "",
            ".",
            "..",
            ".hidden",
            "../etc/passwd",
            "foo/bar",
            r"foo\bar",
            r"foo\..\bar",
            "my node!",
        ] {
            assert!(id.parse::<NodeId>().is_err(), "{id:?} must be rejected");
        }
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
}
