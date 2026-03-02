use eyre::{Context, eyre};
use std::path::{Path, PathBuf};
use uuid::Uuid;

/// Simple file-backed artifact store for HTTP binary distribution.
///
/// Stores built binaries by `(build_id, node_id)` so daemons can
/// pull them from the coordinator via HTTP before spawning.
pub(crate) struct ArtifactStore {
    base_dir: PathBuf,
}

#[allow(dead_code)]
impl ArtifactStore {
    /// Create a new artifact store in a temp directory.
    pub fn new() -> eyre::Result<Self> {
        let base_dir = std::env::temp_dir().join("adora-artifacts");
        std::fs::create_dir_all(&base_dir)
            .with_context(|| format!("failed to create artifact dir {}", base_dir.display()))?;
        Ok(Self { base_dir })
    }

    /// Store binary data for a specific build/node pair. Returns the file path.
    pub fn store(&self, build_id: Uuid, node_id: &str, data: &[u8]) -> eyre::Result<PathBuf> {
        let dir = self.base_dir.join(build_id.to_string());
        std::fs::create_dir_all(&dir)?;
        let sanitized = sanitize_node_id(node_id)
            .ok_or_else(|| eyre!("invalid node_id for artifact storage: {node_id}"))?;
        let path = dir.join(sanitized);
        std::fs::write(&path, data)
            .with_context(|| format!("failed to write artifact to {}", path.display()))?;
        Ok(path)
    }

    /// Get the path to a stored artifact, if it exists.
    pub fn get_path(&self, build_id: &Uuid, node_id: &str) -> Option<PathBuf> {
        let sanitized = sanitize_node_id(node_id)?;
        let path = self.base_dir.join(build_id.to_string()).join(sanitized);
        if path.exists() { Some(path) } else { None }
    }

    /// Remove all artifacts for a given build.
    pub fn cleanup_build(&self, build_id: &Uuid) {
        let dir = self.base_dir.join(build_id.to_string());
        if dir.exists() {
            if let Err(e) = std::fs::remove_dir_all(&dir) {
                tracing::warn!("failed to clean up artifacts for build {build_id}: {e}");
            }
        }
    }

    /// Base directory for constructing artifact URLs.
    pub fn base_dir(&self) -> &Path {
        &self.base_dir
    }
}

/// Sanitize a node_id for safe use as a filename.
/// Returns None if the id is empty or contains path traversal sequences.
fn sanitize_node_id(node_id: &str) -> Option<String> {
    if node_id.is_empty() || node_id.contains("..") || node_id.contains('/') {
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
    fn store_and_get() {
        let store = ArtifactStore::new().unwrap();
        let build_id = Uuid::new_v4();
        let data = b"hello binary";

        let path = store.store(build_id, "my-node", data).unwrap();
        assert!(path.exists());

        let got_path = store.get_path(&build_id, "my-node").unwrap();
        assert_eq!(std::fs::read(&got_path).unwrap(), data);

        store.cleanup_build(&build_id);
        assert!(store.get_path(&build_id, "my-node").is_none());
    }

    #[test]
    fn rejects_path_traversal() {
        assert!(sanitize_node_id("../etc/passwd").is_none());
        assert!(sanitize_node_id("foo/bar").is_none());
        assert!(sanitize_node_id("").is_none());
    }

    #[test]
    fn sanitizes_special_chars() {
        assert_eq!(sanitize_node_id("my node!"), Some("my_node_".into()));
        assert_eq!(sanitize_node_id("ok-node_1"), Some("ok-node_1".into()));
    }
}
