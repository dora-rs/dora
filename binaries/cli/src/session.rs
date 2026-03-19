use std::{
    collections::BTreeMap,
    collections::hash_map::DefaultHasher,
    hash::{Hash, Hasher},
    path::{Path, PathBuf},
};

use dora_core::{
    build::BuildInfo,
    descriptor::{Descriptor, DescriptorExt},
};
use dora_message::{BuildId, SessionId, common::GitSource, id::NodeId};
use eyre::{Context, ContextCompat};

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DataflowSession {
    pub build_id: Option<BuildId>,
    pub session_id: SessionId,
    pub git_sources: BTreeMap<NodeId, GitSource>,
    pub local_build: Option<BuildInfo>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub descriptor_fingerprint: Option<String>,
}

impl Default for DataflowSession {
    fn default() -> Self {
        Self {
            build_id: None,
            session_id: SessionId::generate(),
            git_sources: Default::default(),
            local_build: Default::default(),
            descriptor_fingerprint: None,
        }
    }
}

impl DataflowSession {
    pub fn read_session(dataflow_path: &Path) -> eyre::Result<Self> {
        let session_file = session_file_path(dataflow_path)?;
        if session_file.exists() {
            if let Ok(parsed) = deserialize(&session_file) {
                return Ok(parsed);
            } else {
                tracing::warn!(
                    "failed to read dataflow session file, regenerating (you might need to run `dora build` again)"
                );
            }
        }

        let default_session = DataflowSession::default();
        default_session.write_out_for_dataflow(dataflow_path)?;
        Ok(default_session)
    }

    pub fn read_and_sync_for_dataflow(
        dataflow_path: &Path,
        descriptor: &Descriptor,
    ) -> eyre::Result<Self> {
        let mut session = Self::read_session(dataflow_path)?;
        if session.sync_with_dataflow(descriptor)? {
            session
                .write_out_for_dataflow(dataflow_path)
                .context("failed to update DataflowSession after fingerprint mismatch")?;
        }
        Ok(session)
    }

    /// Syncs the stored fingerprint with the current descriptor.
    ///
    /// Returns `true` when the descriptor changed and stale build state was invalidated.
    pub fn sync_with_dataflow(&mut self, descriptor: &Descriptor) -> eyre::Result<bool> {
        let fingerprint = descriptor_fingerprint(descriptor)?;
        if self.descriptor_fingerprint.as_deref() == Some(&fingerprint) {
            return Ok(false);
        }

        let had_stale_build = self.build_id.is_some() || self.local_build.is_some();
        if had_stale_build {
            tracing::warn!(
                "dataflow descriptor changed since last build; clearing stale build session state"
            );
        }

        self.build_id = None;
        self.local_build = None;
        self.git_sources.clear();
        self.descriptor_fingerprint = Some(fingerprint);
        Ok(true)
    }

    pub fn write_out_for_dataflow(&self, dataflow_path: &Path) -> eyre::Result<()> {
        let session_file = session_file_path(dataflow_path)?;
        let filename = session_file
            .file_name()
            .context("session file has no file name")?
            .to_str()
            .context("session file name is no utf8")?;
        if let Some(parent) = session_file.parent() {
            std::fs::create_dir_all(parent).context("failed to create out dir")?;
        }
        std::fs::write(&session_file, self.serialize()?)
            .context("failed to write dataflow session file")?;
        let gitignore = session_file.with_file_name(".gitignore");
        if gitignore.exists() {
            let existing =
                std::fs::read_to_string(&gitignore).context("failed to read gitignore")?;
            if !existing
                .lines()
                .any(|l| l.split_once('/') == Some(("", filename)))
            {
                let new = existing + &format!("\n/{filename}\n");
                std::fs::write(gitignore, new).context("failed to update gitignore")?;
            }
        } else {
            std::fs::write(gitignore, format!("/{filename}\n"))
                .context("failed to write gitignore")?;
        }
        Ok(())
    }

    fn serialize(&self) -> eyre::Result<String> {
        serde_yaml::to_string(&self).context("failed to serialize dataflow session file")
    }
}

fn deserialize(session_file: &Path) -> eyre::Result<DataflowSession> {
    std::fs::read_to_string(session_file)
        .context("failed to read DataflowSession file")
        .and_then(|s| {
            serde_yaml::from_str(&s).context("failed to deserialize DataflowSession file")
        })
}

fn descriptor_fingerprint(descriptor: &Descriptor) -> eyre::Result<String> {
    let resolved_nodes = descriptor
        .resolve_aliases_and_set_defaults()
        .context("failed to resolve nodes for dataflow fingerprinting")?;
    let serialized =
        serde_json::to_vec(&resolved_nodes).context("failed to serialize resolved nodes")?;
    let mut hasher = DefaultHasher::new();
    serialized.hash(&mut hasher);
    Ok(format!("{:016x}", hasher.finish()))
}

fn session_file_path(dataflow_path: &Path) -> eyre::Result<PathBuf> {
    let file_stem = dataflow_path
        .file_stem()
        .wrap_err("dataflow path has no file stem")?
        .to_str()
        .wrap_err("dataflow file stem is not valid utf-8")?;
    let session_file = dataflow_path
        .with_file_name("out")
        .join(format!("{file_stem}.dora-session.yaml"));
    Ok(session_file)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn descriptor(path: &str) -> Descriptor {
        serde_yaml::from_str::<Descriptor>(&format!("nodes:\n  - id: n1\n    path: {path}\n"))
            .unwrap()
    }

    #[test]
    fn sync_clears_stale_build_state_when_descriptor_changes() {
        let mut session = DataflowSession {
            build_id: Some(BuildId::generate()),
            local_build: Some(BuildInfo {
                node_working_dirs: BTreeMap::new(),
            }),
            git_sources: BTreeMap::from([(
                NodeId::from("n1".to_owned()),
                GitSource {
                    repo: "https://example.com/repo.git".to_owned(),
                    commit_hash: "deadbeef".to_owned(),
                },
            )]),
            descriptor_fingerprint: Some("old".to_owned()),
            ..DataflowSession::default()
        };

        let changed = session.sync_with_dataflow(&descriptor("./node_a")).unwrap();
        assert!(changed);
        assert!(session.build_id.is_none());
        assert!(session.local_build.is_none());
        assert!(session.git_sources.is_empty());
        assert!(session.descriptor_fingerprint.is_some());
    }

    #[test]
    fn sync_keeps_build_state_when_descriptor_is_unchanged() {
        let mut session = DataflowSession::default();
        let descriptor = descriptor("./node_a");
        session.sync_with_dataflow(&descriptor).unwrap();

        session.build_id = Some(BuildId::generate());
        session.local_build = Some(BuildInfo {
            node_working_dirs: BTreeMap::new(),
        });
        session.git_sources.insert(
            NodeId::from("n1".to_owned()),
            GitSource {
                repo: "https://example.com/repo.git".to_owned(),
                commit_hash: "deadbeef".to_owned(),
            },
        );

        let changed = session.sync_with_dataflow(&descriptor).unwrap();
        assert!(!changed);
        assert!(session.build_id.is_some());
        assert!(session.local_build.is_some());
        assert!(!session.git_sources.is_empty());
    }
}
