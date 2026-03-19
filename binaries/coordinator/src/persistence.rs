use crate::ArchivedDataflow;
use dora_message::{DataflowId, common::DaemonId, daemon_to_coordinator::DataflowDaemonResult};
use eyre::Context;
use std::path::{Path, PathBuf};

const COORDINATOR_STATE_VERSION: u32 = 1;

#[derive(Debug, Clone)]
pub struct CoordinatorPersistence {
    path: PathBuf,
}

impl CoordinatorPersistence {
    pub fn new(path: PathBuf) -> Self {
        Self { path }
    }

    pub async fn load(&self) -> eyre::Result<PersistedCoordinatorState> {
        if !self.path.exists() {
            return Ok(PersistedCoordinatorState::default());
        }

        let raw = tokio::fs::read_to_string(&self.path)
            .await
            .with_context(|| {
                format!("failed to read coordinator state `{}`", self.path.display())
            })?;
        let persisted: PersistedCoordinatorState =
            serde_json::from_str(&raw).with_context(|| {
                format!(
                    "failed to parse coordinator state file `{}`",
                    self.path.display()
                )
            })?;

        if persisted.version != COORDINATOR_STATE_VERSION {
            eyre::bail!(
                "unsupported coordinator state version {} in `{}` (expected {})",
                persisted.version,
                self.path.display(),
                COORDINATOR_STATE_VERSION
            );
        }

        Ok(persisted)
    }

    pub async fn save(&self, state: &PersistedCoordinatorState) -> eyre::Result<()> {
        if let Some(parent) = self.path.parent() {
            tokio::fs::create_dir_all(parent).await.with_context(|| {
                format!(
                    "failed to create coordinator state directory `{}`",
                    parent.display()
                )
            })?;
        }

        let serialized =
            serde_json::to_vec_pretty(state).context("failed to serialize coordinator state")?;

        let tmp_path = tmp_path(&self.path)?;
        tokio::fs::write(&tmp_path, serialized)
            .await
            .with_context(|| format!("failed to write temp file `{}`", tmp_path.display()))?;
        tokio::fs::rename(&tmp_path, &self.path)
            .await
            .with_context(|| {
                format!(
                    "failed to atomically replace coordinator state file `{}`",
                    self.path.display()
                )
            })?;
        Ok(())
    }

    pub fn path(&self) -> &PathBuf {
        &self.path
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PersistedCoordinatorState {
    pub version: u32,
    #[serde(default)]
    pub archived_dataflows: Vec<PersistedArchivedDataflow>,
    #[serde(default)]
    pub dataflow_results: Vec<PersistedDataflowResult>,
}

impl Default for PersistedCoordinatorState {
    fn default() -> Self {
        Self {
            version: COORDINATOR_STATE_VERSION,
            archived_dataflows: Vec::new(),
            dataflow_results: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PersistedArchivedDataflow {
    pub dataflow_id: DataflowId,
    pub dataflow: ArchivedDataflow,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PersistedDataflowResult {
    pub dataflow_id: DataflowId,
    #[serde(default)]
    pub daemon_results: Vec<PersistedDaemonResult>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PersistedDaemonResult {
    pub daemon_id: DaemonId,
    pub result: DataflowDaemonResult,
}

fn tmp_path(path: &Path) -> eyre::Result<PathBuf> {
    let file_name = path
        .file_name()
        .ok_or_else(|| eyre::eyre!("coordinator state path has no file name"))?
        .to_str()
        .ok_or_else(|| eyre::eyre!("coordinator state file name is not valid utf-8"))?;
    Ok(path.with_file_name(format!("{file_name}.tmp")))
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_message::id::NodeId;
    use uuid::Uuid;

    #[tokio::test]
    async fn save_and_load_roundtrip() {
        let dir = std::env::temp_dir().join(format!("dora-coordinator-state-{}", Uuid::new_v4()));
        std::fs::create_dir_all(&dir).unwrap();
        let state_path = dir.join("state.json");
        let persistence = CoordinatorPersistence::new(state_path);

        let dataflow_id = Uuid::new_v4();
        let daemon_id = DaemonId::new(Some("daemon-a".to_owned()));
        let snapshot = PersistedCoordinatorState {
            dataflow_results: vec![PersistedDataflowResult {
                dataflow_id,
                daemon_results: vec![PersistedDaemonResult {
                    daemon_id,
                    result: DataflowDaemonResult {
                        timestamp: dora_core::uhlc::HLC::default().new_timestamp(),
                        node_results: std::collections::BTreeMap::from([(
                            NodeId::from("n1".to_owned()),
                            Ok(()),
                        )]),
                    },
                }],
            }],
            ..PersistedCoordinatorState::default()
        };

        persistence.save(&snapshot).await.unwrap();
        let loaded = persistence.load().await.unwrap();
        assert_eq!(loaded.dataflow_results.len(), 1);
        assert!(loaded.archived_dataflows.is_empty());

        let _ = std::fs::remove_dir_all(dir);
    }
}
