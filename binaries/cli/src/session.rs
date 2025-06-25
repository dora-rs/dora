use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

use dora_core::build::BuildInfo;
use dora_message::{common::GitSource, id::NodeId, BuildId, SessionId};
use eyre::{Context, ContextCompat};

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DataflowSession {
    pub build_id: Option<BuildId>,
    pub session_id: SessionId,
    pub git_sources: BTreeMap<NodeId, GitSource>,
    pub local_build: Option<BuildInfo>,
}

impl Default for DataflowSession {
    fn default() -> Self {
        Self {
            build_id: None,
            session_id: SessionId::generate(),
            git_sources: Default::default(),
            local_build: Default::default(),
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
                tracing::warn!("failed to read dataflow session file, regenerating (you might need to run `dora build` again)");
            }
        }

        let default_session = DataflowSession::default();
        default_session.write_out_for_dataflow(dataflow_path)?;
        Ok(default_session)
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
