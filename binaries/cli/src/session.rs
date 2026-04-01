use std::{
    collections::BTreeMap,
    fs::OpenOptions,
    path::{Path, PathBuf},
};

use dora_core::build::BuildInfo;
use dora_message::{BuildId, SessionId, common::GitSource, id::NodeId};
use eyre::{Context, ContextCompat};
use fs2::FileExt;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DataflowSession {
    #[serde(default)]
    pub build_id: Option<BuildId>,
    pub session_id: SessionId,
    #[serde(default)]
    pub git_sources: BTreeMap<NodeId, GitSource>,
    #[serde(default)]
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
        with_session_file_lock(&session_file, || {
            if session_file.exists() {
                match deserialize(&session_file) {
                    Ok(parsed) => return Ok(parsed),
                    Err(err) => {
                        tracing::warn!(
                            "failed to parse dataflow session at `{}` ({err:#}); regenerating",
                            session_file.display()
                        );
                    }
                }
            }

            let default_session = DataflowSession::default();
            default_session.write_out_for_session_file(&session_file)?;
            Ok(default_session)
        })
    }

    pub fn write_out_for_dataflow(&self, dataflow_path: &Path) -> eyre::Result<()> {
        let session_file = session_file_path(dataflow_path)?;
        with_session_file_lock(&session_file, || {
            self.write_out_for_session_file(&session_file)
        })
    }

    fn write_out_for_session_file(&self, session_file: &Path) -> eyre::Result<()> {
        let filename = session_file
            .file_name()
            .context("session file has no file name")?
            .to_str()
            .context("session file name is no utf8")?;
        if let Some(parent) = session_file.parent() {
            std::fs::create_dir_all(parent).context("failed to create out dir")?;
        }
        std::fs::write(session_file, self.serialize()?)
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

fn with_session_file_lock<T>(
    session_file: &Path,
    f: impl FnOnce() -> eyre::Result<T>,
) -> eyre::Result<T> {
    let lock_file = session_lock_file_path(session_file)?;
    if let Some(parent) = lock_file.parent() {
        std::fs::create_dir_all(parent).context("failed to create out dir")?;
    }

    let lock = OpenOptions::new()
        .create(true)
        .read(true)
        .write(true)
        .open(&lock_file)
        .with_context(|| format!("failed to open session lock file `{}`", lock_file.display()))?;

    lock.lock_exclusive()
        .with_context(|| format!("failed to lock session lock file `{}`", lock_file.display()))?;

    let result = f();
    lock.unlock().with_context(|| {
        format!(
            "failed to unlock session lock file `{}`",
            lock_file.display()
        )
    })?;
    result
}

fn session_lock_file_path(session_file: &Path) -> eyre::Result<PathBuf> {
    let file_name = session_file
        .file_name()
        .wrap_err("session path has no file name")?
        .to_str()
        .wrap_err("session file name is not valid utf-8")?;
    Ok(session_file.with_file_name(format!("{file_name}.lock")))
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
    use super::DataflowSession;
    use std::{
        fs,
        path::PathBuf,
        time::{SystemTime, UNIX_EPOCH},
    };

    fn test_root() -> PathBuf {
        std::env::temp_dir().join(format!(
            "dora-session-test-{}-{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ))
    }

    fn test_dataflow_path(root: &std::path::Path) -> PathBuf {
        let dataflow = root.join("dataflow.yml");
        fs::create_dir_all(root).expect("failed to create test dir");
        fs::write(&dataflow, "nodes: []\n").expect("failed to write test dataflow");
        dataflow
    }

    fn session_file_for(dataflow_path: &std::path::Path) -> PathBuf {
        let stem = dataflow_path
            .file_stem()
            .expect("dataflow path should have file stem")
            .to_string_lossy();
        dataflow_path
            .with_file_name("out")
            .join(format!("{stem}.dora-session.yaml"))
    }

    #[test]
    fn read_session_regenerates_on_invalid_yaml() {
        let root = test_root();
        let dataflow_path = test_dataflow_path(&root);
        let session_file = session_file_for(&dataflow_path);
        fs::create_dir_all(
            session_file
                .parent()
                .expect("session file should have parent"),
        )
        .expect("failed to create out dir");
        fs::write(&session_file, "session_id: [\n").expect("failed to write invalid session file");

        let result = DataflowSession::read_session(&dataflow_path)
            .expect("invalid yaml should be replaced with a default session");

        assert!(result.build_id.is_none());
        assert!(result.git_sources.is_empty());
        assert!(result.local_build.is_none());

        let after = fs::read_to_string(&session_file).expect("failed to read session file");
        assert!(
            after.contains("session_id:"),
            "session file should be regenerated"
        );
        assert_ne!(
            after, "session_id: [\n",
            "invalid session file must be replaced"
        );
    }

    #[test]
    fn read_session_regenerates_on_truncated_yaml() {
        let root = test_root();
        let dataflow_path = test_dataflow_path(&root);
        let session_file = session_file_for(&dataflow_path);
        fs::create_dir_all(
            session_file
                .parent()
                .expect("session file should have parent"),
        )
        .expect("failed to create out dir");
        fs::write(
            &session_file,
            "build_id: null\nsession_id: 0195f7e0-3f4a-7e22-b13f-41f0327de0f8\ngit_sources: [\n",
        )
        .expect("failed to write truncated session file");

        let result = DataflowSession::read_session(&dataflow_path)
            .expect("truncated yaml should be replaced with a default session");

        assert!(result.build_id.is_none());
        assert!(result.git_sources.is_empty());
        assert!(result.local_build.is_none());

        let after = fs::read_to_string(&session_file).expect("failed to read session file");
        assert!(
            after.contains("session_id:"),
            "session file should be regenerated"
        );
        assert!(
            !after.contains("git_sources: ["),
            "broken/truncated session content should be replaced"
        );
    }

    #[test]
    fn read_session_accepts_missing_optional_fields() {
        let root = test_root();
        let dataflow_path = test_dataflow_path(&root);
        let session_file = session_file_for(&dataflow_path);
        fs::create_dir_all(
            session_file
                .parent()
                .expect("session file should have parent"),
        )
        .expect("failed to create out dir");
        fs::write(
            &session_file,
            "session_id: 0195f7e0-3f4a-7e22-b13f-41f0327de0f8\n",
        )
        .expect("failed to write legacy session file");

        let parsed = DataflowSession::read_session(&dataflow_path)
            .expect("legacy session with missing optional fields should still parse");

        assert!(parsed.build_id.is_none());
        assert!(parsed.git_sources.is_empty());
        assert!(parsed.local_build.is_none());
    }
}
