use std::{
    collections::BTreeMap,
    io::Write,
    path::{Path, PathBuf},
    time::{SystemTime, UNIX_EPOCH},
};

use dora_core::build::BuildInfo;
use dora_message::{BuildId, SessionId, common::GitSource, id::NodeId};
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
            return deserialize(&session_file).wrap_err_with(|| {
                format!(
                    "failed to parse existing dataflow session at `{}`; \
remove or fix the file and run `dora build` to regenerate it",
                    session_file.display()
                )
            });
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
        write_atomic(&session_file, self.serialize()?.as_bytes())
            .context("failed to write dataflow session file atomically")?;
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

fn write_atomic(path: &Path, bytes: &[u8]) -> eyre::Result<()> {
    let parent = path
        .parent()
        .context("output path has no parent directory")?;
    let filename = path
        .file_name()
        .context("output path has no file name")?
        .to_string_lossy();
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    let tmp_path = parent.join(format!(".{filename}.{nanos}.tmp"));

    let write_result = (|| -> eyre::Result<()> {
        let mut tmp_file = std::fs::OpenOptions::new()
            .create_new(true)
            .write(true)
            .open(&tmp_path)
            .with_context(|| format!("failed to create temporary file `{}`", tmp_path.display()))?;
        tmp_file
            .write_all(bytes)
            .with_context(|| format!("failed to write temporary file `{}`", tmp_path.display()))?;
        tmp_file
            .sync_all()
            .with_context(|| format!("failed to sync temporary file `{}`", tmp_path.display()))?;
        drop(tmp_file);

        if let Err(err) = std::fs::rename(&tmp_path, path) {
            if err.kind() == std::io::ErrorKind::AlreadyExists {
                std::fs::remove_file(path).with_context(|| {
                    format!(
                        "failed to replace existing output file `{}`",
                        path.display()
                    )
                })?;
                std::fs::rename(&tmp_path, path).with_context(|| {
                    format!(
                        "failed to move temporary file `{}` to `{}`",
                        tmp_path.display(),
                        path.display()
                    )
                })?;
            } else {
                return Err(err).with_context(|| {
                    format!(
                        "failed to move temporary file `{}` to `{}`",
                        tmp_path.display(),
                        path.display()
                    )
                });
            }
        }

        Ok(())
    })();

    if write_result.is_err() {
        let _ = std::fs::remove_file(&tmp_path);
    }
    write_result
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
    fn read_session_errors_on_invalid_yaml() {
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

        let result = DataflowSession::read_session(&dataflow_path);

        let err = result.expect_err("invalid yaml must fail instead of regenerating");
        let msg = format!("{err:#}");
        assert!(
            msg.contains("failed to parse existing dataflow session"),
            "error should explain parse failure, got: {msg}"
        );
        assert!(
            msg.contains("run `dora build`"),
            "error should contain recovery guidance, got: {msg}"
        );

        let after = fs::read_to_string(&session_file).expect("failed to read session file");
        assert_eq!(
            after, "session_id: [\n",
            "invalid session file should not be silently overwritten"
        );
    }

    #[test]
    fn read_session_errors_on_truncated_yaml() {
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
            "build_id: null\nsession_id: 0195f7e0-3f4a-7e22-b13f-41f0327de0f8\n",
        )
        .expect("failed to write truncated session file");

        let result = DataflowSession::read_session(&dataflow_path);

        let err = result.expect_err("truncated yaml must fail instead of regenerating");
        let msg = format!("{err:#}");
        assert!(
            msg.contains("failed to parse existing dataflow session"),
            "error should explain parse failure, got: {msg}"
        );

        let after = fs::read_to_string(&session_file).expect("failed to read session file");
        assert!(
            after.contains("session_id"),
            "truncated session file should stay unchanged"
        );
    }
}
