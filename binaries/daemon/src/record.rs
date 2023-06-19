use std::path::PathBuf;

use dora_core::{daemon_messages::DataflowId, message::uhlc::Timestamp};
use eyre::Context;
use tokio::io::AsyncWriteExt;

use crate::Event;

pub struct Recorder {
    working_dir: PathBuf,
    machine_id: String,
    dataflow_id: DataflowId,
}

impl Recorder {
    pub fn new(working_dir: PathBuf, machine_id: String, dataflow_id: DataflowId) -> Self {
        Self {
            working_dir,
            machine_id,
            dataflow_id,
        }
    }

    pub async fn record(&mut self, event: &crate::Event, timestamp: Timestamp) -> eyre::Result<()> {
        let entry = RecordEntry { timestamp, event };
        let rendered = serde_json::to_string(&entry).context("failed to serialize record entry")?;

        let record_folder = self.record_folder().await?;
        let record_file_path = record_folder.join(format!("events-{}.json", self.machine_id));
        let mut record_file = tokio::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(&record_file_path)
            .await
            .wrap_err_with(|| {
                format!(
                    "failed to open record file at {}",
                    record_file_path.display()
                )
            })?;
        record_file
            .write_all(rendered.as_bytes())
            .await
            .context("failed to write event to record file")?;
        record_file
            .write_all("\n".as_bytes())
            .await
            .context("failed to write newline to record file")?;

        Ok(())
    }

    async fn record_folder(&mut self) -> Result<PathBuf, eyre::ErrReport> {
        let record_folder = self
            .working_dir
            .join("record")
            .join(self.dataflow_id.to_string());
        tokio::fs::create_dir_all(&record_folder)
            .await
            .wrap_err_with(|| {
                format!(
                    "failed to create record folder at {}",
                    record_folder.display()
                )
            })?;
        Ok(record_folder)
    }
}

#[derive(Debug, serde::Serialize)]
struct RecordEntry<'a> {
    timestamp: Timestamp,
    event: &'a Event,
}
