use std::path::{Path, PathBuf};

use dora_core::{daemon_messages::DataflowId, message::uhlc::Timestamp};
use eyre::Context;

use self::{json::JsonFile, rosbag::RosbagFile};

mod json;
mod rosbag;

pub struct Recorder {
    json_file: JsonFile,
    rosbag_file: RosbagFile,
}

impl Recorder {
    pub async fn new(
        working_dir: PathBuf,
        machine_id: String,
        dataflow_id: DataflowId,
    ) -> eyre::Result<Self> {
        let record_folder = Self::record_folder(&working_dir, dataflow_id).await?;

        let json_file_path = record_folder.join(format!("events-{}.json", machine_id));
        let json_file = JsonFile::new(&json_file_path).await?;

        let rosbag_file_path = record_folder.join(format!("events-{}.bag", machine_id));
        let rosbag_file = RosbagFile::new(&rosbag_file_path).await?;

        Ok(Self {
            json_file,
            rosbag_file,
        })
    }

    pub async fn record(&mut self, event: &crate::Event, timestamp: Timestamp) -> eyre::Result<()> {
        self.json_file.record(timestamp, event).await?;
        self.rosbag_file.record(timestamp, event).await?;

        Ok(())
    }

    pub async fn finish(self) -> eyre::Result<()> {
        self.rosbag_file.finish().await?;
        Ok(())
    }

    async fn record_folder(
        working_dir: &Path,
        dataflow_id: DataflowId,
    ) -> Result<PathBuf, eyre::ErrReport> {
        let record_folder = working_dir.join("record").join(dataflow_id.to_string());
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
