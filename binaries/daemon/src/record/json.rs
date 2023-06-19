use std::path::Path;

use dora_core::message::uhlc::Timestamp;
use eyre::Context;
use tokio::{fs::File, io::AsyncWriteExt};

use crate::Event;

pub struct JsonFile {
    file: File,
}

impl JsonFile {
    pub async fn new(path: &Path) -> eyre::Result<Self> {
        let file = tokio::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(&path)
            .await
            .wrap_err_with(|| format!("failed to open record file at {}", path.display()))?;
        Ok(Self { file })
    }

    pub async fn record(&mut self, timestamp: Timestamp, event: &Event) -> eyre::Result<()> {
        let json = format(timestamp, event)?;
        self.file
            .write_all(json.as_bytes())
            .await
            .context("failed to write event to record file")?;
        Ok(())
    }
}

fn format(
    timestamp: dora_core::message::uhlc::Timestamp,
    event: &crate::Event,
) -> eyre::Result<String> {
    let entry = RecordEntry { timestamp, event };
    serde_json::to_string(&entry).context("failed to serialize record entry")
}

#[derive(Debug, serde::Serialize)]
struct RecordEntry<'a> {
    timestamp: Timestamp,
    event: &'a Event,
}
