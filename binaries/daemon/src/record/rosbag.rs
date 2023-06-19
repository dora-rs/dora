use std::path::Path;

use dora_core::message::uhlc::Timestamp;
use eyre::Context;
use tokio::{
    fs::File,
    io::{AsyncWrite, AsyncWriteExt},
};

use crate::Event;

pub struct RosbagFile {
    file: File,
    record: Record,
}

impl RosbagFile {
    pub async fn new(path: &Path) -> eyre::Result<Self> {
        let mut file = tokio::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(&path)
            .await
            .wrap_err_with(|| format!("failed to open record file at {}", path.display()))?;
        file.write_all("#ROSBAG V2.0\n".as_bytes())
            .await
            .context("failed to write rosbag header")?;
        Ok(Self {
            file,
            record: Record {
                header: Vec::new(),
                data: Vec::new(),
            },
        })
    }

    pub async fn record(&mut self, timestamp: Timestamp, event: &Event) -> eyre::Result<()> {
        tracing::warn!("rosbag recording is not implemented yet");
        Ok(())
    }

    pub async fn finish(mut self) -> eyre::Result<()> {
        self.record.serialize(&mut self.file).await
    }
}

struct Record {
    header: Vec<HeaderField>,
    data: Vec<u8>,
}

impl Record {
    async fn serialize(&self, writer: &mut (impl AsyncWrite + Unpin)) -> eyre::Result<()> {
        let serialized_header = {
            let mut buf = Vec::new();
            for field in &self.header {
                field.serialize(&mut buf).await?;
            }
            buf
        };

        writer
            .write_all(&u32::try_from(serialized_header.len())?.to_le_bytes())
            .await?;
        writer.write_all(&serialized_header).await?;
        writer
            .write_all(&u32::try_from(self.data.len())?.to_le_bytes())
            .await?;
        writer.write_all(&self.data).await?;

        Ok(())
    }
}

struct HeaderField {
    name: String,
    value: Vec<u8>,
}

impl HeaderField {
    async fn serialize(&self, writer: &mut (impl AsyncWrite + Unpin)) -> eyre::Result<()> {
        let len = self.name.len() + self.value.len() + 5;
        writer.write_all(&u32::try_from(len)?.to_le_bytes()).await?;
        writer.write_all(self.name.as_bytes()).await?;
        writer.write_all(&[b'=']).await?;
        writer.write_all(&self.value).await?;

        Ok(())
    }
}
