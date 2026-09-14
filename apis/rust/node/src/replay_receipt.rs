use dora_core::uhlc::Timestamp;
use dora_recording::replay_receipt::{ReplayReceiptConfig, ReplayReceiptRecorder};

pub(crate) const OUTPUT_RECEIPT_ENV: &str = "DORA_REPLAY_OUTPUT_RECEIPT";
pub(crate) const INPUT_RECEIPT_ENV: &str = "DORA_REPLAY_INPUT_RECEIPT";

struct ActiveReceipt {
    node_id: String,
    recorder: ReplayReceiptRecorder,
}

pub(crate) struct ReceiptRecorder {
    active: Option<ActiveReceipt>,
    configured: bool,
}

impl ReceiptRecorder {
    pub(crate) fn from_env(env_key: &str, node_id: &str) -> eyre::Result<Self> {
        match std::env::var(env_key) {
            Ok(value) => Self::from_json(Some(&value), node_id),
            Err(std::env::VarError::NotPresent) => Ok(Self {
                active: None,
                configured: false,
            }),
            Err(std::env::VarError::NotUnicode(_)) => {
                eyre::bail!("{env_key} is not valid UTF-8")
            }
        }
    }

    fn from_json(value: Option<&str>, node_id: &str) -> eyre::Result<Self> {
        let Some(value) = value else {
            return Ok(Self {
                active: None,
                configured: false,
            });
        };
        let config: ReplayReceiptConfig = serde_json::from_str(value)
            .map_err(|error| eyre::eyre!("invalid replay receipt config: {error}"))?;
        Self::from_config(config, node_id)
    }

    fn from_config(config: ReplayReceiptConfig, node_id: &str) -> eyre::Result<Self> {
        if config.node_id != node_id {
            eyre::bail!(
                "replay receipt node mismatch: config names `{}`, process is `{node_id}`",
                config.node_id
            );
        }
        let node_id = config.node_id.clone();
        let recorder = ReplayReceiptRecorder::new(config)?;
        Ok(Self {
            active: Some(ActiveReceipt { node_id, recorder }),
            configured: true,
        })
    }

    pub(crate) fn is_configured(&self) -> bool {
        self.configured
    }

    pub(crate) fn record(&mut self, stream_id: &str, timestamp: Timestamp) {
        let Some(active) = &mut self.active else {
            return;
        };
        let identity = timestamp_identity(timestamp);
        if let Err(error) = active.recorder.record(stream_id, &identity) {
            let node_id = active.node_id.clone();
            self.active = None;
            tracing::error!(
                "replay receipt disabled for node `{node_id}` after record failure on stream `{stream_id}`: {error:?}"
            );
        }
    }

    pub(crate) fn finish(&mut self) {
        let Some(active) = self.active.take() else {
            return;
        };
        if let Err(error) = active.recorder.finish() {
            tracing::error!(
                "failed to publish replay receipt for node `{}`: {error:?}",
                active.node_id
            );
        }
    }

    #[cfg(test)]
    pub(crate) fn from_config_for_testing(
        config: ReplayReceiptConfig,
        node_id: &str,
    ) -> eyre::Result<Self> {
        Self::from_config(config, node_id)
    }
}

fn timestamp_identity(timestamp: Timestamp) -> [u8; 24] {
    let mut identity = [0; 24];
    identity[..8].copy_from_slice(&timestamp.get_time().as_u64().to_le_bytes());
    identity[8..].copy_from_slice(&timestamp.get_id().to_le_bytes());
    identity
}

#[cfg(test)]
mod tests {
    use std::{collections::BTreeSet, path::PathBuf};

    use dora_core::uhlc::{ID, NTP64, Timestamp};
    use dora_recording::replay_receipt::ReplayReceiptConfig;
    use uuid::Uuid;

    use super::{ReceiptRecorder, timestamp_identity};

    #[test]
    fn absent_config_disables_receipt() {
        let recorder = ReceiptRecorder::from_json(None, "node").unwrap();
        assert!(recorder.active.is_none());
    }

    #[test]
    fn config_requires_the_actual_node_identity() {
        let config = ReplayReceiptConfig {
            session_id: Uuid::nil(),
            node_id: "other".to_owned(),
            stream_ids: BTreeSet::from(["out".to_owned()]),
            receipt_path: PathBuf::from("receipt.json"),
        };
        let value = serde_json::to_string(&config).unwrap();

        let error = ReceiptRecorder::from_json(Some(&value), "node")
            .err()
            .unwrap();
        assert!(error.to_string().contains("node mismatch"));
    }

    #[test]
    fn timestamp_identity_uses_fixed_raw_time_and_id_bytes() {
        let id = ID::try_from(&[1, 2, 3][..]).unwrap();
        let timestamp = Timestamp::new(NTP64(0x0807_0605_0403_0201), id);

        let identity = timestamp_identity(timestamp);
        assert_eq!(&identity[..8], &[1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(&identity[8..11], &[1, 2, 3]);
        assert_eq!(&identity[11..], &[0; 13]);

        assert_ne!(
            identity,
            timestamp_identity(Timestamp::new(
                NTP64(0x0807_0605_0403_0201),
                ID::try_from(&[4][..]).unwrap(),
            ))
        );
        assert_ne!(
            identity,
            timestamp_identity(Timestamp::new(NTP64(0x0807_0605_0403_0202), id,))
        );
    }
}
