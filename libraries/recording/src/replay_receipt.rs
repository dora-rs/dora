//! Ordered stream receipts for checking finite local replays.

use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    io::{Read, Write},
    path::PathBuf,
};

use eyre::{Context, ContextCompat};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use tempfile::NamedTempFile;
use uuid::Uuid;

/// Version of the replay receipt JSON schema.
pub const REPLAY_RECEIPT_VERSION: u16 = 1;
const MAX_RECEIPT_BYTES: usize = 1024 * 1024;

/// Identifies where a node writes or reads its receipt and which streams it tracks.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReplayReceiptConfig {
    pub session_id: Uuid,
    pub node_id: String,
    pub stream_ids: BTreeSet<String>,
    pub receipt_path: PathBuf,
}

/// Count and ordered identity digest for one stream.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StreamReceipt {
    pub count: u64,
    pub sha256: [u8; 32],
}

/// Receipt published by one replay participant.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReplayReceipt {
    pub version: u16,
    pub session_id: Uuid,
    pub node_id: String,
    pub streams: BTreeMap<String, StreamReceipt>,
}

struct StreamState {
    count: u64,
    hasher: Sha256,
}

/// Collects ordered identity digests and publishes a receipt without overwriting an existing one.
pub struct ReplayReceiptRecorder {
    config: ReplayReceiptConfig,
    streams: BTreeMap<String, StreamState>,
}

impl ReplayReceiptRecorder {
    pub fn new(config: ReplayReceiptConfig) -> eyre::Result<Self> {
        if config.node_id.is_empty() {
            eyre::bail!("receipt node identity must not be empty");
        }
        if config.stream_ids.iter().any(String::is_empty) {
            eyre::bail!("receipt stream identity must not be empty");
        }

        let streams = config
            .stream_ids
            .iter()
            .map(|stream_id| {
                (
                    stream_id.clone(),
                    StreamState {
                        count: 0,
                        hasher: Sha256::new(),
                    },
                )
            })
            .collect();
        Ok(Self { config, streams })
    }

    /// Records one caller-encoded identity. Returns `false` for an unselected stream.
    pub fn record(&mut self, stream_id: &str, identity: &[u8]) -> eyre::Result<bool> {
        let Some(stream) = self.streams.get_mut(stream_id) else {
            return Ok(false);
        };
        let identity_len =
            u64::try_from(identity.len()).wrap_err("stream identity length does not fit in u64")?;
        let next_count = stream
            .count
            .checked_add(1)
            .wrap_err_with(|| format!("receipt count overflow for stream `{stream_id}`"))?;
        stream.hasher.update(identity_len.to_le_bytes());
        stream.hasher.update(identity);
        stream.count = next_count;
        Ok(true)
    }

    /// Finalizes and atomically publishes the receipt, failing if the path already exists.
    pub fn finish(self) -> eyre::Result<ReplayReceipt> {
        let receipt = ReplayReceipt {
            version: REPLAY_RECEIPT_VERSION,
            session_id: self.config.session_id,
            node_id: self.config.node_id,
            streams: self
                .streams
                .into_iter()
                .map(|(stream_id, state)| {
                    (
                        stream_id,
                        StreamReceipt {
                            count: state.count,
                            sha256: state.hasher.finalize().into(),
                        },
                    )
                })
                .collect(),
        };

        let parent = self
            .config
            .receipt_path
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
            .unwrap_or_else(|| std::path::Path::new("."));
        let mut temporary = NamedTempFile::new_in(parent).wrap_err_with(|| {
            format!(
                "failed to create temporary receipt beside `{}`",
                self.config.receipt_path.display()
            )
        })?;
        serde_json::to_writer(temporary.as_file_mut(), &receipt)
            .wrap_err("failed to serialize replay receipt")?;
        let receipt_len = temporary
            .as_file()
            .metadata()
            .wrap_err("failed to measure replay receipt")?
            .len();
        if receipt_len + 1 > MAX_RECEIPT_BYTES as u64 {
            eyre::bail!(
                "replay receipt for node `{}` is too large: more than {MAX_RECEIPT_BYTES} bytes",
                receipt.node_id
            );
        }
        temporary
            .as_file_mut()
            .write_all(b"\n")
            .wrap_err("failed to write replay receipt")?;
        temporary
            .as_file_mut()
            .sync_all()
            .wrap_err("failed to sync replay receipt")?;
        temporary
            .persist_noclobber(&self.config.receipt_path)
            .map_err(|error| {
                eyre::eyre!(
                    "failed to publish replay receipt `{}`: {}",
                    self.config.receipt_path.display(),
                    error.error
                )
            })?;
        Ok(receipt)
    }
}

/// Reads a bounded receipt and validates its exact identity and stream set.
pub fn read_receipt(config: &ReplayReceiptConfig) -> eyre::Result<ReplayReceipt> {
    let mut file = File::open(&config.receipt_path).wrap_err_with(|| {
        format!(
            "failed to open replay receipt `{}`",
            config.receipt_path.display()
        )
    })?;
    let mut bytes = Vec::new();
    (&mut file)
        .take((MAX_RECEIPT_BYTES + 1) as u64)
        .read_to_end(&mut bytes)
        .wrap_err_with(|| {
            format!(
                "failed to read replay receipt `{}`",
                config.receipt_path.display()
            )
        })?;
    if bytes.len() > MAX_RECEIPT_BYTES {
        eyre::bail!(
            "replay receipt `{}` is too large: more than {MAX_RECEIPT_BYTES} bytes",
            config.receipt_path.display()
        );
    }

    let receipt: ReplayReceipt = serde_json::from_slice(&bytes).wrap_err_with(|| {
        format!(
            "failed to parse replay receipt `{}`",
            config.receipt_path.display()
        )
    })?;
    validate_receipt(config, &receipt)?;
    Ok(receipt)
}

fn validate_receipt(config: &ReplayReceiptConfig, receipt: &ReplayReceipt) -> eyre::Result<()> {
    if receipt.version != REPLAY_RECEIPT_VERSION {
        eyre::bail!(
            "unsupported replay receipt version {} for node `{}`; expected {REPLAY_RECEIPT_VERSION}",
            receipt.version,
            config.node_id
        );
    }
    if receipt.session_id != config.session_id {
        eyre::bail!(
            "replay receipt session mismatch for node `{}`: got {}, expected {}",
            config.node_id,
            receipt.session_id,
            config.session_id
        );
    }
    if receipt.node_id != config.node_id {
        eyre::bail!(
            "replay receipt node mismatch: got `{}`, expected `{}`",
            receipt.node_id,
            config.node_id
        );
    }
    let actual_streams: BTreeSet<_> = receipt.streams.keys().cloned().collect();
    if actual_streams != config.stream_ids {
        eyre::bail!(
            "replay receipt stream set mismatch for node `{}`: got {:?}, expected {:?}",
            config.node_id,
            actual_streams,
            config.stream_ids
        );
    }
    Ok(())
}

/// Verifies that a producer emitted exactly the recorded number of messages per stream.
pub fn verify_recording_counts(
    expected_counts: &BTreeMap<String, u64>,
    producer: &ReplayReceipt,
) -> eyre::Result<()> {
    let producer_streams: BTreeSet<_> = producer.streams.keys().cloned().collect();
    let expected_streams: BTreeSet<_> = expected_counts.keys().cloned().collect();
    if producer_streams != expected_streams {
        eyre::bail!(
            "producer receipt stream set mismatch for node `{}`: got {:?}, expected {:?}",
            producer.node_id,
            producer_streams,
            expected_streams
        );
    }
    for (stream_id, expected_count) in expected_counts {
        let actual_count = producer.streams[stream_id].count;
        if actual_count != *expected_count {
            eyre::bail!(
                "producer receipt count mismatch for node `{}` stream `{stream_id}`: got {actual_count}, expected {expected_count}",
                producer.node_id
            );
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::{collections::BTreeMap, fs, path::Path};

    use tempfile::tempdir;
    use uuid::Uuid;

    use super::{
        MAX_RECEIPT_BYTES, ReplayReceiptConfig, ReplayReceiptRecorder, read_receipt,
        verify_recording_counts,
    };

    fn config(
        path: &Path,
        session_id: Uuid,
        node_id: &str,
        streams: &[&str],
    ) -> ReplayReceiptConfig {
        ReplayReceiptConfig {
            session_id,
            node_id: node_id.to_owned(),
            stream_ids: streams.iter().map(|stream| (*stream).to_owned()).collect(),
            receipt_path: path.to_owned(),
        }
    }

    fn record(config: ReplayReceiptConfig, entries: &[(&str, &[u8])]) -> super::ReplayReceipt {
        let mut recorder = ReplayReceiptRecorder::new(config).unwrap();
        for (stream_id, identity) in entries {
            assert!(recorder.record(stream_id, identity).unwrap());
        }
        recorder.finish().unwrap()
    }

    #[test]
    fn exact_sequences_match() {
        let dir = tempdir().unwrap();
        let session_id = Uuid::from_u128(1);
        let producer = record(
            config(
                &dir.path().join("producer.json"),
                session_id,
                "camera",
                &["image"],
            ),
            &[("image", b"hlc-1"), ("image", b"hlc-2")],
        );
        let receiver = record(
            config(
                &dir.path().join("receiver.json"),
                session_id,
                "detector",
                &["frames"],
            ),
            &[("frames", b"hlc-1"), ("frames", b"hlc-2")],
        );

        assert_eq!(producer.streams["image"], receiver.streams["frames"]);
        verify_recording_counts(&BTreeMap::from([("image".to_owned(), 2)]), &producer).unwrap();
    }

    #[test]
    fn equal_count_with_duplicate_and_missing_identity_does_not_match() {
        let dir = tempdir().unwrap();
        let session_id = Uuid::from_u128(2);
        let producer = record(
            config(
                &dir.path().join("producer.json"),
                session_id,
                "producer",
                &["out"],
            ),
            &[("out", b"one"), ("out", b"two")],
        );
        let receiver = record(
            config(
                &dir.path().join("receiver.json"),
                session_id,
                "receiver",
                &["out"],
            ),
            &[("out", b"one"), ("out", b"one")],
        );

        let produced = &producer.streams["out"];
        let received = &receiver.streams["out"];
        assert_eq!(produced.count, received.count);
        assert_ne!(produced.sha256, received.sha256);
    }

    #[test]
    fn reordered_identities_do_not_match() {
        let dir = tempdir().unwrap();
        let session_id = Uuid::from_u128(3);
        let producer = record(
            config(
                &dir.path().join("producer.json"),
                session_id,
                "producer",
                &["out"],
            ),
            &[("out", b"one"), ("out", b"two")],
        );
        let receiver = record(
            config(
                &dir.path().join("receiver.json"),
                session_id,
                "receiver",
                &["out"],
            ),
            &[("out", b"two"), ("out", b"one")],
        );

        let produced = &producer.streams["out"];
        let received = &receiver.streams["out"];
        assert_eq!(produced.count, received.count);
        assert_ne!(produced.sha256, received.sha256);
    }

    #[test]
    fn missing_suffix_does_not_match() {
        let dir = tempdir().unwrap();
        let session_id = Uuid::from_u128(4);
        let producer = record(
            config(
                &dir.path().join("producer.json"),
                session_id,
                "producer",
                &["out"],
            ),
            &[("out", b"one"), ("out", b"two")],
        );
        let receiver = record(
            config(
                &dir.path().join("receiver.json"),
                session_id,
                "receiver",
                &["out"],
            ),
            &[("out", b"one")],
        );

        assert_ne!(producer.streams["out"].count, receiver.streams["out"].count);
    }

    #[test]
    fn unselected_streams_are_ignored() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("receipt.json");
        let mut recorder =
            ReplayReceiptRecorder::new(config(&path, Uuid::from_u128(5), "node", &["selected"]))
                .unwrap();

        assert!(!recorder.record("unselected", b"identity").unwrap());
        let receipt = recorder.finish().unwrap();
        assert_eq!(receipt.streams["selected"].count, 0);
    }

    #[test]
    fn read_rejects_wrong_session_node_and_stream_set() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("receipt.json");
        let session_id = Uuid::from_u128(6);
        record(config(&path, session_id, "node", &["a", "b"]), &[]);

        let wrong_session = config(&path, Uuid::from_u128(7), "node", &["a", "b"]);
        assert!(
            read_receipt(&wrong_session)
                .unwrap_err()
                .to_string()
                .contains("session")
        );

        let wrong_node = config(&path, session_id, "other", &["a", "b"]);
        assert!(
            read_receipt(&wrong_node)
                .unwrap_err()
                .to_string()
                .contains("node")
        );

        let wrong_streams = config(&path, session_id, "node", &["a"]);
        assert!(
            read_receipt(&wrong_streams)
                .unwrap_err()
                .to_string()
                .contains("stream set")
        );
    }

    #[test]
    fn read_rejects_wrong_version() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("receipt.json");
        let expected = config(&path, Uuid::from_u128(11), "node", &["out"]);
        let mut receipt = record(expected.clone(), &[]);
        receipt.version += 1;
        fs::write(&path, serde_json::to_vec(&receipt).unwrap()).unwrap();

        assert!(
            read_receipt(&expected)
                .unwrap_err()
                .to_string()
                .contains("version")
        );
    }

    #[test]
    fn read_rejects_missing_truncated_and_oversize_receipts() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("receipt.json");
        let expected = config(&path, Uuid::from_u128(8), "node", &["out"]);

        assert!(
            read_receipt(&expected)
                .unwrap_err()
                .to_string()
                .contains("open")
        );

        fs::write(&path, br#"{"version":1,"session_id":"#).unwrap();
        assert!(
            read_receipt(&expected)
                .unwrap_err()
                .to_string()
                .contains("parse")
        );

        fs::write(&path, vec![b'x'; MAX_RECEIPT_BYTES + 1]).unwrap();
        assert!(
            read_receipt(&expected)
                .unwrap_err()
                .to_string()
                .contains("too large")
        );
    }

    #[test]
    fn finish_does_not_overwrite_an_existing_receipt() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("receipt.json");
        fs::write(&path, b"existing").unwrap();
        let mut recorder =
            ReplayReceiptRecorder::new(config(&path, Uuid::from_u128(9), "node", &["out"]))
                .unwrap();
        recorder.record("out", b"identity").unwrap();

        let error = recorder.finish().unwrap_err();
        assert!(error.to_string().contains("publish"));
        assert_eq!(fs::read(path).unwrap(), b"existing");
    }

    #[test]
    fn finish_rejects_a_receipt_larger_than_the_read_limit() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("receipt.json");
        let stream_ids = (0..10_000)
            .map(|index| format!("stream-{index:05}-{}", "x".repeat(32)))
            .collect();
        let recorder = ReplayReceiptRecorder::new(ReplayReceiptConfig {
            session_id: Uuid::from_u128(12),
            node_id: "node".to_owned(),
            stream_ids,
            receipt_path: path.clone(),
        })
        .unwrap();

        let error = recorder.finish().unwrap_err();
        assert!(error.to_string().contains("too large"));
        assert!(!path.exists());
    }

    #[test]
    fn recording_count_validation_requires_exact_stream_set_and_count() {
        let dir = tempdir().unwrap();
        let receipt = record(
            config(
                &dir.path().join("receipt.json"),
                Uuid::from_u128(10),
                "producer",
                &["a", "b"],
            ),
            &[("a", b"one")],
        );

        let wrong_count = BTreeMap::from([("a".to_owned(), 2), ("b".to_owned(), 0)]);
        assert!(
            verify_recording_counts(&wrong_count, &receipt)
                .unwrap_err()
                .to_string()
                .contains("count mismatch")
        );

        let wrong_set = BTreeMap::from([("a".to_owned(), 1)]);
        assert!(
            verify_recording_counts(&wrong_set, &receipt)
                .unwrap_err()
                .to_string()
                .contains("stream set")
        );
    }
}
