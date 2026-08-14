//! Adapter between this transport and dora's generic extension channel.
//!
//! dora brokers the lifetime of an opaque byte value and nothing else (see
//! `docs/extensions.md`). The pool's descriptor — shape, dtype, segment name,
//! whether the source was pinned, whether a CUDA IPC handle is present — is
//! that value, serialized here and never interpreted by dora.
//!
//! Before the extension channel existed, this rode on three pool-specific
//! daemon requests. Keeping the descriptor opaque is what lets the transport
//! change its own metadata without touching dora's wire protocol.

use dora_node_api::{DoraNode, Metadata, MetadataParameters};

/// Namespace for every key this transport stores. Scoped so a second
/// extension in the same process cannot see or drop our entries.
pub const NAMESPACE: &str = "dora-tensor-pool";

/// Serialize a descriptor for the wire.
///
/// JSON rather than a compact binary encoding: descriptors are a handful per
/// pool registration, not per frame, so legibility in a daemon dump is worth
/// more than the bytes.
pub fn encode(params: &MetadataParameters) -> Result<Vec<u8>, String> {
    serde_json::to_vec(params).map_err(|e| format!("failed to encode pool descriptor: {e}"))
}

/// Parse a descriptor back. An error here means the value was written by a
/// different version of this extension, not by dora.
pub fn decode(bytes: &[u8]) -> Result<MetadataParameters, String> {
    serde_json::from_slice(bytes).map_err(|e| format!("failed to decode pool descriptor: {e}"))
}

/// Publish a pool descriptor.
pub fn store(node: &mut DoraNode, buffer_id: &str, meta: &Metadata) -> Result<(), String> {
    let bytes = encode(&meta.parameters)?;
    node.extension_store(NAMESPACE, buffer_id, bytes)
        .map_err(|e| format!("{e:#}"))
}

/// Fetch a pool descriptor, or `None` if the pool is gone.
///
/// `take` drops the entry in the same round trip, which is the read-with-free
/// path: one request rather than a read followed by a racing drop.
pub fn load(
    node: &mut DoraNode,
    buffer_id: &str,
    take: bool,
) -> Result<Option<MetadataParameters>, String> {
    let bytes = node
        .extension_load(NAMESPACE, buffer_id, take)
        .map_err(|e| format!("{e:#}"))?;
    bytes.as_deref().map(decode).transpose()
}

/// Withdraw a pool descriptor, notifying every node that touched it.
pub fn drop_key(node: &mut DoraNode, buffer_id: &str) -> Result<(), String> {
    node.extension_drop(NAMESPACE, buffer_id)
        .map_err(|e| format!("{e:#}"))
}

/// Fetch a descriptor as [`Metadata`], the shape the transport's read paths
/// already expect.
///
/// A missing pool is an error rather than `Ok(None)`, matching the behaviour
/// of the daemon request this replaced: every caller treats "no descriptor" as
/// "fall back", not as a value.
pub fn load_metadata(node: &mut DoraNode, buffer_id: &str, take: bool) -> eyre::Result<Metadata> {
    let timestamp = node.timestamp();
    let params = load(node, buffer_id, take)
        .map_err(|e| eyre::eyre!("{e}"))?
        .ok_or_else(|| eyre::eyre!("tensor pool `{buffer_id}` has no descriptor"))?;
    Ok(Metadata::from_parameters(timestamp, params))
}

/// Buffer ids whose descriptor has gone away since the last call — because
/// another node dropped it, or because the daemon reclaimed it after the
/// owner exited. Each one means this process should release whatever it
/// mapped for that pool.
pub fn drain_dropped(namespace: &str) -> Vec<String> {
    dora_node_api::event_stream::extensions::drain_dropped_keys(namespace)
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_node_api::Parameter;

    fn sample() -> MetadataParameters {
        let mut p = MetadataParameters::new();
        p.insert("dtype".into(), Parameter::String("float32".into()));
        p.insert("shape".into(), Parameter::ListInt(vec![3, 224, 224]));
        p.insert("is_pinned".into(), Parameter::Bool(true));
        p.insert("ipc_present".into(), Parameter::Bool(false));
        p
    }

    #[test]
    fn descriptor_round_trips() {
        let encoded = encode(&sample()).expect("encode");
        assert_eq!(decode(&encoded).expect("decode"), sample());
    }

    #[test]
    fn every_parameter_kind_survives() {
        // The descriptor is opaque to dora, so nothing else will catch a
        // variant that fails to round-trip.
        let mut p = MetadataParameters::new();
        p.insert("b".into(), Parameter::Bool(true));
        p.insert("i".into(), Parameter::Integer(-7));
        p.insert("s".into(), Parameter::String("x".into()));
        p.insert("li".into(), Parameter::ListInt(vec![1, 2]));
        p.insert("f".into(), Parameter::Float(1.5));
        p.insert("lf".into(), Parameter::ListFloat(vec![1.5, 2.5]));
        p.insert("ls".into(), Parameter::ListString(vec!["a".into()]));

        let encoded = encode(&p).expect("encode");
        assert_eq!(decode(&encoded).expect("decode"), p);
    }

    #[test]
    fn garbage_is_an_error_not_a_panic() {
        let err = decode(b"not json").expect_err("must reject");
        assert!(err.contains("failed to decode pool descriptor"), "{err}");
    }

    #[test]
    fn an_empty_descriptor_round_trips() {
        let empty = MetadataParameters::new();
        let encoded = encode(&empty).expect("encode");
        assert_eq!(decode(&encoded).expect("decode"), empty);
    }
}
