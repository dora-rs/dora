//! Arrow IPC stream framing for the schema-once wire format.
//!
//! A small node output travels schema-once: the node publishes the Arrow
//! schema once, on the output's `@schema` key, and each sample carries only
//! the schema-less record batch, tagged with the schema's hash. The producer
//! and the receive path in `dora-node-api`, and the `dora topic` rebuild in
//! [`crate::debug_topic`], all find the schema block and its hash through the
//! functions here, so they can't drift apart.

/// IPC stream continuation marker (precedes every message length prefix).
/// `dora-node-api`'s encoder writes its framing with these constants too.
pub const CONTINUATION_MARKER: [u8; 4] = [0xff, 0xff, 0xff, 0xff];
/// Bytes of the message length/continuation prefix (continuation + i32 length).
pub const PREFIX_LEN: usize = 8;

/// Length of the leading schema-message block of a full IPC stream produced by
/// `dora-node-api`'s `encode_ipc_into` (so a receiver can split it into the
/// schema prefix and the record-batch+body), or `None` if `stream` is not a
/// framed IPC message.
pub fn schema_block_len(stream: &[u8]) -> Option<usize> {
    if stream.len() < PREFIX_LEN || stream[0..4] != CONTINUATION_MARKER {
        return None;
    }
    let metadata_len = i32::from_le_bytes(stream[4..8].try_into().ok()?);
    let block = PREFIX_LEN.checked_add(usize::try_from(metadata_len).ok()?)?;
    (block <= stream.len()).then_some(block)
}

/// Schema identity of a full IPC stream: the FNV-1a hash of its leading schema
/// block, plus the block itself. This pairing IS the schema-once wire contract
/// — the producer (`publish_schema_once`), the node receive path (in-band
/// priming), and the `dora topic` rebuild all derive the hash from exactly
/// these bytes via this function (the producer via
/// [`schema_block_and_hash_reusing`], which this wraps); independent
/// re-derivations could drift and silently break batch↔schema matching.
pub fn schema_block_and_hash(stream: &[u8]) -> Option<(u64, &[u8])> {
    schema_block_and_hash_reusing(stream, None)
}

/// [`schema_block_and_hash`], but returns `known`'s hash without hashing again
/// when the schema block is byte-identical to `known`'s block. `known` must be
/// a pair this function returned earlier.
pub fn schema_block_and_hash_reusing<'a>(
    stream: &'a [u8],
    known: Option<(u64, &[u8])>,
) -> Option<(u64, &'a [u8])> {
    let block = schema_block_len(stream)?;
    let schema = stream.get(..block)?;
    let hash = match known {
        Some((hash, known_schema)) if known_schema == schema => hash,
        _ => dora_message::metadata::fnv1a(schema),
    };
    Some((hash, schema))
}

#[cfg(test)]
mod tests {
    use super::{schema_block_and_hash, schema_block_and_hash_reusing, schema_block_len};
    use dora_message::metadata::fnv1a;

    /// Wrap `msg` in an IPC message frame (continuation marker + length prefix).
    fn framed(msg: &[u8]) -> Vec<u8> {
        let mut out = vec![0xff, 0xff, 0xff, 0xff];
        out.extend_from_slice(&(msg.len() as i32).to_le_bytes());
        out.extend_from_slice(msg);
        out
    }

    #[test]
    fn schema_block_len_splits_off_the_first_message() {
        let schema = framed(b"SCHEMA");
        let mut stream = schema.clone();
        stream.extend_from_slice(&framed(b"BATCH"));
        assert_eq!(schema_block_len(&stream), Some(schema.len()));
        assert_eq!(schema_block_len(&schema), Some(schema.len()));
    }

    #[test]
    fn schema_block_len_rejects_what_is_not_a_framed_message() {
        assert_eq!(
            schema_block_len(&[0xff; 7]),
            None,
            "shorter than the prefix"
        );
        assert_eq!(schema_block_len(b"not-an-ipc-stream"), None, "no marker");

        let mut negative = vec![0xff; 4];
        negative.extend_from_slice(&(-1i32).to_le_bytes());
        assert_eq!(schema_block_len(&negative), None, "negative length");

        let mut truncated = framed(b"SCHEMA");
        truncated.pop();
        assert_eq!(schema_block_len(&truncated), None, "length past the end");
    }

    /// The producer passes its last `(hash, schema)` back in, and only gets the
    /// hash back unhashed while the schema bytes are the same.
    #[test]
    fn a_known_hash_is_reused_only_for_the_same_schema_bytes() {
        let mut stream = framed(b"SCHEMA-A");
        stream.extend_from_slice(&framed(b"BATCH"));
        let (hash, schema) = schema_block_and_hash(&stream).unwrap();
        assert_eq!(hash, fnv1a(schema));

        // 42 stands in for a hash the function must not recompute.
        let reused = schema_block_and_hash_reusing(&stream, Some((42, schema)));
        assert_eq!(reused, Some((42, schema)));

        let other = framed(b"SCHEMA-B");
        let (other_hash, other_schema) =
            schema_block_and_hash_reusing(&other, Some((42, schema))).unwrap();
        assert_eq!(other_hash, fnv1a(other_schema));
    }
}
