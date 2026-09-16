//! Topic debug streams: relaying observed outputs to coordinator-side
//! watchers (`dora topic echo`/`hz`), with the `@schema` cache that lets a
//! late watcher decode Arrow IPC frames.

use crate::{Daemon, InterDaemonEvent, OutputId};
use aligned_vec::AVec;
use dora_message::{DataflowId, node_to_daemon::Timestamped};
use eyre::Context;
use std::sync::Arc;

/// Cached `@schema` bytes (keyed by their FNV-1a hash, most-recent last) for
/// one debug-watched output, shared between the `@schema` subscriber callback
/// and the data task. Retains several schemas — mirroring the node receive
/// path's `InputDecoder` retention — so an output that interleaves schemas
/// (e.g. schema-once batches of schema A between large full streams of schema
/// B) doesn't have its schema-once schema clobbered, and a stale `@schema`
/// history-query result arriving after an in-band prime can't evict the
/// fresher schema (both stay retained).
pub(crate) type DebugSchemaCache = Arc<std::sync::Mutex<Vec<(u64, Vec<u8>)>>>;

/// Retention bound for [`DebugSchemaCache`], matching the node receive path's
/// `InputDecoder` (see `MAX_RETAINED_SCHEMAS` in dora-node-api's `ipc_encode`).
pub(crate) const MAX_DEBUG_SCHEMAS: usize = 8;

/// Remember `schema` for `hash` in the debug cache (most-recent last), evicting
/// the oldest entry beyond [`MAX_DEBUG_SCHEMAS`]. An already-retained hash is
/// moved to the back without re-copying its bytes.
pub(crate) fn retain_debug_schema(cache: &mut Vec<(u64, Vec<u8>)>, hash: u64, schema: &[u8]) {
    if let Some(pos) = cache.iter().position(|(h, _)| *h == hash) {
        let entry = cache.remove(pos);
        cache.push(entry);
        return;
    }
    cache.push((hash, schema.to_vec()));
    if cache.len() > MAX_DEBUG_SCHEMAS {
        cache.remove(0);
    }
}

/// Rebuild a self-describing Arrow IPC stream for a `dora topic` debug frame.
///
/// Small node→node outputs travel as schema-once batches: the schema is
/// published once on the `@schema` subtopic and each data sample carries only
/// the schema-less record batch (`DoraNode::publish_schema_once`). The
/// `dora topic` decoder expects a full self-describing stream, so for such a
/// batch we prepend the retained schema block — concatenation reconstructs the
/// exact original stream (`schema_block ++ batch_slice == full_stream`).
///
/// Returns `None` (drop the inspection frame) when the batch is schema-once but
/// no schema matching its `SCHEMA_HASH` has been retained yet — a transient,
/// inspection-only gap. A full self-describing payload (no `SCHEMA_HASH`) is
/// forwarded as-is.
pub(crate) fn rebuild_debug_topic_stream(
    cache: &mut Vec<(u64, Vec<u8>)>,
    parameters: &dora_message::metadata::MetadataParameters,
    payload: &[u8],
) -> Option<Vec<u8>> {
    use dora_message::metadata::{SCHEMA_HASH, carries_pattern_correlation, get_integer_param};

    match get_integer_param(parameters, SCHEMA_HASH) {
        Some(hash) => {
            // Only rebuild with the schema matching this batch's hash —
            // otherwise wait for the right `@schema` rather than emit a stream
            // the CLI would decode against the wrong (stale/future) schema.
            let (_, schema) = cache.iter().find(|(h, _)| *h == hash as u64)?;
            let mut full = Vec::with_capacity(schema.len() + payload.len());
            full.extend_from_slice(schema);
            full.extend_from_slice(payload);
            Some(full)
        }
        None => {
            // In-band priming, mirroring the node receive path: a full
            // self-describing stream carries the same schema block the producer
            // publishes on `@schema`, so retain it here — then the schema-less
            // batches that follow rebuild even if the `@schema` subscription
            // missed the (single) schema emission. The producer guarantees such
            // a full stream for the first message of every output and
            // periodically thereafter (`DoraNode::publish_schema_once`).
            // Service/action messages (pattern correlation) are excluded from
            // schema-once; don't let their per-request schemas churn the cache.
            if !carries_pattern_correlation(parameters)
                && let Some((hash, schema)) =
                    dora_node_api::arrow_utils::ipc_encode::schema_block_and_hash(payload)
            {
                retain_debug_schema(cache, hash, schema);
            }
            Some(payload.to_vec())
        }
    }
}

impl Daemon {
    /// Forward a debug-observed local output to coordinator debug watchers.
    ///
    /// Reconstructs the same `InterDaemonEvent::Output` frame that `send_out`
    /// produced before #1787 moved data routing off the daemon, so the CLI's
    /// `dora topic` inspection sees an unchanged wire format. Does **not**
    /// deliver to local receivers — the publishing node already did that over
    /// Zenoh.
    pub(crate) async fn handle_debug_topic_data(
        &self,
        dataflow_id: DataflowId,
        output_id: OutputId,
        metadata: dora_message::metadata::Metadata,
        data: Option<Vec<u8>>,
    ) -> eyre::Result<()> {
        // Skip (re)serialization when no CLI is currently watching this topic.
        let has_watchers = self
            .running
            .get(&dataflow_id)
            .map(|df| df.debug_topic_watchers.contains_key(&output_id))
            .unwrap_or(false);
        if !has_watchers {
            return Ok(());
        }

        let event = InterDaemonEvent::Output {
            dataflow_id,
            node_id: output_id.0.clone(),
            output_id: output_id.1.clone(),
            metadata,
            data: data.map(|d| AVec::from_slice(128, &d)),
        };
        let serialized_event = Timestamped {
            inner: event,
            timestamp: self.clock.new_timestamp(),
        }
        .serialize()
        .wrap_err("failed to serialize debug topic event")?;

        self.send_topic_debug_frames(dataflow_id, &output_id, serialized_event)
            .await
    }
}

#[cfg(test)]
mod debug_topic_tests {
    use super::rebuild_debug_topic_stream;
    use dora_message::metadata::{
        MetadataParameters, Parameter, SCHEMA_HASH, WIRE_SIZE, debug_frame_wire_size, fnv1a,
        get_integer_param, strip_internal_parameters,
    };

    fn params_with_schema_hash(hash: u64) -> MetadataParameters {
        let mut params = MetadataParameters::default();
        params.insert(SCHEMA_HASH.to_string(), Parameter::Integer(hash as i64));
        params
    }

    /// Wrap `msg` in an IPC message frame (continuation marker + length prefix)
    /// so `schema_block_len` parses it like a real schema block.
    fn framed(msg: &[u8]) -> Vec<u8> {
        let mut out = vec![0xff, 0xff, 0xff, 0xff];
        out.extend_from_slice(&(msg.len() as i32).to_le_bytes());
        out.extend_from_slice(msg);
        out
    }

    #[test]
    fn full_stream_payload_is_forwarded_verbatim() {
        // No SCHEMA_HASH ⇒ already a self-describing stream (large/SHM or
        // service/action output); forward the bytes unchanged.
        let payload = b"self-describing-stream".to_vec();
        let mut cache = Vec::new();
        let out = rebuild_debug_topic_stream(&mut cache, &MetadataParameters::default(), &payload);
        assert_eq!(out, Some(payload));
        assert!(
            cache.is_empty(),
            "an unframed payload must not prime the cache"
        );
    }

    #[test]
    fn schema_once_batch_is_prepended_with_cached_schema() {
        let schema = b"SCHEMA-BLOCK".to_vec();
        let hash = fnv1a(&schema);
        let batch = b"schema-less-batch".to_vec();
        let mut cache = vec![(hash, schema.clone())];
        let out = rebuild_debug_topic_stream(&mut cache, &params_with_schema_hash(hash), &batch);
        let mut expected = schema;
        expected.extend_from_slice(&batch);
        assert_eq!(out, Some(expected));
    }

    /// End-to-end guard for the `dora topic info` bandwidth fix, mirroring the
    /// daemon debug path (the topic subscriber in `Daemon::spawn_dataflow`):
    /// rebuild the inspection stream, strip the internal wire keys, then stamp
    /// the *input* payload length under `WIRE_SIZE`. The CLI reader
    /// ([`debug_frame_wire_size`]) must then charge the schema-less on-wire
    /// size, not the rebuilt stream — otherwise bandwidth is over-reported by
    /// the prepended schema on every schema-once frame, worst for the
    /// small/frequent primitives the schema-once optimization targets (#2584).
    /// Keep this in sync with the stamp site in `spawn_dataflow`.
    #[test]
    fn on_wire_size_excludes_prepended_schema() {
        // --- schema-once frame: rebuilt = schema ++ batch, only batch on-wire.
        let schema = b"SCHEMA-BLOCK".to_vec();
        let hash = fnv1a(&schema);
        let batch = b"schema-less-batch".to_vec();
        let mut cache = vec![(hash, schema.clone())];
        let mut params = params_with_schema_hash(hash);

        let rebuilt =
            rebuild_debug_topic_stream(&mut cache, &params, &batch).expect("schema cached");
        assert_eq!(rebuilt.len(), schema.len() + batch.len());

        // Replicate the daemon stamp: strip wire keys, record the input length.
        strip_internal_parameters(&mut params);
        assert!(
            get_integer_param(&params, SCHEMA_HASH).is_none(),
            "internal wire keys must be stripped before stamping"
        );
        params.insert(
            WIRE_SIZE.to_string(),
            Parameter::Integer(batch.len() as i64),
        );

        // The CLI reader charges the schema-less size, not the rebuilt stream.
        let charged = debug_frame_wire_size(&params, Some(&rebuilt));
        assert_eq!(charged, batch.len());
        assert!(
            charged < rebuilt.len(),
            "schema-once accounting must exclude the prepended schema"
        );

        // --- full self-describing frame: rebuilt == payload, charge full len.
        let full = b"self-describing-stream".to_vec();
        let mut full_params = MetadataParameters::default();
        let out = rebuild_debug_topic_stream(&mut Vec::new(), &full_params, &full)
            .expect("full stream forwarded");
        assert_eq!(out.len(), full.len());
        strip_internal_parameters(&mut full_params);
        full_params.insert(WIRE_SIZE.to_string(), Parameter::Integer(full.len() as i64));
        assert_eq!(debug_frame_wire_size(&full_params, Some(&out)), full.len());
    }

    #[test]
    fn schema_once_batch_dropped_when_schema_not_cached() {
        let batch = b"schema-less-batch".to_vec();
        let out = rebuild_debug_topic_stream(&mut Vec::new(), &params_with_schema_hash(42), &batch);
        assert_eq!(out, None);
    }

    #[test]
    fn schema_once_batch_dropped_on_hash_mismatch() {
        // Retained schema is for a different hash (e.g. a stale schema
        // mid-change): drop rather than emit a stream the CLI would decode
        // incorrectly.
        let schema = b"OTHER-SCHEMA".to_vec();
        let out = rebuild_debug_topic_stream(
            &mut vec![(999, schema)],
            &params_with_schema_hash(42),
            b"batch",
        );
        assert_eq!(out, None);
    }

    /// A full self-describing stream must prime the debug schema cache in-band
    /// (mirroring the node receive path), so `dora topic` can rebuild the
    /// schema-less batches that follow even when the `@schema` subscription
    /// missed the (single) schema emission — the producer guarantees a full
    /// stream for the first message of every output and periodically
    /// thereafter (dora-rs/dora#2366 review).
    #[test]
    fn full_stream_primes_debug_schema_cache_in_band() {
        let schema_block = framed(b"SCHEMA-FLATBUFFER");
        let hash = fnv1a(&schema_block);
        let batch = framed(b"BATCH-FLATBUFFER");
        let mut full = schema_block.clone();
        full.extend_from_slice(&batch);

        // The full stream is forwarded verbatim AND primes the cache.
        let mut cache = Vec::new();
        let out = rebuild_debug_topic_stream(&mut cache, &MetadataParameters::default(), &full);
        assert_eq!(out, Some(full.clone()));
        assert_eq!(cache, vec![(hash, schema_block)]);

        // A schema-less batch tagged with that hash now rebuilds, without any
        // `@schema` sample ever having been received.
        let out = rebuild_debug_topic_stream(&mut cache, &params_with_schema_hash(hash), &batch);
        assert_eq!(out, Some(full));
    }

    /// The cache retains multiple schemas: a full stream of a different schema
    /// (e.g. a large message interleaved with schema-once batches on the same
    /// output) must not clobber the schema those batches reference — the node
    /// receive path retains 8 schemas for exactly this interleave, and the
    /// `dora topic` mirror must not silently diverge from what nodes decode.
    #[test]
    fn interleaved_full_stream_does_not_clobber_schema_once_schema() {
        let schema_a = framed(b"SCHEMA-A");
        let hash_a = fnv1a(&schema_a);
        let batch_a = framed(b"BATCH-A");

        // Prime A (as the @schema subscriber or an earlier full stream would).
        let mut cache = vec![(hash_a, schema_a.clone())];

        // A large full stream of schema B passes through on the same output...
        let mut full_b = framed(b"SCHEMA-B");
        full_b.extend_from_slice(&framed(b"BATCH-B"));
        let out = rebuild_debug_topic_stream(&mut cache, &MetadataParameters::default(), &full_b);
        assert_eq!(out, Some(full_b));

        // ...and the next A-tagged schema-once batch still rebuilds.
        let out =
            rebuild_debug_topic_stream(&mut cache, &params_with_schema_hash(hash_a), &batch_a);
        let mut expected = schema_a;
        expected.extend_from_slice(&batch_a);
        assert_eq!(out, Some(expected));
    }

    /// Service/action full streams (pattern-correlated) are excluded from
    /// schema-once and must not churn the debug schema cache.
    #[test]
    fn pattern_correlated_stream_does_not_prime_debug_cache() {
        let schema_block = framed(b"SCHEMA-FLATBUFFER");
        let hash = fnv1a(&schema_block);

        let mut cache = vec![(hash, schema_block.clone())];
        let mut params = MetadataParameters::default();
        params.insert(
            dora_message::metadata::REQUEST_ID.to_string(),
            Parameter::String("req-1".into()),
        );
        let mut service_full = framed(b"OTHER-SCHEMA");
        service_full.extend_from_slice(&framed(b"OTHER-BATCH"));
        let out = rebuild_debug_topic_stream(&mut cache, &params, &service_full);
        assert_eq!(out, Some(service_full));
        assert_eq!(
            cache,
            vec![(hash, schema_block)],
            "a service reply must not churn the schema-once cache"
        );
    }
}
