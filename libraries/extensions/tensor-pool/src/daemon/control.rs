//! Control plane: zenoh publication of pool events and the pending-ack registries.

use super::*;

/// Publish one [`PeerMessage`] to the dataflow's extension topic, wrapped in
/// dora's opaque [`InterDaemonEvent::ExtensionMessage`] envelope.
///
/// `target_machine` addresses a single daemon (`None` = every daemon in the
/// dataflow); dora drops the message on every daemon it does not name, so
/// this side needs no gating of its own.
///
/// serialize + declare + put all run off the event loop (Block congestion
/// control can block declare_publisher on a degraded link). Logs the
/// declare and put timing — encoding the 61.44MB write payload takes
/// hundreds of ms (3s+ in debug builds), so the timing is worth knowing on
/// every publish path.
pub(crate) async fn publish_pool_message(
    session: &zenoh::Session,
    clock: &Arc<HLC>,
    dataflow_id: &Uuid,
    target_machine: Option<&str>,
    message: &PeerMessage,
    shm_provider: Option<&ShmProvider<PosixShmProviderBackend>>,
) -> eyre::Result<()> {
    let is_bulk = matches!(message, PeerMessage::Write { .. });
    let event = InterDaemonEvent::ExtensionMessage {
        dataflow_id: *dataflow_id,
        namespace: NAMESPACE.to_string(),
        target_machine: target_machine.map(str::to_string),
        payload: postcard::to_allocvec(message)
            .map_err(|e| eyre!("memory pool: encoding peer message failed: {e}"))?,
    };
    let serialized = Timestamped {
        inner: event,
        timestamp: clock.new_timestamp(),
    }
    .serialize()?;
    let payload_len = serialized.len();
    let topic = dataflow_extension_topic(dataflow_id, NAMESPACE);
    // Zenoh errors are boxed trait objects — eyre's `From` conversion
    // needs a Sized error, so convert explicitly instead of `?`.
    let declared = std::time::Instant::now();
    let publisher = session
        .declare_publisher(topic.clone())
        .congestion_control(CongestionControl::Block)
        // Remote-only: with the default Locality::Any the publisher's own
        // subscriber receives its own put, and on the RegisterPoolAck path
        // that local echo would be a self-ack that races (and beats) the
        // remote ack (see the RegisterPool handler).
        .allowed_destination(Locality::Remote)
        .await
        .map_err(|e| eyre!("memory pool: declare_publisher({topic}) failed: {e}"))?;
    tracing::info!(
        "memory pool: declared {topic} in {:?}, starting put ({payload_len} bytes)",
        declared.elapsed()
    );
    let started = std::time::Instant::now();
    // Control messages (Register/RegisterAck/Free) go over zenoh SHM when a
    // provider exists: same-host daemons map the payload zero-copy,
    // cross-host receivers get an implicit copy from the zenoh transport.
    // A Write carries the cross-machine tensor data and only ever happens
    // cross-host, where an SHM segment would be an extra copy with no
    // benefit — keep it on the plain path.
    let put_result = if is_bulk {
        publisher.put(serialized).await
    } else if let Some(provider) = shm_provider {
        // Synchronous wait: control payloads are KB-scale, so the shm
        // segment allocation is microseconds — no need for the async
        // allocation policy machinery.
        match provider.alloc(payload_len).wait() {
            // `alloc` guarantees a buffer of at least `payload_len` bytes
            // (alignment may round up), so the copy cannot overflow.
            Ok(mut buf) => {
                let buf_slice: &mut [u8] = buf.as_mut();
                buf_slice[..payload_len].copy_from_slice(&serialized);
                let payload: ZBytes = buf.into();
                publisher.put(payload).await
            }
            Err(e) => {
                tracing::warn!(
                    "memory pool: SHM alloc failed ({e}), falling back to regular payload"
                );
                publisher.put(serialized).await
            }
        }
    } else {
        publisher.put(serialized).await
    };
    put_result.map_err(|e| eyre!("memory pool: publish to {topic} failed: {e}"))?;
    tracing::info!(
        "memory pool: put to {topic} completed in {:?}",
        started.elapsed()
    );
    Ok(())
}

/// Release a cross-machine pool from the freeing daemon: unlink this
/// machine's mirror (self-machine-qualified name; on the origin daemon
/// the unlink is a harmless NotFound no-op) and publish a targeted
/// `FreePool` so the peer drops its tracking entry. The publish is
/// Remote-only — the initiator never receives its own echo, so it must
/// unlink its own mirror here. The caller has already removed the
/// cross_pools entry and passes the recorded peer (the pool's other
/// machine) as the free target.
pub(crate) async fn release_cross_pool(
    session: &zenoh::Session,
    clock: &Arc<HLC>,
    dataflow_id: &Uuid,
    machine_id: &str,
    peer_machine_id: &str,
    shared_memory_id: &str,
    shm_provider: Option<&ShmProvider<PosixShmProviderBackend>>,
) {
    if !unlink_cross_pool_segment(machine_id, &dataflow_id.to_string(), shared_memory_id) {
        return;
    }
    tracing::info!("memory pool: forwarding free of {shared_memory_id} to peer {peer_machine_id}");
    if let Err(e) = publish_pool_message(
        session,
        clock,
        dataflow_id,
        Some(peer_machine_id),
        &PeerMessage::Free {
            shared_memory_id: shared_memory_id.to_string(),
        },
        shm_provider,
    )
    .await
    {
        tracing::warn!("memory pool: failed to publish the free of {shared_memory_id}: {e}");
    }
}

/// Pending synchronous register confirmations:
/// (dataflow id, pool id) -> ack channel. Keyed by dataflow too: every
/// node process restarts its pool counter from zero, so a bare pool id
/// repeats across concurrently running dataflows and an ack could
/// satisfy the wrong registration.
pub(crate) type RegisterAckSenders = std::sync::Mutex<
    std::collections::HashMap<
        (Uuid, String),
        tokio::sync::oneshot::Sender<(bool, bool, Option<u16>, Option<std::net::SocketAddr>)>,
    >,
>;
pub(crate) static CROSS_REGISTER_PENDING: std::sync::LazyLock<RegisterAckSenders> =
    std::sync::LazyLock::new(RegisterAckSenders::default);

/// Pending cross-machine write replies:
/// (dataflow id, pool id, write seq) -> the node's reply channel. The
/// write reply is withheld until the mirror daemon confirms the segment
/// write (`MemoryPoolWriteAck`), so the output notification that follows
/// the write can never overtake the tensor data.
pub(crate) type CrossWriteReplySenders = std::sync::Mutex<
    std::collections::HashMap<(Uuid, String, u64), tokio::sync::oneshot::Sender<DaemonReply>>,
>;
pub(crate) static CROSS_WRITE_PENDING: std::sync::LazyLock<CrossWriteReplySenders> =
    std::sync::LazyLock::new(CrossWriteReplySenders::default);

/// Per-pool write sequence counters: (dataflow id, pool id) -> next seq.
/// Assigned at the origin, echoed by the mirror's commit ack, so the
/// ack can never resolve a reply for a different write.
pub(crate) static CROSS_WRITE_SEQ: std::sync::LazyLock<
    std::sync::Mutex<std::collections::HashMap<(Uuid, String), u64>>,
> = std::sync::LazyLock::new(std::sync::Mutex::default);

/// Pools whose direct-TCP write path is currently degraded to the zenoh
/// relay. The fallback is the steady state on a broken link, so without
/// this tracking every frame would warn — flooding the log. Keyed like
/// [`CROSS_WRITE_PENDING`]: `(dataflow id, pool id)`. Drained per
/// dataflow in `finish_dataflow`.
pub(crate) static CROSS_DIRECT_DEGRADED: std::sync::LazyLock<
    std::sync::Mutex<std::collections::HashSet<(Uuid, String)>>,
> = std::sync::LazyLock::new(std::sync::Mutex::default);

/// Mark a pool's direct-TCP path as degraded. Returns `true` only on the
/// **first** degradation — the caller warns exactly then; repeated
/// failures while already degraded stay silent.
pub(crate) fn note_direct_degraded(dataflow_id: Uuid, shared_memory_id: &str) -> bool {
    CROSS_DIRECT_DEGRADED
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .insert((dataflow_id, shared_memory_id.to_string()))
}

/// Mark a pool's direct-TCP path as recovered. Returns `true` only if the
/// pool was actually degraded — the caller logs the recovery exactly
/// once, on the first successful direct write after a fallback.
pub(crate) fn note_direct_recovered(dataflow_id: Uuid, shared_memory_id: &str) -> bool {
    CROSS_DIRECT_DEGRADED
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .remove(&(dataflow_id, shared_memory_id.to_string()))
}

/// How long a cross-machine write waits for the remote commit ack before
/// failing loudly. Generous: the WAN transfer of a near-limit frame alone
/// can take tens of seconds; a dead link fails earlier via the publish
/// error path.
pub(crate) const CROSS_WRITE_ACK_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(120);
/// Upper bound for reading one direct-TCP data frame (header + payload).
/// A peer that sends a header with a large `size` and then stalls would
/// otherwise hold the per-pool lock and leave the seqlock odd
/// indefinitely, wedging every subsequent write to that pool. 300s covers
/// a 1 GiB frame on a ~5 MB/s slow WAN link; on timeout the connection is
/// dropped, the lock guard drops, and the odd generation marks the frame
/// torn (next write self-heals).
pub(crate) const CROSS_DATA_READ_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(300);

/// Resolve the pending cross-machine write reply for a commit ack.
/// Only the seq-matched pending entry is resolved — an ack for a previous
/// write can never satisfy a newer pending reply. Returns whether a
/// matching pending entry existed and was resolved.
pub(crate) fn resolve_cross_write_ack(
    dataflow_id: Uuid,
    shared_memory_id: String,
    seq: u64,
    ok: bool,
    error: Option<String>,
) -> bool {
    if let Some(tx) = CROSS_WRITE_PENDING
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .remove(&(dataflow_id, shared_memory_id, seq))
    {
        let result = if ok {
            Ok(())
        } else {
            Err(error.unwrap_or_else(|| "remote mirror write failed".to_string()))
        };
        let _ = tx.send(DaemonReply::Result(result));
        true
    } else {
        false
    }
}

/// Fire the safety-net timeout for a pending cross-machine write: remove the
/// entry under the write's *effective* seq and fail the node's blocked
/// `WritePinnedMemory`. Returns whether an entry existed.
///
/// `effective_seq` is the seq the entry is live under now, which advances from
/// the original `seq` to a fresh `relay_seq` on a direct-TCP → zenoh failover.
/// Keying the timeout to the pre-failover seq makes it a no-op after a re-key,
/// leaving the write hung forever (#3193).
///
/// The load happens *under the pending-map guard*, the same one
/// `rekey_cross_write_to_relay` holds while it re-keys and stores. Loading
/// outside it would reopen #3193 as a race: a failover landing between the load
/// and the `remove` would leave the removal targeting a key that has already
/// moved, and nothing retries.
pub(crate) fn fire_cross_write_timeout(
    dataflow_id: Uuid,
    shared_memory_id: &str,
    effective_seq: &std::sync::atomic::AtomicU64,
) -> bool {
    let mut pending = CROSS_WRITE_PENDING
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let seq = effective_seq.load(std::sync::atomic::Ordering::SeqCst);
    let Some(tx) = pending.remove(&(dataflow_id, shared_memory_id.to_string(), seq)) else {
        return false;
    };
    drop(pending);
    let _ = tx.send(DaemonReply::Result(Err(
        "cross-machine write timed out waiting for the remote commit ack".to_string(),
    )));
    true
}

/// Re-key a pending cross-machine write from `seq` to a fresh relay seq for the
/// zenoh fallback hop, advancing `effective_seq` to match. Returns the new
/// relay seq, or `None` if nothing was pending under `seq` (already resolved).
///
/// The re-key drops the stale direct-plane `ok:false` (published the instant a
/// mid-frame direct read fails) via the first-wins resolver, and keeps the
/// per-pool counter monotonic so the next write cannot collide with the
/// re-keyed seq. Advancing `effective_seq` is the crucial half: the safety-net
/// timeout task reads it, so a relay ack lost after the re-key is still
/// reclaimed rather than hanging the node forever (#3193).
pub(crate) fn rekey_cross_write_to_relay(
    dataflow_id: Uuid,
    shared_memory_id: &str,
    seq: u64,
    effective_seq: &std::sync::atomic::AtomicU64,
) -> Option<u64> {
    // Remove, re-key and store under a *single* pending-map guard. Releasing
    // it between the remove and the insert would leave the entry in no map at
    // all, so a concurrent `drain_cross_write_pending` walks past it and the
    // re-insert then outlives `finish_dataflow` — the node stays blocked until
    // the 120s safety net and the entry leaks past the dataflow it belongs to.
    // The counter bump nests `CROSS_WRITE_SEQ` inside this guard; that is the
    // only nesting of the two (every other site drops the seq lock before
    // touching the pending map), so the order cannot deadlock.
    let mut pending = CROSS_WRITE_PENDING
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let tx = pending.remove(&(dataflow_id, shared_memory_id.to_string(), seq))?;
    let relay_seq = {
        let mut seqs = CROSS_WRITE_SEQ.lock().unwrap_or_else(|e| e.into_inner());
        let counter = seqs
            .entry((dataflow_id, shared_memory_id.to_string()))
            .or_insert(0);
        *counter += 1;
        *counter
    };
    pending.insert((dataflow_id, shared_memory_id.to_string(), relay_seq), tx);
    // Still under the guard: `fire_cross_write_timeout` loads this while
    // holding the same lock, so it can never observe a stale seq (#3193).
    effective_seq.store(relay_seq, std::sync::atomic::Ordering::SeqCst);
    Some(relay_seq)
}

/// Fail and drop every cross-machine write reply still pending for
/// `dataflow_id`, unblocking any node stuck in `WritePinnedMemory`. Returns the
/// number of stranded writes reclaimed.
///
/// Called from `finish_dataflow`: the per-write safety-net timeout normally
/// reclaims these, but a dataflow that finishes first (node crash, `dora stop`,
/// or a relay entry orphaned by a failover) would otherwise leave the entry —
/// and the node's blocked write — hanging until that timeout, and leak the
/// entry for the daemon's lifetime if the reply channel is already gone (#3193).
/// Mirrors how the sibling cross-write maps are already drained here.
pub(crate) fn drain_cross_write_pending(dataflow_id: Uuid) -> usize {
    let stranded: Vec<_> = {
        let mut pending = CROSS_WRITE_PENDING
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        // One pass, no key clones: the previous scan-clone-then-remove shape
        // allocated a `String` per match just to look each entry back up.
        pending
            .extract_if(|(df, _, _), _| *df == dataflow_id)
            .map(|(_, tx)| tx)
            .collect()
    };
    let count = stranded.len();
    for tx in stranded {
        let _ = tx.send(DaemonReply::Result(Err(
            "dataflow finished before the cross-machine write was acknowledged".to_string(),
        )));
    }
    count
}

/// Size of the daemon's zenoh SHM provider segment. Memory-pool control
/// notifications (RegisterPool/RegisterPoolAck/FreePool) are KB-scale,
/// so a small segment carries all in-flight control traffic with headroom;
/// the cross-machine tensor payload (MemoryPoolWrite) deliberately stays
/// on the plain path and never allocates from here.
pub(crate) const MEMORY_POOL_SHM_PROVIDER_SIZE: usize = 8 * 1024 * 1024;

#[cfg(test)]
mod cross_write_failover_tests {
    use super::{
        CROSS_WRITE_PENDING, CROSS_WRITE_SEQ, drain_cross_write_pending, fire_cross_write_timeout,
        rekey_cross_write_to_relay, resolve_cross_write_ack,
    };
    use dora_message::DataflowId;
    use dora_message::daemon_to_node::DaemonReply;

    #[test]
    fn failover_rekey_drops_stale_direct_ack_and_resolves_via_relay() {
        // The WriteMemoryPool failover rekeys the pending entry to a
        // fresh seq for the zenoh relay hop, so the mirror's ok:false
        // for the abandoned direct seq (published the instant its
        // mid-frame read fails) is dropped by the first-wins resolver,
        // and the relay's ok:true resolves the node's write. Without the
        // rekey, the ok:false would win the race and fail a write the
        // relay delivered byte-for-byte (bot review 5307022693).
        let df = DataflowId::new_v4();
        let pool = "pool_sender_node_1".to_string();
        let (tx, rx) = tokio::sync::oneshot::channel();
        // Write-side seq allocation (counter 0 -> 1), mirroring the
        // WriteMemoryPool handler.
        let mut seqs = CROSS_WRITE_SEQ.lock().unwrap_or_else(|e| e.into_inner());
        let counter = seqs.entry((df, pool.clone())).or_insert(0);
        *counter += 1;
        let seq = *counter;
        drop(seqs);
        assert_eq!(seq, 1, "test setup assumes the first write takes seq 1");
        CROSS_WRITE_PENDING
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .insert((df, pool.clone(), seq), tx);

        // Rekey via the production helper the direct-failure arm calls, so
        // this asserts against the real path rather than a copy of it.
        let effective_seq = std::sync::atomic::AtomicU64::new(seq);
        let relay_seq = rekey_cross_write_to_relay(df, &pool, seq, &effective_seq)
            .expect("pending entry present");
        assert_ne!(relay_seq, seq, "rekey must produce a fresh seq");

        // The stale direct-plane ok:false arrives first: no pending entry
        // under the old seq, so it is dropped without touching the node.
        assert!(!resolve_cross_write_ack(
            df,
            pool.clone(),
            seq,
            false,
            Some("mid-frame direct read failed".to_string())
        ));

        // The relay's ok:true resolves the node's write as a success.
        assert!(resolve_cross_write_ack(
            df,
            pool.clone(),
            relay_seq,
            true,
            None
        ));
        let result = rx.blocking_recv().expect("node reply delivered");
        assert!(
            matches!(result, DaemonReply::Result(Ok(()))),
            "node must see the relay's success, not the stale direct failure"
        );
    }

    #[test]
    fn rekey_advances_effective_seq_so_the_timeout_follows_it() {
        // #3193: drives the production re-key path (`rekey_cross_write_to_relay`,
        // the direct-TCP → zenoh failover arm) and pins the actual fix — that
        // it advances the shared `effective_seq` the safety-net timeout reads.
        // Deleting the `effective_seq.store(relay_seq)` inside the helper makes
        // the `effective_seq` assertion below fail, and the timeout then reads
        // the stale seq (a no-op) so the final resolve fails too — i.e. this
        // test fails exactly when the write would hang forever.
        let df = DataflowId::new_v4();
        let pool = "pool_rekey_atomic".to_string();
        let (tx, mut rx) = tokio::sync::oneshot::channel();

        // Allocate the seq exactly as the handler does (counter 0 -> 1) and
        // insert the pending reply under it.
        let seq = {
            let mut seqs = CROSS_WRITE_SEQ.lock().unwrap_or_else(|e| e.into_inner());
            let counter = seqs.entry((df, pool.clone())).or_insert(0);
            *counter += 1;
            *counter
        };
        CROSS_WRITE_PENDING
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .insert((df, pool.clone(), seq), tx);
        let effective_seq = std::sync::atomic::AtomicU64::new(seq);

        // Run the real re-key the handler runs on a direct-send failure.
        let relay_seq = rekey_cross_write_to_relay(df, &pool, seq, &effective_seq)
            .expect("a pending entry under the original seq");
        assert_ne!(relay_seq, seq, "re-key must allocate a fresh monotonic seq");
        // The crux of the fix: the atomic the timeout task reads was advanced
        // to the re-keyed seq.
        assert_eq!(
            effective_seq.load(std::sync::atomic::Ordering::SeqCst),
            relay_seq,
            "re-key must advance the effective seq the timeout follows"
        );

        // A timeout still keyed to the pre-failover seq is a no-op: the entry
        // moved. This is the #3193 regression, reproduced by handing the
        // helper an atomic frozen at the stale seq.
        let stale_seq = std::sync::atomic::AtomicU64::new(seq);
        assert!(!fire_cross_write_timeout(df, &pool, &stale_seq));
        assert!(
            rx.try_recv().is_err(),
            "write must still be pending after a timeout keyed to the stale seq"
        );

        // The real safety net passes the shared atomic the re-key advanced —
        // the helper loads it under the pending guard — so it resolves the
        // re-keyed entry and fails the node's write cleanly instead of hanging.
        assert!(fire_cross_write_timeout(df, &pool, &effective_seq));
        assert!(
            matches!(rx.try_recv(), Ok(DaemonReply::Result(Err(_)))),
            "the timeout keyed to the effective seq must fail the node's write"
        );
    }

    #[test]
    fn finish_dataflow_drain_fails_pending_cross_writes() {
        // #3193: a dataflow that finishes with a write still pending (node
        // crash / stop, or a relay entry orphaned by failover) must have that
        // entry reclaimed and the node unblocked, not left to leak. An
        // unrelated dataflow's pending write must survive the drain.
        let df = DataflowId::new_v4();
        let other_df = DataflowId::new_v4();
        let pool = "pool_finish_drain".to_string();
        let (tx, mut rx) = tokio::sync::oneshot::channel();
        let (other_tx, mut other_rx) = tokio::sync::oneshot::channel();

        {
            let mut pending = CROSS_WRITE_PENDING
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            pending.insert((df, pool.clone(), 7), tx);
            pending.insert((other_df, pool.clone(), 7), other_tx);
        }

        assert_eq!(drain_cross_write_pending(df), 1);
        assert!(
            matches!(rx.try_recv(), Ok(DaemonReply::Result(Err(_)))),
            "the finishing dataflow's pending write must be failed"
        );
        assert!(
            other_rx.try_recv().is_err(),
            "an unrelated dataflow's pending write must be untouched"
        );

        // Leave the shared static tidy for other tests.
        drain_cross_write_pending(other_df);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn create_cross_pool_shmem_idempotent_on_matching_size() {
        // A retried registration whose first ack was lost re-runs
        // create_cross_pool_shmem against the daemon's own live mirror:
        // same (dataflow, pool) and same size must be an idempotent
        // success, not an unlink+recreate that strands receivers already
        // mapping the old inode (bot review 5307022693).
        let df = DataflowId::new_v4();
        let pool = "pool_sender_node_1".to_string();
        let size = 61_440_000usize;
        super::create_cross_pool_shmem(
            &df,
            "M",
            &pool,
            size,
            "torch.int64",
            &[7_680_000],
            "cpu",
            None,
        )
        .expect("first create");
        let name = super::TensorPoolManager::cross_pool_shmem_name("M", &df.to_string(), &pool)
            .expect("name");
        let path = format!("/dev/shm/{name}");
        let ino_after_create = {
            use std::os::unix::fs::MetadataExt;
            std::fs::metadata(&path).expect("segment exists").ino()
        };
        // Second attempt (same everything): idempotent success — the
        // inode must survive, so receivers mapping it keep reading the
        // live mirror rather than a frozen orphan.
        super::create_cross_pool_shmem(
            &df,
            "M",
            &pool,
            size,
            "torch.int64",
            &[7_680_000],
            "cpu",
            None,
        )
        .expect("retry must be idempotent");
        let ino_after_idempotent = {
            use std::os::unix::fs::MetadataExt;
            std::fs::metadata(&path).expect("segment exists").ino()
        };
        assert_eq!(
            ino_after_create, ino_after_idempotent,
            "idempotent retry must not unlink+recreate the live mirror"
        );

        // A stale mirror of a different size is still replaced (new
        // inode, new size).
        super::create_cross_pool_shmem(
            &df,
            "M",
            &pool,
            size + 1,
            "torch.int64",
            &[7_680_000],
            "cpu",
            None,
        )
        .expect("stale size replaced");
        let meta = std::fs::metadata(&path).expect("segment exists");
        let ino_after_replace = {
            use std::os::unix::fs::MetadataExt;
            meta.ino()
        };
        assert_ne!(
            ino_after_create, ino_after_replace,
            "stale-size mirror must be replaced with a fresh segment"
        );
        let _ = std::fs::remove_file(&path);
    }
}
