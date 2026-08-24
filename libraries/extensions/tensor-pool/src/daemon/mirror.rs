//! Mirror segments: the DORADMA header, the seqlock, and the `/dev/shm`
//! segment lifetime on the receiving machine.

use super::*;

/// Per-pool synchronous write lock for the zenoh-relay data plane.
/// Keyed by `(dataflow id, pool id)` like the direct-TCP twin
/// `CROSS_POOL_WRITE_LOCKS_ASYNC`: pool ids repeat across dataflows (each
/// node process restarts its counter), and a bare pool-id key would
/// serialize writes to different segments of concurrent dataflows.
pub(crate) type CrossPoolWriteLocksSync = std::sync::Mutex<
    std::collections::HashMap<(Uuid, String), std::sync::Arc<std::sync::Mutex<()>>>,
>;
pub(crate) static CROSS_POOL_WRITE_LOCKS: std::sync::LazyLock<CrossPoolWriteLocksSync> =
    std::sync::LazyLock::new(CrossPoolWriteLocksSync::default);

/// Async per-pool write lock for the direct-TCP data plane: a
/// `std::sync::MutexGuard` cannot be held across an `.await`, so the
/// receive-into-mirror path (which reads the stream while holding the
/// per-pool serialization lock) uses an async mutex instead. Serializes
/// concurrent direct writes to the **same pool of the same dataflow**
/// across connections. Keyed by `(dataflow id, pool id)` like the other
/// cross-write state: pool ids repeat across dataflows (each node process
/// restarts its counter), and a bare pool-id key would needlessly
/// serialize writes to *different* segments of concurrent dataflows.
/// Per-plane only: the relay path serializes on the separate sync
/// `CROSS_POOL_WRITE_LOCKS`; cross-plane exclusion is provided by the
/// node's turn-based write cadence, not by these locks (bot review
/// 5306582566; the dual-lock split is a decided design).
pub(crate) type CrossPoolWriteLocks = tokio::sync::Mutex<
    std::collections::HashMap<(Uuid, String), std::sync::Arc<tokio::sync::Mutex<()>>>,
>;
pub(crate) static CROSS_POOL_WRITE_LOCKS_ASYNC: std::sync::LazyLock<CrossPoolWriteLocks> =
    std::sync::LazyLock::new(CrossPoolWriteLocks::default);

// DORADMA shmem layout — must match the node API exactly
// (apis/python/node/src/lib.rs): [magic:8][json_len:8][data_offset:8]
// [ipc_present:8][ipc_handle:64][write_gen:8 @96][reserved:152][json:256]
// [data:data_offset]. write_gen is the seqlock generation: even =
// complete, odd = write in progress.
pub(crate) const DORADMA_HEADER_SIZE: usize = 256;
pub(crate) const DORADMA_MAGIC: &[u8; 8] = b"DORADMA\x00";

/// Read 8 consecutive bytes from `ptr` as a little-endian u64. Same
/// implementation as the node API's `read_header_u64` — the wire layout
/// of the DORADMA header must be identical on both sides.
pub(crate) fn read_header_u64(ptr: *const u8) -> u64 {
    const { assert!(std::mem::size_of::<u64>() == 8) };
    let mut buf = [0u8; 8];
    unsafe { std::ptr::copy_nonoverlapping(ptr, buf.as_mut_ptr(), 8) };
    u64::from_le_bytes(buf)
}

/// Write 8 bytes as a little-endian u64 at `ptr`. Mirror of the node
/// API's header writes (`json_len_le` / `data_off_le` byte copies).
pub(crate) fn write_header_u64(ptr: *mut u8, value: u64) {
    const { assert!(std::mem::size_of::<u64>() == 8) };
    let le = value.to_le_bytes();
    unsafe { std::ptr::copy_nonoverlapping(le.as_ptr(), ptr, 8) };
}

/// Begins a memory-pool seqlock write at `gen_ptr` (header offset 96)
/// **if the generation is even**: marks the generation odd (write in
/// progress) and returns the even pre-write generation. If the
/// generation is already odd (leftover from a previous failed write),
/// the increment is skipped and the previous even generation is
/// returned, so `seqlock_end`'s `pre + 2` always produces an even
/// generation. Bit-identical to the node API's `seqlock_begin_if_even`.
pub(crate) unsafe fn seqlock_begin_if_even(gen_ptr: *mut u64) -> u64 {
    unsafe {
        let cur = std::ptr::read_volatile(gen_ptr);
        if cur.is_multiple_of(2) {
            std::ptr::write_volatile(gen_ptr, cur.wrapping_add(1));
            std::sync::atomic::fence(std::sync::atomic::Ordering::Release);
        }
        cur & !1 // always return the even baseline
    }
}

/// Closes a memory-pool seqlock write (header offset 96): publishes
/// `pre_write_gen + 2` (even = complete) when the copy succeeded, or
/// rolls back to `pre_write_gen` when it failed. Bit-identical to the
/// node API's `seqlock_end`.
pub(crate) unsafe fn seqlock_end(gen_ptr: *mut u64, pre_write_gen: u64, copy_ok: bool) {
    unsafe {
        // Release fence BEFORE the completion store, so the writer's payload
        // writes are ordered before the even ("complete") generation becomes
        // visible. A fence placed *after* the store orders nothing on a
        // weakly-ordered CPU (dora-rs/dora#3288).
        std::sync::atomic::fence(std::sync::atomic::Ordering::Release);
        if copy_ok {
            std::ptr::write_volatile(gen_ptr, pre_write_gen.wrapping_add(2));
        } else {
            std::ptr::write_volatile(gen_ptr, pre_write_gen);
        }
    }
}

/// Remove stale segments left on this machine by dataflows whose daemon
/// was killed without running shutdown cleanup. Only segments carrying
/// THIS machine's id prefix (`dora_pool_{machine_id}_...`) are touched: a
/// daemon restart implies its own dataflows died (nodes are daemon
/// children), and sibling daemons on the same host use their own prefixes
/// — so nothing live is ever unlinked. Machine-qualified LOCAL pool
/// segments (the python side now qualifies auto names with
/// `DORA_MACHINE_ID`) are swept here too — they are attributable to this
/// machine and can only be leftovers of this daemon's own dead dataflows.
#[cfg(target_os = "linux")]
pub(crate) fn cleanup_orphan_mirrors(machine_id: &str) -> usize {
    let prefix = format!("dora_pool_{machine_id}_");
    let Ok(entries) = std::fs::read_dir("/dev/shm") else {
        return 0;
    };
    let mut removed = 0;
    for entry in entries.flatten() {
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if name.starts_with(&prefix) {
            match std::fs::remove_file(entry.path()) {
                Ok(()) => {
                    tracing::info!("memory pool: removed orphan mirror segment {name}");
                    removed += 1;
                }
                Err(e) => tracing::warn!("memory pool: failed to remove orphan mirror {name}: {e}"),
            }
        }
    }
    if removed > 0 {
        tracing::info!("memory pool: cleaned {removed} orphan mirror segment(s)");
    }
    removed
}

/// The pool descriptor replicated into this daemon's extension table when
/// a cross-machine pool is mirrored (`PeerMessage::Register`). The sender's node
/// stores the descriptor on its own daemon only; without this replication
/// a receiver's daemon query (`load_metadata`) misses, so a GPU receiver
/// never obtains the daemon-trusted size it requires before HtoD staging
/// (`GPU_BUF_SIZES`) and every read fails as "pool does not exist". CPU
/// receivers never notice — their fast path reads the mirror header
/// directly — which is why CPU-only deployments cannot trip this.
///
/// Field set mirrors the python transport's `seam::store` descriptor so
/// the receiver-side consumers (`load_metadata`, `GPU_BUF_SIZES`) find
/// the same keys. `pinned_type` carries the receiver device ("cuda:0")
/// exactly like the mirror header JSON (same input, same value).
/// NOT cfg-gated: it only builds a JSON descriptor (no /dev/shm access),
/// and the `PeerMessage::Register` handler calls it unconditionally — gating it
/// broke the macOS daemon build (何勇 review 5302853212).
pub(crate) fn build_mirror_descriptor(
    size: usize,
    dtype: &str,
    shape: &[i64],
    mirror_shmem_name: &str,
    device: &str,
) -> Vec<u8> {
    let mut params = MetadataParameters::new();
    params.insert(
        "size".to_string(),
        dora_message::metadata::Parameter::Integer(size as i64),
    );
    params.insert(
        "dtype".to_string(),
        dora_message::metadata::Parameter::String(dtype.to_string()),
    );
    params.insert(
        "shape".to_string(),
        dora_message::metadata::Parameter::ListInt(shape.to_vec()),
    );
    params.insert(
        "shared_memory_name".to_string(),
        dora_message::metadata::Parameter::String(mirror_shmem_name.to_string()),
    );
    params.insert(
        "pinned_type".to_string(),
        dora_message::metadata::Parameter::String(device.to_string()),
    );
    params.insert(
        "ipc_present".to_string(),
        dora_message::metadata::Parameter::Bool(false),
    );
    serde_json::to_vec(&params).unwrap_or_default()
}

/// Create a CPU DORADMA pool mirror on this machine. Mirrors the node
/// API's register_memory_pool shmem layout. The generation is
/// initialized to an odd (in-progress) value with all-zero data so
/// receivers do not read the empty segment as a valid frame before the
/// first direct write lands (the reader retries while the generation is
/// odd). The segment is deliberately left owner-less (`set_owner(false)`)
/// so the /dev/shm name survives the handle drop — on Linux a created
/// (owner) Shmem shm_unlinks on drop, which would remove the name local
/// receivers open for the zero-copy fast path.
#[allow(clippy::too_many_arguments)]
pub(crate) fn create_cross_pool_shmem(
    dataflow_id: &Uuid,
    machine_id: &str,
    shared_memory_id: &str,
    size: usize,
    dtype: &str,
    shape: &[i64],
    device: &str,
    sender_shmem: Option<&str>,
) -> eyre::Result<()> {
    // Machine-qualified OS id: the mirror lives on the target machine's
    // /dev/shm, which on a dual-daemon test host is the SAME namespace as
    // the sender's local pool. An unqualified id would collide with the
    // sender's local segment (create fails with EEXIST) whenever both
    // daemons run on one host.
    let shmem_name = TensorPoolManager::cross_pool_shmem_name(
        machine_id,
        &dataflow_id.to_string(),
        shared_memory_id,
    )
    .ok_or_else(|| eyre::eyre!("invalid pool id: {shared_memory_id}"))?;
    // `device` is the receiver's device (the mirror's consumer): the
    // sender relays it in `PeerMessage::Register`. A GPU receiver ("cuda:0") reads
    // the mirror's CPU data region and stages it HtoD into its own GPU
    // buffer; "cpu" readers consume the data region directly.
    // `dtype`/`device` arrive from the remote `PeerMessage::Register` event
    // (untrusted cross-machine strings) — build the header JSON with
    // serde_json so quotes/backslashes are escaped instead of corrupting
    // or injecting into the parsed structure.
    // `sender_shmem`: the sender's local segment name, relayed so a
    // same-host reader can open it directly (the direct-TCP zero-copy
    // fast path — the mirror is only a fallback that would otherwise
    // serve stale frames, since the origin skips the per-frame push for
    // same-host pools). Absent on cross-host mirrors, where readers
    // consume the mirror's own data region.
    let mut header_json = serde_json::json!({
        "size": size,
        "dtype": dtype,
        "shape": shape,
        "pinned_type": device,
    });
    if let Some(name) = sender_shmem {
        header_json["sender_shmem"] = serde_json::Value::String(name.to_string());
    }
    let json = serde_json::to_string(&header_json)
        .map_err(|e| eyre::eyre!("failed to serialize mirror header JSON: {e}"))?;
    let data_offset = DORADMA_HEADER_SIZE + json.len();
    let make_conf = || ShmemConf::new().os_id(&shmem_name).size(size + data_offset);
    let mut shmem = match make_conf().create() {
        Ok(s) => s,
        Err(e) => {
            // EEXIST: either a stale mirror left by a daemon that was
            // killed without running shutdown cleanup, or — on a retried
            // registration whose first ack was lost — this daemon's own
            // live mirror created moments ago. Unlinking the latter
            // strands receivers that already mapped the old inode (they
            // read a frozen frame forever while writes land in the new
            // one). Distinguish by segment size: a live mirror from this
            // same (dataflow, pool) registration has exactly this size
            // and its header is already written — treat it as idempotent
            // success; anything else is stale and safe to replace (bot
            // review 5307022693).
            let shm_path = format!("/dev/shm/{shmem_name}");
            if let Ok(meta) = std::fs::metadata(&shm_path)
                && meta.len() == (size + data_offset) as u64
            {
                tracing::info!(
                    "memory pool: mirror {shmem_name} already exists at the registered size \
                     — idempotent success (registration retried after a lost ack)"
                );
                return Ok(());
            }
            if std::path::Path::new(&shm_path).exists() {
                tracing::warn!("memory pool: stale mirror {shmem_name} exists, replacing");
                std::fs::remove_file(&shm_path)
                    .map_err(|e| eyre::eyre!("remove stale mirror {shmem_name}: {e}"))?;
                make_conf()
                    .create()
                    .map_err(|re| eyre::eyre!("recreate mirror {shmem_name} after unlink: {re}"))?
            } else {
                return Err(eyre::eyre!("create shmem: {e}"));
            }
        }
    };
    unsafe {
        let ptr = shmem.as_ptr();
        std::ptr::copy_nonoverlapping(DORADMA_MAGIC.as_ptr(), ptr, 8);
        write_header_u64(ptr.add(8), json.len() as u64);
        write_header_u64(ptr.add(16), data_offset as u64);
        std::ptr::copy_nonoverlapping(json.as_ptr(), ptr.add(DORADMA_HEADER_SIZE), json.len());
        // Odd generation = write in progress. The mirror starts with
        // all-zero data; an even (complete) generation would let a
        // receiver read that as a valid frame before the first direct
        // write, so begin odd and let the first `seqlock_end` publish
        // the first even (complete) generation.
        write_header_u64(ptr.add(96), 1);
    }
    shmem.set_owner(false);
    Ok(())
}

/// Write tensor bytes into a mirrored cross-machine pool under the
/// DORADMA seqlock protocol (odd gen during write, even after).
///
/// A 61.44MB mirror write is a 10-30ms synchronous memcpy, so callers
/// must not run it on the event loop — spawn it (see the `PeerMessage::Write`
/// handler). Not async: there is nothing to await, the work is the copy.
/// Returns whether the mirror write completed (the caller publishes the
/// commit ack with this outcome).
pub(crate) fn write_cross_pool_data(
    dataflow_id: &Uuid,
    machine_id: &str,
    shared_memory_id: &str,
    tensor_data: &[u8],
    size: usize,
) -> bool {
    // Serialise concurrent relay-path writes to the same pool: two
    // overlapping memcpys would interleave bytes and leave a mixed frame
    // that the seqlock (odd = in-progress) cannot detect once both
    // writers have completed an even generation. Per-pool lock, held
    // across the whole write (open + seqlock begin + copy + end).
    // NOTE: this lock is per-plane — the direct-TCP path serializes on
    // the separate `CROSS_POOL_WRITE_LOCKS_ASYNC`. Cross-plane exclusion
    // relies on the node's turn-based write cadence (each write awaits
    // its seq-matched ack before the next), which the design assumes
    // (bot review 5306582566; the dual-lock split is a decided design).
    let write_lock = {
        let mut locks = CROSS_POOL_WRITE_LOCKS
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let key = (*dataflow_id, shared_memory_id.to_string());
        match locks.get(&key) {
            Some(lock) => lock.clone(),
            None => {
                let lock = std::sync::Arc::new(std::sync::Mutex::new(()));
                locks.insert(key, lock.clone());
                lock
            }
        }
    };
    let _guard = write_lock.lock().unwrap_or_else(|e| e.into_inner());
    // Must match the machine-qualified id used by `create_cross_pool_shmem`.
    let Some(shmem_name) = TensorPoolManager::cross_pool_shmem_name(
        machine_id,
        &dataflow_id.to_string(),
        shared_memory_id,
    ) else {
        tracing::warn!("memory pool: invalid pool id {shared_memory_id}, dropping frame");
        return false;
    };
    let Ok(shmem) = ShmemConf::new().os_id(&shmem_name).open() else {
        tracing::warn!(
            "memory pool: pool {shared_memory_id} missing at write \
             (sync register should have prevented this), dropping frame"
        );
        return false;
    };
    let shmem_ptr = shmem.as_ptr();
    // Guard against a corrupt/truncated header before any pointer math.
    let magic = unsafe { std::slice::from_raw_parts(shmem_ptr, 8) };
    if magic != DORADMA_MAGIC {
        tracing::warn!("memory pool: {shared_memory_id} header magic mismatch, dropping frame");
        return false;
    }
    // The payload must cover the registered pool size recorded in the
    // mirror header — a short payload would otherwise publish an even
    // generation over a truncated tensor whose tail `[copy_len..]` is
    // stale, and readers accept it as complete (the same gap the direct
    // path's registered-size check closes; the zenoh relay is the active
    // data plane whenever the direct endpoint is unavailable, not just a
    // fallback).
    let registered_size = {
        let json_len = unsafe { read_header_u64(shmem_ptr.add(8)) } as usize;
        let json_bytes =
            unsafe { std::slice::from_raw_parts(shmem_ptr.add(DORADMA_HEADER_SIZE), json_len) };
        serde_json::from_slice::<serde_json::Value>(json_bytes)
            .ok()
            .and_then(|v| v.get("size").and_then(|s| s.as_u64()))
            .unwrap_or(0) as usize
    };
    if tensor_data.len() != registered_size || size != registered_size {
        tracing::warn!(
            "memory pool: {shared_memory_id} payload {} bytes / declared {size} \
             != registered pool size {registered_size}, dropping frame",
            tensor_data.len()
        );
        return false;
    }
    let data_offset = unsafe { read_header_u64(shmem_ptr.add(16)) } as usize;
    let copy_len = tensor_data.len().min(size);
    // Checked add: a corrupt header's data_offset could otherwise wrap
    // `data_offset + copy_len` past the bounds check.
    let Some(end) = data_offset.checked_add(copy_len) else {
        tracing::warn!(
            "memory pool: {shared_memory_id} data_offset {data_offset} + {copy_len} overflows usize, dropping frame"
        );
        return false;
    };
    if end > shmem.len() {
        tracing::warn!(
            "memory pool: {shared_memory_id} data_offset {data_offset} + {copy_len} exceeds shmem size {}, dropping frame",
            shmem.len()
        );
        return false;
    }
    unsafe {
        let gen_ptr = shmem_ptr.add(96) as *mut u64;
        let pre = seqlock_begin_if_even(gen_ptr);
        std::ptr::copy_nonoverlapping(tensor_data.as_ptr(), shmem_ptr.add(data_offset), copy_len);
        seqlock_end(gen_ptr, pre, true);
    }
    true
}

/// Read a pool's tensor bytes from its local segment — the
/// shared-memory-reference write path: the node's write request carries
/// only `(id, size)` metadata, so the daemon opens the sender's segment
/// (name recorded at registration) and copies `size` bytes from the
/// DORADMA data region. Keeping the node→daemon request KB-scale removes
/// the transport cap on cross-machine pool size (the request previously
/// carried the whole tensor, bounded by `dora_message::MAX_MESSAGE_BYTES`).
pub(crate) fn read_pool_segment_data(shmem_name: &str, size: usize) -> Result<Vec<u8>, String> {
    let shmem = ShmemConf::new()
        .os_id(shmem_name)
        .open()
        .map_err(|e| format!("cannot open segment {shmem_name}: {e}"))?;
    let shmem_ptr = shmem.as_ptr();
    // Guard against a corrupt/truncated header before any pointer math.
    let magic = unsafe { std::slice::from_raw_parts(shmem_ptr, 8) };
    if magic != DORADMA_MAGIC {
        return Err(format!("segment {shmem_name} header magic mismatch"));
    }
    let data_offset = unsafe { read_header_u64(shmem_ptr.add(16)) } as usize;
    // Checked add: same wrapping concern as the direct-TCP frame path
    // (size comes from the node request, not the socket, but a corrupt
    // header's data_offset must not wrap the bounds check either).
    let Some(end) = data_offset.checked_add(size) else {
        return Err(format!(
            "segment {shmem_name} data_offset {data_offset} + {size} overflows usize"
        ));
    };
    if end > shmem.len() {
        return Err(format!(
            "segment {shmem_name} data_offset {data_offset} + {size} exceeds shmem size {}",
            shmem.len()
        ));
    }
    let mut data = vec![0u8; size];
    unsafe {
        std::ptr::copy_nonoverlapping(shmem_ptr.add(data_offset), data.as_mut_ptr(), size);
    }
    Ok(data)
}

/// Remove a mirrored cross-machine pool's shmem segment. Linux keeps
/// pools in /dev/shm; the name is only removable by file unlink because
/// the mirror handle was dropped owner-less (`set_owner(false)`).
/// Path-traversal guarded, mirroring the memory-pool crate's
/// `free_shared_memory` checks.
pub(crate) fn remove_cross_pool_shmem(shmem_name: &str) {
    if !shmem_name.starts_with("dora_pool_")
        || shmem_name.contains('/')
        || shmem_name.contains("..")
    {
        tracing::warn!("memory pool: refusing to remove shmem `{shmem_name}`: unexpected name");
        return;
    }
    #[cfg(target_os = "linux")]
    {
        let shm_path = format!("/dev/shm/{shmem_name}");
        match std::fs::remove_file(&shm_path) {
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => {
                tracing::warn!(
                    "memory pool: failed to unlink shared memory file {}: {}. \
                     The file may still be in use by other processes.",
                    shm_path,
                    e
                );
            }
        }
    }
    #[cfg(not(target_os = "linux"))]
    {
        // Not an anomaly worth a warning per pool per free/finish: the
        // whole cross-machine mirror path is Linux-only.
        tracing::debug!("memory pool: shmem removal only implemented on Linux");
    }
}

/// Unlink the `/dev/shm` segment a cross-machine pool resolves to under
/// `machine_id`, returning whether the name could be derived.
///
/// Used by the free paths that only know the pool id. Paths that hold
/// the name the segment was created under should unlink that instead —
/// see `cross_pool_shmem_name`'s own warning: "create, write, and the
/// free paths must agree or a mirror leaks under a name nobody unlinks."
pub(crate) fn unlink_cross_pool_segment(
    machine_id: &str,
    dataflow_id: &str,
    shared_memory_id: &str,
) -> bool {
    let Some(shmem_name) =
        TensorPoolManager::cross_pool_shmem_name(machine_id, dataflow_id, shared_memory_id)
    else {
        tracing::warn!("memory pool: invalid pool id {shared_memory_id}, cannot unlink mirror");
        return false;
    };
    remove_cross_pool_shmem(&shmem_name);
    true
}

#[cfg(test)]
mod cross_mirror_descriptor_tests {
    use super::build_mirror_descriptor;
    use dora_message::metadata::{MetadataParameters, Parameter};

    fn decode(bytes: &[u8]) -> MetadataParameters {
        serde_json::from_slice(bytes).expect("descriptor must decode")
    }

    fn get_str<'a>(params: &'a MetadataParameters, key: &str) -> &'a str {
        match params.get(key) {
            Some(Parameter::String(s)) => s,
            other => panic!("expected string {key}, got {other:?}"),
        }
    }

    #[test]
    fn gpu_mirror_descriptor_carries_receiver_device() {
        // A GPU receiver's mirror: pinned_type must be the receiver
        // device ("cuda:0"), not "cpu" — the python receiver derives
        // `effective_as_cuda` from it and stages the mirror's CPU data
        // region HtoD.
        let bytes = build_mirror_descriptor(
            61_440_000,
            "torch.int64",
            &[7_680_000],
            "dora_pool_D_01a00445-b12d-734b-852b-463d76c749b1_sender_node_1",
            "cuda:0",
        );
        let params = decode(&bytes);
        assert_eq!(
            get_str(&params, "pinned_type"),
            "cuda:0",
            "GPU mirror must advertise the receiver device"
        );
        assert_eq!(
            get_str(&params, "dtype"),
            "torch.int64",
            "dtype must round-trip"
        );
        assert_eq!(
            params.get("size"),
            Some(&Parameter::Integer(61_440_000)),
            "size must round-trip"
        );
        assert_eq!(
            params.get("shape"),
            Some(&Parameter::ListInt(vec![7_680_000])),
            "shape must round-trip"
        );
        assert_eq!(
            get_str(&params, "shared_memory_name",),
            "dora_pool_D_01a00445-b12d-734b-852b-463d76c749b1_sender_node_1",
            "shared_memory_name must be the machine-qualified mirror segment"
        );
        assert_eq!(
            params.get("ipc_present"),
            Some(&Parameter::Bool(false)),
            "cross-machine mirrors never export a host-local IPC handle"
        );
    }

    #[test]
    fn cpu_mirror_descriptor_is_cpu() {
        // A CPU receiver's mirror: pinned_type "cpu" keeps the python
        // receiver on the CPU read path (no HtoD staging).
        let bytes = build_mirror_descriptor(
            61_440_000,
            "torch.int64",
            &[7_680_000],
            "dora_pool_D_01a00445-b12d-734b-852b-463d76c749b1_sender_node_1",
            "cpu",
        );
        assert_eq!(get_str(&decode(&bytes), "pinned_type"), "cpu");
    }
}
