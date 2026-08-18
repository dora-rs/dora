//! Direct-TCP data plane: the mirror-side listener and the origin-side sender.

use super::*;

/// Direct-TCP cross-machine data plane.
///
/// Frame: `[u32 magic][16-byte dataflow UUID][u32 pool_id_len][pool_id]
/// [u64 seq][u64 size][size bytes of tensor data]`. The mirror daemon
/// reads the tensor straight into the mirror segment's data region
/// (zero user-space copies on the receive side); the origin pays a single
/// user-space copy (segment → send buffer). The commit ack travels over
/// zenoh (existing `MemoryPoolWriteAck` machinery) so the origin's
/// pending resolution is unchanged. When `DORA_MEMORY_POOL_AUTH_TOKEN` is
/// set, a token handshake precedes the first frame on every connection
/// (see [`auth_handshake_send`]/[`auth_handshake_verify`]).
pub(crate) const CROSS_DATA_MAGIC: u32 = 0xD0A0_0011;

/// Keeps a mirror mapping alive while the direct-TCP payload is read
/// straight into its data region. Only Send-safe values (plain addresses
/// and the keep-alive `Shmem`) cross the `.await` in
/// [`serve_cross_data_frame`]; the mapping itself is process-wide and
/// thread-agnostic (same pattern as the python extension's `PoolSlot`).
pub(crate) struct DirectMirrorWriter {
    _shmem: shared_memory_extended::Shmem,
    data_addr: usize,
    data_len: usize,
    gen_addr: usize,
    pre: u64,
}

// SAFETY: the wrapper owns the mapping (`_shmem` keeps it alive until
// `finish`/drop) and carries only plain addresses across awaits. Moving
// the wrapper between threads never invalidates the mapping (mmap is
// process-wide); no thread-bound state is involved.
unsafe impl Send for DirectMirrorWriter {}

impl DirectMirrorWriter {
    /// Begin a seqlock write: computes the data region address from the
    /// (already validated) header and takes the odd (in-progress)
    /// generation.
    fn new(shmem: shared_memory_extended::Shmem, data_offset: usize, size: usize) -> Self {
        let shmem_ptr = shmem.as_ptr();
        let gen_addr = unsafe { shmem_ptr.add(96) } as usize;
        let pre = unsafe { seqlock_begin_if_even(gen_addr as *mut u64) };
        Self {
            _shmem: shmem,
            data_addr: unsafe { shmem_ptr.add(data_offset) } as usize,
            data_len: size,
            gen_addr,
            pre,
        }
    }

    /// The mirror's data region, exactly `size` bytes (validated against
    /// the segment length by the caller before construction).
    fn data_slice_mut(&mut self) -> &mut [u8] {
        unsafe { std::slice::from_raw_parts_mut(self.data_addr as *mut u8, self.data_len) }
    }

    /// Complete the seqlock write (even generation).
    fn finish(self) {
        unsafe { seqlock_end(self.gen_addr as *mut u64, self.pre, true) };
    }
}

// NOTE on the failed-write path (no `Drop` rollback here): a payload
// read that fails mid-frame leaves the generation odd (in-progress).
// That is deliberate — readers reject the torn frame (they never see
// half-written data), and the next full write self-heals the segment
// (`seqlock_begin_if_even` finds the odd generation, keeps it, writes
// the full frame, and publishes the even one). Rolling the generation
// back to `pre` instead would *mark the torn bytes as a complete frame*,
// which is worse than blocking. The origin fails fast through
// `MemoryPoolWriteAck { ok: false }` (see `serve_cross_data_frame`).

/// Port for the mirror daemon's direct-TCP data listener. Overridable for
/// deployment (e.g. the rendezvous machine must publish this port to the
/// origin machine on a routed WAN link).
pub(crate) const CROSS_DATA_PORT_ENV: &str = "DORA_MEMORY_POOL_DATA_PORT";
pub(crate) const CROSS_DATA_PORT_DEFAULT: u16 = 7410;
/// Bind address for the direct-TCP data listener. Overridable for
/// deployments where the listener must not (or cannot) sit on every
/// interface — a host with a site firewall that only opens a specific
/// interface, or a shared machine where port 7410 on `0.0.0.0` would
/// collide with a sibling daemon.
pub(crate) const CROSS_DATA_BIND_ENV: &str = "DORA_MEMORY_POOL_DATA_BIND";
pub(crate) const CROSS_DATA_BIND_DEFAULT: &str = "0.0.0.0";

/// Read and serve one direct-TCP data frame: write the payload straight
/// into the mirror segment's data region (under the per-pool lock and
/// seqlock) and publish the zenoh commit ack. Returns `Ok(true)` for the
/// next frame, `Ok(false)` on clean EOF.
pub(crate) async fn serve_cross_data_frame(
    tensor_pool: &TensorPoolManager,
    machine_id: &str,
    session: &zenoh::Session,
    clock: &Arc<HLC>,
    shm_provider: Option<&ShmProvider<PosixShmProviderBackend>>,
    stream: &mut tokio::net::TcpStream,
) -> Result<bool, String> {
    // Bound the whole frame read: a peer that sends a header with a large
    // `size` and then stalls would otherwise hold the per-pool lock and
    // leave the seqlock odd indefinitely, wedging every subsequent write
    // to that pool. On timeout the connection is dropped; the lock guard
    // drops with the cancelled future, and the odd generation marks the
    // frame torn (next write self-heals).
    let frame = match tokio::time::timeout(
        CROSS_DATA_READ_TIMEOUT,
        handle_cross_data_frame(stream, tensor_pool, machine_id),
    )
    .await
    {
        Ok(frame) => frame,
        Err(_) => {
            return Err(
                "memory pool: direct-TCP data connection stalled (frame read timeout)".to_string(),
            );
        }
    };
    match frame {
        Ok(Some((dataflow_id, shared_memory_id, seq))) => {
            // Remote commit ack via zenoh (the origin's pending reply
            // waits on it).
            publish_memory_pool_event(
                session,
                clock,
                &dataflow_id,
                &InterDaemonEvent::MemoryPoolWriteAck {
                    dataflow_id,
                    shared_memory_id,
                    seq,
                    ok: true,
                    error: None,
                },
                shm_provider,
            )
            .await
            .map_err(|e| format!("failed to publish MemoryPoolWriteAck: {e}"))?;
            Ok(true)
        }
        Ok(None) => Ok(false),
        Err(err) => {
            // The frame's identity is known: fail the origin's pending
            // write fast (mirror could not write the frame — read error,
            // segment missing, bounds violation) instead of making it
            // wait out the commit-ack timeout.
            if let Some((dataflow_id, shared_memory_id, seq)) = err.ack
                && let Err(e) = publish_memory_pool_event(
                    session,
                    clock,
                    &dataflow_id,
                    &InterDaemonEvent::MemoryPoolWriteAck {
                        dataflow_id,
                        shared_memory_id,
                        seq,
                        ok: false,
                        error: Some(err.message.clone()),
                    },
                    shm_provider,
                )
                .await
            {
                tracing::warn!(
                    "memory pool: failed to publish failed-write MemoryPoolWriteAck: {e}"
                );
            }
            Err(err.message)
        }
    }
}

/// Frame-level error from [`handle_cross_data_frame`]. Carries the
/// frame's identity `(dataflow id, pool id, seq)` once the header has
/// been parsed, so the caller can publish `MemoryPoolWriteAck { ok: false }`
/// and let the origin fail fast instead of waiting out the commit-ack
/// timeout. Errors before the header is complete carry no ack info.
#[derive(Debug)]
pub(crate) struct CrossFrameError {
    pub(crate) message: String,
    pub(crate) ack: Option<(Uuid, String, u64)>,
}

impl CrossFrameError {
    fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            ack: None,
        }
    }

    fn with_ack(
        message: impl Into<String>,
        dataflow_id: Uuid,
        shared_memory_id: String,
        seq: u64,
    ) -> Self {
        Self {
            message: message.into(),
            ack: Some((dataflow_id, shared_memory_id, seq)),
        }
    }
}

/// Frame-parse + mirror-write core of the direct-TCP data plane (no zenoh
/// involved), split out for unit testing. Returns the ack info
/// `(dataflow id, pool id, seq)`; `Ok(None)` on clean EOF.
pub(crate) async fn handle_cross_data_frame(
    stream: &mut tokio::net::TcpStream,
    tensor_pool: &TensorPoolManager,
    machine_id: &str,
) -> Result<Option<(Uuid, String, u64)>, CrossFrameError> {
    use tokio::io::AsyncReadExt;

    let mut magic = [0u8; 4];
    match stream.read_exact(&mut magic).await {
        Ok(_) => {}
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(CrossFrameError::new(format!("read magic: {e}"))),
    }
    if u32::from_be_bytes(magic) != CROSS_DATA_MAGIC {
        return Err(CrossFrameError::new(
            "bad frame magic (not a memory-pool data connection?)",
        ));
    }
    let mut df_bytes = [0u8; 16];
    stream
        .read_exact(&mut df_bytes)
        .await
        .map_err(|e| CrossFrameError::new(format!("read dataflow id: {e}")))?;
    let dataflow_id = Uuid::from_bytes(df_bytes);
    let mut pool_len = [0u8; 4];
    stream
        .read_exact(&mut pool_len)
        .await
        .map_err(|e| CrossFrameError::new(format!("read pool id length: {e}")))?;
    let pool_len = u32::from_be_bytes(pool_len) as usize;
    if pool_len > 1024 {
        return Err(CrossFrameError::new(format!(
            "pool id too long ({pool_len} bytes)"
        )));
    }
    let mut pool_bytes = vec![0u8; pool_len];
    stream
        .read_exact(&mut pool_bytes)
        .await
        .map_err(|e| CrossFrameError::new(format!("read pool id: {e}")))?;
    let shared_memory_id =
        String::from_utf8(pool_bytes).map_err(|_| CrossFrameError::new("pool id not UTF-8"))?;
    let mut seq_bytes = [0u8; 8];
    stream
        .read_exact(&mut seq_bytes)
        .await
        .map_err(|e| CrossFrameError::new(format!("read seq: {e}")))?;
    let seq = u64::from_be_bytes(seq_bytes);
    let mut size_bytes = [0u8; 8];
    stream
        .read_exact(&mut size_bytes)
        .await
        .map_err(|e| CrossFrameError::new(format!("read size: {e}")))?;
    let size = u64::from_be_bytes(size_bytes) as usize;

    let dataflow_str = dataflow_id.to_string();
    if !tensor_pool.is_cross(&dataflow_str, &shared_memory_id) {
        return Err(CrossFrameError::with_ack(
            format!("write for a pool without a cross-machine entry: {shared_memory_id}"),
            dataflow_id,
            shared_memory_id.clone(),
            seq,
        ));
    }
    let Some(shmem_name) =
        TensorPoolManager::cross_pool_shmem_name(machine_id, &dataflow_str, &shared_memory_id)
    else {
        return Err(CrossFrameError::with_ack(
            format!("invalid pool id {shared_memory_id}"),
            dataflow_id,
            shared_memory_id.clone(),
            seq,
        ));
    };
    // Serialise concurrent direct writes to the same pool of the same
    // dataflow first (async lock: a std MutexGuard cannot be held across
    // an await), so nothing non-Send crosses this await. Keyed by
    // (dataflow, pool) — a bare pool id would serialize writes to
    // different segments of concurrent dataflows.
    let write_lock = {
        let mut locks = CROSS_POOL_WRITE_LOCKS_ASYNC.lock().await;
        match locks.get(&(dataflow_id, shared_memory_id.clone())) {
            Some(lock) => lock.clone(),
            None => {
                let lock = std::sync::Arc::new(tokio::sync::Mutex::new(()));
                locks.insert((dataflow_id, shared_memory_id.clone()), lock.clone());
                lock
            }
        }
    };
    let _guard = write_lock.lock().await;
    // Open + validate the mirror (all sync, no awaits while raw pointers
    // are live), then read the payload straight into the data region
    // under the seqlock — zero user-space copies on this side. The open
    // lives inside a block so the `Shmem` local (with its drop flag) is
    // consumed before the read await.
    let mut writer = {
        let shmem = ShmemConf::new().os_id(&shmem_name).open().map_err(|e| {
            CrossFrameError::with_ack(
                format!("cannot open mirror {shmem_name}: {e}"),
                dataflow_id,
                shared_memory_id.clone(),
                seq,
            )
        })?;
        let shmem_ptr = shmem.as_ptr();
        let magic8 = unsafe { std::slice::from_raw_parts(shmem_ptr, 8) };
        if magic8 != DORADMA_MAGIC {
            return Err(CrossFrameError::with_ack(
                format!("{shared_memory_id} header magic mismatch"),
                dataflow_id,
                shared_memory_id.clone(),
                seq,
            ));
        }
        // The frame `size` must equal the registered pool size recorded
        // in the mirror header: a short frame would otherwise publish an
        // even generation over a truncated tensor that readers accept as
        // complete (the segment-length bound alone cannot catch it, since
        // the segment is created at the registered size + header).
        let registered_size = {
            let json_len = unsafe { read_header_u64(shmem_ptr.add(8)) } as usize;
            let json_bytes =
                unsafe { std::slice::from_raw_parts(shmem_ptr.add(DORADMA_HEADER_SIZE), json_len) };
            serde_json::from_slice::<serde_json::Value>(json_bytes)
                .ok()
                .and_then(|v| v.get("size").and_then(|s| s.as_u64()))
                .unwrap_or(0) as usize
        };
        if size != registered_size {
            return Err(CrossFrameError::with_ack(
                format!(
                    "{shared_memory_id} frame size {size} != registered pool size {registered_size}"
                ),
                dataflow_id,
                shared_memory_id.clone(),
                seq,
            ));
        }
        let data_offset = unsafe { read_header_u64(shmem_ptr.add(16)) } as usize;
        // Checked add: `size` is wire-controlled (u64 read straight off
        // the socket), so `data_offset + size` can wrap to a small value
        // and pass the bounds check — then a `size`-byte slice would be
        // constructed past the mapping (UB; a remote-triggerable abort in
        // debug builds). Reject the overflow explicitly.
        let Some(end) = data_offset.checked_add(size) else {
            return Err(CrossFrameError::with_ack(
                format!("{shared_memory_id} data_offset {data_offset} + {size} overflows usize"),
                dataflow_id,
                shared_memory_id.clone(),
                seq,
            ));
        };
        if end > shmem.len() {
            return Err(CrossFrameError::with_ack(
                format!(
                    "{shared_memory_id} data_offset {data_offset} + {size} exceeds shmem size {}",
                    shmem.len()
                ),
                dataflow_id,
                shared_memory_id.clone(),
                seq,
            ));
        }
        DirectMirrorWriter::new(shmem, data_offset, size)
    };
    let dst = writer.data_slice_mut();
    if let Err(e) = stream.read_exact(dst).await {
        // The writer drops without `finish`: the seqlock generation stays
        // odd (in-progress) — readers reject the torn frame and the next
        // full write self-heals (see the NOTE on `DirectMirrorWriter`).
        // The ack info lets the caller fail the origin's pending write
        // instead of stranding it for the timeout.
        return Err(CrossFrameError::with_ack(
            format!("read payload: {e}"),
            dataflow_id,
            shared_memory_id,
            seq,
        ));
    }
    writer.finish();
    Ok(Some((dataflow_id, shared_memory_id, seq)))
}

/// Send one direct-TCP data frame to a peer's data listener (origin side).
/// Reuses a persistent connection per endpoint; a dead connection is
/// dropped and re-established lazily. The connection is taken out of the
/// map while in flight (a std `MutexGuard` cannot be held across an
/// `.await`), so concurrent writers to the same endpoint serialize on the
/// map lock instead — fine for the turn-based benchmark cadence. When
/// auth is configured, a freshly connected socket performs the token
/// handshake before the first frame.
pub(crate) async fn send_cross_data_frame(
    conns: &Arc<std::sync::Mutex<HashMap<std::net::SocketAddr, tokio::net::TcpStream>>>,
    endpoint: std::net::SocketAddr,
    dataflow_id: Uuid,
    shared_memory_id: &str,
    seq: u64,
    data: &[u8],
) -> Result<(), String> {
    use tokio::io::AsyncWriteExt;

    let mut stream = {
        // The guard is a temporary: it must be dropped before the connect
        // await below (a std MutexGuard is not Send across awaits).
        let existing = conns
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .remove(&endpoint);
        match existing {
            Some(stream) => stream,
            None => {
                let stream = tokio::time::timeout(
                    std::time::Duration::from_secs(5),
                    tokio::net::TcpStream::connect(endpoint),
                )
                .await
                .map_err(|_| format!("connect timeout to {endpoint}"))?
                .map_err(|e| format!("connect to {endpoint} failed: {e}"))?;
                // Auth handshake on fresh connections only. A rejected
                // handshake is an error on this side too — the caller
                // degrades to the zenoh relay.
                let mut stream = stream;
                if let Some(token) = cross_data_auth_token() {
                    tokio::time::timeout(
                        CROSS_DATA_READ_TIMEOUT,
                        auth_handshake_send(&mut stream, &token),
                    )
                    .await
                    .map_err(|_| format!("auth handshake timeout to {endpoint}"))?
                    .map_err(|e| format!("auth handshake to {endpoint} failed: {e}"))?;
                }
                stream
            }
        }
    };
    let mut buf = Vec::with_capacity(4 + 16 + 4 + shared_memory_id.len() + 8 + 8 + data.len());
    buf.extend_from_slice(&CROSS_DATA_MAGIC.to_be_bytes());
    buf.extend_from_slice(dataflow_id.as_bytes());
    buf.extend_from_slice(&(shared_memory_id.len() as u32).to_be_bytes());
    buf.extend_from_slice(shared_memory_id.as_bytes());
    buf.extend_from_slice(&seq.to_be_bytes());
    buf.extend_from_slice(&(data.len() as u64).to_be_bytes());
    let result = async {
        stream.write_all(&buf).await?;
        stream.write_all(data).await?;
        stream.flush().await?;
        Ok::<(), std::io::Error>(())
    }
    .await;
    if let Err(e) = result {
        // Dead connection — drop it (not re-inserted) so the next write
        // reconnects.
        return Err(format!("direct write to {endpoint} failed: {e}"));
    }
    conns
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .insert(endpoint, stream);
    Ok(())
}
