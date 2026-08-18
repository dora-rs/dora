//! Decoding and servicing the opaque bytes dora delivers.
//!
//! Everything dora knows about this transport is that an extension named
//! [`NAMESPACE`] exists and that opaque bytes must reach it. The bytes are
//! decoded here, into the extension's own protocol types — dora's wire
//! protocol carries no pool-shaped variant. See `docs/extensions.md`.

use super::*;
use dora_tensor_pool::protocol::{NAMESPACE, NodeRequest, NodeResponse, PeerMessage};

impl crate::Daemon {
    /// Route one [`InterDaemonEvent::ExtensionMessage`] to its extension.
    ///
    /// The `target_machine` gate is dora's, not the extension's: the topic is
    /// a dataflow-scope broadcast, so an addressed message must be dropped by
    /// every daemon it does not name. It was previously duplicated inside the
    /// register and free handlers.
    pub(crate) async fn handle_extension_message(
        &mut self,
        dataflow_id: Uuid,
        namespace: String,
        target_machine: Option<String>,
        payload: Vec<u8>,
    ) -> eyre::Result<()> {
        if let Some(target) = target_machine.as_deref()
            && target != self.machine_id.as_deref().unwrap_or("")
        {
            return Ok(());
        }
        if namespace != NAMESPACE {
            tracing::debug!("ignoring inter-daemon message for unknown extension {namespace:?}");
            return Ok(());
        }
        let message: PeerMessage = match postcard::from_bytes(&payload) {
            Ok(m) => m,
            Err(e) => {
                tracing::warn!("memory pool: undecodable peer message: {e}");
                return Ok(());
            }
        };
        self.handle_pool_peer_message(dataflow_id, message).await
    }

    async fn handle_pool_peer_message(
        &mut self,
        dataflow_id: Uuid,
        message: PeerMessage,
    ) -> eyre::Result<()> {
        match message {
            PeerMessage::Write {
                shared_memory_id,
                tensor_data,
                size,
                seq,
            } => {
                // Cross-machine path: pool mirrored here — write the data
                // straight into the DORADMA data region under the seqlock
                // protocol (receiver reads its local pool zero-copy).
                // Pools without a cross-machine entry (local pools, or a
                // daemon that is not this pool's mirror) drop the frame at
                // debug level: the write path publishes unconditionally, so
                // a non-mirror daemon sees every frame of every pool. A
                // genuinely missing mirror still warns inside
                // `write_cross_pool_data`.
                let is_cross = self
                    .pool
                    .tensor_pool
                    .is_cross(&dataflow_id.to_string(), &shared_memory_id);
                if !is_cross {
                    tracing::debug!(
                        pool = %shared_memory_id,
                        "memory pool: dropping write for a pool without a cross-machine entry"
                    );
                    return Ok(());
                }
                // The mirror write is a synchronous 61.44MB memcpy
                // (10-30ms) — off the event loop or it would stall
                // heartbeats, node replies and output delivery.
                let local_machine_id = self.machine_id.clone().unwrap_or_default();
                let session = self.zenoh_session.clone();
                let clock = self.clock.clone();
                let shm_provider = self.pool.shm_provider.clone();
                tokio::spawn(async move {
                    // `dataflow_id` (Uuid) is Copy; captured by copy.
                    let ok = write_cross_pool_data(
                        &dataflow_id,
                        &local_machine_id,
                        &shared_memory_id,
                        &tensor_data,
                        size,
                    );
                    // Remote commit ack: the origin's write reply waits
                    // for this, so its send_output notification cannot
                    // overtake the mirror write.
                    if let Err(e) = publish_pool_message(
                        &session,
                        &clock,
                        &dataflow_id,
                        None,
                        &PeerMessage::WriteAck {
                            shared_memory_id,
                            seq,
                            ok,
                            error: (!ok).then(|| {
                                "remote mirror write failed (missing or invalid segment)"
                                    .to_string()
                            }),
                        },
                        shm_provider.as_deref(),
                    )
                    .await
                    {
                        tracing::warn!("memory pool: failed to publish MemoryPoolWriteAck: {e}");
                    }
                });
                Ok(())
            }
            PeerMessage::WriteAck {
                shared_memory_id,
                seq,
                ok,
                error,
            } => {
                // Complete the synchronous cross-machine write: the mirror
                // daemon confirms the segment write, so the pending reply
                // (and the send_output notification that follows it) can
                // only fire after the remote data is visible.
                resolve_cross_write_ack(dataflow_id, shared_memory_id, seq, ok, error);
                Ok(())
            }
            PeerMessage::RegisterAck {
                shared_memory_id,
                ok,
                direct,
                data_port,
                data_addr,
                ..
            } => {
                // Complete a synchronous cross-machine register: hand the
                // ack to the spawned register task awaiting it (if any).
                if let Some(tx) = CROSS_REGISTER_PENDING
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .remove(&(dataflow_id, shared_memory_id))
                {
                    let _ = tx.send((ok, direct, data_port, data_addr));
                }
                Ok(())
            }
            PeerMessage::Register {
                origin_machine_id,
                shared_memory_id,
                shmem_name,
                size,
                dtype,
                shape,
                device,
                ..
            } => {
                // The envelope's `target_machine` already selected this
                // daemon (see `handle_extension_message`). The opt-in gate
                // keeps a daemon that never participates from mirroring
                // anything even when a peer tries to register with it.
                if !cross_machine_enabled() {
                    return Ok(());
                }
                // Never mirror for a dataflow this daemon has finished —
                // see [`MemoryPoolSubscriber`]. Gated on the subscriber
                // registry rather than `self.running`: the subscriber is
                // declared before the node build, while the dataflow has
                // not reached `self.running` yet, and the origin's sync
                // register fires from its node startup — so a `running`
                // check would reject legitimate registrations during a
                // slow build.
                let Some(dataflow_live) = self
                    .pool
                    .subscribers
                    .get(&dataflow_id)
                    .map(|s| s.dataflow_live.clone())
                else {
                    tracing::debug!(
                        "memory pool: ignoring RegisterPool for {shared_memory_id}: \
                         dataflow {dataflow_id} is no longer running"
                    );
                    return Ok(());
                };
                let session = self.zenoh_session.clone();
                let clock = self.clock.clone();
                // The gating above guarantees this daemon IS the target
                // machine, so its machine id is the mirror's namespace.
                // Lazily open the direct-TCP data listener now that this
                // daemon is actually mirroring something (daemons that
                // never participate in cross-machine pools stay closed).
                self.ensure_cross_data_listener().await;
                let local_machine_id = self.machine_id.clone();
                // Pool creation happens inside spawn (creation is millisecond-scale but publishing may Block)
                let tensor_pool = self.pool.tensor_pool.clone();
                let shm_provider = self.pool.shm_provider.clone();
                // Replicate the pool descriptor into the local extension
                // table before the mirror creation task runs, so a
                // receiver's daemon query resolves it from the first
                // frame (see `build_mirror_descriptor` for why the
                // sender-side store alone is insufficient). Owner is the
                // sender node id, derived from the pool id like
                // `cross_pool_shmem_name`; the sender node never runs on
                // this host, so the node-exit reclaim never fires —
                // FreePool drops the entry explicitly.
                let mirror_shmem_name = TensorPoolManager::cross_pool_shmem_name(
                    self.machine_id.as_deref().unwrap_or_default(),
                    &dataflow_id.to_string(),
                    &shared_memory_id,
                );
                if let Some(mirror_shmem_name) = &mirror_shmem_name {
                    // Fallible owner parse: the node id segment comes from
                    // the wire (`pool_{node}_{counter}`), and
                    // `NodeId::from(String)` panics on an invalid id
                    // (e.g. `pool_!_1` passes the path-component guard).
                    // Reject the registration instead of crashing the
                    // daemon (何勇 review 5302853212).
                    let Some(owner) = shared_memory_id
                        .strip_prefix("pool_")
                        .and_then(|s| s.rsplit_once('_'))
                        .map(|(node, _)| node)
                        .and_then(|node| node.parse::<NodeId>().ok())
                    else {
                        tracing::warn!(
                            "memory pool: rejecting RegisterPool with invalid pool id \
                             {shared_memory_id} (cannot derive a sender node id)"
                        );
                        return Ok(());
                    };
                    let ext_key = ExtensionKey {
                        dataflow_id: dataflow_id.to_string(),
                        namespace: "dora-tensor-pool".to_string(),
                        key: shared_memory_id.clone(),
                    };
                    let value =
                        build_mirror_descriptor(size, &dtype, &shape, mirror_shmem_name, &device);
                    if let Err(e) = self.extensions.store(ext_key, value, &owner) {
                        tracing::warn!(
                            "memory pool: failed to store mirror descriptor for {shared_memory_id}: {e}"
                        );
                    } else {
                        tracing::info!(
                            "memory pool: stored mirror descriptor for {shared_memory_id}"
                        );
                    }
                }
                // Advertise this daemon's direct-TCP data listener so the
                // origin can bypass the zenoh relay for per-frame writes.
                let data_port = self.pool.cross_data_listener_port;
                // Explicit dialable address override: the coordinator only
                // sees this daemon's WS source address, which is the wrong
                // dial target under NAT / multi-homed / same-host
                // coordinator deployment (e.g. 127.0.0.1). The deployer
                // sets DORA_MEMORY_POOL_DATA_ADDR (full `ip:port`) to the
                // address origins can actually reach.
                let data_addr = std::env::var("DORA_MEMORY_POOL_DATA_ADDR")
                    .ok()
                    .and_then(|s| match s.parse::<std::net::SocketAddr>() {
                        Ok(addr) => Some(addr),
                        Err(_) => {
                            tracing::warn!(
                                "memory pool: DORA_MEMORY_POOL_DATA_ADDR `{s}` is not an ip:port \
                                 address; ignoring"
                            );
                            None
                        }
                    });
                tokio::spawn(async move {
                    // Mirror allocation cap: the RegisterPool event's
                    // `size` comes from a remote daemon (untrusted
                    // cross-machine input). Without a cap, a buggy or
                    // corrupted peer could drive an unbounded /dev/shm
                    // allocation here (memory-exhaustion DoS). Matches
                    // the 1 GiB registration cap enforced on the local
                    // side; the error flows back through RegisterPoolAck.
                    let result = if size > 1024 * 1024 * 1024 {
                        Err(eyre::eyre!(
                            "cross-machine pool size {size} exceeds the 1 GiB mirror cap"
                        ))
                    } else {
                        create_cross_pool_shmem(
                            &dataflow_id,
                            local_machine_id.as_deref().unwrap_or_default(),
                            &shared_memory_id,
                            size,
                            &dtype,
                            &shape,
                            &device,
                            Some(&shmem_name),
                        )
                    };
                    let (mut ok, mut error) = match result {
                        Ok(()) => (true, None),
                        Err(e) => (false, Some(e.to_string())),
                    };
                    // Same-host detection: if this daemon can open the
                    // sender's segment (shared /dev/shm), readers can read
                    // it directly and the origin can skip the data push.
                    let mut direct = ok && ShmemConf::new().os_id(&shmem_name).open().is_ok();
                    if ok {
                        // Track the pool's other machine (the origin) so
                        // the targeted free reaches it, mirroring the
                        // origin's `{pool -> target}` entry, and record the
                        // segment just created so the finish-time drain can
                        // unlink exactly it.
                        tensor_pool.register_cross_pool(
                            dataflow_id.to_string(),
                            shared_memory_id.clone(),
                            origin_machine_id,
                            mirror_shmem_name.clone(),
                        );
                        // The dataflow may have finished while this task
                        // ran; see [`MemoryPoolSubscriber`]. Registering
                        // behind the finish-time drain would strand the
                        // segment for the daemon's lifetime, so undo it.
                        if dataflow_live.load(atomic::Ordering::SeqCst) {
                            tracing::info!(
                                "memory pool: mirrored cross-machine pool {shared_memory_id} (size {size})"
                            );
                        } else {
                            tensor_pool
                                .unregister_cross_pool(&dataflow_id.to_string(), &shared_memory_id);
                            if let Some(name) = &mirror_shmem_name {
                                remove_cross_pool_shmem(name);
                            }
                            // Report the failure rather than returning
                            // early: the origin would otherwise sit
                            // through its 5s ack timeout three times over
                            // for a dataflow that is already gone.
                            ok = false;
                            direct = false;
                            error = Some(format!(
                                "dataflow {dataflow_id} finished before the mirror was registered"
                            ));
                        }
                    } else {
                        tracing::warn!(
                            "memory pool: failed to mirror pool {shared_memory_id}: {}",
                            error.as_deref().unwrap_or("unknown")
                        );
                    }
                    if let Err(e) = publish_pool_message(
                        &session,
                        &clock,
                        &dataflow_id,
                        None,
                        &PeerMessage::RegisterAck {
                            shared_memory_id,
                            ok,
                            direct,
                            error,
                            data_port,
                            data_addr,
                        },
                        shm_provider.as_deref(),
                    )
                    .await
                    {
                        tracing::warn!("memory pool: failed to publish RegisterPoolAck: {e}");
                    }
                });
                Ok(())
            }
            PeerMessage::Free { shared_memory_id } => {
                self.pool
                    .tensor_pool
                    .unregister_cross_pool(&dataflow_id.to_string(), &shared_memory_id);
                // The replicated mirror descriptor goes away with the
                // pool — through the notify path so receiver nodes on
                // this machine learn via `drain_dropped` and release
                // their mappings (a bare drop_key discards the
                // touched-node set and strands them, 何勇 review
                // 5302853212).
                let ext_key = ExtensionKey {
                    dataflow_id: dataflow_id.to_string(),
                    namespace: "dora-tensor-pool".to_string(),
                    key: shared_memory_id.clone(),
                };
                let dataflow = self.running.get(&dataflow_id);
                drop_extension_and_notify(&mut self.extensions, dataflow, &ext_key, &self.clock);
                // Drop the per-pool relay write lock: keyed by a fresh
                // per-dataflow UUID like the sibling cross-write maps,
                // so entries would otherwise accumulate for every freed
                // relay-path pool (unbounded on a long-running daemon
                // when direct TCP is unavailable).
                CROSS_POOL_WRITE_LOCKS
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .remove(&(dataflow_id, shared_memory_id.to_string()));
                // The direct-TCP twin lock accumulates the same way: a
                // direct-path pool freed here would otherwise keep its
                // entry until dataflow finish (only the relay lock was
                // drained on free — self-review, 2026-08-16).
                CROSS_POOL_WRITE_LOCKS_ASYNC
                    .lock()
                    .await
                    .remove(&(dataflow_id, shared_memory_id.to_string()));
                // Same machine-qualified id as `create_cross_pool_shmem`.
                if !unlink_cross_pool_segment(
                    self.machine_id.as_deref().unwrap_or_default(),
                    &dataflow_id.to_string(),
                    &shared_memory_id,
                ) {
                    return Ok(());
                }
                tracing::info!("memory pool: freed cross-machine pool {shared_memory_id}");
                Ok(())
            }
        }
    }

    /// Route one `DaemonRequest::ExtensionRequest` to its extension and hand
    /// the opaque reply back to the calling node.
    pub(crate) async fn handle_extension_request(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        namespace: String,
        payload: Vec<u8>,
        reply_sender: tokio::sync::oneshot::Sender<DaemonReply>,
    ) -> eyre::Result<()> {
        if namespace != NAMESPACE {
            let _ = reply_sender.send(DaemonReply::Result(Err(format!(
                "no extension registered under {namespace:?} on this daemon"
            ))));
            return Ok(());
        }
        let request: NodeRequest = match postcard::from_bytes(&payload) {
            Ok(r) => r,
            Err(e) => {
                let _ = reply_sender.send(DaemonReply::Result(Err(format!(
                    "undecodable tensor-pool request: {e}"
                ))));
                return Ok(());
            }
        };
        self.handle_pool_node_request(dataflow_id, node_id, request, reply_sender)
            .await
    }

    async fn handle_pool_node_request(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        request: NodeRequest,
        reply_sender: tokio::sync::oneshot::Sender<DaemonReply>,
    ) -> eyre::Result<()> {
        match request {
            NodeRequest::Write {
                shared_memory_id,
                size,
            } => {
                // Only cross-machine pools need forwarding: the origin
                // records the pool in the cross_pools table when the
                // register ack arrives (before replying to the node), so a
                // pool without an entry is local-only — every remote daemon
                // would drop its frames at debug level anyway. Gate the
                // 61.44MB serialize + WAN put on the entry; local pools
                // just get the Ok reply.
                //
                // Failures are logged loudly: a dropped publish strands
                // remote readers with a never-ready mirror pool.
                // Must match the subscriber's wire format: a
                // `Timestamped<InterDaemonEvent>` (the same framing the
                // regular inter-daemon event path uses) — the framing and
                // publish live in `publish_pool_message`.
                // Run serialize + declare + put all off the event loop:
                // postcard::serialize of the 61.44MB payload takes hundreds
                // of ms (3s+ in debug builds), and with Block congestion
                // control a slow/stalled inter-daemon link blocks
                // declare_publisher() itself — either one wedges the daemon
                // event loop (heartbeats + node replies + output delivery
                // included), backing up the event channels until the
                // sender's WritePinnedMemory hangs forever.
                if !self
                    .pool
                    .tensor_pool
                    .is_cross(&dataflow_id.to_string(), &shared_memory_id)
                {
                    // Reply must stay byte-identical to the forwarded
                    // path below (Result(Ok(()))) — the node cannot
                    // distinguish a gated local write from a forwarded
                    // cross-machine one.
                    let _ = reply_sender.send(DaemonReply::Result(Ok(())));
                    return Ok(());
                }
                // Shared-memory-reference write: the node sends only
                // (id, size) metadata; the tensor is read from the
                // sender's segment (name recorded at registration). The
                // request therefore stays KB-scale and cross-machine
                // pools are no longer bounded by the node→daemon request
                // cap (MAX_MESSAGE_BYTES). Only the cheap name resolution
                // happens here — the actual read (a full-size allocation
                // plus copy) runs inside the spawned task below, so a
                // large frame never blocks the event loop (heartbeats,
                // node replies, output delivery).
                let shmem_name = {
                    // Resolve the sender's segment. The deterministic
                    // machine-qualified auto-name is tried first: the
                    // register-time initial push arrives BEFORE the
                    // python's local registration completes, so the
                    // daemon table does not hold the entry yet. The
                    // table lookup covers explicit `name=` pools (whose
                    // names are not derivable).
                    self.machine_id
                        .as_deref()
                        .filter(|m| !m.is_empty())
                        .and_then(|m| {
                            TensorPoolManager::cross_pool_shmem_name(
                                m,
                                &dataflow_id.to_string(),
                                &shared_memory_id,
                            )
                        })
                        .or_else(|| {
                            self.pool
                                .tensor_pool
                                .read_tensor_pool(
                                    &dora_tensor_pool::TensorPoolId {
                                        dataflow_id: dataflow_id.to_string(),
                                        id: shared_memory_id.clone(),
                                    },
                                    node_id.as_ref(),
                                )
                                .and_then(|m| m.shared_memory_name)
                                .filter(|n| !n.is_empty())
                        })
                };
                let session = self.zenoh_session.clone();
                let clock = self.clock.clone();
                let shm_provider = self.pool.shm_provider.clone();
                // Remote commit acknowledgement: the reply is withheld
                // until the mirror daemon confirms the segment write
                // (MemoryPoolWriteAck). Otherwise the send_output
                // notification that follows this write can overtake the
                // tensor data and the receiver returns the previous
                // stable frame.
                let seq = {
                    let mut seqs = CROSS_WRITE_SEQ.lock().unwrap_or_else(|e| e.into_inner());
                    let counter = seqs
                        .entry((dataflow_id, shared_memory_id.clone()))
                        .or_insert(0);
                    *counter += 1;
                    *counter
                };
                CROSS_WRITE_PENDING
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .insert((dataflow_id, shared_memory_id.clone(), seq), reply_sender);
                // The seq the pending entry is live under. Starts at `seq`;
                // a direct-TCP → zenoh failover re-keys the entry to a fresh
                // `relay_seq` below and advances this to match, so the
                // safety-net timeout removes the entry under whatever seq is
                // actually live. Keying the timeout to the pre-failover seq
                // (as it did) makes it a no-op after a re-key, leaving the
                // node's write blocked forever (#3193).
                let effective_seq = Arc::new(std::sync::atomic::AtomicU64::new(seq));
                let timeout_seq = effective_seq.clone();
                let timeout_pool = shared_memory_id.clone();
                // Direct-TCP data plane: when the register ack reported a
                // data listener, writes bypass the zenoh relay entirely —
                // one user-space copy on this side (segment → send
                // buffer), the mirror daemon reads the stream straight
                // into the mirror segment. The commit ack still arrives
                // via zenoh, so the pending machinery is unchanged. Falls
                // back to zenoh when no endpoint is known or the send
                // fails.
                let direct_endpoint = self
                    .pool
                    .cross_data_endpoints
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .get(&(dataflow_id, shared_memory_id.clone()))
                    .copied();
                let cross_data_conns = self.pool.cross_data_conns.clone();
                tokio::spawn(async move {
                    // The segment read (a full-size allocation plus copy)
                    // runs here, off the event loop.
                    let tensor_data = match shmem_name {
                        Some(name) => match read_pool_segment_data(&name, size) {
                            Ok(data) => data,
                            Err(e) => {
                                resolve_cross_write_ack(
                                    dataflow_id,
                                    shared_memory_id.clone(),
                                    seq,
                                    false,
                                    Some(format!(
                                        "cross-machine write: failed to read sender segment: {e}"
                                    )),
                                );
                                return;
                            }
                        },
                        None => {
                            resolve_cross_write_ack(
                                dataflow_id,
                                shared_memory_id.clone(),
                                seq,
                                false,
                                Some(format!(
                                    "pool {shared_memory_id} has no local segment to read the write from"
                                )),
                            );
                            return;
                        }
                    };
                    let mut relay_seq = seq;
                    if let Some(endpoint) = direct_endpoint {
                        match send_cross_data_frame(
                            &cross_data_conns,
                            endpoint,
                            dataflow_id,
                            &shared_memory_id,
                            seq,
                            &tensor_data,
                        )
                        .await
                        {
                            Ok(()) => {
                                // The commit ack arrives via zenoh. If this
                                // pool was degraded, the direct path just
                                // recovered — report exactly once.
                                if note_direct_recovered(dataflow_id, &shared_memory_id) {
                                    tracing::info!(
                                        "memory pool: direct TCP write to {endpoint} recovered; \
                                         pool {shared_memory_id} back on the direct path"
                                    );
                                }
                                return;
                            }
                            Err(e) => {
                                // Warn exactly once per pool while degraded:
                                // the zenoh fallback is the steady state on a
                                // broken link, and a per-frame warn would
                                // flood the log. Recovery is reported once by
                                // the Ok arm above.
                                if note_direct_degraded(dataflow_id, &shared_memory_id) {
                                    tracing::warn!(
                                        "memory pool: direct TCP write failed ({e}); \
                                         pool {shared_memory_id} degraded to the zenoh relay \
                                         (warned once; recovery will be logged)"
                                    );
                                }
                                // The direct attempt for this seq is
                                // abandoned. On a mid-frame disconnect the
                                // mirror has already parsed the header and
                                // publishes ok:false(old seq) the instant
                                // its read fails — which would win the race
                                // against the relay's ok:true (published
                                // only after the re-sent frame is memcpy'd)
                                // and fail the node's write even though the
                                // zenoh fallback delivers the frame
                                // byte-for-byte. Re-key the pending entry to
                                // a fresh seq for the relay hop so the stale
                                // ok:false is dropped by the first-wins
                                // resolver, keep the per-pool counter
                                // monotonic so the next write cannot collide
                                // with this re-keyed seq (bot review
                                // 5307022693), and advance `effective_seq`
                                // so the safety-net timeout follows it (#3193).
                                match rekey_cross_write_to_relay(
                                    dataflow_id,
                                    &shared_memory_id,
                                    seq,
                                    &effective_seq,
                                ) {
                                    Some(new_seq) => relay_seq = new_seq,
                                    // Nothing was pending under `seq`, so this
                                    // write is already resolved: the mirror's
                                    // stale ok:false won the race, the safety
                                    // net fired, or `finish_dataflow` drained
                                    // it. Falling through would publish the
                                    // whole payload to the mirror anyway — it
                                    // would memcpy the frame and advance the
                                    // seqlock, so a receiver there reads as
                                    // "stable" a frame this daemon already told
                                    // the sender had failed, and a full-size WAN
                                    // transfer burns with nobody awaiting it.
                                    None => return,
                                }
                            }
                        }
                    }
                    let message = PeerMessage::Write {
                        shared_memory_id: shared_memory_id.clone(),
                        tensor_data,
                        size,
                        seq: relay_seq,
                    };
                    if let Err(e) = publish_pool_message(
                        &session,
                        &clock,
                        &dataflow_id,
                        None,
                        &message,
                        shm_provider.as_deref(),
                    )
                    .await
                    {
                        tracing::error!("memory pool: failed to forward WriteMemoryPool: {e}");
                        // No ack will ever arrive — fail the node's write
                        // loudly instead of leaving it hanging.
                        if let Some(tx) = CROSS_WRITE_PENDING
                            .lock()
                            .unwrap_or_else(|e| e.into_inner())
                            .remove(&(dataflow_id, shared_memory_id, relay_seq))
                        {
                            let _ = tx.send(DaemonReply::Result(Err(format!(
                                "cross-machine write failed to reach the remote daemon: {e}"
                            ))));
                        }
                    }
                });
                // Safety net: if the ack never arrives (peer restart,
                // lost ack), fail the write rather than hang the node.
                // `fire_cross_write_timeout` reads the effective seq itself,
                // under the pending-map guard, so the removal follows any
                // direct-TCP → relay re-key that happened meanwhile — including
                // one that lands while this task is waking up (#3193).
                tokio::spawn(async move {
                    tokio::time::sleep(CROSS_WRITE_ACK_TIMEOUT).await;
                    fire_cross_write_timeout(dataflow_id, &timeout_pool, &timeout_seq);
                });
            }
            NodeRequest::RegisterCrossMachine {
                shared_memory_id,
                shmem_name,
                size,
                dtype,
                shape,
                device,
                machine_id,
            } => {
                // Resolve the machine via the coordinator, publish
                // RegisterPool over the memory-pool topic, and await the
                // remote RegisterPoolAck before replying (sync register).
                // The ack is delivered through this daemon's own event
                // loop (`handle_inter_daemon_event`), so awaiting it on
                // the loop itself would deadlock — the loop could never
                // process the ack. Run the whole flow in a spawned task.
                if !cross_machine_enabled() {
                    tracing::warn!(
                        "memory pool: cross-machine register requested but the cross-machine \
                         data plane is disabled (set DORA_MEMORY_POOL_CROSS_MACHINE=1 on every \
                         daemon to enable it)"
                    );
                    let _ = reply_sender.send(node_reply(&NodeResponse::CrossMachineRegistered {
                        result: Err("cross-machine pools are disabled on this daemon \
                             (DORA_MEMORY_POOL_CROSS_MACHINE not set)"
                            .to_string()),
                        direct: false,
                    }));
                    return Ok(());
                }
                let topic = dataflow_extension_topic(&dataflow_id, NAMESPACE);
                let session = self.zenoh_session.clone();
                let clock = self.clock.clone();
                let coordinator_sender = self.coordinator_sender.clone();
                // The origin machine id for the RegisterPool event: the
                // mirror records `{pool -> origin}` and later frees
                // toward it. `self` is not reachable inside the spawn.
                let origin_machine_id = self.machine_id.clone();
                let tensor_pool = self.pool.tensor_pool.clone();
                let shm_provider = self.pool.shm_provider.clone();
                let cross_data_endpoints = self.pool.cross_data_endpoints.clone();
                tokio::spawn(async move {
                    // Clone for the post-flow cleanup below: the inner
                    // async block moves `shared_memory_id` into the pool
                    // map on the success path.
                    let cleanup_pool_id = shared_memory_id.clone();
                    // Same-host flag: set true when the remote ack
                    // confirmed it can open our segment directly.
                    let mut direct = false;
                    let reply = async {
                        // Resolve the target machine through the
                        // coordinator. No coordinator connection means
                        // the machine cannot be resolved either — same
                        // warn-and-skip.
                        let Some(coordinator_sender) = coordinator_sender.as_ref() else {
                            return Err(format!(
                                r#"machine "{machine_id}" could not be resolved: no such machine on the coordinator (or no coordinator); cross-machine memory pool not created"#
                            ));
                        };
                        let Some(peer_addr) =
                            coordinator::resolve_machine(coordinator_sender, &clock, &machine_id)
                                .await
                        else {
                            return Err(format!(
                                r#"machine "{machine_id}" could not be resolved: no such machine on the coordinator (or no coordinator); cross-machine memory pool not created"#
                            ));
                        };
                        // Publish RegisterPool and await the ack, retrying on
                        // timeout: the remote daemon's memory-pool
                        // subscription is established in parallel during
                        // dataflow startup, so the first RegisterPool can
                        // be published before the subscription exists and
                        // be lost (no subscriber yet) — observed as the
                        // register timing out while the remote never
                        // received the event. An explicit ok=false reply
                        // is not retried (the remote was reached and
                        // reported a creation failure).
                        let mut reply = Err(format!(
                            r#"machine "{machine_id}" resolved but remote pool creation failed: RegisterPoolAck timed out (5s); cross-machine memory pool not created"#
                        ));
                        for attempt in 0..3 {
                            // Register the ack channel BEFORE publishing:
                            // the remote acks as soon as it receives
                            // RegisterPool, so a late registration could
                            // race the ack and spuriously time out.
                            let (ack_tx, ack_rx) = oneshot::channel();
                            CROSS_REGISTER_PENDING
                                .lock()
                                .unwrap_or_else(|e| e.into_inner())
                                .insert((dataflow_id, shared_memory_id.clone()), ack_tx);
                            let register = PeerMessage::Register {
                                origin_machine_id: origin_machine_id.clone().unwrap_or_default(),
                                shared_memory_id: shared_memory_id.clone(),
                                shmem_name: shmem_name.clone(),
                                size,
                                dtype: dtype.clone(),
                                shape: shape.clone(),
                                device: device.clone(),
                            };
                            let payload = match postcard::to_allocvec(&register) {
                                Ok(payload) => payload,
                                Err(e) => {
                                    tracing::error!("memory pool: encoding Register failed: {e}");
                                    return Err(format!("Register encoding failed: {e}"));
                                }
                            };
                            let serialized = match (Timestamped {
                                inner: InterDaemonEvent::ExtensionMessage {
                                    dataflow_id,
                                    namespace: NAMESPACE.to_string(),
                                    target_machine: Some(machine_id.clone()),
                                    payload,
                                },
                                timestamp: clock.new_timestamp(),
                            })
                            .serialize()
                            {
                                Ok(serialized) => serialized,
                                Err(e) => {
                                    tracing::error!(
                                        "memory pool: postcard serialize RegisterPool failed: {e}"
                                    );
                                    return Err(format!(
                                        "RegisterPool serialization failed: {e}"
                                    ));
                                }
                            };
                            let publisher = match session
                                .declare_publisher(topic.clone())
                                .congestion_control(CongestionControl::Block)
                                // Remote-only: the local echo of RegisterPool
                                // would fail to mirror (EEXIST — this node
                                // already created the pool) and publish a
                                // false ok=false RegisterPoolAck that beats
                                // the remote's real ack, failing every sync
                                // register.
                                .allowed_destination(Locality::Remote)
                                .await
                            {
                                Ok(publisher) => publisher,
                                Err(e) => {
                                    tracing::error!(
                                        "memory pool: declare_publisher({topic}) failed: {e}"
                                    );
                                    return Err(format!(
                                        "RegisterPool publish failed (declare_publisher): {e}"
                                    ));
                                }
                            };
                            // RegisterPool is a control notification — go
                            // over zenoh SHM when available (same-host
                            // zero-copy; cross-host the transport copies),
                            // falling back to the plain payload on any
                            // alloc/size failure.
                            let payload_len = serialized.len();
                            let put_result = if let Some(provider) = shm_provider.as_ref() {
                                // Synchronous wait: KB-scale control payload,
                                // microsecond allocation. `alloc` guarantees
                                // a buffer of at least `payload_len` bytes.
                                match provider.alloc(payload_len).wait() {
                                    Ok(mut buf) => {
                                        let buf_slice: &mut [u8] = buf.as_mut();
                                        buf_slice[..payload_len].copy_from_slice(&serialized);
                                        let payload: ZBytes = buf.into();
                                        publisher.put(payload).await
                                    }
                                    Err(e) => {
                                        tracing::warn!(
                                            "memory pool: SHM alloc failed ({e}), \
                                             falling back to regular payload"
                                        );
                                        publisher.put(serialized).await
                                    }
                                }
                            } else {
                                publisher.put(serialized).await
                            };
                            if let Err(e) = put_result {
                                tracing::error!(
                                    "memory pool: publish RegisterPool to {topic} failed: {e}"
                                );
                                return Err(format!("RegisterPool publish failed: {e}"));
                            }
                            match tokio::time::timeout(
                                coordinator::CROSS_REGISTER_TIMEOUT,
                                ack_rx,
                            )
                            .await
                            {
                                Ok(Ok((true, ack_direct, ack_data_port, ack_data_addr))) => {
                                    // Direct-TCP data plane: remember the
                                    // mirror's data listener so per-frame
                                    // writes bypass the zenoh relay. The
                                    // explicitly advertised address wins —
                                    // the coordinator-derived one
                                    // (`peer_addr.ip()`, the mirror daemon's
                                    // WS source address) is the wrong dial
                                    // target under NAT / multi-homed /
                                    // same-host coordinator deployment.
                                    let endpoint = ack_data_addr.or_else(|| {
                                        ack_data_port.map(|data_port| {
                                            std::net::SocketAddr::new(
                                                peer_addr.ip(),
                                                data_port,
                                            )
                                        })
                                    });
                                    if let Some(endpoint) = endpoint {
                                        cross_data_endpoints
                                            .lock()
                                            .unwrap_or_else(|e| e.into_inner())
                                            .insert(
                                                (dataflow_id, shared_memory_id.clone()),
                                                endpoint,
                                            );
                                    }
                                    // No mirror name: this is the origin
                                    // side, which created no segment of its
                                    // own — the mirror lives on the target
                                    // machine. The sender node's live local
                                    // segment is tracked separately, by the
                                    // `register_tensor_pool` below.
                                    tensor_pool.register_cross_pool(
                                        dataflow_id.to_string(),
                                        shared_memory_id.clone(),
                                        machine_id,
                                        None,
                                    );
                                    // Make the explicit `name=` segment
                                    // discoverable for the
                                    // shared-memory-reference write path:
                                    // `write_pinned_memory` falls back to
                                    // this table when the derived
                                    // auto-name does not match the actual
                                    // segment (the register-time initial
                                    // push arrives right after this ack,
                                    // so the entry must exist before the
                                    // node's first write).
                                    tensor_pool.register_tensor_pool(
                                        dora_tensor_pool::TensorPoolId {
                                            dataflow_id: dataflow_id.to_string(),
                                            id: shared_memory_id.clone(),
                                        },
                                        dora_tensor_pool::TensorPoolMetadata {
                                            size,
                                            dtype: dtype.clone(),
                                            shape: shape
                                                .iter()
                                                .map(|s| *s as usize)
                                                .collect(),
                                            shared_memory_name: Some(shmem_name.clone()),
                                            buffer_id: Some(shared_memory_id.clone()),
                                            ..Default::default()
                                        },
                                        node_id.to_string(),
                                        HashSet::new(),
                                    )
                                    .unwrap_or_else(|e| {
                                        tracing::warn!(
                                            "memory pool: failed to record {} in the pool table: {e}",
                                            shared_memory_id
                                        );
                                    });
                                    reply = Ok(());
                                    direct = ack_direct;
                                    break;
                                }
                                Ok(Ok((false, _, _, _))) => {
                                    reply = Err(format!(
                                        r#"machine "{machine_id}" resolved but remote pool creation failed: remote returned ok=false; cross-machine memory pool not created"#
                                    ));
                                    break;
                                }
                                Ok(Err(_)) => {
                                    reply = Err(format!(
                                        r#"machine "{machine_id}" resolved but remote pool creation failed: ack channel closed (remote daemon disconnected); cross-machine memory pool not created"#
                                    ));
                                    break;
                                }
                                Err(_) => {
                                    reply = Err(format!(
                                        r#"machine "{machine_id}" resolved but remote pool creation failed: RegisterPoolAck timed out (5s); cross-machine memory pool not created"#
                                    ));
                                    if attempt < 2 {
                                        tracing::warn!(
                                            "memory pool: RegisterPool attempt {} for {shared_memory_id} timed out — remote subscription may not be ready yet, retrying",
                                            attempt + 1
                                        );
                                        continue;
                                    }
                                }
                            }
                        }
                        reply
                    }
                    .await;
                    // Drop the pending ack entry if the ack never arrived
                    // (publish failure or timeout); on the success path the
                    // ack delivery already removed it, so this is a no-op.
                    CROSS_REGISTER_PENDING
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .remove(&(dataflow_id, cleanup_pool_id));
                    if let Err(err) = &reply {
                        tracing::warn!("memory pool: cross-machine register failed: {err}");
                    }
                    let _ = reply_sender.send(node_reply(&NodeResponse::CrossMachineRegistered {
                        result: reply,
                        direct,
                    }));
                });
            }
            NodeRequest::Free { shared_memory_id } => {
                // Drop the cross_pools entry and, when the pool was
                // cross-machine, publish the targeted FreePool and unlink
                // this machine's mirror.
                let peer = self
                    .pool
                    .tensor_pool
                    .unregister_cross_pool(&dataflow_id.to_string(), &shared_memory_id)
                    .map(|entry| entry.peer_machine);
                if let Some(peer) = &peer {
                    release_cross_pool(
                        &self.zenoh_session,
                        &self.clock,
                        &dataflow_id,
                        self.machine_id.as_deref().unwrap_or_default(),
                        peer,
                        &shared_memory_id,
                        self.pool.shm_provider.as_deref(),
                    )
                    .await;
                }
                // A cross-machine registration with an explicit `name=`
                // recorded an entry in the pool table (so the write path
                // resolves the segment). Remove it here — and let
                // `free_tensor_pool` unlink the *actual* sender segment
                // (the metadata's `shared_memory_name` is the real
                // `name=`, which the derived-name unlink in
                // `release_cross_pool` misses, leaving the /dev/shm
                // object behind). Not-found (local pools, auto-named
                // cross-machine pools) is fine — the table is only
                // populated on the explicit-name path (何勇 review
                // 5302853212).
                if let Err(e) = self.pool.tensor_pool.free_tensor_pool(
                    &dora_tensor_pool::TensorPoolId {
                        dataflow_id: dataflow_id.to_string(),
                        id: shared_memory_id.clone(),
                    },
                    node_id.as_ref(),
                ) {
                    tracing::debug!("memory pool: no pool-table entry for {shared_memory_id}: {e}");
                }
                // Drop this machine's replicated mirror descriptor. The
                // sender-free path reaches it via the FreePool event; the
                // receiver-free path lands here directly and would
                // otherwise leave the descriptor (and any receiver's
                // ExtensionDropped notification) until dataflow finish —
                // asymmetric with the FreePool path (self-review,
                // 2026-08-16). A no-op for pools whose registration never
                // completed (drop_key returns None).
                let ext_key = ExtensionKey {
                    dataflow_id: dataflow_id.to_string(),
                    namespace: "dora-tensor-pool".to_string(),
                    key: shared_memory_id.clone(),
                };
                let dataflow = self.running.get(&dataflow_id);
                drop_extension_and_notify(&mut self.extensions, dataflow, &ext_key, &self.clock);
                let _ = reply_sender.send(DaemonReply::Result(Ok(())));
            }
        }
        Ok(())
    }
    /// Lazily start this daemon's direct-TCP data listener (mirror side):
    /// the listener opens only when the first `RegisterPool` asks this
    /// daemon to mirror a pool — daemons that never participate in
    /// cross-machine pools do not open the port. The bound port is
    /// reported to origins in `RegisterPoolAck.data_port`. The bind
    /// address is configurable (`DORA_MEMORY_POOL_DATA_BIND`, default
    /// `0.0.0.0`) for hosts where the listener must not sit on every
    /// interface.
    async fn ensure_cross_data_listener(&mut self) {
        if !cross_machine_enabled() {
            return;
        }
        if self.pool.cross_data_listener_port.is_some() {
            return;
        }
        let port = std::env::var(CROSS_DATA_PORT_ENV)
            .ok()
            .and_then(|p| p.parse().ok())
            .unwrap_or(CROSS_DATA_PORT_DEFAULT);
        let bind: String = std::env::var(CROSS_DATA_BIND_ENV)
            .unwrap_or_else(|_| CROSS_DATA_BIND_DEFAULT.to_string());
        let listener = match tokio::net::TcpListener::bind((bind.as_str(), port)).await {
            Ok(l) => l,
            Err(e) => {
                tracing::warn!(
                    "memory pool: direct-TCP data listener bind failed on {bind}:{port}: {e}"
                );
                return;
            }
        };
        self.pool.cross_data_listener_port = listener.local_addr().ok().map(|a| a.port());
        tracing::info!(
            "memory pool: direct-TCP data listener on port {:?}",
            self.pool.cross_data_listener_port
        );

        let tensor_pool = self.pool.tensor_pool.clone();
        let machine_id = self.machine_id.clone().unwrap_or_default();
        let session = self.zenoh_session.clone();
        let clock = self.clock.clone();
        let shm_provider = self.pool.shm_provider.clone();
        // Auth is read per connection (env lookup), so the daemon needs no
        // state here beyond the option the handshake checks.
        tokio::spawn(async move {
            loop {
                let Ok((stream, peer)) = listener.accept().await else {
                    // Transient errors (ECONNABORTED) are fine to retry
                    // immediately, but a persistent one (e.g. EMFILE/ENFILE
                    // fd exhaustion) would otherwise spin at 100% CPU —
                    // bound the retry with a short sleep.
                    tokio::time::sleep(std::time::Duration::from_millis(50)).await;
                    continue;
                };
                tracing::debug!("memory pool: direct-TCP data connection from {peer}");
                let (tensor_pool, machine_id, session, clock, shm_provider) = (
                    tensor_pool.clone(),
                    machine_id.clone(),
                    session.clone(),
                    clock.clone(),
                    shm_provider.clone(),
                );
                tokio::spawn(async move {
                    let mut stream = stream;
                    // Auth handshake first, once per connection: a peer
                    // without the shared token is dropped before any frame
                    // can touch a mirror segment. Only when the daemon
                    // itself has a token configured (a token-less daemon
                    // accepts token-less peers — the handshake is a
                    // configured-deployment option, both ends set it or
                    // neither does).
                    if let Err(e) = cross_data_auth_gate(&mut stream).await {
                        tracing::warn!("memory pool: direct-TCP auth rejected for {peer}: {e}");
                        return;
                    }
                    loop {
                        match serve_cross_data_frame(
                            &tensor_pool,
                            &machine_id,
                            &session,
                            &clock,
                            shm_provider.as_deref(),
                            &mut stream,
                        )
                        .await
                        {
                            Ok(true) => continue,
                            Ok(false) => break, // clean EOF
                            Err(e) => {
                                tracing::warn!(
                                    "memory pool: direct-TCP data connection closed: {e}"
                                );
                                break;
                            }
                        }
                    }
                });
            }
        });
    }

    /// Subscribe to this dataflow's extension topic for the cross-machine
    /// messages arriving over zenoh.
    pub(crate) fn pool_subscribe_dataflow(&mut self, dataflow_id: Uuid) {
        // Subscribe to the dataflow memory-pool topic for cross-machine
        // events arriving through Zenoh (WriteMemoryPool, RegisterPool,
        // RegisterPoolAck, FreePool). Declared FIRST, before the node
        // build: the sender's sync register fires from its node startup,
        // and the node build (pip install etc.) can take tens of seconds —
        // a subscription declared after the build makes the register's
        // retries land before any subscriber exists (observed: RegisterPool
        // published into the void, ack timeout). The whole
        // declare+receive loop runs OFF the event loop: with a degraded
        // inter-daemon link, declare_subscriber() itself can block, which
        // would otherwise wedge this spawn handler and stall every
        // subsequent event (WriteMemoryPool included) — the sender then
        // hangs on its daemon reply forever. Skipped entirely when the
        // cross-machine data plane is not enabled.
        if cross_machine_enabled() {
            let mp_topic = dataflow_extension_topic(&dataflow_id, NAMESPACE);
            let mp_session = self.zenoh_session.clone();
            let mp_events_tx = self.events_tx.clone();
            let subscriber = tokio::spawn(async move {
                let Ok(subscriber) = mp_session.declare_subscriber(&mp_topic).await else {
                    tracing::warn!(
                        "memory pool: declare_subscriber({mp_topic}) failed; \
                         cross-machine pool reads will not see remote writes"
                    );
                    return;
                };
                while let Ok(sample) = subscriber.recv_async().await {
                    let bytes = sample.payload().to_bytes();
                    if let Ok(event) =
                        Timestamped::<InterDaemonEvent>::deserialize_inter_daemon_event(&bytes)
                    {
                        tracing::info!("memory pool: received inter-daemon event on {mp_topic}");
                        let _ = mp_events_tx
                            .send(Timestamped {
                                inner: Event::Daemon(event.inner),
                                timestamp: event.timestamp,
                            })
                            .await;
                    }
                }
            });
            // Retain the handle: the loop has no shutdown branch, so an
            // abandoned task would keep the subscriber, its session clone,
            // and its event sender alive for the daemon's lifetime.
            // Aborted in `finish_dataflow` and on the failed-spawn path.
            self.pool.subscribers.insert(
                dataflow_id,
                MemoryPoolSubscriber {
                    handle: subscriber,
                    dataflow_live: Arc::new(AtomicBool::new(true)),
                },
            );
        }
    }

    /// Release this dataflow's pool state when the dataflow finishes.
    pub(crate) async fn pool_cleanup_dataflow(&mut self, dataflow_id: Uuid) {
        if let Some(subscriber) = self.pool.subscribers.remove(&dataflow_id) {
            subscriber.shutdown();
        }

        // Drain this dataflow's per-(dataflow, pool) cross-write state:
        // these maps are keyed by a fresh per-dataflow UUID and are never
        // touched again after finish, so without this a long-lived daemon
        // cycling many cross-machine dataflows accumulates one entry per
        // (dataflow, pool) forever. The direct-write lock map and the
        // write-seq counters; the degradation set is drained too (a pool
        // that never recovered would otherwise linger).
        {
            let mut locks = CROSS_POOL_WRITE_LOCKS_ASYNC.lock().await;
            locks.retain(|(df, _), _| *df != dataflow_id);
        }
        {
            // The zenoh-relay write lock is the same fresh per-dataflow
            // UUID key; drain it here too — a relay-path pool whose
            // dataflow ends without an explicit free (e.g. a node crash,
            // so no FreePool is published) would otherwise leak its entry
            // for the daemon's lifetime (bot review 5301862843).
            let mut locks = CROSS_POOL_WRITE_LOCKS
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            locks.retain(|(df, _), _| *df != dataflow_id);
        }
        // Fail any cross-machine write replies still pending for this
        // dataflow. Without it a node whose write was in flight at finish
        // hangs until the 120s safety-net timeout, and a relay entry orphaned
        // by a failover (#3193) would leak for the daemon's lifetime.
        //
        // Must run *before* the `CROSS_WRITE_SEQ` retain below: a spawned
        // write task that fails over concurrently re-keys under the pending
        // guard, and it bumps the per-pool counter only if it found something
        // pending. Draining first therefore makes the re-key a no-op instead
        // of letting it resurrect a counter this function has already cleared
        // — a resurrected counter restarts at 0, mints a relay seq *below* the
        // abandoned one (breaking the monotonicity the write path relies on)
        // and leaks for the daemon's lifetime, since finish runs once.
        let stranded = drain_cross_write_pending(dataflow_id);
        if stranded > 0 {
            tracing::debug!(
                %dataflow_id,
                stranded,
                "failed cross-machine writes still pending at dataflow finish"
            );
        }
        {
            let mut seqs = CROSS_WRITE_SEQ.lock().unwrap_or_else(|e| e.into_inner());
            seqs.retain(|(df, _), _| *df != dataflow_id);
        }
        {
            let mut degraded = CROSS_DIRECT_DEGRADED
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            degraded.retain(|(df, _)| *df != dataflow_id);
        }
        // `cross_data_endpoints` is keyed by the same fresh per-dataflow
        // UUID (register-ack path); drain it here too so a long-lived
        // daemon does not accumulate one `SocketAddr` per (dataflow,
        // pool) forever.
        {
            let mut endpoints = self
                .pool
                .cross_data_endpoints
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            endpoints.retain(|(df, _), _| *df != dataflow_id);
        }
        // Reclaim this dataflow's pools. `cleanup_dataflow` covers the
        // local pool table; `cleanup_cross_pools` drains the cross-machine
        // table and unlinks the mirror segments among it (#3194).
        //
        // Neither guarantees every local reader is gone: dynamic nodes are
        // excluded from `should_finish` and may outlive the dataflow, and
        // an already-mapped reader survives an unlink but a re-open by
        // name does not. That window is not new — `cleanup_dataflow` has
        // reclaimed ordinary local pool segments at finish since
        // dora-rs/dora#2881; mirrors now behave the same way rather than
        // leaking for the daemon's lifetime.
        let dataflow_str = dataflow_id.to_string();
        self.pool.tensor_pool.cleanup_dataflow(&dataflow_str);
        self.pool.tensor_pool.cleanup_cross_pools(&dataflow_str);
    }
}

/// Encode a [`NodeResponse`] into the opaque reply dora hands back.
pub(crate) fn node_reply(response: &NodeResponse) -> DaemonReply {
    match postcard::to_allocvec(response) {
        Ok(payload) => DaemonReply::ExtensionReply { payload },
        Err(e) => DaemonReply::Result(Err(format!("tensor-pool reply encode failed: {e}"))),
    }
}
