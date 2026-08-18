//! Cross-machine memory-pool transport (opt-in, outside dora's 1.0
//! guarantees — see `libraries/extensions/tensor-pool/README.md`).
//!
//! Extracted from `lib.rs`, where it had grown to ~2.7k lines inline.
//! Split by plane so each piece is reviewable on its own:
//!
//! | module | what it owns |
//! |---|---|
//! | [`auth`] | shared-token handshake for the direct-TCP plane |
//! | [`mirror`] | DORADMA header, seqlock, `/dev/shm` mirror segments |
//! | [`data_plane`] | direct-TCP listener and sender |
//! | [`control`] | zenoh control events and pending-ack registries |

// ---------------------------------------------------------------------------
// Cross-machine memory pool data plane (ports the pre-tensor-pool branch;
// see `docs/design.md` in the hetero-pool PR). All of it is opt-in behind
// `DORA_MEMORY_POOL_CROSS_MACHINE=1`; a daemon without the env never opens
// the direct-TCP data port and never mirrors remote pools.
// ---------------------------------------------------------------------------

/// Opt-in switch for the whole cross-machine memory-pool data plane.
/// Off by default: no mirror segments, no direct-TCP listener, no
/// RegisterPool/RegisterCrossMachinePool processing. Deployments that
/// enable it must set the same value on every participating daemon.
pub(crate) const CROSS_MACHINE_ENV: &str = "DORA_MEMORY_POOL_CROSS_MACHINE";

pub(crate) fn cross_machine_enabled() -> bool {
    matches!(
        std::env::var(CROSS_MACHINE_ENV).as_deref(),
        Ok("1") | Ok("true") | Ok("TRUE") | Ok("yes")
    )
}

pub(crate) mod auth;
pub(crate) mod control;
pub(crate) mod daemon;
pub(crate) mod data_plane;
pub(crate) mod mirror;
pub(crate) mod state;

pub(crate) use auth::*;
pub(crate) use control::*;
pub(crate) use data_plane::*;
pub(crate) use mirror::*;
pub(crate) use state::{MemoryPoolSubscriber, PoolState};

use super::*;
// Imports this subsystem owns. They live here rather than in the crate root so
// that a build without the `tensor-pool` feature pulls in neither the
// dependencies nor the `unsafe` that comes with them.
pub(crate) use dora_core::topics::dataflow_extension_topic;
pub(crate) use dora_tensor_pool::TensorPoolManager;
pub(crate) use dora_tensor_pool::protocol::{NAMESPACE, PeerMessage};
pub(crate) use shared_memory_extended::ShmemConf;
pub(crate) use std::sync::atomic::AtomicBool;
pub(crate) use zenoh::Wait;
pub(crate) use zenoh::bytes::ZBytes;
pub(crate) use zenoh::sample::Locality;
pub(crate) use zenoh::shm::{PosixShmProviderBackend, ShmProvider, ShmProviderBuilder};

#[cfg(test)]
mod cross_pool_write_tests {
    use super::*;

    /// Serializes tests that mutate the process-wide
    /// `DORA_MEMORY_POOL_AUTH_TOKEN` env var. Rust tests run on parallel
    /// threads; the auth-handshake tests key off that env on both the
    /// send and verify sides, so a parallel `set_var` would flip the
    /// handshake contract out from under them mid-flight.
    static CROSS_DATA_AUTH_ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// Panic-safe cleanup: unlink the test mirror from /dev/shm even when
    /// an assertion fails mid-test (a leaked segment would pollute the
    /// bench host's zero-residue checks).
    struct ShmemCleanup(String);

    impl Drop for ShmemCleanup {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(format!("/dev/shm/{}", self.0));
        }
    }

    /// Orphan sweep removes only segments under this machine's id prefix;
    /// segments of other machines (sibling daemons on the same host) and
    /// local (un-prefixed) pool segments survive.
    #[test]
    #[cfg(target_os = "linux")]
    fn orphan_sweep_only_touches_own_machine_prefix() {
        let dir = "/dev/shm";
        // "orphanB" prefix isolates this test from the other cross-pool
        // tests, which run in parallel and use segments under dora_pool_B_.
        let own = "dora_pool_orphanB_orphantest_node_0";
        let sibling = "dora_pool_orphanC_orphantest_node_0";
        let local = "dora_pool_orphantest_node_0";
        std::fs::write(format!("{dir}/{own}"), vec![0u8; 64]).unwrap();
        std::fs::write(format!("{dir}/{sibling}"), vec![0u8; 64]).unwrap();
        std::fs::write(format!("{dir}/{local}"), vec![0u8; 64]).unwrap();

        let removed = cleanup_orphan_mirrors("orphanB");

        assert_eq!(removed, 1);
        assert!(!std::path::Path::new(&format!("{dir}/{own}")).exists());
        assert!(std::path::Path::new(&format!("{dir}/{sibling}")).exists());
        assert!(std::path::Path::new(&format!("{dir}/{local}")).exists());
        // test hygiene
        let _ = std::fs::remove_file(format!("{dir}/{sibling}"));
        let _ = std::fs::remove_file(format!("{dir}/{local}"));
    }

    /// `finish_dataflow` must unlink the mirror segments this daemon
    /// created for the finishing dataflow (dora-rs/dora#3194) — an
    /// explicit `FreePool` never arrives when the dataflow ends by node
    /// crash, `dora stop`, or abort — and must leave alone both the
    /// origin-side entry (which owns no segment) and another dataflow's
    /// still-live mirrors.
    #[test]
    #[cfg(target_os = "linux")]
    fn finish_unlinks_own_mirrors_only() {
        let finishing = Uuid::new_v4().to_string();
        let other = Uuid::new_v4().to_string();

        let name = |dataflow_id: &str, pool_id: &str| {
            TensorPoolManager::cross_pool_shmem_name("finishB", dataflow_id, pool_id).unwrap()
        };
        // (dataflow, pool, is a mirror this daemon created, survives finish)
        let fixture = [
            (&finishing, "pool_node_0", true, false),
            (&finishing, "pool_node_1", true, false),
            // Origin-side entry of the finishing dataflow: no segment of
            // its own, so the sender's live local pool must survive.
            (&finishing, "pool_sender_0", false, true),
            // Mirror of a dataflow that is still running.
            (&other, "pool_node_0", true, true),
        ];

        let tensor_pool = TensorPoolManager::new();
        let mut cleanup = Vec::new();
        for (dataflow_id, pool_id, is_mirror, _) in fixture {
            let shmem_name = name(dataflow_id, pool_id);
            std::fs::write(format!("/dev/shm/{shmem_name}"), vec![0u8; 4096]).unwrap();
            cleanup.push(ShmemCleanup(shmem_name.clone()));
            tensor_pool.register_cross_pool(
                dataflow_id.to_string(),
                pool_id.to_string(),
                "A".to_string(),
                is_mirror.then_some(shmem_name),
            );
        }

        assert_eq!(tensor_pool.cleanup_cross_pools(&finishing), 2);

        for (dataflow_id, pool_id, _, survives) in fixture {
            let path = format!("/dev/shm/{}", name(dataflow_id, pool_id));
            assert_eq!(
                std::path::Path::new(&path).exists(),
                survives,
                "{pool_id} of {dataflow_id}"
            );
        }
    }

    /// A stale mirror segment (leftover from a killed daemon that never
    /// ran shutdown cleanup) must be replaced on re-register instead of
    /// failing with EEXIST.
    #[test]
    #[cfg(target_os = "linux")]
    fn stale_mirror_is_replaced_on_register() {
        let dataflow_id = Uuid::new_v4();
        let pool_id = "pool_node_0";
        const SIZE: usize = 4096;
        let shmem_name =
            TensorPoolManager::cross_pool_shmem_name("B", &dataflow_id.to_string(), pool_id)
                .unwrap();
        // Simulate the leftover: a plain file under the mirror's name.
        std::fs::write(format!("/dev/shm/{shmem_name}"), vec![0u8; 512]).unwrap();
        let _cleanup = ShmemCleanup(shmem_name.clone());

        create_cross_pool_shmem(
            &dataflow_id,
            "B",
            pool_id,
            SIZE,
            "int64",
            &[512],
            "cpu",
            None,
        )
        .unwrap();

        // The recreated segment must be a valid DORADMA mirror.
        let shmem = ShmemConf::new().os_id(&shmem_name).open().unwrap();
        let magic = unsafe { std::slice::from_raw_parts(shmem.as_ptr(), 8) };
        assert_eq!(magic, DORADMA_MAGIC);
    }

    #[test]
    #[cfg(target_os = "linux")] // /dev/shm 段名（mirror 创建/校验）仅 Linux 有效
    fn mirror_json_records_receiver_device() {
        let dataflow_id = Uuid::new_v4();
        let pool_id = "pool_node_0";
        const SIZE: usize = 4096;
        for device in ["cpu", "cuda:0"] {
            let shmem_name =
                TensorPoolManager::cross_pool_shmem_name("B", &dataflow_id.to_string(), pool_id)
                    .unwrap();
            let _cleanup = ShmemCleanup(shmem_name.clone());
            create_cross_pool_shmem(
                &dataflow_id,
                "B",
                pool_id,
                SIZE,
                "int64",
                &[512],
                device,
                None,
            )
            .unwrap();
            let shmem = ShmemConf::new().os_id(&shmem_name).open().unwrap();
            let json_len = unsafe { read_header_u64(shmem.as_ptr().add(8)) } as usize;
            let json_bytes = unsafe {
                std::slice::from_raw_parts(shmem.as_ptr().add(DORADMA_HEADER_SIZE), json_len)
            };
            let json = String::from_utf8(json_bytes.to_vec()).unwrap();
            assert!(
                json.contains(&format!("\"pinned_type\":\"{device}\"")),
                "mirror json {json} must record receiver device {device}"
            );
            std::fs::remove_file(format!("/dev/shm/{shmem_name}")).unwrap();
        }
    }

    /// Concurrent writers to the same mirror must not interleave bytes:
    /// after every round the data region holds one writer's complete
    /// pattern, never a mixture. Without the per-pool lock, overlapping
    /// memcpys of distinct fill bytes interleave and the final frame
    /// passes the seqlock (even generation) with mixed bytes.
    #[test]
    #[cfg(target_os = "linux")]
    fn concurrent_mirror_writes_never_interleave_bytes() {
        let dataflow_id = Uuid::new_v4();
        let pool_id = "pool_node_0";
        const SIZE: usize = 4 * 1024 * 1024;
        create_cross_pool_shmem(
            &dataflow_id,
            "B",
            pool_id,
            SIZE,
            "int64",
            &[8192],
            "cpu",
            None,
        )
        .unwrap();
        let shmem_name =
            TensorPoolManager::cross_pool_shmem_name("B", &dataflow_id.to_string(), pool_id)
                .unwrap();
        let _cleanup = ShmemCleanup(shmem_name.clone());
        let shmem = ShmemConf::new().os_id(&shmem_name).open().unwrap();
        // data_offset comes from the header (json_len varies), not a constant.
        let data_offset = unsafe { read_header_u64(shmem.as_ptr().add(16)) as usize };

        // Writers with distinct fill bytes race the same mirror.
        // Patterns are allocated once and reused across rounds.
        const WRITERS: u8 = 8;
        let patterns: Vec<Vec<u8>> = (0..WRITERS).map(|w| vec![w; SIZE]).collect();
        for round in 0..200 {
            std::thread::scope(|scope| {
                for pattern in patterns.iter() {
                    let pattern = pattern.as_slice();
                    scope.spawn(move || {
                        write_cross_pool_data(&dataflow_id, "B", pool_id, pattern, SIZE);
                    });
                }
            });
            // The data region must hold exactly one writer's full pattern.
            let first = unsafe { *shmem.as_ptr().add(data_offset) };
            let data = unsafe { std::slice::from_raw_parts(shmem.as_ptr().add(data_offset), SIZE) };
            assert!(
                data.iter().all(|b| *b == first),
                "round {round}: interleaved bytes in mirror data"
            );
            // Seqlock: generation is even (complete) after the round.
            let generation =
                unsafe { std::ptr::read_volatile(shmem.as_ptr().add(96) as *const u64) };
            assert_eq!(
                generation % 2,
                0,
                "round {round}: odd generation after write"
            );
        }
    }

    /// The write-commit ack resolves only the seq-matched pending reply.
    ///
    /// The same-host cross-daemon smoke test reads via the `direct ==
    /// true` path and bypasses the MemoryPoolWrite/MemoryPoolWriteAck
    /// machinery entirely (only the manual two-host runs exercise it), so
    /// this test pins the ack semantics directly: a stale ack (a previous
    /// write's seq) must not resolve anything, the seq-matched ack
    /// resolves exactly its own pending reply, and a failed mirror write
    /// surfaces as an error reply.
    #[test]
    fn write_ack_resolves_only_seq_matched_pending_reply() {
        use tokio::sync::oneshot;

        let df = Uuid::new_v4();
        let pool = "pool_sender_node_1".to_string();

        // Two in-flight writes to the same pool: seq 1 then seq 2.
        let (tx1, mut rx1) = oneshot::channel();
        let (tx2, mut rx2) = oneshot::channel();
        {
            let mut pending = CROSS_WRITE_PENDING
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            pending.insert((df, pool.clone(), 1), tx1);
            pending.insert((df, pool.clone(), 2), tx2);
        }

        // A stale ack (a previous write's seq) resolves nothing.
        assert!(!resolve_cross_write_ack(df, pool.clone(), 0, true, None));
        assert!(matches!(
            rx1.try_recv(),
            Err(oneshot::error::TryRecvError::Empty)
        ));
        assert!(matches!(
            rx2.try_recv(),
            Err(oneshot::error::TryRecvError::Empty)
        ));

        // The seq-matched ack resolves exactly its own pending reply.
        assert!(resolve_cross_write_ack(df, pool.clone(), 1, true, None));
        match rx1.try_recv() {
            Ok(DaemonReply::Result(Ok(()))) => {}
            other => panic!("seq-1 ack resolved the wrong reply: {other:?}"),
        }
        // seq 2 is untouched by the seq-1 ack.
        assert!(matches!(
            rx2.try_recv(),
            Err(oneshot::error::TryRecvError::Empty)
        ));

        // A failed mirror write surfaces as an error reply.
        assert!(resolve_cross_write_ack(
            df,
            pool.clone(),
            2,
            false,
            Some("mirror segment missing".to_string()),
        ));
        match rx2.try_recv() {
            Ok(DaemonReply::Result(Err(e))) => {
                assert!(e.contains("mirror segment missing"));
            }
            other => panic!("failed write must resolve to an error reply: {other:?}"),
        }
    }

    #[tokio::test]
    async fn write_lock_is_dataflow_scoped() {
        let df1 = Uuid::new_v4();
        let df2 = Uuid::new_v4();
        let pool = "pool_sender_node_1".to_string();
        let mut locks = CROSS_POOL_WRITE_LOCKS_ASYNC.lock().await;
        let mut entry = |df, pool: &str| {
            locks
                .entry((df, pool.to_string()))
                .or_insert_with(|| std::sync::Arc::new(tokio::sync::Mutex::new(())))
                .clone()
        };
        // Same pool id in two dataflows → distinct locks (concurrent
        // dataflows never serialize on each other's segments).
        let l1 = entry(df1, &pool);
        let l2 = entry(df2, &pool);
        assert!(!std::sync::Arc::ptr_eq(&l1, &l2));
        // Same dataflow, same pool → the same lock (writes to one segment
        // stay serialized).
        assert!(std::sync::Arc::ptr_eq(&l1, &entry(df1, &pool)));
        // Same dataflow, different pool → distinct locks.
        assert!(!std::sync::Arc::ptr_eq(
            &l1,
            &entry(df1, "pool_sender_node_2")
        ));
    }

    /// The direct-TCP fallback warns exactly once per pool: repeated
    /// failures while degraded stay silent, and recovery is reported once.
    ///
    /// A broken link is the steady state for the zenoh fallback — a
    /// per-frame warn would flood the daemon log for the whole outage.
    #[test]
    fn direct_fallback_warns_once_per_pool_and_reports_recovery() {
        let df = Uuid::new_v4();
        let pool = "pool_sender_node_1".to_string();

        // First failure: newly degraded, the caller warns.
        assert!(note_direct_degraded(df, &pool));
        // Further failures while degraded: silent.
        assert!(!note_direct_degraded(df, &pool));
        assert!(!note_direct_degraded(df, &pool));
        // Recovery: was degraded, the caller logs it once.
        assert!(note_direct_recovered(df, &pool));
        // Recovery without being degraded: nothing to report.
        assert!(!note_direct_recovered(df, &pool));

        // Pools are tracked independently.
        assert!(note_direct_degraded(df, "pool_sender_node_2"));
        assert!(!note_direct_degraded(df, "pool_sender_node_2"));
        assert!(note_direct_recovered(df, "pool_sender_node_2"));
        // A fresh dataflow never collides (keyed by UUID).
        assert!(note_direct_degraded(Uuid::new_v4(), &pool));
    }

    /// Build a direct-TCP frame header (magic + dataflow + pool + seq +
    /// size) with no payload bytes.
    fn build_frame_header(dataflow_id: Uuid, pool_id: &str, seq: u64, size: u64) -> Vec<u8> {
        let mut frame = Vec::new();
        frame.extend_from_slice(&CROSS_DATA_MAGIC.to_be_bytes());
        frame.extend_from_slice(dataflow_id.as_bytes());
        let pool_bytes = pool_id.as_bytes();
        frame.extend_from_slice(&(pool_bytes.len() as u32).to_be_bytes());
        frame.extend_from_slice(pool_bytes);
        frame.extend_from_slice(&seq.to_be_bytes());
        frame.extend_from_slice(&size.to_be_bytes());
        frame
    }

    /// A wire-controlled `size` near `u64::MAX` must be rejected, not
    /// wrap `data_offset + size` past the bounds check (which would
    /// construct a `size`-byte slice past the mapping — UB, and a
    /// remote-triggerable abort in debug builds).
    #[tokio::test]
    #[cfg(target_os = "linux")]
    async fn wire_size_overflow_is_rejected_not_aborted() {
        use tokio::io::AsyncWriteExt;

        let dataflow_id = Uuid::new_v4();
        let pool_id = "pool_node_0";
        const SIZE: usize = 64 * 1024;
        create_cross_pool_shmem(
            &dataflow_id,
            "B",
            pool_id,
            SIZE,
            "int64",
            &[8192],
            "cpu",
            None,
        )
        .unwrap();
        let shmem_name =
            TensorPoolManager::cross_pool_shmem_name("B", &dataflow_id.to_string(), pool_id)
                .unwrap();
        let _cleanup = ShmemCleanup(shmem_name.clone());
        let tensor_pool = TensorPoolManager::new();
        tensor_pool.register_cross_pool(
            dataflow_id.to_string(),
            pool_id.to_string(),
            "A".to_string(),
            Some(shmem_name.clone()),
        );

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            handle_cross_data_frame(&mut stream, &tensor_pool, "B").await
        });

        // data_offset + u64::MAX wraps to a small value without the
        // checked add — the frame must be rejected with the ack info.
        let frame = build_frame_header(dataflow_id, pool_id, 1, u64::MAX);
        let mut client = tokio::net::TcpStream::connect(addr).await.unwrap();
        client.write_all(&frame).await.unwrap();

        let err = server.await.unwrap().unwrap_err();
        assert!(
            err.ack == Some((dataflow_id, pool_id.to_string(), 1)),
            "overflow rejection must carry the ack info: {err:?}"
        );
        // Rejection can come from the registered-size check (newer,
        // fires first: u64::MAX != registered size) or the checked-add
        // overflow guard — either way the wire-controlled size is
        // refused without aborting.
        assert!(
            err.message.contains("overflow") || err.message.contains("registered pool size"),
            "expected size rejection, got: {}",
            err.message
        );
        // The mirror's seqlock generation is untouched: no writer began,
        // so it stays at the initial odd (in-progress) value 1.
        let shmem = ShmemConf::new().os_id(&shmem_name).open().unwrap();
        let generation = unsafe { std::ptr::read_volatile(shmem.as_ptr().add(96) as *const u64) };
        assert_eq!(
            generation, 1,
            "generation must be untouched (no write began)"
        );
    }

    /// The direct-TCP serve layer fails the origin fast on a bad frame:
    /// `serve_cross_data_frame` publishes `MemoryPoolWriteAck { ok: false }`
    /// over zenoh when the frame's identity is parsed but the write cannot
    /// proceed. This closes the last untested half of the standing
    /// non-blocker — the codec round-trip covers the happy path, this
    /// covers the error path's zenoh ack publish (only the true two-host
    /// transfer remains manual).
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    #[cfg(target_os = "linux")]
    async fn serve_error_publishes_failed_write_ack() {
        use tokio::io::AsyncWriteExt;

        // Hermetic zenoh pair: mirror listens, origin dials; no scouting.
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        drop(listener);
        let mut mirror_cfg = zenoh::Config::default();
        let mut origin_cfg = zenoh::Config::default();
        for cfg in [&mut mirror_cfg, &mut origin_cfg] {
            cfg.insert_json5("scouting/multicast/enabled", "false")
                .unwrap();
            cfg.insert_json5("scouting/gossip/enabled", "false")
                .unwrap();
        }
        mirror_cfg
            .insert_json5(
                "listen/endpoints",
                &format!(r#"{{ peer: ["tcp/127.0.0.1:{port}"] }}"#),
            )
            .unwrap();
        mirror_cfg
            .insert_json5("listen/exit_on_failure", "false")
            .unwrap();
        origin_cfg
            .insert_json5(
                "connect/endpoints",
                &format!(r#"{{ peer: ["tcp/127.0.0.1:{port}"] }}"#),
            )
            .unwrap();
        let mirror_session = zenoh::open(mirror_cfg).await.unwrap();
        let origin_session = zenoh::open(origin_cfg).await.unwrap();

        // Mirror pool the frame will target.
        let dataflow_id = Uuid::new_v4();
        let pool_id = "pool_node_0";
        const SIZE: usize = 64 * 1024;
        create_cross_pool_shmem(
            &dataflow_id,
            "B",
            pool_id,
            SIZE,
            "int64",
            &[8192],
            "cpu",
            None,
        )
        .unwrap();
        let shmem_name =
            TensorPoolManager::cross_pool_shmem_name("B", &dataflow_id.to_string(), pool_id)
                .unwrap();
        let _cleanup = ShmemCleanup(shmem_name.clone());
        let tensor_pool = TensorPoolManager::new();
        tensor_pool.register_cross_pool(
            dataflow_id.to_string(),
            pool_id.to_string(),
            "A".to_string(),
            Some(shmem_name.clone()),
        );

        // Data listener served by the mirror session's process context.
        let data_listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let data_addr = data_listener.local_addr().unwrap();
        let clock = Arc::new(HLC::default());
        let serve = tokio::spawn(async move {
            let (mut stream, _) = data_listener.accept().await.unwrap();
            serve_cross_data_frame(
                &tensor_pool,
                "B",
                &mirror_session,
                &clock,
                None,
                &mut stream,
            )
            .await
        });

        // Origin-side subscriber on the memory-pool topic (the daemon's
        // per-dataflow loop in miniature).
        let topic = dataflow_extension_topic(&dataflow_id, NAMESPACE);
        let subscriber = origin_session.declare_subscriber(&topic).await.unwrap();
        // The serve layer publishes the failed-write ack exactly once
        // (event-driven, no retry — fine in production, where the
        // subscription is established long before any write). Give the
        // subscription interest time to propagate to the mirror session,
        // or the single ack put would be dropped into the void.
        tokio::time::sleep(std::time::Duration::from_millis(300)).await;

        // A bad frame (size near u64::MAX → checked-add rejection) with a
        // parsed identity: the serve layer must publish ok=false.
        let mut client = tokio::net::TcpStream::connect(data_addr).await.unwrap();
        let frame = build_frame_header(dataflow_id, pool_id, 1, u64::MAX);
        client.write_all(&frame).await.unwrap();

        let serve_result = serve.await.unwrap();
        assert!(serve_result.is_err(), "serve must report the frame error");

        // The origin receives the failed-write ack with matching identity.
        let mut ack_received = false;
        for _ in 0..10 {
            match tokio::time::timeout(std::time::Duration::from_secs(2), subscriber.recv_async())
                .await
            {
                Ok(Ok(sample)) => {
                    let bytes = sample.payload().to_bytes();
                    let event =
                        Timestamped::<InterDaemonEvent>::deserialize_inter_daemon_event(&bytes)
                            .unwrap();
                    match event.inner {
                        InterDaemonEvent::ExtensionMessage {
                            dataflow_id: df,
                            namespace,
                            payload,
                            ..
                        } => {
                            assert_eq!(namespace, NAMESPACE);
                            let PeerMessage::WriteAck {
                                shared_memory_id,
                                seq,
                                ok,
                                error,
                            } = postcard::from_bytes(&payload).unwrap()
                            else {
                                panic!("expected a WriteAck")
                            };
                            assert_eq!(df, dataflow_id);
                            assert_eq!(shared_memory_id, pool_id);
                            assert_eq!(seq, 1);
                            assert!(!ok, "failed write must ack ok=false");
                            assert!(
                                error.is_some()
                                    && (error.as_deref().unwrap().contains("overflow")
                                        || error
                                            .as_deref()
                                            .unwrap()
                                            .contains("registered pool size")),
                                "failed write ack must carry the error: {error:?}"
                            );
                            ack_received = true;
                        }
                        other => panic!("unexpected event: {other:?}"),
                    }
                }
                Ok(Err(e)) => panic!("subscriber closed: {e}"),
                Err(_) => {} // interest not yet propagated; keep waiting
            }
            if ack_received {
                break;
            }
        }
        assert!(ack_received, "failed-write ack never arrived over zenoh");
    }

    /// A payload read that fails mid-frame (TCP drop) leaves the seqlock
    /// generation odd — readers reject the torn frame and never see
    /// half-written bytes; the next full write self-heals — and the
    /// error carries the ack info so the origin fails fast instead of
    /// waiting out the commit-ack timeout.
    #[tokio::test]
    #[cfg(target_os = "linux")]
    async fn payload_read_failure_stays_odd_and_carries_ack() {
        use tokio::io::AsyncWriteExt;

        let dataflow_id = Uuid::new_v4();
        let pool_id = "pool_node_0";
        const SIZE: usize = 64 * 1024;
        create_cross_pool_shmem(
            &dataflow_id,
            "B",
            pool_id,
            SIZE,
            "int64",
            &[8192],
            "cpu",
            None,
        )
        .unwrap();
        let shmem_name =
            TensorPoolManager::cross_pool_shmem_name("B", &dataflow_id.to_string(), pool_id)
                .unwrap();
        let _cleanup = ShmemCleanup(shmem_name.clone());
        let tensor_pool = TensorPoolManager::new();
        tensor_pool.register_cross_pool(
            dataflow_id.to_string(),
            pool_id.to_string(),
            "A".to_string(),
            Some(shmem_name.clone()),
        );

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let tensor_pool_1 = tensor_pool.clone();
        let server = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            handle_cross_data_frame(&mut stream, &tensor_pool_1, "B").await
        });
        let mut client = tokio::net::TcpStream::connect(addr).await.unwrap();

        // Frame 1: a full, valid write — the generation advances to an
        // even baseline (2) that the failure must leave odd-on-top-of.
        let mut frame1 = build_frame_header(dataflow_id, pool_id, 1, SIZE as u64);
        frame1.extend(std::iter::repeat_n(7u8, SIZE));
        client.write_all(&frame1).await.unwrap();
        let ack1 = server.await.unwrap().unwrap().unwrap();
        assert_eq!(ack1.2, 1);
        let shmem = ShmemConf::new().os_id(&shmem_name).open().unwrap();
        let generation = unsafe { std::ptr::read_volatile(shmem.as_ptr().add(96) as *const u64) };
        assert_eq!(
            generation, 2,
            "frame 1 must complete the write (even gen 2)"
        );

        // Frame 2: header + half the payload, then the connection drops —
        // the read fails mid-frame.
        let listener2 = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr2 = listener2.local_addr().unwrap();
        let tensor_pool_2 = tensor_pool.clone();
        let server2 = tokio::spawn(async move {
            let (mut stream, _) = listener2.accept().await.unwrap();
            handle_cross_data_frame(&mut stream, &tensor_pool_2, "B").await
        });
        let mut client2 = tokio::net::TcpStream::connect(addr2).await.unwrap();
        let mut frame2 = build_frame_header(dataflow_id, pool_id, 2, SIZE as u64);
        frame2.extend(std::iter::repeat_n(9u8, SIZE / 2));
        client2.write_all(&frame2).await.unwrap();
        // Drop the connection: the mirror's read_exact fails with EOF.
        drop(client2);

        let err = server2.await.unwrap().unwrap_err();
        assert!(
            err.ack == Some((dataflow_id, pool_id.to_string(), 2)),
            "mid-frame read failure must carry the ack info: {err:?}"
        );
        assert!(err.message.contains("read payload"), "got: {}", err.message);

        // The generation stays odd (in-progress) — the fail-safe: readers
        // reject the torn frame rather than reading half-written bytes.
        let generation = unsafe { std::ptr::read_volatile(shmem.as_ptr().add(96) as *const u64) };
        assert_eq!(generation, 3, "generation must stay odd on the torn frame");
        // A subsequent full write self-heals: begin keeps the odd
        // generation, writes the frame, and publishes the even one.
        let listener3 = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr3 = listener3.local_addr().unwrap();
        let tensor_pool_3 = tensor_pool.clone();
        let server3 = tokio::spawn(async move {
            let (mut stream, _) = listener3.accept().await.unwrap();
            handle_cross_data_frame(&mut stream, &tensor_pool_3, "B").await
        });
        let mut client3 = tokio::net::TcpStream::connect(addr3).await.unwrap();
        let mut frame3 = build_frame_header(dataflow_id, pool_id, 3, SIZE as u64);
        frame3.extend(std::iter::repeat_n(11u8, SIZE));
        client3.write_all(&frame3).await.unwrap();
        let ack3 = server3.await.unwrap().unwrap().unwrap();
        assert_eq!(ack3.2, 3);
        let generation = unsafe { std::ptr::read_volatile(shmem.as_ptr().add(96) as *const u64) };
        assert_eq!(
            generation, 4,
            "the next full write must self-heal to an even generation"
        );
        let data_offset = unsafe { read_header_u64(shmem.as_ptr().add(16)) } as usize;
        let data = unsafe { std::slice::from_raw_parts(shmem.as_ptr().add(data_offset), SIZE) };
        assert!(
            data.iter().all(|b| *b == 11),
            "self-healed frame must be fully visible"
        );
    }

    /// The zenoh ack publish itself: the mirror-side publish helper over a
    /// real zenoh transport (loopback TCP), received and deserialized by
    /// the origin-side subscriber, resolving the seq-matched pending reply.
    ///
    /// `write_ack_resolves_only_seq_matched_pending_reply` pins the
    /// resolver in isolation; this test covers the publish → transport →
    /// subscribe → deserialize chain that feeds it — the last uncovered
    /// link of the commit protocol (only the true two-host transfer still
    /// needs manual runs). Two sessions are required because the mirror
    /// publishes with `Locality::Remote`: a same-session subscriber would
    /// never receive its own put.
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn zenoh_ack_publish_resolves_pending_reply() {
        // Free loopback port for the mirror session's listener.
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        drop(listener);

        let mut mirror_cfg = zenoh::Config::default();
        let mut origin_cfg = zenoh::Config::default();
        for cfg in [&mut mirror_cfg, &mut origin_cfg] {
            // Hermetic: no scouting, so the test can neither touch nor be
            // touched by other zenoh instances on the host.
            cfg.insert_json5("scouting/multicast/enabled", "false")
                .unwrap();
            cfg.insert_json5("scouting/gossip/enabled", "false")
                .unwrap();
        }
        mirror_cfg
            .insert_json5(
                "listen/endpoints",
                &format!(r#"{{ peer: ["tcp/127.0.0.1:{port}"] }}"#),
            )
            .unwrap();
        mirror_cfg
            .insert_json5("listen/exit_on_failure", "false")
            .unwrap();
        origin_cfg
            .insert_json5(
                "connect/endpoints",
                &format!(r#"{{ peer: ["tcp/127.0.0.1:{port}"] }}"#),
            )
            .unwrap();
        let mirror_session = zenoh::open(mirror_cfg).await.unwrap();
        let origin_session = zenoh::open(origin_cfg).await.unwrap();

        let df = Uuid::new_v4();
        let pool = "pool_sender_node_1".to_string();
        let seq = 7u64;
        let topic = dataflow_extension_topic(&df, NAMESPACE);

        // Origin-side subscriber, mirroring the daemon's per-dataflow loop.
        let subscriber = origin_session.declare_subscriber(&topic).await.unwrap();

        // The write this ack commits is pending its reply.
        let (tx, rx) = tokio::sync::oneshot::channel();
        CROSS_WRITE_PENDING
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .insert((df, pool.clone(), seq), tx);

        // Mirror-side publish through the production helper. Retry the put
        // until the ack arrives: the subscriber interest must propagate to
        // the mirror session over the fresh link, and a put that precedes
        // the interest is dropped (Block congestion control never drops, so
        // retries converge).
        let clock = Arc::new(HLC::default());
        let mut ack_received = false;
        for _ in 0..10 {
            publish_pool_message(
                &mirror_session,
                &clock,
                &df,
                None,
                &PeerMessage::WriteAck {
                    shared_memory_id: pool.clone(),
                    seq,
                    ok: true,
                    error: None,
                },
                None,
            )
            .await
            .unwrap();
            match tokio::time::timeout(std::time::Duration::from_secs(2), subscriber.recv_async())
                .await
            {
                Ok(Ok(sample)) => {
                    // Deserialize with the production method and resolve.
                    let bytes = sample.payload().to_bytes();
                    let event =
                        Timestamped::<InterDaemonEvent>::deserialize_inter_daemon_event(&bytes)
                            .unwrap();
                    match event.inner {
                        InterDaemonEvent::ExtensionMessage {
                            dataflow_id,
                            namespace,
                            payload,
                            ..
                        } => {
                            assert_eq!(namespace, NAMESPACE);
                            let PeerMessage::WriteAck {
                                shared_memory_id,
                                seq: ack_seq,
                                ok,
                                error,
                            } = postcard::from_bytes(&payload).unwrap()
                            else {
                                panic!("expected a WriteAck")
                            };
                            assert_eq!(dataflow_id, df);
                            assert_eq!(shared_memory_id, pool);
                            assert_eq!(ack_seq, seq);
                            assert!(ok);
                            assert!(error.is_none());
                            assert!(resolve_cross_write_ack(
                                dataflow_id,
                                shared_memory_id,
                                ack_seq,
                                ok,
                                error,
                            ));
                        }
                        other => panic!("unexpected event on memory-pool topic: {other:?}"),
                    }
                    ack_received = true;
                }
                Ok(Err(e)) => panic!("memory-pool subscriber closed: {e}"),
                Err(_) => {} // interest not yet propagated; put again
            }
            if ack_received {
                break;
            }
        }
        assert!(ack_received, "ack never arrived over the zenoh link");

        // The pending write's reply is resolved by the zenoh-delivered ack.
        match rx.await {
            Ok(DaemonReply::Result(Ok(()))) => {}
            other => panic!("pending reply not resolved by the zenoh ack: {other:?}"),
        }
    }

    /// The direct-TCP data-plane codec: a frame sent via
    /// `send_cross_data_frame` over a loopback connection is parsed by
    /// `handle_cross_data_frame` and written straight into the mirror
    /// segment — payload bytes land in the data region under the seqlock,
    /// and the returned ack info matches (dataflow, pool, seq). This is
    /// the new steady-state cross-machine write path, which the same-host
    /// smoke (direct == true) bypasses entirely.
    #[test]
    #[cfg(target_os = "linux")]
    fn direct_tcp_frame_round_trip_writes_mirror() {
        // This test exercises the token-less contract (both ends skip the
        // handshake when no token is configured). Serialize against the
        // auth test's env mutation and clear the variable defensively so
        // a token in the developer's shell cannot flip this test into
        // handshake mode mid-flight.
        //
        // SAFETY (env mutation in tests): `CROSS_DATA_AUTH_ENV_LOCK`
        // serializes every test that reads or writes this variable; the
        // guard runs on this test's own thread and is never awaited on.
        let _env_lock = CROSS_DATA_AUTH_ENV_LOCK.lock().unwrap();
        unsafe { std::env::remove_var("DORA_MEMORY_POOL_AUTH_TOKEN") };
        struct TokenEnvGuard;
        impl Drop for TokenEnvGuard {
            fn drop(&mut self) {
                unsafe { std::env::remove_var("DORA_MEMORY_POOL_AUTH_TOKEN") };
            }
        }
        let _token_guard = TokenEnvGuard;

        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        runtime.block_on(async {
            let dataflow_id = Uuid::new_v4();
            let pool_id = "pool_node_0";
            const SIZE: usize = 64 * 1024;
            create_cross_pool_shmem(
                &dataflow_id,
                "B",
                pool_id,
                SIZE,
                "int64",
                &[8192],
                "cpu",
                None,
            )
            .unwrap();
            let shmem_name =
                TensorPoolManager::cross_pool_shmem_name("B", &dataflow_id.to_string(), pool_id)
                    .unwrap();
            let _cleanup = ShmemCleanup(shmem_name.clone());
            let tensor_pool = TensorPoolManager::new();
            tensor_pool.register_cross_pool(
                dataflow_id.to_string(),
                pool_id.to_string(),
                "A".to_string(),
                Some(shmem_name.clone()),
            );

            let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
            let addr = listener.local_addr().unwrap();
            let server = tokio::spawn(async move {
                let (mut stream, _) = listener.accept().await.unwrap();
                handle_cross_data_frame(&mut stream, &tensor_pool, "B").await
            });
            let payload: Vec<u8> = (0..SIZE).map(|i| (i % 251) as u8).collect();
            let conns = Arc::new(std::sync::Mutex::new(HashMap::new()));
            send_cross_data_frame(&conns, addr, dataflow_id, pool_id, 42, &payload)
                .await
                .unwrap();
            let ack_info = server.await.unwrap().unwrap().unwrap();
            assert_eq!(ack_info.0, dataflow_id);
            assert_eq!(ack_info.1, pool_id);
            assert_eq!(ack_info.2, 42);

            let shmem = ShmemConf::new().os_id(&shmem_name).open().unwrap();
            let data_offset = unsafe { read_header_u64(shmem.as_ptr().add(16)) } as usize;
            let data = unsafe { std::slice::from_raw_parts(shmem.as_ptr().add(data_offset), SIZE) };
            assert_eq!(
                data,
                payload.as_slice(),
                "mirror data region must equal the payload"
            );
            // Seqlock: generation is even (complete) after the write.
            let generation =
                unsafe { std::ptr::read_volatile(shmem.as_ptr().add(96) as *const u64) };
            assert_eq!(generation % 2, 0, "odd generation after write");
        });
    }

    /// The direct-TCP auth handshake: with `DORA_MEMORY_POOL_AUTH_TOKEN`
    /// set, a sender with the shared token is accepted and its frame
    /// served; a sender without it (or with the wrong token) is rejected
    /// before any frame is parsed.
    #[tokio::test]
    async fn auth_handshake_accepts_shared_token_and_rejects_others() {
        // Hermetic env for this test: set and restore.
        //
        // SAFETY (env mutation in tests): `CROSS_DATA_AUTH_ENV_LOCK`
        // serializes every test that reads or writes this variable, so no
        // other test thread can observe the mutation mid-test. The std
        // mutex is only ever locked briefly by this test's own thread
        // (never awaited on), so it cannot deadlock the async body.
        let _env_lock = CROSS_DATA_AUTH_ENV_LOCK.lock().unwrap();
        unsafe { std::env::set_var("DORA_MEMORY_POOL_AUTH_TOKEN", "test-shared-secret") };
        struct EnvGuard;
        impl Drop for EnvGuard {
            fn drop(&mut self) {
                unsafe { std::env::remove_var("DORA_MEMORY_POOL_AUTH_TOKEN") };
            }
        }
        let _guard = EnvGuard;

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            match tokio::time::timeout(
                std::time::Duration::from_secs(5),
                auth_handshake_verify(&mut stream),
            )
            .await
            {
                Ok(result) => result,
                Err(_) => Err("auth handshake timed out".to_string()),
            }
        });

        // Correct token: accepted.
        let mut client = tokio::net::TcpStream::connect(addr).await.unwrap();
        auth_handshake_send(&mut client, "test-shared-secret")
            .await
            .unwrap();
        server.await.unwrap().unwrap();
        assert_eq!((), (), "shared token must be accepted");

        // Wrong token: rejected on the verify side and an error on the
        // send side (the peer answers AUTH_FAIL).
        let listener2 = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr2 = listener2.local_addr().unwrap();
        let server2 = tokio::spawn(async move {
            let (mut stream, _) = listener2.accept().await.unwrap();
            tokio::time::timeout(
                std::time::Duration::from_secs(5),
                auth_handshake_verify(&mut stream),
            )
            .await
        });
        let mut client2 = tokio::net::TcpStream::connect(addr2).await.unwrap();
        let send_err = auth_handshake_send(&mut client2, "wrong-token")
            .await
            .expect_err("wrong token must fail the handshake");
        assert!(
            send_err.contains("rejected"),
            "send side must report the rejection: {send_err}"
        );
        let verify_err = server2.await.unwrap().unwrap().unwrap_err();
        assert!(
            verify_err.contains("mismatch"),
            "verify side must report the mismatch: {verify_err}"
        );
    }

    /// The listener's auth gate, driven through a real socket: a peer
    /// with the wrong token must be rejected (no frame may be served)
    /// and a peer with the shared token must pass. This pins the
    /// nested-result flatten in `cross_data_auth_gate` — a regression
    /// there (binding only the outer timeout, as the listener did
    /// before) would let a mismatched peer through, while the
    /// direct-call tests of `auth_handshake_verify` would stay green.
    #[tokio::test]
    async fn auth_gate_rejects_wrong_token_through_real_socket() {
        let _env_lock = CROSS_DATA_AUTH_ENV_LOCK.lock().unwrap();
        unsafe { std::env::set_var("DORA_MEMORY_POOL_AUTH_TOKEN", "test-shared-secret") };
        struct EnvGuard;
        impl Drop for EnvGuard {
            fn drop(&mut self) {
                unsafe { std::env::remove_var("DORA_MEMORY_POOL_AUTH_TOKEN") };
            }
        }
        let _guard = EnvGuard;

        // Wrong token: the gate must reject, mirroring the listener's
        // drop-before-serving behavior.
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            cross_data_auth_gate(&mut stream).await
        });
        let mut client = tokio::net::TcpStream::connect(addr).await.unwrap();
        auth_handshake_send(&mut client, "wrong-token")
            .await
            .expect_err("wrong token must fail the send side");
        let gate_err = server.await.unwrap().unwrap_err();
        assert!(
            gate_err.contains("mismatch"),
            "gate must reject the mismatched peer: {gate_err}"
        );

        // Correct token: the gate must pass, exactly as the listener
        // would then serve frames.
        let listener2 = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr2 = listener2.local_addr().unwrap();
        let server2 = tokio::spawn(async move {
            let (mut stream, _) = listener2.accept().await.unwrap();
            cross_data_auth_gate(&mut stream).await
        });
        let mut client2 = tokio::net::TcpStream::connect(addr2).await.unwrap();
        auth_handshake_send(&mut client2, "test-shared-secret")
            .await
            .unwrap();
        server2.await.unwrap().unwrap();
        assert_eq!((), (), "shared token must pass the gate");
    }
}
