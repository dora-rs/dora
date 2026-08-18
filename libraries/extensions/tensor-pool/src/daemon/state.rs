//! The daemon-side state this extension owns.
//!
//! One field on `Daemon` rather than six, so the whole subsystem — and the
//! `dora-tensor-pool` dependency behind it — compiles out with the
//! `tensor-pool` feature.

use super::*;

/// A dataflow's memory-pool zenoh subscriber, plus the liveness flag the
/// detached mirror-creation tasks it feeds check before they publish a
/// mirror into the tensor pool.
///
/// Mirror creation runs off the event loop, so a `finish_dataflow` can
/// land between the handler's liveness check and the task's
/// `register_cross_pool`. Without the flag that task would register a
/// segment *behind* the finish-time drain, leaking it for the daemon's
/// lifetime (dora-rs/dora#3194).
pub(crate) struct MemoryPoolSubscriber {
    /// The receive loop has no shutdown branch of its own, so a dropped
    /// handle would leak the task, its session clone, and its event
    /// sender for the daemon's lifetime.
    pub(crate) handle: tokio::task::JoinHandle<()>,
    /// Cloned into every mirror task; cleared by [`Self::shutdown`].
    pub(crate) dataflow_live: Arc<AtomicBool>,
}

impl MemoryPoolSubscriber {
    /// Stop feeding this dataflow: clear the liveness flag, then abort
    /// the receive loop.
    ///
    /// The flag is cleared *before* the caller drains the cross-pool
    /// table, which is what makes the drain authoritative: a mirror task
    /// that still reads `true` after its `register_cross_pool` is
    /// guaranteed to be visible to that drain, and one that reads `false`
    /// rolls its own registration back.
    pub(crate) fn shutdown(self) {
        self.dataflow_live.store(false, atomic::Ordering::SeqCst);
        self.handle.abort();
    }
}

pub struct PoolState {
    /// Cross-machine memory-pool state: the `cross_pools` table (which pools
    /// this daemon mirrors, and who their peer machine is). The main pool
    /// table is node-side; the daemon only ever needs the cross-machine
    /// entries (mirror tracking, direct-TCP gating, targeted frees).
    pub(crate) tensor_pool: TensorPoolManager,
    /// SHM provider for inter-daemon control notifications. Same-host daemons
    /// receive the payload as a zero-copy shared-memory reference; cross-host
    /// receivers get an implicit regular-buffer copy from the zenoh transport
    /// (SHM only works within a host). `None` when the provider could not be
    /// created — control messages then fall back to regular payloads.
    pub(crate) shm_provider: Option<Arc<ShmProvider<PosixShmProviderBackend>>>,
    /// Per-dataflow zenoh subscriber tasks, each with the liveness flag its
    /// detached mirror tasks check before registering (#3194).
    pub(crate) subscribers: HashMap<DataflowId, MemoryPoolSubscriber>,
    /// Port of this daemon's direct-TCP data listener (the mirror side of the
    /// cross-machine data plane), when it was started. Reported to the origin
    /// in `PeerMessage::RegisterAck.data_port`.
    pub(crate) cross_data_listener_port: Option<u16>,
    /// Persistent direct-TCP connections to peer daemons' data listeners (the
    /// origin side). Keyed by the peer's `SocketAddr`; a dead connection is
    /// dropped on write failure and re-established lazily.
    pub(crate) cross_data_conns:
        Arc<std::sync::Mutex<HashMap<std::net::SocketAddr, tokio::net::TcpStream>>>,
    /// Direct-TCP endpoint per cross-machine pool: `(dataflow id, pool id)` ->
    /// peer's `SocketAddr`. Populated when the register ack carries a data
    /// port; absent pools fall back to the zenoh relay.
    pub(crate) cross_data_endpoints:
        Arc<std::sync::Mutex<HashMap<(Uuid, String), std::net::SocketAddr>>>,
}

impl PoolState {
    /// Build the state, including the zenoh SHM provider.
    ///
    /// Provider failure is non-fatal: control messages fall back to regular
    /// payloads (see [`publish_pool_message`]).
    pub fn new() -> Self {
        let shm_provider =
            match ShmProviderBuilder::default_backend(MEMORY_POOL_SHM_PROVIDER_SIZE).wait() {
                Ok(provider) => Some(Arc::new(provider)),
                Err(e) => {
                    tracing::warn!(
                        "memory pool: zenoh SHM provider creation failed ({e}); \
                         control events will use regular payloads"
                    );
                    None
                }
            };
        Self {
            tensor_pool: TensorPoolManager::new(),
            shm_provider,
            subscribers: HashMap::new(),
            cross_data_listener_port: None,
            cross_data_conns: Arc::new(std::sync::Mutex::new(HashMap::new())),
            cross_data_endpoints: Arc::new(std::sync::Mutex::new(HashMap::new())),
        }
    }

    /// Reclaim `/dev/shm` segments left by a previous incarnation of this
    /// daemon. Only this machine's own prefix is swept, so a sibling daemon's
    /// live segments are never touched.
    ///
    /// Do not widen this gate: the prefix also matches a *node's own* local
    /// pool segment (a node with `DORA_MACHINE_ID` set names it the same way),
    /// and on the `dora up` path nodes deliberately outlive a daemon restart
    /// (see `daemon-reconnect-e2e`).
    pub fn sweep_orphans_at_startup(machine_id: Option<&str>) {
        #[cfg(target_os = "linux")]
        if cross_machine_enabled()
            && let Some(machine_id) = machine_id.filter(|m| !m.is_empty())
        {
            cleanup_orphan_mirrors(machine_id);
        }
        #[cfg(not(target_os = "linux"))]
        let _ = machine_id;
    }

    /// Reclaim segments a previous crash of *this dataflow's* nodes left
    /// behind. Scoped to the nodes this daemon spawns, since a co-located
    /// daemon may be starting the other half of the same dataflow right now.
    pub fn sweep_orphans_for_dataflow(dataflow_id: Uuid, is_local_node: impl Fn(&str) -> bool) {
        TensorPoolManager::cleanup_orphans(&dataflow_id.to_string(), is_local_node);
    }

    /// Stop feeding a dataflow whose spawn failed before it started running.
    pub fn abort_subscriber(&mut self, dataflow_id: &DataflowId) {
        if let Some(subscriber) = self.subscribers.remove(dataflow_id) {
            subscriber.shutdown();
        }
    }

    /// Release bookkeeping on daemon exit. Mirror segments are deliberately
    /// NOT unlinked: they may still be open by a peer daemon's readers on a
    /// shared host, and [`Self::sweep_orphans_at_startup`] sweeps this
    /// machine's own leftovers on the next start.
    pub fn cleanup_all(&self) {
        self.tensor_pool.cleanup_all().ok();
    }
}

impl Default for PoolState {
    fn default() -> Self {
        Self::new()
    }
}
