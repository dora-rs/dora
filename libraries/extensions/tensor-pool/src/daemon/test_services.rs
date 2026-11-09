//! An in-memory [`DaemonServices`] for driving the daemon half in tests.

use super::*;

/// Records what the extension stores and drops, and hands everything posted
/// through the [`PeerMessageSink`] to the test, which plays the event loop.
pub(crate) struct FakeServices {
    pub machine_id: String,
    pub session: zenoh::Session,
    pub clock: Arc<HLC>,
    pub table: HashMap<(Uuid, String), Vec<u8>>,
    pub dropped: Vec<(Uuid, String)>,
    pub sink_tx: tokio::sync::mpsc::UnboundedSender<Timestamped<InterDaemonEvent>>,
}

impl FakeServices {
    /// A daemon on `machine_id` with a zenoh session that neither scouts
    /// nor listens, so the test cannot reach (or be reached by) any other
    /// zenoh instance on the host. Returns the receiving end of the sink.
    pub async fn new(
        machine_id: &str,
    ) -> (
        Self,
        tokio::sync::mpsc::UnboundedReceiver<Timestamped<InterDaemonEvent>>,
    ) {
        let mut cfg = zenoh::Config::default();
        cfg.insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        cfg.insert_json5("scouting/gossip/enabled", "false")
            .unwrap();
        cfg.insert_json5("listen/endpoints", "{ peer: [] }")
            .unwrap();
        let session = zenoh::open(cfg).await.unwrap();
        let (sink_tx, sink_rx) = tokio::sync::mpsc::unbounded_channel();
        let services = Self {
            machine_id: machine_id.to_string(),
            session,
            clock: Arc::new(HLC::default()),
            table: HashMap::new(),
            dropped: Vec::new(),
            sink_tx,
        };
        (services, sink_rx)
    }
}

impl DaemonServices for FakeServices {
    fn machine_id(&self) -> Option<String> {
        Some(self.machine_id.clone())
    }

    fn zenoh_session(&self) -> zenoh::Session {
        self.session.clone()
    }

    fn clock(&self) -> Arc<HLC> {
        self.clock.clone()
    }

    fn extension_store(
        &mut self,
        dataflow_id: Uuid,
        namespace: &str,
        key: &str,
        value: Vec<u8>,
        _owner: &str,
    ) -> Result<(), String> {
        assert_eq!(namespace, NAMESPACE);
        self.table.insert((dataflow_id, key.to_string()), value);
        Ok(())
    }

    fn extension_drop_notify(&mut self, dataflow_id: Uuid, namespace: &str, key: &str) {
        assert_eq!(namespace, NAMESPACE);
        self.table.remove(&(dataflow_id, key.to_string()));
        self.dropped.push((dataflow_id, key.to_string()));
    }

    fn resolve_machine(&self, _machine_id: String) -> BoxFuture<Option<std::net::SocketAddr>> {
        Box::pin(async { None })
    }

    fn peer_message_sink(&self) -> PeerMessageSink {
        let tx = self.sink_tx.clone();
        Arc::new(move |event| {
            let _ = tx.send(event);
            Box::pin(async {}) as BoxFuture<()>
        })
    }
}

/// Serializes tests that enable the cross-machine opt-in through the
/// process-wide environment.
static CROSS_MACHINE_ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// Turn the cross-machine data plane on for the calling test. The data
/// listener binds an ephemeral loopback port so parallel tests never fight
/// over the default one. Held for the whole test.
pub(crate) fn enable_cross_machine() -> std::sync::MutexGuard<'static, ()> {
    let guard = CROSS_MACHINE_ENV_LOCK
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    // SAFETY (env mutation in tests): every test that sets these holds
    // `CROSS_MACHINE_ENV_LOCK`, and no other test depends on them
    // being unset.
    unsafe {
        std::env::set_var(CROSS_MACHINE_ENV, "1");
        std::env::set_var(CROSS_DATA_PORT_ENV, "0");
        std::env::set_var(CROSS_DATA_BIND_ENV, "127.0.0.1");
    }
    guard
}
