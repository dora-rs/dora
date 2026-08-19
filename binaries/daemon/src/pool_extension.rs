//! Wiring for the tensor-pool extension.
//!
//! The extension lives in `dora-tensor-pool` and reaches this daemon only
//! through [`DaemonServices`]. This file is the whole of dora's side of that
//! seam: the capability implementations, and the call sites that drive the
//! extension's lifecycle. Nothing pool-shaped — no segment layout, no CUDA,
//! no `unsafe` — appears in this crate.

use std::{net::SocketAddr, sync::Arc};

use dora_core::uhlc::HLC;
use dora_message::{
    daemon_to_daemon::InterDaemonEvent, daemon_to_node::DaemonReply, id::NodeId,
    node_to_daemon::Timestamped,
};
use dora_tensor_pool::daemon::{BoxFuture, DaemonServices, PeerMessageSink, PoolState};
use uuid::Uuid;

use crate::{
    Daemon, Event, coordinator,
    extension_table::{ExtensionKey, ExtensionTable},
};

/// Borrows exactly the daemon state the extension is granted, so the
/// extension can hold `&mut` on it without borrowing the whole `Daemon`.
struct Services<'a> {
    machine_id: Option<String>,
    zenoh_session: zenoh::Session,
    clock: Arc<HLC>,
    events_tx: tokio::sync::mpsc::Sender<Timestamped<Event>>,
    coordinator_sender: Option<coordinator::CoordinatorSender>,
    extensions: &'a mut ExtensionTable,
    running: &'a std::collections::HashMap<
        dora_message::DataflowId,
        crate::running_dataflow::RunningDataflow,
    >,
}

impl DaemonServices for Services<'_> {
    fn machine_id(&self) -> Option<String> {
        self.machine_id.clone()
    }

    fn zenoh_session(&self) -> zenoh::Session {
        self.zenoh_session.clone()
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
        owner: &str,
    ) -> Result<(), String> {
        let owner = NodeId::from(owner.to_owned());
        self.extensions
            .store(ext_key(dataflow_id, namespace, key), value, &owner)
    }

    fn extension_drop_notify(&mut self, dataflow_id: Uuid, namespace: &str, key: &str) {
        let key = ext_key(dataflow_id, namespace, key);
        let dataflow = self.running.get(&dataflow_id);
        crate::drop_extension_and_notify(self.extensions, dataflow, &key, &self.clock);
    }

    fn resolve_machine(&self, machine_id: String) -> BoxFuture<Option<SocketAddr>> {
        let sender = self.coordinator_sender.clone();
        let clock = self.clock.clone();
        Box::pin(async move {
            let sender = sender.as_ref()?;
            coordinator::resolve_machine(sender, &clock, &machine_id).await
        })
    }

    fn peer_message_sink(&self) -> PeerMessageSink {
        let events_tx = self.events_tx.clone();
        Arc::new(move |event: Timestamped<InterDaemonEvent>| {
            let events_tx = events_tx.clone();
            Box::pin(async move {
                let _ = events_tx
                    .send(Timestamped {
                        inner: Event::Daemon(event.inner),
                        timestamp: event.timestamp,
                    })
                    .await;
            }) as BoxFuture<()>
        })
    }
}

fn ext_key(dataflow_id: Uuid, namespace: &str, key: &str) -> ExtensionKey {
    ExtensionKey {
        dataflow_id: dataflow_id.to_string(),
        namespace: namespace.to_string(),
        key: key.to_string(),
    }
}

impl Daemon {
    /// Split `&mut self` into the extension's state and the capabilities it
    /// is granted, so both can be borrowed at once.
    fn pool_split(&mut self) -> (&mut PoolState, Services<'_>) {
        let services = Services {
            machine_id: self.machine_id.clone(),
            zenoh_session: self.zenoh_session.clone(),
            clock: self.clock.clone(),
            events_tx: self.events_tx.clone(),
            coordinator_sender: self.coordinator_sender.clone(),
            extensions: &mut self.extensions,
            running: &self.running,
        };
        (&mut self.pool, services)
    }

    /// Hand one `InterDaemonEvent::ExtensionMessage` to its extension.
    pub(crate) async fn handle_extension_message(
        &mut self,
        dataflow_id: Uuid,
        namespace: String,
        target_machine: Option<String>,
        payload: Vec<u8>,
    ) -> eyre::Result<()> {
        let (pool, mut services) = self.pool_split();
        pool.handle_extension_message(
            &mut services,
            dataflow_id,
            namespace,
            target_machine,
            payload,
        )
        .await
    }

    /// Hand one `DaemonRequest::ExtensionRequest` to its extension.
    pub(crate) async fn handle_extension_request(
        &mut self,
        dataflow_id: Uuid,
        node_id: NodeId,
        namespace: String,
        payload: Vec<u8>,
        reply_sender: tokio::sync::oneshot::Sender<DaemonReply>,
    ) -> eyre::Result<()> {
        let (pool, mut services) = self.pool_split();
        pool.handle_extension_request(
            &mut services,
            dataflow_id,
            node_id,
            namespace,
            payload,
            reply_sender,
        )
        .await
    }

    /// Let the extension subscribe to this dataflow's peer traffic.
    pub(crate) fn pool_subscribe_dataflow(&mut self, dataflow_id: Uuid) {
        let (pool, mut services) = self.pool_split();
        pool.subscribe_dataflow(&mut services, dataflow_id);
    }

    /// Let the extension release this dataflow's state.
    pub(crate) async fn pool_cleanup_dataflow(&mut self, dataflow_id: Uuid) {
        self.pool.cleanup_dataflow(dataflow_id).await;
    }
}
