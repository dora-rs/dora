//! What this extension needs from the daemon hosting it.
//!
//! The seam runs in one direction by design: dora hands the extension
//! opaque bytes and the few daemon-only capabilities below, and the
//! extension hands back opaque bytes. It has no access to `Daemon`, to the
//! running-dataflow table, or to anything else in the daemon's state — so
//! nothing here can grow into the pool-shaped socket in dora's core that
//! `docs/extensions.md` exists to prevent.
//!
//! Two of the seven are the reason an extension needs a daemon at all: a
//! node that crashed cannot withdraw the descriptor it published, so
//! [`DaemonServices::extension_store`] and
//! [`DaemonServices::extension_drop_notify`] put that descriptor's lifetime
//! in the daemon's hands (dora-rs/dora#2881 is the failure mode with a real
//! transport attached). The rest are plumbing the extension cannot reach on
//! its own: the machine identity and coordinator topology dora owns, and
//! the channels it already holds open.

use std::{net::SocketAddr, pin::Pin, sync::Arc};

use dora_core::uhlc::HLC;
use dora_message::{daemon_to_daemon::InterDaemonEvent, node_to_daemon::Timestamped};
use uuid::Uuid;

/// A future the extension awaits without knowing what produced it.
pub type BoxFuture<T> = Pin<Box<dyn Future<Output = T> + Send>>;

/// Hands a decoded peer message back to the daemon's event loop.
///
/// The extension's per-dataflow subscriber task decodes what arrives on the
/// zenoh topic but cannot construct the daemon's internal event type, so it
/// posts through this instead.
pub type PeerMessageSink =
    Arc<dyn Fn(Timestamped<InterDaemonEvent>) -> BoxFuture<()> + Send + Sync>;

/// The daemon capabilities this extension is granted. Implemented by the
/// daemon; the extension only ever sees the trait.
pub trait DaemonServices: Send {
    /// This daemon's machine id, as registered with the coordinator.
    /// `None` on a daemon that never registered one.
    fn machine_id(&self) -> Option<String>;

    /// The daemon's zenoh session. Cheap to clone — it is `Arc`-backed.
    fn zenoh_session(&self) -> zenoh::Session;

    /// The daemon's hybrid logical clock, for stamping published messages.
    fn clock(&self) -> Arc<HLC>;

    /// Store `value` under `(dataflow_id, namespace, key)` in the daemon's
    /// extension table, owned by `owner`.
    ///
    /// The daemon reclaims it when `owner` exits and when the dataflow
    /// finishes — the one thing a crashed node cannot do for itself.
    fn extension_store(
        &mut self,
        dataflow_id: Uuid,
        namespace: &str,
        key: &str,
        value: Vec<u8>,
        owner: &str,
    ) -> Result<(), String>;

    /// Drop `(dataflow_id, namespace, key)` and notify every node that
    /// stored or read it, so each can release what it referred to promptly
    /// rather than at process exit.
    fn extension_drop_notify(&mut self, dataflow_id: Uuid, namespace: &str, key: &str);

    /// Resolve a machine id to the address of the daemon registered for it.
    /// `None` when the machine is unknown or no coordinator is reachable.
    fn resolve_machine(&self, machine_id: String) -> BoxFuture<Option<SocketAddr>>;

    /// Sink for messages this extension's subscriber task decodes.
    fn peer_message_sink(&self) -> PeerMessageSink;
}
