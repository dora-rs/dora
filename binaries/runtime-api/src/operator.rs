use dora_core::{
    config::{DataId, NodeId},
    descriptor::{Descriptor, OperatorDefinition},
};
use dora_node_api::{DoraArray, EncodedSample, Event, MetadataParameters, SampleAllocator};
use eyre::Result;
use std::any::Any;
use std::sync::{Arc, OnceLock};
use tokio::sync::{mpsc::Sender, oneshot};

/// Shared slot holding the node's [`SampleAllocator`].
///
/// The operator threads are started *before* `DoraNode::init` runs (the node
/// only reports ready once every operator has initialized), so the allocator
/// cannot be passed in at spawn time. The runtime fills this slot as soon as the
/// node exists, which is strictly before any input — and therefore any output —
/// can reach an operator.
pub type SharedAllocator = Arc<OnceLock<SampleAllocator>>;

/// An operator's side of the runtime: where to send events, and where to get
/// output samples from.
#[derive(Clone)]
pub struct RuntimeHandle {
    events_tx: Sender<OperatorEvent>,
    allocator: SharedAllocator,
}

impl RuntimeHandle {
    pub fn new(events_tx: Sender<OperatorEvent>, allocator: SharedAllocator) -> Self {
        Self {
            events_tx,
            allocator,
        }
    }

    /// Encode `array` into a dora-owned sample **on the calling (operator)
    /// thread** and hand that to the runtime.
    ///
    /// This is the only place an `OperatorEvent::Output` is built, so the
    /// ownership invariant that fixes dora-rs/dora#2742 holds for every operator
    /// backend: `array` is borrowed, never sent, and the caller drops it while
    /// it still holds whatever its language runtime needs to free it.
    pub fn send_output(
        &self,
        output_id: DataId,
        parameters: MetadataParameters,
        array: &DoraArray,
    ) -> Result<()> {
        let allocator = self
            .allocator
            .get()
            .ok_or_else(|| eyre::eyre!("cannot send an output before the node is initialized"))?;
        let encoded = allocator.encode_arrow(array)?;
        self.events_tx
            .blocking_send(OperatorEvent::Output {
                output_id,
                parameters,
                encoded,
            })
            .map_err(|_| eyre::eyre!("failed to send output to runtime"))
    }

    /// Report a lifecycle event (finished, error, panic) to the runtime.
    pub fn report(&self, event: OperatorEvent) {
        let _ = self.events_tx.blocking_send(event);
    }
}

/// A language/ABI-specific operator backend.
///
/// Each runtime backend (shared-library, Python, WASM, third-party, …)
/// implements this trait and hands it to [`crate::main`], which drives the
/// language-neutral event loop. The implementation is invoked once, on the
/// **main thread** (PyO3 and `libloading` both want a dedicated thread), and is
/// responsible for loading the operator described by `operator` and running it
/// until it stops.
///
/// The runtime↔operator contract is language-neutral: consume
/// [`dora_node_api::Event`]s off `incoming_events`, emit outputs and lifecycle
/// events through `handle`, and signal readiness (or an init failure) exactly
/// once on `init_done`.
///
/// A backend that cannot host `operator`'s source kind must return an `Err`
/// **without** signalling `init_done`, so the failure surfaces as a spawn error
/// rather than a runtime hang (dora-rs/dora#2595).
pub trait OperatorRunner {
    fn run_operator(
        &self,
        node_id: &NodeId,
        operator: OperatorDefinition,
        incoming_events: flume::Receiver<Event>,
        handle: RuntimeHandle,
        init_done: oneshot::Sender<Result<()>>,
        dataflow_descriptor: &Descriptor,
    ) -> eyre::Result<RunnerGuard>;
}

/// A backend-owned resource that must stay alive until the runtime's event loop
/// has joined.
///
/// The shared-library backend returns its loaded `libloading::Library` here.
/// Since dora-rs/dora#2742 the main loop no longer holds Arrow arrays exported
/// by the operator — outputs are encoded into dora-owned samples on the operator
/// thread (see [`RuntimeHandle::send_output`]) — but other values can still
/// carry `.so`-resident vtables across the channel, most notably an
/// [`OperatorEvent::Panic`] payload. Unloading the library while the loop may
/// still hold one dangles those, so [`crate::main`] binds the guard for the
/// whole run and drops it last.
///
/// It is deliberately opaque (`Box<dyn Any>`) so `dora-runtime-api` stays
/// language-neutral — it never needs to name `libloading` or any other
/// backend-specific type. Backends with nothing to keep alive return `None`.
pub type RunnerGuard = Option<Box<dyn Any>>;

#[derive(Debug)]
#[allow(clippy::large_enum_variant)]
pub enum OperatorEvent {
    Output {
        output_id: DataId,
        parameters: MetadataParameters,
        /// The payload, already IPC-encoded into a dora-owned sample **by the
        /// operator thread** (see [`RuntimeHandle::send_output`]).
        ///
        /// Deliberately not an `ArrayData`: an operator's array may be backed by
        /// memory its own language runtime owns, and releasing that memory can
        /// need the owning runtime (a `pyarrow` array over a numpy buffer takes
        /// the GIL to drop). Freeing it here would let an operator that holds
        /// the GIL stall the runtime's event loop — dora-rs/dora#2742.
        encoded: EncodedSample,
    },
    Error(eyre::Error),
    Panic(Box<dyn Any + Send>),
    Finished {
        reason: StopReason,
    },
}

#[derive(Debug)]
pub enum StopReason {
    InputsClosed,
    ExplicitStop,
    ExplicitStopAll,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The #2742 invariant, at the level the fix actually lives: an operator's
    /// array is *borrowed* by `send_output`, never sent. By the time the event
    /// is on the channel the source payload has been released — on this thread,
    /// where the operator still holds whatever its language runtime needs to
    /// free it — and what crosses is a dora-owned `EncodedSample`.
    #[test]
    fn send_output_releases_the_operator_payload_before_it_crosses_the_channel() {
        use arrow::buffer::Buffer;
        use std::ptr::NonNull;
        use std::sync::atomic::{AtomicBool, Ordering};

        /// Stands in for a foreign owner of the payload — numpy behind pyarrow,
        /// or a buffer owned by an operator's `.so`.
        struct ForeignOwner {
            released: Arc<AtomicBool>,
            _backing: Vec<u8>,
        }
        impl Drop for ForeignOwner {
            fn drop(&mut self) {
                self.released.store(true, Ordering::SeqCst);
            }
        }

        let backing = vec![0xCDu8; 4096];
        let ptr = NonNull::new(backing.as_ptr() as *mut u8).expect("non-null");
        let released = Arc::new(AtomicBool::new(false));
        let owner = Arc::new(ForeignOwner {
            released: released.clone(),
            _backing: backing,
        });
        let buffer = unsafe { Buffer::from_custom_allocation(ptr, 4096, owner) };
        let array = dora_node_api::DoraArray::from_array(arrow::array::make_array(
            arrow::array::ArrayData::builder(dora_node_api::arrow_v59::datatypes::DataType::UInt8)
                .len(4096)
                .add_buffer(buffer)
                .build()
                .expect("valid UInt8 array"),
        ));

        let (events_tx, mut events_rx) = tokio::sync::mpsc::channel(1);
        let allocator: SharedAllocator = Arc::new(OnceLock::new());
        allocator
            .set(SampleAllocator::heap())
            .expect("fresh OnceLock");
        let handle = RuntimeHandle::new(events_tx, allocator);

        handle
            .send_output(
                DataId::from("out".to_string()),
                MetadataParameters::default(),
                &array,
            )
            .expect("send_output");

        // The operator drops its array here, on its own thread.
        drop(array);
        assert!(
            released.load(Ordering::SeqCst),
            "the queued event must not keep the operator's payload alive"
        );

        match events_rx.try_recv().expect("an output event was queued") {
            OperatorEvent::Output { encoded, .. } => {
                assert!(
                    encoded.as_bytes().len() > 4096,
                    "the sample must hold the encoded payload, not a reference to it"
                );
            }
            other => panic!("expected an Output event, got {other:?}"),
        }
    }

    /// Sending before the node exists must fail with a clear message rather
    /// than panic.
    #[test]
    fn send_output_before_node_init_is_a_clear_error() {
        let (events_tx, _events_rx) = tokio::sync::mpsc::channel(1);
        let handle = RuntimeHandle::new(events_tx, SharedAllocator::default());
        let array = dora_node_api::DoraArray::from_array(arrow::array::NullArray::new(1));

        let err = handle
            .send_output(
                DataId::from("out".to_string()),
                MetadataParameters::default(),
                &array,
            )
            .expect_err("no allocator yet");
        assert!(
            err.to_string().contains("before the node is initialized"),
            "unexpected error: {err}"
        );
    }
}
