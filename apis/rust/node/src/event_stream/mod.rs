use std::{
    collections::{BTreeMap, HashMap, VecDeque},
    path::PathBuf,
    pin::pin,
    sync::Arc,
    time::{Duration, Instant},
};

use dora_message::{
    DataflowId,
    daemon_to_node::{DaemonCommunication, DaemonReply, DataMessage, NodeEvent},
    id::DataId,
    node_to_daemon::{DaemonRequest, Timestamped},
};
pub use event::{Event, StopCause};
use futures::{
    FutureExt, Stream,
    future::{Either, select},
};
use futures_timer::Delay;
use scheduler::{NON_INPUT_EVENT, Scheduler};

use self::thread::{EventItem, EventStreamThreadHandle};
use crate::{
    DaemonCommunicationWrapper, PatternError,
    daemon_connection::{DaemonChannel, node_integration_testing::convert_output_to_json},
    event_stream::data_conversion::RawData,
    node::{ZENOH_TEARDOWN_TIMEOUT, teardown_with_timeout},
};
use dora_arrow_convert::IntoArrow;
use dora_core::{
    config::{Input, NodeId},
    uhlc,
};
use eyre::{Context, eyre};

pub use scheduler::Scheduler as EventScheduler;

mod data_conversion;
mod event;
pub mod input_tracker;
pub mod merged;
mod scheduler;
mod thread;

/// Asynchronous iterator over the incoming [`Event`]s destined for this node.
///
/// This struct [implements](#impl-Stream-for-EventStream) the [`Stream`] trait,
/// so you can use methods of the [`StreamExt`](futures::StreamExt) trait
/// on this struct. A common pattern is `while let Some(event) = event_stream.next().await`.
///
/// Nodes should iterate over this event stream and react to events that they are interested in.
/// Typically, the most important event type is [`Event::Input`].
/// You don't need to handle all events, it's fine to ignore events that are not relevant to your node.
///
/// The event stream will close itself after a [`Event::Stop`] was received.
/// A manual `break` on [`Event::Stop`] is typically not needed.
/// _(You probably do need to use a manual `break` on stop events when using the
/// [`StreamExt::merge`][`futures_concurrency::stream::StreamExt::merge`] implementation on
/// [`EventStream`] to combine the stream with an external one.)_
///
/// Once the event stream finished, nodes should exit.
/// Note that Dora kills nodes that don't exit quickly after a [`Event::Stop`] of type
/// [`StopCause::Manual`] was received.
pub struct EventStream {
    node_id: NodeId,
    // Drop order: Rust drops fields in declaration order (top to bottom).
    // receiver must drop FIRST so tx_clone.send() in subscriber threads
    // returns Err, causing threads to exit before JoinHandles are dropped.
    // Using `tokio::sync::mpsc::Receiver` — instead of `flume` — avoids the
    // AB-BA deadlock between flume 0.10's spinlock and pyo3's GIL-acquiring
    // waker when a Python coroutine polls this stream
    // (upstream dora-rs/dora#1603).
    receiver: tokio::sync::mpsc::Receiver<EventItem>,
    _thread_handle: EventStreamThreadHandle,
    /// Callback subscribers — kept alive for the lifetime of the
    /// EventStream. Dropping a subscriber undeclares it and stops further
    /// callbacks. The callbacks use `try_send` (never block), so undeclaring
    /// does not depend on `receiver` being dropped first. `EventStream::drop`
    /// explicitly tears these down under a deadline (see there); by the time
    /// this field drops it is already empty.
    _zenoh_subscribers: Vec<zenoh::pubsub::Subscriber<()>>,
    /// Per-input `@schema` subscribers (zenoh-ext AdvancedSubscriber) that prime
    /// the data subscribers' decoders. Kept alive like `_zenoh_subscribers`.
    _zenoh_schema_subscribers: Vec<zenoh_ext::AdvancedSubscriber<()>>,
    /// Per-input liveliness tokens announcing "my data subscriber is declared"
    /// so producers can count subscribers and only switch an output to the direct
    /// zenoh data plane once every subscriber is wired (startup no-loss barrier;
    /// see [`dora_core::topics::zenoh_input_ready_liveliness_topic`]). Kept alive
    /// for the EventStream's lifetime; dropping a token undeclares it.
    _zenoh_liveliness_tokens: Vec<zenoh::liveliness::LivelinessToken>,
    close_channel: DaemonChannel,
    clock: Arc<uhlc::HLC>,
    scheduler: Scheduler,
    write_events_to: Option<WriteEventsTo>,
    start_timestamp: uhlc::Timestamp,
    use_scheduler: bool,
    /// Expected input types from YAML descriptor (for first-message validation).
    /// Each input is checked once; after the first message, the entry is removed.
    input_type_checks: HashMap<DataId, arrow_schema::DataType>,
    /// Events that were consumed by a pattern-aware helper
    /// (`recv_service_response`, `recv_action_result`) while searching
    /// for a correlation match. Drained first on the next `recv()` so
    /// the caller's main event loop never loses intermediate events
    /// (dora-rs/adora#148).
    pending_passthrough: std::collections::VecDeque<Event>,
    /// Set to true after an `Event::Stop` has been delivered. Zenoh
    /// subscriber threads hold clones of the event channel sender, so
    /// the daemon thread's sender drop alone is not enough to close
    /// the receiver — subsequent `recv_async`/`poll_next` would hang.
    /// Returning `None` here lets the caller exit and drops the
    /// `EventStream`, which signals subscriber shutdown.
    stop_received: bool,
}

impl EventStream {
    #[allow(clippy::too_many_arguments)]
    #[tracing::instrument(level = "trace", skip(clock, zenoh_session))]
    pub(crate) fn init(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        daemon_communication: &DaemonCommunicationWrapper,
        input_config: BTreeMap<DataId, Input>,
        input_types: &BTreeMap<DataId, String>,
        clock: Arc<uhlc::HLC>,
        write_events_to: Option<PathBuf>,
        zenoh_session: Option<&zenoh::Session>,
        dynamic: bool,
    ) -> eyre::Result<Self> {
        let channel = match daemon_communication {
            DaemonCommunicationWrapper::Standard(daemon_communication) => {
                match daemon_communication {
                    DaemonCommunication::Tcp { socket_addr } => {
                        DaemonChannel::new_tcp(*socket_addr).wrap_err_with(|| {
                            format!("failed to connect event stream for node `{node_id}`")
                        })?
                    }

                    DaemonCommunication::Interactive => {
                        DaemonChannel::Interactive(Default::default())
                    }
                }
            }

            DaemonCommunicationWrapper::Testing { channel } => {
                DaemonChannel::IntegrationTestChannel(channel.clone())
            }
        };

        let close_channel = match daemon_communication {
            DaemonCommunicationWrapper::Standard(daemon_communication) => {
                match daemon_communication {
                    DaemonCommunication::Tcp { socket_addr } => {
                        DaemonChannel::new_tcp(*socket_addr).wrap_err_with(|| {
                            format!("failed to connect event close channel for node `{node_id}`")
                        })?
                    }
                    DaemonCommunication::Interactive => {
                        DaemonChannel::Interactive(Default::default())
                    }
                }
            }
            DaemonCommunicationWrapper::Testing { channel } => {
                DaemonChannel::IntegrationTestChannel(channel.clone())
            }
        };

        let mut queue_size_limit: HashMap<DataId, (usize, VecDeque<EventItem>)> = input_config
            .iter()
            .map(|(input, config)| {
                (
                    input.clone(),
                    (
                        config
                            .queue_size
                            .unwrap_or(dora_message::config::DEFAULT_QUEUE_SIZE),
                        VecDeque::new(),
                    ),
                )
            })
            .collect();

        queue_size_limit.insert(
            DataId::from(NON_INPUT_EVENT.to_string()),
            (1_000, VecDeque::new()),
        );

        let queue_policies: HashMap<DataId, dora_message::config::QueuePolicy> = input_config
            .iter()
            .filter_map(|(input, config)| config.queue_policy.map(|p| (input.clone(), p)))
            .collect();

        let scheduler = Scheduler::with_policies(queue_size_limit, queue_policies);

        let total_queue_capacity: usize = input_config
            .values()
            .map(|c| {
                c.queue_size
                    .unwrap_or(dora_message::config::DEFAULT_QUEUE_SIZE)
            })
            .sum::<usize>()
            .max(64);

        let write_events_to = match write_events_to {
            Some(path) => {
                if let Some(parent) = path.parent() {
                    std::fs::create_dir_all(parent).wrap_err_with(|| {
                        format!(
                            "failed to create parent directories for event output file `{}` for node `{}`",
                            path.display(),
                            node_id
                        )
                    })?;
                }

                let file = std::fs::File::create(&path).wrap_err_with(|| {
                    format!(
                        "failed to create event output file `{}` for node `{}`",
                        path.display(),
                        node_id
                    )
                })?;

                Some(WriteEventsTo {
                    node_id: node_id.clone(),
                    file,
                    events_buffer: Vec::new(),
                    poisoned: None,
                })
            }
            None => None,
        };

        // Resolve input type URNs to Arrow DataTypes for first-message validation.
        let mut input_type_checks = HashMap::new();
        {
            let registry = dora_core::types::TypeRegistry::new();
            for (input_id, type_urn) in input_types {
                match registry.resolve_arrow_type(type_urn) {
                    Some(dt) => {
                        input_type_checks.insert(input_id.clone(), dt);
                    }
                    None => {
                        // Complex or custom types not resolvable to a simple Arrow DataType
                        if registry.resolve(type_urn).is_some() {
                            tracing::debug!(
                                input = %input_id,
                                "skipping type check for complex type \"{type_urn}\""
                            );
                        } else {
                            tracing::warn!(
                                input = %input_id,
                                "unknown input type URN \"{type_urn}\" — skipping type check"
                            );
                        }
                    }
                }
            }
        }

        Self::init_on_channel(
            dataflow_id,
            node_id,
            channel,
            close_channel,
            clock,
            scheduler,
            write_events_to,
            input_type_checks,
            total_queue_capacity,
            zenoh_session,
            &input_config,
            dynamic,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn init_on_channel(
        dataflow_id: DataflowId,
        node_id: &NodeId,
        mut channel: DaemonChannel,
        mut close_channel: DaemonChannel,
        clock: Arc<uhlc::HLC>,
        scheduler: Scheduler,
        write_events_to: Option<WriteEventsTo>,
        input_type_checks: HashMap<DataId, arrow_schema::DataType>,
        channel_capacity: usize,
        zenoh_session: Option<&zenoh::Session>,
        input_config: &BTreeMap<DataId, Input>,
        dynamic: bool,
    ) -> eyre::Result<Self> {
        channel.register(dataflow_id, node_id.clone(), clock.new_timestamp())?;
        let (tx, rx) = tokio::sync::mpsc::channel(channel_capacity);

        let use_scheduler = match &channel {
            DaemonChannel::IntegrationTestChannel(_) => {
                // don't use the scheduler for integration tests because it leads to
                // non-deterministic event ordering
                false
            }
            _ => true,
        };

        // Declare zenoh subscribers for each input that has a source node.
        // We use callback subscribers so the zenoh IO thread delivers the
        // sample directly into the event channel without an intermediate
        // dora-side thread + recv_async wakeup. The `Subscriber<()>` handle
        // must be kept alive for the lifetime of the EventStream — we store
        // them in `_zenoh_subscribers` and drop them after `receiver`.
        let mut zenoh_subscribers = Vec::new();
        let mut zenoh_schema_subscribers = Vec::new();
        let mut zenoh_liveliness_tokens = Vec::new();
        if let Some(session) = zenoh_session {
            use zenoh::Wait;
            for (input_id, input) in input_config {
                let mapping = &input.mapping;
                if let dora_message::config::InputMapping::User(user_mapping) = mapping {
                    let source_node = &user_mapping.source;
                    let source_output = &user_mapping.output;
                    let topic = dora_core::topics::zenoh_output_publish_topic(
                        dataflow_id,
                        source_node,
                        source_output,
                    );
                    let key_expr = match zenoh::key_expr::KeyExpr::new(topic.clone()) {
                        Ok(k) => k.into_owned(),
                        Err(e) => {
                            tracing::warn!(input = %input_id, "invalid zenoh key ({e}), using daemon path");
                            continue;
                        }
                    };
                    // Per-input persistent decoder for the schema-once path,
                    // shared between this data subscriber (which decodes batches
                    // in zenoh receipt order) and the `@schema` subscriber (which
                    // primes it from the cached/live schema). `Mutex` because the
                    // zenoh callbacks are `Fn`; uncontended in practice (each
                    // subscriber delivers its samples serially).
                    let decoder = std::sync::Arc::new(std::sync::Mutex::new(
                        crate::arrow_utils::ipc_encode::InputDecoder::new(),
                    ));
                    // Set if the `@schema` subscriber fails to declare: the
                    // schema plane is then dead for this input and only the
                    // producer's periodic in-band full-stream refresh can prime
                    // the decoder. The data callback surfaces a `FatalError` if
                    // the input stays undecodable past a grace window (see
                    // below), distinguishing a genuinely dead input from the
                    // transient "schema not arrived yet" drop.
                    let schema_plane_failed =
                        std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
                    // Start of the current run of undecodable schema-once
                    // batches while the schema plane is dead; cleared by any
                    // successful decode. Only touched when `schema_plane_failed`
                    // is set, so it costs nothing on the healthy path.
                    let first_undecodable =
                        std::sync::Arc::new(std::sync::Mutex::new(Option::<Instant>::None));
                    // The `@schema` subscriber primes `decoder`; its history query
                    // fetches the cached schema on join (late joiners), and
                    // `detect_late_publishers` covers a producer that starts later.
                    declare_schema_subscriber(
                        session,
                        dataflow_id,
                        source_node,
                        source_output,
                        input_id,
                        decoder.clone(),
                        tx.clone(),
                        schema_plane_failed.clone(),
                        &mut zenoh_schema_subscribers,
                    );

                    let tx_cb = tx.clone();
                    let input_id_cb = input_id.clone();
                    let decoder = decoder.clone();
                    let first_undecodable_cb = first_undecodable.clone();
                    let subscriber = session
                        .declare_subscriber(key_expr)
                        .callback(move |sample| {
                            // catch_unwind: a panic inside the callback would
                            // otherwise unwind through zenoh's IO worker, which
                            // is unsafe. Surface as FatalError so the node sees
                            // it and exits cleanly.
                            let result =
                                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                                    use dora_message::metadata::Metadata;
                                    let metadata = match sample.attachment() {
                                        Some(att) => {
                                            match bincode::deserialize::<Metadata>(&att.to_bytes())
                                            {
                                                // A version mismatch that still happens
                                                // to deserialize: reject with a clear
                                                // message rather than acting on data from
                                                // an incompatible wire format.
                                                Ok(m)
                                                    if m.metadata_version()
                                                        != Metadata::CURRENT_VERSION =>
                                                {
                                                    tracing::warn!(
                                                        "dropping zenoh sample: incompatible \
                                                     metadata wire version {} (this node \
                                                     speaks {})",
                                                        m.metadata_version(),
                                                        Metadata::CURRENT_VERSION
                                                    );
                                                    return;
                                                }
                                                Ok(m) => m,
                                                Err(e) => {
                                                    // A pre-1.0 peer (old ArrowTypeInfo
                                                    // sidecar layout) misaligns here; name
                                                    // the likely cause so the failure isn't
                                                    // a bare bincode error.
                                                    tracing::warn!(
                                                        "zenoh metadata deserialization failed \
                                                     (possibly a peer using an incompatible \
                                                     wire format/version): {e}"
                                                    );
                                                    return;
                                                }
                                            }
                                        }
                                        None => {
                                            tracing::warn!(
                                                "zenoh sample missing metadata attachment"
                                            );
                                            return;
                                        }
                                    };
                                    let payload = sample.payload().clone();
                                    // Decode here (receipt order) so the per-input
                                    // persistent decoder stays in sync. This runs on
                                    // zenoh's IO worker: the aligned-SHM path is
                                    // zero-copy, but an under-aligned heap payload
                                    // copies its buffers on this thread (acceptable —
                                    // only small messages take the heap path).
                                    let mut decoder = decoder.lock().unwrap_or_else(|poison| {
                                        // A prior callback panicked mid-decode
                                        // (caught below), poisoning the lock and
                                        // leaving the decoder in an undefined
                                        // state. Reset it so the next batch is not
                                        // fed into a corrupt decoder.
                                        let mut guard = poison.into_inner();
                                        guard.reset();
                                        guard
                                    });
                                    let data =
                                        match decode_zenoh_sample(&mut decoder, &metadata, payload)
                                        {
                                            Ok(Some(data)) => data,
                                            // Schema-less batch we can't decode yet.
                                            // Normally transient: the priming schema
                                            // hasn't arrived (lossy data plane), so drop
                                            // and wait — the window is bounded by the
                                            // producer's periodic full-stream refresh,
                                            // which re-primes in-band. But if the
                                            // `@schema` subscriber failed to declare, the
                                            // schema plane is dead (a degraded zenoh
                                            // session) and only that refresh can save the
                                            // input — give it a grace window, then surface
                                            // a `FatalError` so the node exits loudly
                                            // rather than drop messages forever. Only
                                            // clear the flag once the error is actually
                                            // queued, so a momentarily-full channel can't
                                            // swallow the one-shot fatal signal forever; it
                                            // retries on the next undecodable batch. This
                                            // callback is the sole reader, so the
                                            // load-then-store is race-free. `Relaxed` is
                                            // sufficient: the flag is stored in
                                            // `declare_schema_subscriber` before the data
                                            // subscriber that drives this callback is even
                                            // declared, so that write happens-before any
                                            // read here.
                                            Ok(None) => {
                                                if schema_plane_failed
                                                    .load(std::sync::atomic::Ordering::Relaxed)
                                                {
                                                    let mut first = first_undecodable_cb
                                                        .lock()
                                                        .unwrap_or_else(|p| p.into_inner());
                                                    if first.is_none() {
                                                        tracing::warn!(
                                                            input = %input_id_cb,
                                                            "schema-once batch arrived unprimed \
                                                             while the `@schema` subscriber is \
                                                             not declared; dropping, waiting up \
                                                             to {}s for the producer's in-band \
                                                             full-stream refresh",
                                                            SCHEMA_PLANE_FATAL_GRACE.as_secs()
                                                        );
                                                    }
                                                    if schema_plane_fatal_due(
                                                        &mut first,
                                                        Instant::now(),
                                                    ) && tx_cb
                                                        .try_send(EventItem::FatalError(eyre!(
                                                            "input `{input_id_cb}`: the `@schema` \
                                                             subscriber failed to declare (a \
                                                             degraded zenoh session) and the \
                                                             input stayed undecodable for {}s \
                                                             despite the producer's periodic \
                                                             full-stream refresh — messages on \
                                                             this input are being dropped",
                                                            SCHEMA_PLANE_FATAL_GRACE.as_secs()
                                                        )))
                                                        .is_ok()
                                                    {
                                                        schema_plane_failed.store(
                                                            false,
                                                            std::sync::atomic::Ordering::Relaxed,
                                                        );
                                                    }
                                                }
                                                return;
                                            }
                                            Err(e) => {
                                                tracing::warn!(
                                                    input = %input_id_cb,
                                                    "zenoh payload decode failed: {e}"
                                                );
                                                return;
                                            }
                                        };
                                    drop(decoder);
                                    // A successful decode means the input is healthy
                                    // (in-band priming worked); reset the fatal grace
                                    // window. Only costs a lock when the schema plane
                                    // is actually dead.
                                    if schema_plane_failed
                                        .load(std::sync::atomic::Ordering::Relaxed)
                                    {
                                        *first_undecodable_cb
                                            .lock()
                                            .unwrap_or_else(|p| p.into_inner()) = None;
                                    }
                                    // Callback runs on zenoh's tokio IO worker —
                                    // `blocking_send` panics from a tokio context, so
                                    // use `try_send`. If the channel is full the event
                                    // is dropped (logged); receiver-dropped also
                                    // surfaces here, in which case there's nothing to do.
                                    if let Err(e) = tx_cb.try_send(EventItem::ZenohInput {
                                        id: input_id_cb.clone(),
                                        metadata: std::sync::Arc::new(metadata),
                                        data,
                                    }) {
                                        use tokio::sync::mpsc::error::TrySendError;
                                        match e {
                                            TrySendError::Full(_) => {
                                                tracing::warn!(
                                                    "event channel full; dropping zenoh input"
                                                );
                                            }
                                            TrySendError::Closed(_) => {
                                                // normal shutdown
                                            }
                                        }
                                    }
                                }));
                            if result.is_err() {
                                tracing::error!(
                                    input = %input_id_cb,
                                    "zenoh subscriber callback panicked"
                                );
                                let _ = tx_cb.try_send(EventItem::FatalError(eyre!(
                                    "zenoh subscriber callback for input `{input_id_cb}` panicked"
                                )));
                            }
                        })
                        .wait();
                    match subscriber {
                        Ok(s) => {
                            tracing::debug!(input = %input_id, %topic, "zenoh subscriber declared (callback)");
                            zenoh_subscribers.push(s);

                            // Announce readiness for this link so the producer
                            // counts us before switching the output to the direct
                            // zenoh data plane. Declared only after a *successful*
                            // data subscriber (so a counted token always implies
                            // the subscription is in flight) and never for dynamic
                            // nodes — they connect at arbitrary times and are not
                            // part of the startup barrier (the producer reaches
                            // them via normal matching once they join). A failure
                            // here is non-fatal: the producer just keeps this
                            // output on the reliable daemon path.
                            if !dynamic {
                                let ready_key =
                                    dora_core::topics::zenoh_input_ready_liveliness_topic(
                                        dataflow_id,
                                        source_node,
                                        source_output,
                                        node_id,
                                        input_id,
                                    );
                                match session.liveliness().declare_token(ready_key).wait() {
                                    Ok(token) => zenoh_liveliness_tokens.push(token),
                                    Err(e) => {
                                        tracing::warn!(
                                            input = %input_id,
                                            "failed to declare zenoh readiness token ({e}); \
                                             producer will keep this output on the daemon path"
                                        );
                                    }
                                }
                            }
                        }
                        Err(e) => {
                            tracing::warn!(
                                input = %input_id,
                                "failed to declare zenoh subscriber ({e}), using daemon path"
                            );
                        }
                    }
                }
            }
        }

        let reply = channel
            .request(&Timestamped {
                inner: DaemonRequest::Subscribe,
                timestamp: clock.new_timestamp(),
            })
            .map_err(|e| eyre!(e))
            .wrap_err("failed to create subscription with dora-daemon")?;

        match reply {
            DaemonReply::Result(Ok(())) => {}
            DaemonReply::Result(Err(err)) => {
                eyre::bail!("subscribe failed: {err}")
            }
            other => eyre::bail!("unexpected subscribe reply: {other:?}"),
        }

        close_channel.register(dataflow_id, node_id.clone(), clock.new_timestamp())?;

        let thread_handle = thread::init(node_id.clone(), tx, channel, clock.clone())?;

        Ok(EventStream {
            node_id: node_id.clone(),
            receiver: rx,
            _thread_handle: thread_handle,
            _zenoh_subscribers: zenoh_subscribers,
            _zenoh_schema_subscribers: zenoh_schema_subscribers,
            _zenoh_liveliness_tokens: zenoh_liveliness_tokens,
            close_channel,
            start_timestamp: clock.new_timestamp(),
            clock,
            scheduler,
            write_events_to,
            use_scheduler,
            input_type_checks,
            pending_passthrough: std::collections::VecDeque::new(),
            stop_received: false,
        })
    }

    /// Synchronously waits for the next event.
    ///
    /// Blocks the thread until the next event arrives.
    /// Returns [`None`] once the event stream is closed.
    ///
    /// For an asynchronous variant of this method see [`recv_async`][Self::recv_async].
    ///
    /// ## Event Reordering
    ///
    /// This method uses an [`EventScheduler`] internally to **reorder events**. This means that the
    /// events might be returned in a different order than they occurred. For details, check the
    /// documentation of the [`EventScheduler`] struct.
    ///
    /// If you want to receive the events in their original chronological order, use the
    /// asynchronous [`StreamExt::next`](futures::StreamExt::next) method instead ([`EventStream`] implements the
    /// [`Stream`] trait).
    pub fn recv(&mut self) -> Option<Event> {
        futures::executor::block_on(self.recv_async())
    }

    /// Receives the next incoming [`Event`] synchronously with a timeout.
    ///
    /// Blocks the thread until the next event arrives or the timeout is reached.
    /// Returns a [`Event::Error`] if no event was received within the given duration.
    ///
    /// Returns [`None`] once the event stream is closed.
    ///
    /// For an asynchronous variant of this method see [`recv_async_timeout`][Self::recv_async_timeout].
    ///
    /// ## Event Reordering
    ///
    /// This method uses an [`EventScheduler`] internally to **reorder events**. This means that the
    /// events might be returned in a different order than they occurred. For details, check the
    /// documentation of the [`EventScheduler`] struct.
    ///
    /// If you want to receive the events in their original chronological order, use the
    /// asynchronous [`StreamExt::next`](futures::StreamExt::next) method instead ([`EventStream`] implements the
    /// [`Stream`] trait).
    pub fn recv_timeout(&mut self, dur: Duration) -> Option<Event> {
        futures::executor::block_on(self.recv_async_timeout(dur))
    }

    /// Receives the next incoming [`Event`] asynchronously, using an [`EventScheduler`] for fairness.
    ///
    /// Returns [`None`] once the event stream is closed.
    ///
    /// ## Event Reordering
    ///
    /// This method uses an [`EventScheduler`] internally to **reorder events**. This means that the
    /// events might be returned in a different order than they occurred. For details, check the
    /// documentation of the [`EventScheduler`] struct.
    ///
    /// If you want to receive the events in their original chronological order, use the
    /// [`StreamExt::next`](futures::StreamExt::next) method with a custom timeout future instead
    /// ([`EventStream`] implements the [`Stream`] trait).
    pub async fn recv_async(&mut self) -> Option<Event> {
        // Drain any events that were stashed by pattern-aware helpers
        // (`recv_service_response`, `recv_action_result`) while they
        // were waiting for a specific correlation. These must be
        // returned to the caller before we poll the underlying
        // scheduler, so the caller's main event loop never loses
        // events that arrived during a helper wait (dora-rs/adora#148).
        if let Some(event) = self.pending_passthrough.pop_front() {
            return Some(event);
        }
        self.recv_from_stream().await
    }

    /// Receive the next event straight from the scheduler/receiver,
    /// **without** draining `pending_passthrough` first.
    ///
    /// The pattern-aware wait loop ([`wait_for_correlation`](Self::wait_for_correlation))
    /// buffers every non-matching event into `pending_passthrough` itself. If
    /// it pumped the stream through [`recv_async`](Self::recv_async), that
    /// drain would immediately hand the just-buffered event straight back, the
    /// classifier would re-buffer it, and the loop would spin on the same event
    /// forever — never reading the awaited response off the receiver — until it
    /// hit its deadline and wrongly reported a timeout (while pinning a CPU
    /// core). Reading through this bypass keeps the buffered events reserved for
    /// the caller's own `recv`/`recv_async` while the wait loop makes real
    /// progress.
    async fn recv_from_stream(&mut self) -> Option<Event> {
        // Close the stream after a Stop event: the daemon thread has
        // already dropped its sender, but zenoh subscriber threads
        // hold clones that would otherwise keep `receiver` open.
        if self.stop_received {
            // The scheduler gives `Stop` (a NON_INPUT_EVENT) strict priority
            // over buffered inputs (`scheduler::next`), so inputs enqueued
            // before Stop can still be queued when Stop is delivered. Drain
            // those buffered *inputs* before closing instead of dropping them
            // silently. Trailing non-input control events (a second Stop /
            // AllInputsClosed / InputClosed / Reload) are discarded rather than
            // re-delivered after Stop — the contract is "nothing after Stop
            // except the inputs that were already queued". Do NOT pull new
            // events from `receiver`; the dataflow is stopping, and on the
            // non-scheduler path returning `None` closes the stream against
            // zenoh-held senders.
            if self.use_scheduler {
                while let Some(item) = self.scheduler.next() {
                    if matches!(
                        &item,
                        EventItem::NodeEvent {
                            event: NodeEvent::Input { .. },
                            ..
                        } | EventItem::ZenohInput { .. }
                    ) {
                        return Some(Self::convert_event_item(item));
                    }
                }
            }
            return None;
        }
        let event = if !self.use_scheduler {
            self.receiver.recv().await.map(Self::convert_event_item)
        } else {
            loop {
                if self.scheduler.is_empty() {
                    if let Some(event) = self.receiver.recv().await {
                        self.add_event(event);
                    } else {
                        break;
                    }
                } else {
                    match self.receiver.try_recv() {
                        Ok(event) => self.add_event(event),
                        Err(_) => break, // empty or disconnected
                    };
                }
            }
            self.scheduler.next().map(Self::convert_event_item)
        };

        // First-message type validation: check once per input, then remove.
        // Zero cost after first message per input.
        //
        // Skip the check when the message carries pattern metadata
        // (`request_id`, `goal_id`, or `goal_status`) — the input is
        // polymorphic by pattern design and a single declared type
        // cannot cover all variants. The check stays armed so a later
        // non-pattern message can still validate (dora-rs/adora#150).
        if let Some(Event::Input {
            ref id,
            ref metadata,
            ref data,
        }) = event
            && let Some(expected) = self.input_type_checks.get(id).cloned()
        {
            let is_pattern_message = metadata
                .parameters
                .contains_key(dora_message::metadata::REQUEST_ID)
                || metadata
                    .parameters
                    .contains_key(dora_message::metadata::GOAL_ID)
                || metadata
                    .parameters
                    .contains_key(dora_message::metadata::GOAL_STATUS);
            if !is_pattern_message {
                // Consume the check and validate.
                self.input_type_checks.remove(id);
                let actual = data.data_type();
                // Skip check for Null type (timer ticks, empty payloads)
                // to avoid spurious warnings on annotated timer inputs.
                if *actual != arrow_schema::DataType::Null && *actual != expected {
                    tracing::warn!(
                        input = %id,
                        expected = ?expected,
                        actual = ?actual,
                        "input type mismatch on first message"
                    );
                }
            }
        }

        if matches!(&event, Some(Event::Stop(_))) {
            self.stop_received = true;
        }
        event
    }

    /// Check if there are any buffered events in the scheduler, the
    /// receiver, or the passthrough buffer used by pattern-aware helpers.
    pub fn is_empty(&self) -> bool {
        self.pending_passthrough.is_empty() && self.scheduler.is_empty() && self.receiver.is_empty()
    }

    /// Returns and resets the accumulated drop counts per input ID.
    ///
    /// When inputs overflow their queue limits, the oldest messages are discarded.
    /// For `drop_oldest` inputs this happens at `queue_size`. For `backpressure`
    /// inputs this happens at a hard safety cap of 10x `queue_size`.
    /// This method returns a map from input ID to the number of messages dropped
    /// since the last call.
    pub fn drain_drop_counts(&mut self) -> HashMap<DataId, u64> {
        self.scheduler.drain_drop_counts()
    }

    fn add_event(&mut self, event: EventItem) {
        // Event recording is observability-only (writes to the optional
        // `write_events_to` log). A write failure must not panic the event
        // loop — drop the log line and continue scheduling.
        if let Err(err) = self.record_event(&event) {
            tracing::warn!(
                node = %self.node_id,
                "failed to record event to write_events_to log: {err:?}"
            );
            // Mark the recording poisoned so consumers can detect events
            // are missing from the final JSON. `write_out()` surfaces this
            // as a top-level `recording_status` field (#1857).
            if let Some(write_events_to) = self.write_events_to.as_mut() {
                let time_offset_secs = self
                    .clock
                    .new_timestamp()
                    .get_diff_duration(&self.start_timestamp)
                    .as_secs_f64();
                write_events_to.mark_poisoned(&err, time_offset_secs);
            }
        }
        self.scheduler.add_event(event);
    }

    fn record_event(&mut self, event: &EventItem) -> eyre::Result<()> {
        if let Some(write_events_to) = &mut self.write_events_to {
            let event_json = match event {
                EventItem::NodeEvent { event, .. } => match event {
                    NodeEvent::Stop => {
                        let time_offset = self
                            .clock
                            .new_timestamp()
                            .get_diff_duration(&self.start_timestamp);
                        let event_json = serde_json::json!({
                            "type": "Stop",
                            "time_offset_secs": time_offset.as_secs_f64(),
                        });
                        Some(event_json)
                    }
                    NodeEvent::Reload { .. } => None,
                    NodeEvent::Input { id, metadata, data } => {
                        let mut event_json = convert_output_to_json(
                            id,
                            metadata,
                            data,
                            self.start_timestamp,
                            false,
                        )?;
                        event_json.insert("type".into(), "Input".into());
                        Some(event_json.into())
                    }
                    NodeEvent::InputClosed { id } => {
                        let time_offset = self
                            .clock
                            .new_timestamp()
                            .get_diff_duration(&self.start_timestamp);
                        let event_json = serde_json::json!({
                            "type": "InputClosed",
                            "id": id.to_string(),
                            "time_offset_secs": time_offset.as_secs_f64(),
                        });
                        Some(event_json)
                    }
                    NodeEvent::InputRecovered { id } => {
                        let time_offset = self
                            .clock
                            .new_timestamp()
                            .get_diff_duration(&self.start_timestamp);
                        let event_json = serde_json::json!({
                            "type": "InputRecovered",
                            "id": id.to_string(),
                            "time_offset_secs": time_offset.as_secs_f64(),
                        });
                        Some(event_json)
                    }
                    NodeEvent::NodeRestarted { id } => {
                        let time_offset = self
                            .clock
                            .new_timestamp()
                            .get_diff_duration(&self.start_timestamp);
                        let event_json = serde_json::json!({
                            "type": "NodeRestarted",
                            "id": id.to_string(),
                            "time_offset_secs": time_offset.as_secs_f64(),
                        });
                        Some(event_json)
                    }
                    NodeEvent::AllInputsClosed => {
                        let time_offset = self
                            .clock
                            .new_timestamp()
                            .get_diff_duration(&self.start_timestamp);
                        let event_json = serde_json::json!({
                            "type": "AllInputsClosed",
                            "time_offset_secs": time_offset.as_secs_f64(),
                        });
                        Some(event_json)
                    }
                    _ => None,
                },
                _ => None,
            };
            if let Some(event_json) = event_json {
                write_events_to.events_buffer.push(event_json);
            }
        }
        Ok(())
    }

    /// Receives the next buffered [`Event`] (if any) without blocking, using an
    /// [`EventScheduler`] for fairness.
    ///
    /// Returns [`TryRecvError::Empty`] if no event is available right now.
    /// Returns [`TryRecvError::Closed`] once the event stream is closed.
    ///
    /// This method never blocks and is safe to use in asynchronous contexts.
    ///
    /// ## Event Reordering
    ///
    /// This method uses an [`EventScheduler`] internally to **reorder events**. This means that the
    /// events might be returned in a different order than they occurred. For details, check the
    /// documentation of the [`EventScheduler`] struct.
    ///
    /// If you want to receive the events in their original chronological order, use the
    /// [`StreamExt::next`](futures::StreamExt::next) method with a custom timeout future instead
    /// ([`EventStream`] implements the [`Stream`] trait).
    pub fn try_recv(&mut self) -> Result<Event, TryRecvError> {
        match self.recv_async().now_or_never() {
            Some(Some(event)) => Ok(event),
            Some(None) => Err(TryRecvError::Closed),
            None => Err(TryRecvError::Empty),
        }
    }

    /// Receives all buffered [`Event`]s without blocking, using an [`EventScheduler`] for fairness.
    ///
    /// Return `Some(Vec::new())` if no events are ready.
    /// Returns [`None`] once the event stream is closed and no events are buffered anymore.
    ///
    /// This method never blocks and is safe to use in asynchronous contexts.
    ///
    /// This method is equivalent to repeatedly calling [`try_recv`][Self::try_recv]. See its docs
    /// for details on event reordering.
    pub fn drain(&mut self) -> Option<Vec<Event>> {
        let mut events = Vec::new();
        loop {
            match self.try_recv() {
                Ok(event) => events.push(event),
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Closed) => {
                    if events.is_empty() {
                        return None;
                    } else {
                        break;
                    }
                }
            }
        }
        Some(events)
    }

    /// Receives the next incoming [`Event`] asynchronously with a timeout.
    ///
    /// Returns a [`Event::Error`] if no event was received within the given duration.
    ///
    /// Returns [`None`] once the event stream is closed.
    ///
    /// ## Event Reordering
    ///
    /// This method uses an [`EventScheduler`] internally to **reorder events**. This means that the
    /// events might be returned in a different order than they occurred. For details, check the
    /// documentation of the [`EventScheduler`] struct.
    ///
    /// If you want to receive the events in their original chronological order, use the
    /// [`StreamExt::next`](futures::StreamExt::next) method with a custom timeout future instead
    /// ([`EventStream`] implements the [`Stream`] trait).
    pub async fn recv_async_timeout(&mut self, dur: Duration) -> Option<Event> {
        match select(Delay::new(dur), pin!(self.recv_async())).await {
            Either::Left((_elapsed, _)) => Some(Self::convert_event_item(EventItem::TimeoutError(
                eyre!("Receiver timed out"),
            ))),
            Either::Right((event, _)) => event,
        }
    }

    /// Waits for a service response carrying `request_id` in its metadata.
    ///
    /// Drives the event loop internally and returns the matching
    /// [`Event::Input`] as soon as it arrives. Non-matching events are
    /// buffered and replayed on the next call to `recv()` / `recv_async()`,
    /// so your main event loop does not lose intermediate events.
    ///
    /// Terminal conditions return a [`PatternError`]:
    ///
    /// - `Timeout` — `timeout` elapsed before any matching response arrived.
    /// - `ServerRestarted(expected_server)` — the expected server node
    ///   restarted, which means its in-flight `request_id` correlation
    ///   was orphaned. The caller should retry against the new instance.
    /// - `StreamEnded` — the event stream closed (dataflow stopping)
    ///   before a response arrived. The terminal `Stop` event is still
    ///   returned to the caller's next `recv()`.
    /// - `StreamError` — an upstream error event surfaced during the wait.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let request_id = node.send_service_request(...)?;
    /// match events
    ///     .recv_service_response(&request_id, &server_id, Duration::from_secs(5))
    ///     .await
    /// {
    ///     Ok(Event::Input { data, .. }) => handle_response(data),
    ///     Err(PatternError::Timeout) => fallback_path(),
    ///     Err(PatternError::ServerRestarted(_)) => retry_with_new_instance(),
    ///     Err(e) => return Err(e.into()),
    ///     _ => unreachable!(),
    /// }
    /// ```
    pub async fn recv_service_response(
        &mut self,
        request_id: &str,
        expected_server: &NodeId,
        timeout: Duration,
    ) -> Result<Event, PatternError> {
        self.wait_for_correlation(
            timeout,
            expected_server,
            |event, request_id| match event {
                Event::Input { metadata, .. } => {
                    dora_message::metadata::get_string_param(
                        &metadata.parameters,
                        dora_message::metadata::REQUEST_ID,
                    ) == Some(request_id)
                }
                _ => false,
            },
            request_id,
        )
        .await
    }

    /// Waits for a terminal action result (`goal_status` ∈
    /// {`succeeded`, `aborted`, `canceled`}) with a matching `goal_id`.
    ///
    /// Semantics mirror [`recv_service_response`](Self::recv_service_response)
    /// but match on `goal_id` + a terminal `goal_status` instead of
    /// `request_id`. Intermediate feedback events (matching `goal_id`
    /// without a terminal `goal_status`) are stashed for the caller's
    /// main loop — use `recv()` separately if you need to observe them.
    pub async fn recv_action_result(
        &mut self,
        goal_id: &str,
        expected_server: &NodeId,
        timeout: Duration,
    ) -> Result<Event, PatternError> {
        self.wait_for_correlation(
            timeout,
            expected_server,
            |event, goal_id| match event {
                Event::Input { metadata, .. } => {
                    let matches_goal = dora_message::metadata::get_string_param(
                        &metadata.parameters,
                        dora_message::metadata::GOAL_ID,
                    ) == Some(goal_id);
                    if !matches_goal {
                        return false;
                    }
                    matches!(
                        dora_message::metadata::get_string_param(
                            &metadata.parameters,
                            dora_message::metadata::GOAL_STATUS,
                        ),
                        Some(dora_message::metadata::GOAL_STATUS_SUCCEEDED)
                            | Some(dora_message::metadata::GOAL_STATUS_ABORTED)
                            | Some(dora_message::metadata::GOAL_STATUS_CANCELED)
                    )
                }
                _ => false,
            },
            goal_id,
        )
        .await
    }

    /// Core loop for the pattern-aware helpers. Waits up to `timeout`
    /// for an event that satisfies `is_match(event, needle)`. Buffers
    /// every non-matching event so the caller's main event loop can
    /// still see them via `recv()`.
    async fn wait_for_correlation<F>(
        &mut self,
        timeout: Duration,
        expected_server: &NodeId,
        is_match: F,
        needle: &str,
    ) -> Result<Event, PatternError>
    where
        F: Fn(&Event, &str) -> bool,
    {
        let deadline = std::time::Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(std::time::Instant::now());
            if remaining.is_zero() {
                return Err(PatternError::Timeout);
            }
            // Pump the stream via `recv_from_stream`, NOT `recv_async`: the
            // latter drains `pending_passthrough` first, which would re-hand us
            // the very events we buffer below and livelock the loop (see
            // `recv_from_stream`'s docs).
            let event = match select(Delay::new(remaining), pin!(self.recv_from_stream())).await {
                Either::Left((_elapsed, _)) => return Err(PatternError::Timeout),
                Either::Right((None, _)) => return Err(PatternError::StreamEnded),
                Either::Right((Some(e), _)) => e,
            };

            match classify_correlation_event(&event, expected_server, |e| is_match(e, needle)) {
                CorrelationOutcome::Match => return Ok(event),
                CorrelationOutcome::ServerRestarted => {
                    self.pending_passthrough.push_back(event);
                    return Err(PatternError::ServerRestarted(expected_server.to_string()));
                }
                CorrelationOutcome::StreamEnded => {
                    self.pending_passthrough.push_back(event);
                    return Err(PatternError::StreamEnded);
                }
                CorrelationOutcome::StreamError => {
                    if let Event::Error(err) = event {
                        return Err(PatternError::StreamError(err));
                    }
                    unreachable!("StreamError only returned for Event::Error");
                }
                CorrelationOutcome::Passthrough => {
                    self.pending_passthrough.push_back(event);
                }
            }
        }
    }
}

/// Outcome of classifying a single event during a pattern-aware wait.
/// Separated from `wait_for_correlation` so the decision logic can be
/// unit-tested without a live `EventStream`.
#[derive(Debug, PartialEq, Eq)]
enum CorrelationOutcome {
    /// The event satisfies the caller's predicate — return it.
    Match,
    /// `Event::NodeRestarted { id }` where `id == expected_server`.
    ServerRestarted,
    /// `Event::Stop(_)` — the dataflow is shutting down.
    StreamEnded,
    /// `Event::Error(_)` — the stream surfaced an error.
    StreamError,
    /// Unrelated event — buffer it and keep waiting.
    Passthrough,
}

fn classify_correlation_event<F>(
    event: &Event,
    expected_server: &NodeId,
    is_match: F,
) -> CorrelationOutcome
where
    F: Fn(&Event) -> bool,
{
    if is_match(event) {
        return CorrelationOutcome::Match;
    }
    match event {
        Event::NodeRestarted { id } if id == expected_server => CorrelationOutcome::ServerRestarted,
        Event::Stop(_) => CorrelationOutcome::StreamEnded,
        Event::Error(_) => CorrelationOutcome::StreamError,
        _ => CorrelationOutcome::Passthrough,
    }
}

impl EventStream {
    fn convert_event_item(item: EventItem) -> Event {
        match item {
            EventItem::NodeEvent { event } => match event {
                NodeEvent::Stop => Event::Stop(event::StopCause::Manual),
                NodeEvent::Reload { operator_id } => Event::Reload { operator_id },
                NodeEvent::InputClosed { id } => Event::InputClosed { id },
                NodeEvent::InputRecovered { id } => Event::InputRecovered { id },
                NodeEvent::NodeRestarted { id } => Event::NodeRestarted { id },
                NodeEvent::Input { id, metadata, data } => {
                    let data_inner = data.map(Arc::unwrap_or_clone);
                    let result = data_to_arrow_array(data_inner);
                    match result {
                        Ok(data) => {
                            let mut metadata = Arc::unwrap_or_clone(metadata);
                            dora_message::metadata::strip_internal_parameters(
                                &mut metadata.parameters,
                            );
                            Event::Input {
                                id,
                                metadata,
                                data: data.into(),
                            }
                        }
                        Err(err) => Event::Error(format!("{err:?}")),
                    }
                }
                NodeEvent::AllInputsClosed => Event::Stop(event::StopCause::AllInputsClosed),
                NodeEvent::ParamUpdate { key, value_json } => {
                    match serde_json::from_slice(&value_json) {
                        Ok(value) => Event::ParamUpdate { key, value },
                        Err(err) => Event::Error(format!(
                            "failed to deserialize ParamUpdate value for `{key}`: {err}"
                        )),
                    }
                }
                NodeEvent::ParamDeleted { key } => Event::ParamDeleted { key },
                NodeEvent::NodeFailed {
                    affected_input_ids,
                    error,
                    source_node_id,
                } => Event::NodeFailed {
                    affected_input_ids,
                    error,
                    source_node_id,
                },
                other => {
                    tracing::warn!("ignoring unrecognized NodeEvent variant: {other:?}");
                    Event::Error(format!("unrecognized node event: {other:?}"))
                }
            },

            EventItem::ZenohInput { id, metadata, data } => {
                let mut metadata = Arc::unwrap_or_clone(metadata);
                dora_message::metadata::strip_internal_parameters(&mut metadata.parameters);
                Event::Input {
                    id,
                    metadata,
                    // Already decoded in the subscriber callback (receipt order).
                    data: arrow::array::make_array(data).into(),
                }
            }

            EventItem::FatalError(err) => {
                Event::Error(format!("fatal event stream error: {err:?}"))
            }
            EventItem::TimeoutError(err) => {
                Event::Error(format!("Timeout event stream error: {err:?}"))
            }
        }
    }
}

/// No event is available right now or the event stream has been closed.
#[derive(Debug)]
pub enum TryRecvError {
    /// No new event is available right now.
    Empty,
    /// The event stream has been closed.
    Closed,
}

/// Convert a zenoh `ZBytes` payload into an Arrow array without copying
/// for contiguous buffers (e.g. Zenoh SHM).
///
/// For `Cow::Borrowed` payloads (SHM), the Arrow `Buffer` is backed by
/// the original `ZBytes` allocation via `Buffer::from_custom_allocation`,
/// achieving true zero-copy. For `Cow::Owned` (normal network path),
/// copy into Dora's aligned buffer type before reconstructing Arrow arrays.
/// Newtype that owns a Zenoh [`ZBytes`](zenoh::bytes::ZBytes) payload so it can
/// back an Arrow `Buffer` via `Buffer::from_custom_allocation`. Keeping the
/// `ZBytes` alive keeps the underlying SHM mapping (or heap buffer) valid for
/// the lifetime of the zero-copy Arrow buffer.
#[allow(dead_code)] // field kept alive to own the zenoh buffer
struct ZBytesAllocation(zenoh::bytes::ZBytes);
// SAFETY: the wrapped `ZBytes` is only used to keep the backing allocation
// alive; the bytes are treated as immutable for the Buffer's lifetime.
unsafe impl Sync for ZBytesAllocation {}
unsafe impl Send for ZBytesAllocation {}
impl std::panic::RefUnwindSafe for ZBytesAllocation {}

/// Convert a zenoh payload to an Arrow array (dora-rs/adora#132).
///
/// Every data-plane payload is a self-describing Arrow IPC stream, so the
/// decode needs no type sidecar. An empty payload is a metadata-only message
/// and maps to the unit array.
/// Wrap a zenoh payload as an Arrow `Buffer` — aliasing the zenoh SHM mapping
/// for borrowed payloads (zero-copy), owning the materialized `Vec` otherwise.
fn zenoh_payload_to_buffer(payload: zenoh::bytes::ZBytes) -> arrow::buffer::Buffer {
    use std::ptr::NonNull;
    match payload.to_bytes() {
        std::borrow::Cow::Borrowed(slice) => {
            let ptr =
                NonNull::new(slice.as_ptr() as *mut u8).expect("zenoh SHM payload ptr is null");
            let len = slice.len();
            // SAFETY: `ptr` points into the SHM region owned by `payload`;
            // moving `payload` into the Arc keeps the region mapped for the
            // lifetime of the Buffer.
            unsafe {
                arrow::buffer::Buffer::from_custom_allocation(
                    ptr,
                    len,
                    Arc::new(ZBytesAllocation(payload)),
                )
            }
        }
        std::borrow::Cow::Owned(vec) => arrow::buffer::Buffer::from_vec(vec),
    }
}

/// Declare the `@schema` AdvancedSubscriber for an input: its callback primes
/// `decoder` from the schema published on the output's schema subtopic. The
/// history query fetches the cached schema on join; `detect_late_publishers`
/// re-queries a producer that appears after this subscriber.
///
/// On failure, `schema_plane_failed` is set so the data subscriber surfaces a
/// `FatalError` if the input stays undecodable past
/// [`SCHEMA_PLANE_FATAL_GRACE`]: a failed declare means a degraded zenoh
/// session, so exit loudly rather than limp along on the in-band full-stream
/// refresh alone — but give that refresh (which fully heals the input) its
/// chance first instead of killing a node that would recover within seconds.
/// It never blocks the data subscriber.
#[allow(clippy::too_many_arguments)]
fn declare_schema_subscriber(
    session: &zenoh::Session,
    dataflow_id: DataflowId,
    source_node: &NodeId,
    source_output: &DataId,
    input_id: &DataId,
    decoder: Arc<std::sync::Mutex<crate::arrow_utils::ipc_encode::InputDecoder>>,
    tx: tokio::sync::mpsc::Sender<EventItem>,
    schema_plane_failed: Arc<std::sync::atomic::AtomicBool>,
    out: &mut Vec<zenoh_ext::AdvancedSubscriber<()>>,
) {
    use zenoh::Wait;
    use zenoh_ext::{AdvancedSubscriberBuilderExt, HistoryConfig};

    let topic =
        dora_core::topics::zenoh_output_schema_topic(dataflow_id, source_node, source_output);
    let key = match zenoh::key_expr::KeyExpr::new(topic) {
        Ok(k) => k.into_owned(),
        Err(e) => {
            tracing::warn!(input = %input_id, "invalid @schema zenoh key ({e}); schema-once disabled for this input");
            schema_plane_failed.store(true, std::sync::atomic::Ordering::Relaxed);
            return;
        }
    };
    let input_id_cb = input_id.clone();
    let sub = session
        .declare_subscriber(key)
        .history(HistoryConfig::default().detect_late_publishers())
        .callback(move |sample| {
            // catch_unwind: a panic must not unwind through zenoh's IO worker.
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let buffer = zenoh_payload_to_buffer(sample.payload().clone());
                let hash = crate::node::fnv1a(buffer.as_slice());
                let mut decoder = decoder.lock().unwrap_or_else(|poison| {
                    let mut guard = poison.into_inner();
                    guard.reset();
                    guard
                });
                if let Err(e) = decoder.set_schema(hash, buffer) {
                    tracing::warn!(input = %input_id_cb, "failed to prime decoder from @schema sample: {e}");
                }
            }));
            if result.is_err() {
                tracing::error!(input = %input_id_cb, "zenoh @schema subscriber callback panicked");
                let _ = tx.try_send(EventItem::FatalError(eyre!(
                    "zenoh @schema subscriber for input `{input_id_cb}` panicked"
                )));
            }
        })
        .wait();
    match sub {
        Ok(s) => out.push(s),
        Err(e) => {
            tracing::warn!(input = %input_id, "failed to declare @schema subscriber ({e}); schema-once disabled for this input");
            schema_plane_failed.store(true, std::sync::atomic::Ordering::Relaxed);
        }
    }
}

/// How long a schema-once input may stay continuously undecodable — with the
/// `@schema` plane dead — before the node exits with a `FatalError`. Three
/// producer full-stream refresh intervals: the refresh re-primes the input
/// in-band, so if it hasn't healed after three periods the input is genuinely
/// dead (producer gone or data plane dropping every refresh), not just waiting
/// out the documented recovery window.
const SCHEMA_PLANE_FATAL_GRACE: Duration =
    crate::node::SCHEMA_ONCE_REFRESH_INTERVAL.saturating_mul(3);

/// Whether the undecodable-input condition has persisted past
/// [`SCHEMA_PLANE_FATAL_GRACE`]. Records the start of the window on first call;
/// the caller clears `first_undecodable` on any successful decode.
fn schema_plane_fatal_due(first_undecodable: &mut Option<Instant>, now: Instant) -> bool {
    let start = *first_undecodable.get_or_insert(now);
    now.duration_since(start) >= SCHEMA_PLANE_FATAL_GRACE
}

/// Decode a zenoh data sample in receipt order. A schema-less batch (the
/// `SCHEMA_HASH` parameter is present) decodes against the per-input decoder
/// primed from the output's `@schema` subtopic or in-band from an earlier full
/// stream — returning `Ok(None)` if that schema hasn't been received yet, in
/// which case the caller drops it. Other messages (large/SHM full streams,
/// daemon-path payloads) decode standalone — and additionally prime the decoder
/// in-band (see [`prime_in_band`]).
fn decode_zenoh_sample(
    decoder: &mut crate::arrow_utils::ipc_encode::InputDecoder,
    metadata: &dora_message::metadata::Metadata,
    payload: zenoh::bytes::ZBytes,
) -> eyre::Result<Option<arrow::array::ArrayData>> {
    use crate::arrow_utils::decode_arrow_ipc_zero_copy;
    use dora_message::metadata::{SCHEMA_HASH, get_integer_param};

    if payload.is_empty() {
        return Ok(Some(().into_arrow().into()));
    }
    let buffer = zenoh_payload_to_buffer(payload);
    match get_integer_param(&metadata.parameters, SCHEMA_HASH) {
        Some(hash) => {
            tracing::debug!("received schema-less batch with SCHEMA_HASH={}", hash);
            decoder.decode_batch(buffer, hash as u64)
        }
        None => {
            tracing::debug!("received full IPC stream (no SCHEMA_HASH)");
            // Service/action messages are excluded from schema-once (a server
            // multiplexes per-request schemas through one output); don't let
            // them churn the retained schema set.
            if !crate::node::carries_pattern_correlation(&metadata.parameters) {
                prime_in_band(decoder, &buffer);
            }
            decode_arrow_ipc_zero_copy(buffer).map(Some)
        }
    }
}

/// Prime the per-input decoder from the schema block of a full self-describing
/// stream received on the data topic.
///
/// The producer sends a full stream for the first message of an output (and
/// after every schema change, failed `@schema` publish, or periodic refresh —
/// see `publish_schema_once`). Since all data-plane puts share one publisher,
/// zenoh delivers them in order, so this priming happens strictly before the
/// schema-less batches that reference the schema — closing the QoS race where
/// an express batch overtakes the `@schema` plane's non-express schema, and
/// re-priming (within the refresh interval) any consumer whose `@schema`
/// history query missed the single schema emission.
fn prime_in_band(
    decoder: &mut crate::arrow_utils::ipc_encode::InputDecoder,
    buffer: &arrow::buffer::Buffer,
) {
    let Some((hash, schema)) =
        crate::arrow_utils::ipc_encode::schema_block_and_hash(buffer.as_slice())
    else {
        return;
    };
    // A known schema (live or retained) needs no eager re-prime: the full
    // stream decodes standalone, and `decode_batch` re-primes lazily from the
    // retained set when a schema-less batch actually references it.
    if decoder.knows_schema(hash) {
        return;
    }
    // Copy the schema block out of the payload: retaining a slice of an
    // SHM-backed buffer would pin the whole segment for the decoder's lifetime.
    let schema = arrow::buffer::Buffer::from(schema);
    if let Err(e) = decoder.set_schema(hash, schema) {
        tracing::debug!("in-band schema priming failed: {e}");
    }
}

pub fn data_to_arrow_array(
    data: Option<DataMessage>,
) -> eyre::Result<Arc<dyn arrow::array::Array>> {
    let data: eyre::Result<Option<RawData>> = match data {
        None => Ok(None),
        Some(DataMessage::Vec(v)) => Ok(Some(RawData::Vec(v))),
    };

    data.and_then(|data| {
        let raw_data = data.unwrap_or(RawData::Empty);
        raw_data.into_arrow_array().map(arrow::array::make_array)
    })
}

impl Stream for EventStream {
    type Item = Event;

    fn poll_next(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Option<Self::Item>> {
        // Drain events that were buffered by pattern-aware helpers
        // (`recv_service_response`, `recv_action_result`) before
        // polling the underlying receiver. Mirrors the drain at the
        // top of `recv_async` so `StreamExt::next()` and `recv()`
        // return the same events in the same order
        // (dora-rs/adora#172).
        if let Some(event) = self.pending_passthrough.pop_front() {
            return std::task::Poll::Ready(Some(event));
        }

        // Close the stream after a Stop event: zenoh subscriber threads
        // hold sender clones that would otherwise keep `receiver` open.
        if self.stop_received {
            return std::task::Poll::Ready(None);
        }

        let poll = self
            .receiver
            .poll_recv(cx)
            .map(|item| item.map(Self::convert_event_item));

        // Run first-message type check on the Stream path too.
        //
        // Mirror the recv_async() logic: skip the check (and keep it
        // armed) when the message carries pattern metadata, so a later
        // non-pattern message can still validate (dora-rs/adora#174).
        if let std::task::Poll::Ready(Some(Event::Input {
            ref id,
            ref metadata,
            ref data,
        })) = poll
            && let Some(expected) = self.input_type_checks.get(id).cloned()
        {
            let is_pattern_message = metadata
                .parameters
                .contains_key(dora_message::metadata::REQUEST_ID)
                || metadata
                    .parameters
                    .contains_key(dora_message::metadata::GOAL_ID)
                || metadata
                    .parameters
                    .contains_key(dora_message::metadata::GOAL_STATUS);
            if !is_pattern_message {
                self.input_type_checks.remove(id);
                let actual = data.data_type();
                if *actual != arrow_schema::DataType::Null && *actual != expected {
                    tracing::warn!(
                        input = %id,
                        expected = ?expected,
                        actual = ?actual,
                        "input type mismatch on first message (Stream path)"
                    );
                }
            }
        }

        if matches!(&poll, std::task::Poll::Ready(Some(Event::Stop(_)))) {
            self.stop_received = true;
        }
        poll
    }
}

impl Drop for EventStream {
    fn drop(&mut self) {
        // Tear down the per-input zenoh callback subscribers under a deadline.
        // `Subscriber::drop` undeclares the subscription on the shared zenoh
        // session, which blocks indefinitely when zenoh's net runtime is
        // wedged (e.g. stuck retrying an unreachable scouted peer). Left
        // unbounded, that hangs the whole node in `EventStream`'s field-drop
        // sequence — before `DoraNode::Drop` (which already bounds its own
        // session teardown) even runs — so the daemon never sees the node
        // finish and the dataflow stalls until an outer timeout (dora-rs/dora#2425).
        // The callbacks use `try_send`, so undeclaring before `receiver` drops
        // cannot deadlock on a blocked callback.
        //
        // The `@schema` `AdvancedSubscriber`s (added with the Arrow IPC data
        // plane in #2366) undeclare on the same shared session and can wedge
        // the same way, so they must be torn down under the same deadline —
        // mirroring `DoraNode::Drop`, which drops both publisher maps inside
        // one guard. Left out, they would otherwise drop unbounded in the
        // implicit field-drop phase after this `Drop` body returns (#2583).
        let subscribers = std::mem::take(&mut self._zenoh_subscribers);
        let schema_subscribers = std::mem::take(&mut self._zenoh_schema_subscribers);
        if !subscribers.is_empty() || !schema_subscribers.is_empty() {
            let completed =
                teardown_with_timeout("zenoh-subscribers", ZENOH_TEARDOWN_TIMEOUT, move || {
                    drop(subscribers);
                    drop(schema_subscribers);
                });
            if !completed {
                tracing::warn!(
                    "zenoh subscriber teardown timed out after {}s; continuing node shutdown",
                    ZENOH_TEARDOWN_TIMEOUT.as_secs()
                );
            }
        }

        // Undeclare readiness tokens under the same deadline: like subscriber
        // teardown, `LivelinessToken::drop` undeclares on the shared session and
        // can block if zenoh's net runtime is wedged (dora-rs/dora#2425).
        let tokens = std::mem::take(&mut self._zenoh_liveliness_tokens);
        if !tokens.is_empty() {
            let completed = teardown_with_timeout(
                "zenoh-liveliness-tokens",
                ZENOH_TEARDOWN_TIMEOUT,
                move || {
                    drop(tokens);
                },
            );
            if !completed {
                tracing::warn!(
                    "zenoh readiness-token teardown timed out after {}s; continuing node shutdown",
                    ZENOH_TEARDOWN_TIMEOUT.as_secs()
                );
            }
        }

        let request = Timestamped {
            inner: DaemonRequest::EventStreamDropped,
            timestamp: self.clock.new_timestamp(),
        };
        let result = self
            .close_channel
            .request(&request)
            .map_err(|e| eyre!(e))
            .wrap_err("failed to signal event stream closure to dora-daemon")
            .and_then(|r| match r {
                DaemonReply::Result(Ok(())) => Ok(()),
                DaemonReply::Result(Err(err)) => Err(eyre!("EventStreamClosed failed: {err}")),
                other => Err(eyre!("unexpected EventStreamClosed reply: {other:?}")),
            });
        if let Err(err) = result {
            tracing::warn!("{err:?}")
        }

        if let Some(write_events_to) = self.write_events_to.take()
            && let Err(err) = write_events_to.write_out()
        {
            tracing::warn!(
                "failed to write out events for node {}: {err:?}",
                self.node_id
            );
        }
    }
}

pub(crate) struct WriteEventsTo {
    node_id: NodeId,
    file: std::fs::File,
    events_buffer: Vec<serde_json::Value>,
    /// `None` while the recording is complete. Becomes `Some(...)` on
    /// the first `record_event` failure; subsequent failures bump the
    /// counter inside. Surfaced in `write_out()` as a top-level
    /// `recording_status` field so consumers (replay tools, audit
    /// pipelines) can detect partial recordings instead of silently
    /// treating a syntactically-valid file as complete (#1857).
    poisoned: Option<PoisonInfo>,
}

#[derive(Debug)]
pub(crate) struct PoisonInfo {
    /// `events_buffer.len()` at the moment of the first failure — i.e.
    /// the number of events successfully recorded before the gap.
    first_failure_event_index: usize,
    /// Seconds since `EventStream::start_timestamp` at the first failure.
    first_failure_time_offset_secs: f64,
    /// `format!("{err:?}")` of the first `record_event()` error.
    first_failure_error: String,
    /// Count of subsequent failures after the first one.
    additional_failures: u64,
}

impl WriteEventsTo {
    /// Mark the recording poisoned. First call captures the failure
    /// detail; later calls just bump `additional_failures`.
    fn mark_poisoned(&mut self, err: &eyre::Report, time_offset_secs: f64) {
        match &mut self.poisoned {
            None => {
                self.poisoned = Some(PoisonInfo {
                    first_failure_event_index: self.events_buffer.len(),
                    first_failure_time_offset_secs: time_offset_secs,
                    first_failure_error: format!("{err:?}"),
                    additional_failures: 0,
                });
            }
            Some(info) => {
                info.additional_failures += 1;
            }
        }
    }

    fn write_out(self) -> eyre::Result<()> {
        use dora_message::integration_testing_format::RecordingStatus;

        let Self {
            node_id,
            file,
            events_buffer,
            poisoned,
        } = self;
        let mut inputs_file = serde_json::Map::new();
        inputs_file.insert("id".into(), node_id.to_string().into());
        // Emit `recording_status` for clean recordings too, so consumers
        // can rely on its presence as a definitive signal rather than
        // having to treat "field absent" as ambiguous between "clean"
        // and "older format" (#1857). Serialized via the canonical
        // `RecordingStatus` enum in `dora-message` so the wire format
        // stays in lockstep with the consumer-side type. The wire
        // shape is unaffected by `IntegrationTestInput`'s
        // `Option<Box<RecordingStatus>>` storage choice — serde
        // transparently serializes through the `Box`.
        let recording_status = match poisoned {
            None => RecordingStatus::Clean,
            Some(info) => RecordingStatus::Poisoned {
                first_failure_event_index: info.first_failure_event_index,
                first_failure_time_offset_secs: info.first_failure_time_offset_secs,
                first_failure_error: info.first_failure_error,
                additional_failures: info.additional_failures,
            },
        };
        inputs_file.insert(
            "recording_status".into(),
            serde_json::to_value(&recording_status)
                .context("failed to serialize recording_status")?,
        );
        inputs_file.insert("events".into(), events_buffer.into());

        serde_json::to_writer_pretty(file, &inputs_file)
            .context("failed to write events to file")?;
        Ok(())
    }
}

#[cfg(test)]
impl EventStream {
    /// Test-only: inject an event into the passthrough buffer so we can
    /// verify that `is_empty`, `recv_async`, and `Stream::poll_next` all
    /// drain it correctly (dora-rs/adora#172).
    fn push_passthrough_for_testing(&mut self, event: Event) {
        self.pending_passthrough.push_back(event);
    }

    /// Test-only: buffer an empty input directly in the scheduler and force
    /// scheduler mode, simulating an input the scheduler held back while
    /// prioritizing `Stop`. Used to verify `recv_async` drains buffered inputs
    /// after `Stop` instead of dropping them (dora-rs/dora#2027).
    fn push_scheduler_input_for_testing(&mut self, id: &str) {
        use crate::event_stream::thread::EventItem;
        use dora_message::{daemon_to_node::NodeEvent, metadata::Metadata};
        self.use_scheduler = true;
        let meta = Metadata::new(dora_core::uhlc::HLC::default().new_timestamp());
        self.scheduler.add_event(EventItem::NodeEvent {
            event: NodeEvent::Input {
                id: id.into(),
                metadata: std::sync::Arc::new(meta),
                data: None,
            },
        });
    }

    /// Test-only: buffer a `Stop` directly in the scheduler (a NON_INPUT_EVENT)
    /// and force scheduler mode, to verify the post-Stop drain discards trailing
    /// control events instead of re-delivering a second `Stop` (dora-rs/dora#2027).
    fn push_scheduler_stop_for_testing(&mut self) {
        use crate::event_stream::thread::EventItem;
        use dora_message::daemon_to_node::NodeEvent;
        self.use_scheduler = true;
        self.scheduler.add_event(EventItem::NodeEvent {
            event: NodeEvent::Stop,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn convert_param_update() {
        let item = EventItem::NodeEvent {
            event: NodeEvent::ParamUpdate {
                key: "fps".into(),
                value_json: serde_json::to_vec(&serde_json::json!(60)).unwrap(),
            },
        };
        let event = EventStream::convert_event_item(item);
        match event {
            Event::ParamUpdate { key, value } => {
                assert_eq!(key, "fps");
                assert_eq!(value, serde_json::json!(60));
            }
            other => panic!("expected ParamUpdate, got {other:?}"),
        }
    }

    /// Regression test for the daemon↔node wire protocol: `NodeEvent`
    /// is sent over TCP with bincode, so any field type that uses
    /// `Deserializer::deserialize_any` (like `serde_json::Value`)
    /// breaks the channel and kills the node at the next receive.
    /// `NodeEvent::ParamUpdate` carries its value as JSON-encoded
    /// bytes for that reason. This test pins the invariant so we
    /// don't regress back to a `deserialize_any` field.
    #[test]
    fn node_event_param_update_round_trips_through_bincode() {
        let cases = [
            serde_json::json!(42),
            serde_json::json!(1.5),
            serde_json::json!("hello"),
            serde_json::json!(null),
            serde_json::json!([1, 2, 3]),
            serde_json::json!({"nested": {"array": [true, false]}}),
        ];
        for value in cases {
            let event = NodeEvent::ParamUpdate {
                key: "rate".into(),
                value_json: serde_json::to_vec(&value).unwrap(),
            };
            let bytes = bincode::serialize(&event).expect("bincode serialize");
            let back: NodeEvent = bincode::deserialize(&bytes).expect("bincode deserialize");
            match back {
                NodeEvent::ParamUpdate { key, value_json } => {
                    assert_eq!(key, "rate");
                    let decoded: serde_json::Value =
                        serde_json::from_slice(&value_json).expect("value_json is JSON");
                    assert_eq!(decoded, value);
                }
                other => panic!("expected ParamUpdate, got {other:?}"),
            }
        }
    }

    // -- WriteEventsTo poisoned-state tests (#1857) ------------------------
    //
    // Build a `WriteEventsTo` against a tempfile, exercise the public
    // surface (push events / mark_poisoned / write_out), then parse the
    // resulting JSON and assert on the `recording_status` field shape.
    // No new dev-deps — uses std::env::temp_dir() + uuid (already a dep).

    fn write_events_to_with_tempfile() -> (WriteEventsTo, std::path::PathBuf) {
        let path = std::env::temp_dir().join(format!(
            "dora-write-events-test-{}.json",
            uuid::Uuid::new_v4()
        ));
        let file = std::fs::File::create(&path).expect("create tempfile");
        let w = WriteEventsTo {
            node_id: "test-node".parse().unwrap(),
            file,
            events_buffer: Vec::new(),
            poisoned: None,
        };
        (w, path)
    }

    fn read_back(path: &std::path::Path) -> serde_json::Value {
        let s = std::fs::read_to_string(path).expect("read back tempfile");
        std::fs::remove_file(path).ok();
        serde_json::from_str(&s).expect("output is valid JSON")
    }

    #[test]
    fn write_events_clean_recording_emits_state_clean() {
        let (mut w, path) = write_events_to_with_tempfile();
        w.events_buffer.push(serde_json::json!({"type": "Stop"}));
        w.write_out().expect("write_out clean recording");

        let v = read_back(&path);
        assert_eq!(v["recording_status"]["state"], "clean");
        assert_eq!(v["events"].as_array().unwrap().len(), 1);
        assert_eq!(v["id"], "test-node");
    }

    #[test]
    fn write_events_poisoned_recording_emits_state_poisoned_with_first_failure() {
        let (mut w, path) = write_events_to_with_tempfile();
        // 2 events recorded cleanly, then a failure, then 1 more event
        w.events_buffer.push(serde_json::json!({"type": "Input"}));
        w.events_buffer.push(serde_json::json!({"type": "Input"}));
        w.mark_poisoned(&eyre!("arrow conversion failed: bad type"), 1.5);
        w.events_buffer.push(serde_json::json!({"type": "Stop"}));
        w.write_out().expect("write_out poisoned recording");

        let v = read_back(&path);
        let status = &v["recording_status"];
        assert_eq!(status["state"], "poisoned");
        assert_eq!(status["first_failure_event_index"], 2);
        assert_eq!(status["first_failure_time_offset_secs"], 1.5);
        assert!(
            status["first_failure_error"]
                .as_str()
                .unwrap()
                .contains("arrow conversion failed: bad type")
        );
        assert_eq!(status["additional_failures"], 0);
        // `events` keeps the 2 clean + 1 post-failure-but-successful events.
        assert_eq!(v["events"].as_array().unwrap().len(), 3);
    }

    #[test]
    fn write_events_multiple_failures_keep_first_and_count_rest() {
        let (mut w, path) = write_events_to_with_tempfile();
        w.mark_poisoned(&eyre!("first error"), 0.5);
        w.mark_poisoned(&eyre!("second error"), 1.0);
        w.mark_poisoned(&eyre!("third error"), 1.5);
        w.write_out().expect("write_out with multiple failures");

        let v = read_back(&path);
        let status = &v["recording_status"];
        assert_eq!(status["state"], "poisoned");
        // First failure detail is preserved, NOT overwritten by later ones.
        assert_eq!(status["first_failure_event_index"], 0);
        assert_eq!(status["first_failure_time_offset_secs"], 0.5);
        assert!(
            status["first_failure_error"]
                .as_str()
                .unwrap()
                .contains("first error")
        );
        // Two additional failures after the first.
        assert_eq!(status["additional_failures"], 2);
    }

    #[test]
    fn convert_param_deleted() {
        let item = EventItem::NodeEvent {
            event: NodeEvent::ParamDeleted { key: "fps".into() },
        };
        let event = EventStream::convert_event_item(item);
        match event {
            Event::ParamDeleted { key } => {
                assert_eq!(key, "fps");
            }
            other => panic!("expected ParamDeleted, got {other:?}"),
        }
    }

    #[test]
    fn convert_stop_event() {
        let item = EventItem::NodeEvent {
            event: NodeEvent::Stop,
        };
        let event = EventStream::convert_event_item(item);
        assert!(matches!(event, Event::Stop(StopCause::Manual)));
    }

    #[test]
    fn convert_all_inputs_closed() {
        let item = EventItem::NodeEvent {
            event: NodeEvent::AllInputsClosed,
        };
        let event = EventStream::convert_event_item(item);
        assert!(matches!(event, Event::Stop(StopCause::AllInputsClosed)));
    }

    #[test]
    fn convert_input_closed() {
        let item = EventItem::NodeEvent {
            event: NodeEvent::InputClosed {
                id: "input_1".to_string().into(),
            },
        };
        let event = EventStream::convert_event_item(item);
        match event {
            Event::InputClosed { id } => assert_eq!(AsRef::<str>::as_ref(&id), "input_1"),
            other => panic!("expected InputClosed, got {other:?}"),
        }
    }

    #[test]
    fn convert_node_restarted() {
        let item = EventItem::NodeEvent {
            event: NodeEvent::NodeRestarted {
                id: "upstream".to_string().into(),
            },
        };
        let event = EventStream::convert_event_item(item);
        match event {
            Event::NodeRestarted { id } => assert_eq!(AsRef::<str>::as_ref(&id), "upstream"),
            other => panic!("expected NodeRestarted, got {other:?}"),
        }
    }

    // ---- dora-rs/adora#148: pattern-aware correlation classification ----

    use arrow::array::new_empty_array;
    use arrow::datatypes::DataType as ArrowDataType;
    use dora_arrow_convert::ArrowData;
    use dora_message::metadata::{
        GOAL_ID, GOAL_STATUS, GOAL_STATUS_ABORTED, GOAL_STATUS_SUCCEEDED, Metadata,
        MetadataParameters, Parameter, REQUEST_ID,
    };

    fn make_metadata(params: MetadataParameters) -> Metadata {
        Metadata::from_parameters(dora_core::uhlc::HLC::default().new_timestamp(), params)
    }

    fn make_input_event(id: &str, params: MetadataParameters) -> Event {
        Event::Input {
            id: id.into(),
            metadata: make_metadata(params),
            data: ArrowData(new_empty_array(&ArrowDataType::Null)),
        }
    }

    fn request_id_params(id: &str) -> MetadataParameters {
        let mut p = MetadataParameters::new();
        p.insert(REQUEST_ID.into(), Parameter::String(id.to_string()));
        p
    }

    fn goal_params(goal_id: &str, status: Option<&str>) -> MetadataParameters {
        let mut p = MetadataParameters::new();
        p.insert(GOAL_ID.into(), Parameter::String(goal_id.to_string()));
        if let Some(s) = status {
            p.insert(GOAL_STATUS.into(), Parameter::String(s.to_string()));
        }
        p
    }

    fn is_request_match(needle: &str) -> impl Fn(&Event) -> bool + '_ {
        move |event: &Event| match event {
            Event::Input { metadata, .. } => {
                dora_message::metadata::get_string_param(&metadata.parameters, REQUEST_ID)
                    == Some(needle)
            }
            _ => false,
        }
    }

    fn is_action_result_match(needle: &str) -> impl Fn(&Event) -> bool + '_ {
        move |event: &Event| match event {
            Event::Input { metadata, .. } => {
                let p = &metadata.parameters;
                dora_message::metadata::get_string_param(p, GOAL_ID) == Some(needle)
                    && matches!(
                        dora_message::metadata::get_string_param(p, GOAL_STATUS),
                        Some(GOAL_STATUS_SUCCEEDED)
                            | Some(GOAL_STATUS_ABORTED)
                            | Some(dora_message::metadata::GOAL_STATUS_CANCELED)
                    )
            }
            _ => false,
        }
    }

    #[test]
    fn classify_matching_request_id_returns_match() {
        let server = NodeId::from("calc".to_string());
        let event = make_input_event("response", request_id_params("req-42"));
        assert_eq!(
            classify_correlation_event(&event, &server, is_request_match("req-42")),
            CorrelationOutcome::Match
        );
    }

    #[test]
    fn classify_different_request_id_is_passthrough() {
        let server = NodeId::from("calc".to_string());
        let event = make_input_event("response", request_id_params("req-99"));
        assert_eq!(
            classify_correlation_event(&event, &server, is_request_match("req-42")),
            CorrelationOutcome::Passthrough
        );
    }

    #[test]
    fn classify_expected_server_restart_returns_server_restarted() {
        let server = NodeId::from("calc".to_string());
        let event = Event::NodeRestarted { id: server.clone() };
        assert_eq!(
            classify_correlation_event(&event, &server, is_request_match("req-42")),
            CorrelationOutcome::ServerRestarted
        );
    }

    #[test]
    fn classify_unrelated_node_restart_is_passthrough() {
        let server = NodeId::from("calc".to_string());
        let event = Event::NodeRestarted {
            id: NodeId::from("other".to_string()),
        };
        assert_eq!(
            classify_correlation_event(&event, &server, is_request_match("req-42")),
            CorrelationOutcome::Passthrough
        );
    }

    #[test]
    fn classify_stop_returns_stream_ended() {
        let server = NodeId::from("calc".to_string());
        let event = Event::Stop(StopCause::Manual);
        assert_eq!(
            classify_correlation_event(&event, &server, is_request_match("req-42")),
            CorrelationOutcome::StreamEnded
        );
    }

    #[test]
    fn classify_error_returns_stream_error() {
        let server = NodeId::from("calc".to_string());
        let event = Event::Error("boom".to_string());
        assert_eq!(
            classify_correlation_event(&event, &server, is_request_match("req-42")),
            CorrelationOutcome::StreamError
        );
    }

    #[test]
    fn classify_unrelated_input_is_passthrough() {
        let server = NodeId::from("calc".to_string());
        let event = make_input_event("sensor", MetadataParameters::new());
        assert_eq!(
            classify_correlation_event(&event, &server, is_request_match("req-42")),
            CorrelationOutcome::Passthrough
        );
    }

    #[test]
    fn classify_param_update_is_passthrough() {
        // Runtime parameter updates must survive a helper wait.
        let server = NodeId::from("calc".to_string());
        let event = Event::ParamUpdate {
            key: "threshold".to_string(),
            value: serde_json::json!(0.85),
        };
        assert_eq!(
            classify_correlation_event(&event, &server, is_request_match("req-42")),
            CorrelationOutcome::Passthrough
        );
    }

    #[test]
    fn classify_action_result_terminal_succeeded_matches() {
        let server = NodeId::from("nav".to_string());
        let event = make_input_event("result", goal_params("goal-1", Some(GOAL_STATUS_SUCCEEDED)));
        assert_eq!(
            classify_correlation_event(&event, &server, is_action_result_match("goal-1")),
            CorrelationOutcome::Match
        );
    }

    #[test]
    fn classify_action_result_terminal_aborted_matches() {
        let server = NodeId::from("nav".to_string());
        let event = make_input_event("result", goal_params("goal-1", Some(GOAL_STATUS_ABORTED)));
        assert_eq!(
            classify_correlation_event(&event, &server, is_action_result_match("goal-1")),
            CorrelationOutcome::Match
        );
    }

    #[test]
    fn classify_action_feedback_without_terminal_status_is_passthrough() {
        // Intermediate feedback (no terminal goal_status) should pass
        // through so the caller's main loop can observe it.
        let server = NodeId::from("nav".to_string());
        let event = make_input_event("feedback", goal_params("goal-1", None));
        assert_eq!(
            classify_correlation_event(&event, &server, is_action_result_match("goal-1")),
            CorrelationOutcome::Passthrough
        );
    }

    #[test]
    fn classify_action_result_for_different_goal_is_passthrough() {
        let server = NodeId::from("nav".to_string());
        let event = make_input_event("result", goal_params("goal-2", Some(GOAL_STATUS_SUCCEEDED)));
        assert_eq!(
            classify_correlation_event(&event, &server, is_action_result_match("goal-1")),
            CorrelationOutcome::Passthrough
        );
    }

    // ---- dora-rs/adora#172: pending_passthrough integration ----

    use crate::integration_testing::{
        IntegrationTestInput, TestingInput, TestingOptions, TestingOutput,
        integration_testing_format::{IncomingEvent, TimedIncomingEvent},
    };

    /// Create a minimal EventStream via the testing path.
    fn test_event_stream() -> (crate::DoraNode, EventStream) {
        let events = vec![TimedIncomingEvent {
            time_offset_secs: 0.0,
            event: IncomingEvent::Stop,
        }];
        let inputs = TestingInput::Input(IntegrationTestInput::new(
            "test-node".parse().unwrap(),
            events,
        ));
        let (tx, _rx) = flume::unbounded();
        let outputs = TestingOutput::ToChannel(tx);
        let options = TestingOptions {
            skip_output_time_offsets: true,
        };
        crate::DoraNode::init_testing(inputs, outputs, options).unwrap()
    }

    #[test]
    fn is_empty_reflects_pending_passthrough() {
        let (_node, mut events) = test_event_stream();
        // Drain the initial Stop event so the stream is empty.
        let _ = events.recv();
        assert!(events.is_empty(), "should be empty after draining");

        // Inject a passthrough event — is_empty must now return false.
        events.push_passthrough_for_testing(Event::ParamDeleted {
            key: "k".to_string(),
        });
        assert!(
            !events.is_empty(),
            "should not be empty with pending passthrough"
        );
    }

    #[test]
    fn stream_poll_next_drains_pending_passthrough() {
        use futures::StreamExt;
        let (_node, mut events) = test_event_stream();
        // Drain the initial Stop event.
        let _ = events.recv();

        // Inject a passthrough event.
        events.push_passthrough_for_testing(Event::ParamUpdate {
            key: "threshold".to_string(),
            value: serde_json::json!(42),
        });

        // StreamExt::next() should return the passthrough event, not
        // block waiting on the underlying receiver.
        let next = futures::executor::block_on(events.next());
        match next {
            Some(Event::ParamUpdate { key, value }) => {
                assert_eq!(key, "threshold");
                assert_eq!(value, serde_json::json!(42));
            }
            other => panic!("expected ParamUpdate from passthrough, got {other:?}"),
        }
    }

    #[test]
    fn recv_async_drains_pending_passthrough_before_receiver() {
        let (_node, mut events) = test_event_stream();

        // Inject a passthrough event BEFORE the Stop in the receiver.
        events.push_passthrough_for_testing(Event::ParamDeleted {
            key: "x".to_string(),
        });

        // First recv should return the passthrough event.
        let first = events.recv();
        assert!(
            matches!(first, Some(Event::ParamDeleted { .. })),
            "expected passthrough ParamDeleted first, got {first:?}"
        );

        // Second recv should return the Stop from the receiver.
        let second = events.recv();
        assert!(
            matches!(second, Some(Event::Stop(_))),
            "expected Stop second, got {second:?}"
        );
    }

    /// Regression: a pattern-aware wait (`recv_service_response`) must make
    /// progress when a non-matching event arrives *before* the correlated
    /// response.
    ///
    /// The wait loop buffers every non-matching event into
    /// `pending_passthrough` so the caller's own event loop can still see it.
    /// Before the fix the loop pumped the stream via `recv_async`, which
    /// drains `pending_passthrough` first — so it kept re-serving the buffered
    /// non-matching event, re-buffering it, and spinning forever without ever
    /// reading the response off the receiver. Every such call pinned a CPU core
    /// and returned `Timeout`. The fix pumps via `recv_from_stream`, which
    /// bypasses the passthrough buffer.
    #[test]
    fn recv_service_response_matches_after_non_matching_event() {
        // Delivered FIFO (integration tests disable the reordering scheduler):
        // the non-matching "sensor" input, then the correlated "response".
        let events = vec![
            TimedIncomingEvent {
                time_offset_secs: 0.0,
                event: IncomingEvent::Input {
                    id: "sensor".parse().unwrap(),
                    metadata: None,
                    data: None,
                },
            },
            TimedIncomingEvent {
                time_offset_secs: 0.0,
                event: IncomingEvent::Input {
                    id: "response".parse().unwrap(),
                    metadata: Some(request_id_params("req-1")),
                    data: None,
                },
            },
            TimedIncomingEvent {
                time_offset_secs: 0.0,
                event: IncomingEvent::Stop,
            },
        ];
        let inputs = TestingInput::Input(IntegrationTestInput::new(
            "test-node".parse().unwrap(),
            events,
        ));
        let (tx, _rx) = flume::unbounded();
        let outputs = TestingOutput::ToChannel(tx);
        let options = TestingOptions {
            skip_output_time_offsets: true,
        };
        let (_node, mut events) = crate::DoraNode::init_testing(inputs, outputs, options).unwrap();

        let server = NodeId::from("calc".to_string());
        let response = futures::executor::block_on(events.recv_service_response(
            "req-1",
            &server,
            Duration::from_secs(5),
        ));
        match response {
            Ok(Event::Input { id, .. }) => assert_eq!(id.as_str(), "response"),
            other => panic!("expected the correlated response Input, got {other:?}"),
        }

        // The non-matching "sensor" input must not be lost — it is replayed
        // to the caller's own event loop after the wait returns.
        let buffered = events.recv();
        assert!(
            matches!(&buffered, Some(Event::Input { id, .. }) if id.as_str() == "sensor"),
            "expected the buffered non-matching 'sensor' input, got {buffered:?}"
        );
    }

    /// After a `Stop` event is delivered, subsequent `recv` calls must
    /// return `None` so the node can exit cleanly even when zenoh
    /// subscriber threads still hold clones of the event channel
    /// sender (which would otherwise keep the receiver open).
    #[test]
    fn recv_returns_none_after_stop() {
        let (_node, mut events) = test_event_stream();

        // First recv delivers the seeded Stop.
        let first = events.recv();
        assert!(matches!(first, Some(Event::Stop(_))));

        // Second recv must return None even though the underlying
        // receiver may still have live senders.
        let second = events.recv();
        assert!(
            second.is_none(),
            "recv must return None after Stop, got {second:?}"
        );
    }

    /// #2027: the scheduler gives `Stop` (a NON_INPUT_EVENT) strict priority
    /// over buffered inputs, so an input enqueued before `Stop` is still in the
    /// scheduler when `Stop` is delivered. `recv` must drain that input before
    /// closing rather than dropping it silently (the previous `return None`
    /// after `stop_received` lost it).
    #[test]
    fn recv_drains_buffered_scheduler_inputs_after_stop() {
        let (_node, mut events) = test_event_stream();

        // Deliver the seeded Stop (sets `stop_received`).
        assert!(matches!(events.recv(), Some(Event::Stop(_))));

        // Simulate the input the scheduler held back behind the prioritized Stop.
        events.push_scheduler_input_for_testing("cam");

        let drained = events.recv();
        assert!(
            matches!(&drained, Some(Event::Input { id, .. }) if id.as_str() == "cam"),
            "buffered input must be drained after Stop, got {drained:?}"
        );

        // Once the scheduler is empty the stream closes.
        assert!(
            events.recv().is_none(),
            "stream must close after draining buffered inputs"
        );
    }

    /// #2027 review (P2): the post-Stop drain must deliver buffered *inputs*
    /// only. A non-input control event buffered behind Stop (e.g. a second
    /// `Stop`) must NOT be re-delivered to a loop-until-`None` caller.
    #[test]
    fn recv_after_stop_skips_trailing_control_events() {
        let (_node, mut events) = test_event_stream();

        // Deliver the seeded Stop (sets `stop_received`).
        assert!(matches!(events.recv(), Some(Event::Stop(_))));

        // Buffer a trailing Stop AND a real input behind it. The scheduler
        // prioritizes the Stop (NON_INPUT), so the drain meets it first.
        events.push_scheduler_stop_for_testing();
        events.push_scheduler_input_for_testing("cam");

        // The drain must skip the trailing Stop and return only the input...
        let drained = events.recv();
        assert!(
            matches!(&drained, Some(Event::Input { id, .. }) if id.as_str() == "cam"),
            "expected the buffered input, not a re-delivered Stop, got {drained:?}"
        );
        // ...then close (no second Stop ever surfaces).
        assert!(events.recv().is_none(), "stream must close after the input");
    }

    /// The zenoh receive path is Arrow-IPC-only. An empty payload is a
    /// metadata-only message and maps to the unit array; a non-empty payload is
    /// a self-describing IPC stream and round-trips to its original array (with
    /// no type sidecar involved).
    #[test]
    fn zenoh_payload_ipc_roundtrip_and_empty_is_unit() {
        use crate::arrow_utils::ipc_encode::{InputDecoder, encode_ipc_into, ipc_fast_path_len};
        use arrow::array::{Array, Int32Array};

        // A standalone full stream (no SCHEMA_HASH parameter) decodes directly.
        let metadata =
            dora_message::metadata::Metadata::new(dora_core::uhlc::HLC::default().new_timestamp());
        let mut decoder = InputDecoder::new();

        // Empty payload -> unit array (metadata-only message).
        let unit = decode_zenoh_sample(&mut decoder, &metadata, zenoh::bytes::ZBytes::new())
            .unwrap()
            .unwrap();
        assert_eq!(
            unit.data_type(),
            &arrow_schema::DataType::Null,
            "empty payload maps to the unit array"
        );

        // Non-empty IPC payload round-trips to the original array.
        let data = Int32Array::from(vec![10, 20, 30]).into_data();
        let len = ipc_fast_path_len(&data).expect("primitive is fast-path eligible");
        let mut buf = vec![0u8; len];
        encode_ipc_into(&data, &mut buf).unwrap();

        let decoded = decode_zenoh_sample(&mut decoder, &metadata, zenoh::bytes::ZBytes::from(buf))
            .unwrap()
            .unwrap();
        assert_eq!(decoded.data_type(), &arrow_schema::DataType::Int32);
        assert_eq!(&decoded, &data);
    }

    /// A full self-describing stream on the data topic must prime the per-input
    /// decoder in-band: the schema-less batches that follow decode against it
    /// without any `@schema`-plane delivery. This is what makes the producer's
    /// "full stream until the schema is confirmed published" strategy race-free
    /// — data-plane puts share one publisher, so zenoh preserves their order,
    /// while the separate `@schema` plane can lose the race with an express
    /// batch (dora-rs/dora#2366 review: first-message QoS race).
    #[test]
    fn full_stream_primes_decoder_in_band_for_schema_less_batches() {
        use crate::arrow_utils::ipc_encode::{
            InputDecoder, batch_fast_path_len, encode_batch_into, encode_ipc_into,
            ipc_fast_path_len, schema_block_len,
        };
        use arrow::array::{Array, Int32Array};
        use dora_message::metadata::{Metadata, Parameter, SCHEMA_HASH};

        let hlc = dora_core::uhlc::HLC::default();
        let mut decoder = InputDecoder::new();

        // Message 1: full stream (no SCHEMA_HASH), as the producer sends while
        // the schema is not yet confirmed published on the `@schema` plane.
        let first = Int32Array::from(vec![1, 2]).into_data();
        let mut full = vec![0u8; ipc_fast_path_len(&first).unwrap()];
        encode_ipc_into(&first, &mut full).unwrap();
        let block = schema_block_len(&full).unwrap();
        let hash = dora_message::metadata::fnv1a(&full[..block]);

        let plain = Metadata::new(hlc.new_timestamp());
        let got = decode_zenoh_sample(&mut decoder, &plain, zenoh::bytes::ZBytes::from(full))
            .unwrap()
            .unwrap();
        assert_eq!(&got, &first);

        // Message 2: schema-less batch tagged with the schema hash. No
        // `set_schema` call happened — decoding must succeed purely from the
        // in-band priming above.
        let second = Int32Array::from(vec![3]).into_data();
        let mut batch = vec![0u8; batch_fast_path_len(&second).unwrap()];
        encode_batch_into(&second, &mut batch).unwrap();
        let mut tagged = Metadata::new(hlc.new_timestamp());
        tagged
            .parameters
            .insert(SCHEMA_HASH.to_string(), Parameter::Integer(hash as i64));

        let got = decode_zenoh_sample(&mut decoder, &tagged, zenoh::bytes::ZBytes::from(batch))
            .unwrap()
            .expect("schema-less batch must decode against the in-band-primed decoder");
        assert_eq!(&got, &second);
    }

    /// Service/action full streams (pattern-correlated) are excluded from
    /// schema-once and must NOT prime the decoder in-band: a server multiplexes
    /// per-request schemas through one output, and letting those prime the
    /// decoder would churn the retained schema set for no benefit.
    #[test]
    fn pattern_correlated_full_stream_does_not_prime_in_band() {
        use crate::arrow_utils::ipc_encode::{
            InputDecoder, batch_fast_path_len, encode_batch_into, encode_ipc_into,
            ipc_fast_path_len, schema_block_len,
        };
        use arrow::array::{Array, Int32Array};
        use dora_message::metadata::{Metadata, Parameter, REQUEST_ID, SCHEMA_HASH};

        let hlc = dora_core::uhlc::HLC::default();
        let mut decoder = InputDecoder::new();

        let reply = Int32Array::from(vec![7]).into_data();
        let mut full = vec![0u8; ipc_fast_path_len(&reply).unwrap()];
        encode_ipc_into(&reply, &mut full).unwrap();
        let block = schema_block_len(&full).unwrap();
        let hash = dora_message::metadata::fnv1a(&full[..block]);

        // A service reply (request_id) decodes standalone…
        let mut service = Metadata::new(hlc.new_timestamp());
        service
            .parameters
            .insert(REQUEST_ID.to_string(), Parameter::String("req-1".into()));
        let got = decode_zenoh_sample(&mut decoder, &service, zenoh::bytes::ZBytes::from(full))
            .unwrap()
            .unwrap();
        assert_eq!(&got, &reply);

        // …but must not have primed the decoder for its schema hash.
        let batch_array = Int32Array::from(vec![8]).into_data();
        let mut batch = vec![0u8; batch_fast_path_len(&batch_array).unwrap()];
        encode_batch_into(&batch_array, &mut batch).unwrap();
        let mut tagged = Metadata::new(hlc.new_timestamp());
        tagged
            .parameters
            .insert(SCHEMA_HASH.to_string(), Parameter::Integer(hash as i64));
        assert!(
            decode_zenoh_sample(&mut decoder, &tagged, zenoh::bytes::ZBytes::from(batch))
                .unwrap()
                .is_none(),
            "a pattern-correlated stream must not prime the schema-once decoder"
        );
    }

    /// Internal wire-protocol keys (`_schema_hash`, `_framing`) must be
    /// stripped from the metadata handed to user code — on both the zenoh and
    /// the daemon receive paths. A forwarded `_schema_hash` would otherwise
    /// ride onto a large/service output (which does not overwrite it) and make
    /// receivers hash-mismatch and silently drop the message
    /// (dora-rs/dora#2366 review).
    #[test]
    fn internal_wire_keys_are_stripped_from_user_visible_metadata() {
        use dora_message::metadata::{
            FRAMING, FRAMING_ARROW_IPC, Metadata, Parameter, SCHEMA_HASH,
        };

        let hlc = dora_core::uhlc::HLC::default();
        let mut metadata = Metadata::new(hlc.new_timestamp());
        metadata
            .parameters
            .insert(SCHEMA_HASH.to_string(), Parameter::Integer(42));
        metadata.parameters.insert(
            FRAMING.to_string(),
            Parameter::String(FRAMING_ARROW_IPC.to_string()),
        );
        metadata
            .parameters
            .insert("user_key".to_string(), Parameter::Integer(7));

        // Zenoh receive path.
        let zenoh_item = EventItem::ZenohInput {
            id: DataId::from("in".to_string()),
            metadata: Arc::new(metadata.clone()),
            data: {
                use arrow::array::Array;
                arrow::array::Int32Array::from(vec![1]).into_data()
            },
        };
        let Event::Input {
            metadata: user_metadata,
            ..
        } = EventStream::convert_event_item(zenoh_item)
        else {
            panic!("expected an input event");
        };
        assert!(!user_metadata.parameters.contains_key(SCHEMA_HASH));
        assert!(!user_metadata.parameters.contains_key(FRAMING));
        assert_eq!(
            user_metadata.parameters.get("user_key"),
            Some(&Parameter::Integer(7)),
            "user-provided keys must survive the strip"
        );

        // Daemon receive path.
        let daemon_item = EventItem::NodeEvent {
            event: dora_message::daemon_to_node::NodeEvent::Input {
                id: DataId::from("in".to_string()),
                metadata: Arc::new(metadata),
                data: None,
            },
        };
        let Event::Input {
            metadata: user_metadata,
            ..
        } = EventStream::convert_event_item(daemon_item)
        else {
            panic!("expected an input event");
        };
        assert!(!user_metadata.parameters.contains_key(SCHEMA_HASH));
        assert!(!user_metadata.parameters.contains_key(FRAMING));
    }

    /// The schema-plane FatalError only fires after the grace window: the
    /// producer's periodic full-stream refresh heals an unprimed input in-band,
    /// so a node with a dead `@schema` plane must not be killed on the first
    /// dropped batch when it would recover within seconds.
    #[test]
    fn schema_plane_fatal_waits_out_the_grace_window() {
        let start = Instant::now();
        let mut first = None;

        // First undecodable batch starts the window — not fatal yet.
        assert!(!schema_plane_fatal_due(&mut first, start));
        assert_eq!(first, Some(start));
        // Still inside the grace window — not fatal.
        assert!(!schema_plane_fatal_due(
            &mut first,
            start + SCHEMA_PLANE_FATAL_GRACE / 2
        ));
        // Past the window — fatal.
        assert!(schema_plane_fatal_due(
            &mut first,
            start + SCHEMA_PLANE_FATAL_GRACE
        ));

        // A successful decode clears the window (caller side); the next drop
        // starts a fresh one.
        let mut first = None;
        let later = start + SCHEMA_PLANE_FATAL_GRACE * 2;
        assert!(!schema_plane_fatal_due(&mut first, later));
        assert_eq!(first, Some(later));
    }

    /// Same invariant as `recv_returns_none_after_stop`, verified via
    /// the `Stream` impl (`StreamExt::next`).
    #[test]
    fn stream_returns_none_after_stop() {
        use futures::StreamExt;
        let (_node, mut events) = test_event_stream();

        let first = futures::executor::block_on(events.next());
        assert!(matches!(first, Some(Event::Stop(_))));

        let second = futures::executor::block_on(events.next());
        assert!(
            second.is_none(),
            "Stream::next must yield None after Stop, got {second:?}"
        );
    }
}
