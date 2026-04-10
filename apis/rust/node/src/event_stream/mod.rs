use std::{
    collections::{BTreeMap, HashMap, VecDeque},
    path::PathBuf,
    pin::pin,
    sync::Arc,
    time::Duration,
};

use adora_message::{
    DataflowId,
    daemon_to_node::{DaemonCommunication, DaemonReply, DataMessage, NodeEvent},
    id::DataId,
    node_to_daemon::{DaemonRequest, Timestamped},
};
pub use event::{Event, StopCause};
use futures::{
    FutureExt, Stream, StreamExt,
    future::{Either, select},
};
use futures_timer::Delay;
use scheduler::{NON_INPUT_EVENT, Scheduler};

use self::thread::{EventItem, EventStreamThreadHandle};
use crate::{
    DaemonCommunicationWrapper, PatternError,
    daemon_connection::{DaemonChannel, node_integration_testing::convert_output_to_json},
    event_stream::data_conversion::{MappedInputData, RawData, SharedMemoryData},
};
use adora_core::{
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
/// so you can use methods of the [`StreamExt`] trait
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
/// Note that Adora kills nodes that don't exit quickly after a [`Event::Stop`] of type
/// [`StopCause::Manual`] was received.
pub struct EventStream {
    node_id: NodeId,
    // Drop order: Rust drops fields in declaration order (top to bottom).
    // receiver must drop FIRST so tx_clone.send() in subscriber threads
    // returns Err, causing threads to exit before JoinHandles are dropped.
    receiver: flume::r#async::RecvStream<'static, EventItem>,
    _thread_handle: EventStreamThreadHandle,
    _zenoh_thread_handles: Vec<std::thread::JoinHandle<()>>,
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
    ) -> eyre::Result<Self> {
        let channel = match daemon_communication {
            DaemonCommunicationWrapper::Standard(daemon_communication) => {
                match daemon_communication {
                    DaemonCommunication::Shmem {
                        daemon_events_region_id,
                        ..
                    } => unsafe { DaemonChannel::new_shmem(daemon_events_region_id) }
                        .wrap_err_with(|| {
                            format!("failed to create shmem event stream for node `{node_id}`")
                        })?,
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
                    DaemonCommunication::Shmem {
                        daemon_events_close_region_id,
                        ..
                    } => unsafe { DaemonChannel::new_shmem(daemon_events_close_region_id) }
                        .wrap_err_with(|| {
                            format!(
                                "failed to create shmem event close channel for node `{node_id}`"
                            )
                        })?,
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
                            .unwrap_or(adora_message::config::DEFAULT_QUEUE_SIZE),
                        VecDeque::new(),
                    ),
                )
            })
            .collect();

        queue_size_limit.insert(
            DataId::from(NON_INPUT_EVENT.to_string()),
            (1_000, VecDeque::new()),
        );

        let queue_policies: HashMap<DataId, adora_message::config::QueuePolicy> = input_config
            .iter()
            .filter_map(|(input, config)| config.queue_policy.map(|p| (input.clone(), p)))
            .collect();

        let scheduler = Scheduler::with_policies(queue_size_limit, queue_policies);

        let total_queue_capacity: usize = input_config
            .values()
            .map(|c| {
                c.queue_size
                    .unwrap_or(adora_message::config::DEFAULT_QUEUE_SIZE)
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
                })
            }
            None => None,
        };

        // Resolve input type URNs to Arrow DataTypes for first-message validation.
        let mut input_type_checks = HashMap::new();
        {
            let registry = adora_core::types::TypeRegistry::new();
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
    ) -> eyre::Result<Self> {
        channel.register(dataflow_id, node_id.clone(), clock.new_timestamp())?;
        let reply = channel
            .request(&Timestamped {
                inner: DaemonRequest::Subscribe,
                timestamp: clock.new_timestamp(),
            })
            .map_err(|e| eyre!(e))
            .wrap_err("failed to create subscription with adora-daemon")?;

        match reply {
            DaemonReply::Result(Ok(())) => {}
            DaemonReply::Result(Err(err)) => {
                eyre::bail!("subscribe failed: {err}")
            }
            other => eyre::bail!("unexpected subscribe reply: {other:?}"),
        }

        close_channel.register(dataflow_id, node_id.clone(), clock.new_timestamp())?;

        let (tx, rx) = flume::bounded(channel_capacity);

        let use_scheduler = match &channel {
            DaemonChannel::IntegrationTestChannel(_) => {
                // don't use the scheduler for integration tests because it leads to
                // non-deterministic event ordering
                false
            }
            _ => true,
        };

        // Spawn zenoh subscribers for each input that has a source node.
        // These feed events directly into the same flume channel as daemon events.
        // Subscriber threads are tracked for cleanup in EventStream::drop.
        let mut zenoh_thread_handles = Vec::new();
        if let Some(session) = zenoh_session {
            use zenoh::Wait;
            for (input_id, input) in input_config {
                let mapping = &input.mapping;
                // Only user inputs from other nodes need zenoh subscribers
                if let adora_message::config::InputMapping::User(user_mapping) = mapping {
                    let source_node = &user_mapping.source;
                    let source_output = &user_mapping.output;
                    let topic = adora_core::topics::zenoh_output_publish_topic(
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
                    let subscriber = match session.declare_subscriber(key_expr).wait() {
                        Ok(s) => s,
                        Err(e) => {
                            // Graceful degradation: this input falls back to daemon delivery
                            tracing::warn!(
                                input = %input_id,
                                "failed to declare zenoh subscriber ({e}), using daemon path"
                            );
                            continue;
                        }
                    };

                    tracing::debug!(
                        input = %input_id,
                        %topic,
                        "zenoh subscriber declared for input"
                    );

                    let tx_clone = tx.clone();
                    let input_id = input_id.clone();
                    // Dummy ack channel — zenoh handles buffer lifecycle via ref counting,
                    // no DropToken needed (unlike the custom shmem path).
                    let dummy_ack = flume::bounded(0).0;
                    let handle = std::thread::Builder::new()
                        .name(format!("zenoh-sub-{input_id}"))
                        .spawn(move || {
                            while let Ok(sample) = subscriber.recv() {
                                // Extract metadata from attachment
                                let metadata = match sample.attachment() {
                                    Some(att) => {
                                        match bincode::deserialize::<
                                            adora_message::metadata::Metadata,
                                        >(
                                            &att.to_bytes()
                                        ) {
                                            Ok(m) => m,
                                            Err(e) => {
                                                tracing::warn!(
                                                    "zenoh metadata deserialization failed: {e}"
                                                );
                                                continue;
                                            }
                                        }
                                    }
                                    None => {
                                        tracing::warn!("zenoh sample missing metadata attachment");
                                        continue;
                                    }
                                };

                                // Extract payload bytes from zenoh sample.
                                // TODO: Use ZShm reference directly for true zero-copy
                                // once data_conversion.rs supports RawData::ZenohShm.
                                let payload = sample.payload();
                                let data_bytes = payload.to_bytes();
                                let data = if data_bytes.is_empty() {
                                    None
                                } else {
                                    let avec = match data_bytes {
                                        std::borrow::Cow::Owned(vec) => {
                                            // Convert Vec<u8> to AVec without copying.
                                            // SAFETY: Vec's allocation is always at least
                                            // 1-byte aligned, which matches the runtime
                                            // alignment we request.
                                            let mut vec = std::mem::ManuallyDrop::new(vec);
                                            let ptr = vec.as_mut_ptr();
                                            let len = vec.len();
                                            let cap = vec.capacity();
                                            unsafe {
                                                aligned_vec::AVec::from_raw_parts(ptr, 1, len, cap)
                                            }
                                        }
                                        std::borrow::Cow::Borrowed(slice) => {
                                            aligned_vec::AVec::from_slice(1, slice)
                                        }
                                    };
                                    Some(std::sync::Arc::new(DataMessage::Vec(avec)))
                                };

                                let event = NodeEvent::Input {
                                    id: input_id.clone(),
                                    metadata: std::sync::Arc::new(metadata),
                                    data,
                                };

                                if tx_clone
                                    .send(EventItem::NodeEvent {
                                        event,
                                        ack_channel: dummy_ack.clone(),
                                    })
                                    .is_err()
                                {
                                    break; // receiver dropped
                                }
                            }
                            tracing::trace!("zenoh subscriber thread exiting");
                        });
                    match handle {
                        Ok(h) => zenoh_thread_handles.push(h),
                        Err(e) => {
                            tracing::warn!(
                                "failed to spawn zenoh subscriber thread ({e}), input will use daemon path"
                            );
                        }
                    }
                }
            }
        }

        let thread_handle = thread::init(node_id.clone(), tx, channel, clock.clone())?;

        Ok(EventStream {
            node_id: node_id.clone(),
            receiver: rx.into_stream(),
            _thread_handle: thread_handle,
            _zenoh_thread_handles: zenoh_thread_handles,
            close_channel,
            start_timestamp: clock.new_timestamp(),
            clock,
            scheduler,
            write_events_to,
            use_scheduler,
            input_type_checks,
            pending_passthrough: std::collections::VecDeque::new(),
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
    /// asynchronous [`StreamExt::next`] method instead ([`EventStream`] implements the
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
    /// asynchronous [`StreamExt::next`] method instead ([`EventStream`] implements the
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
    /// [`StreamExt::next`] method with a custom timeout future instead
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
        let event = if !self.use_scheduler {
            self.receiver.next().await.map(Self::convert_event_item)
        } else {
            loop {
                if self.scheduler.is_empty() {
                    if let Some(event) = self.receiver.next().await {
                        self.add_event(event);
                    } else {
                        break;
                    }
                } else {
                    match self.receiver.next().now_or_never().flatten() {
                        Some(event) => self.add_event(event),
                        None => break, // no other ready events
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
                .contains_key(adora_message::metadata::REQUEST_ID)
                || metadata
                    .parameters
                    .contains_key(adora_message::metadata::GOAL_ID)
                || metadata
                    .parameters
                    .contains_key(adora_message::metadata::GOAL_STATUS);
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

        event
    }

    /// Check if there are any buffered events in the scheduler or the receiver.
    pub fn is_empty(&self) -> bool {
        self.scheduler.is_empty() & self.receiver.is_empty()
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
        self.record_event(&event).unwrap();
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
    /// [`StreamExt::next`] method with a custom timeout future instead
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
    /// [`StreamExt::next`] method with a custom timeout future instead
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
                    adora_message::metadata::get_string_param(
                        &metadata.parameters,
                        adora_message::metadata::REQUEST_ID,
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
                    let matches_goal = adora_message::metadata::get_string_param(
                        &metadata.parameters,
                        adora_message::metadata::GOAL_ID,
                    ) == Some(goal_id);
                    if !matches_goal {
                        return false;
                    }
                    matches!(
                        adora_message::metadata::get_string_param(
                            &metadata.parameters,
                            adora_message::metadata::GOAL_STATUS,
                        ),
                        Some(adora_message::metadata::GOAL_STATUS_SUCCEEDED)
                            | Some(adora_message::metadata::GOAL_STATUS_ABORTED)
                            | Some(adora_message::metadata::GOAL_STATUS_CANCELED)
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
            let event = match select(Delay::new(remaining), pin!(self.recv_async())).await {
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
            EventItem::NodeEvent { event, ack_channel } => match event {
                NodeEvent::Stop => Event::Stop(event::StopCause::Manual),
                NodeEvent::Reload { operator_id } => Event::Reload { operator_id },
                NodeEvent::InputClosed { id } => Event::InputClosed { id },
                NodeEvent::InputRecovered { id } => Event::InputRecovered { id },
                NodeEvent::NodeRestarted { id } => Event::NodeRestarted { id },
                NodeEvent::Input { id, metadata, data } => {
                    let data_inner = data.map(Arc::unwrap_or_clone);
                    let result = data_to_arrow_array(data_inner, &metadata, ack_channel);
                    match result {
                        Ok(data) => Event::Input {
                            id,
                            metadata: Arc::unwrap_or_clone(metadata),
                            data: data.into(),
                        },
                        Err(err) => Event::Error(format!("{err:?}")),
                    }
                }
                NodeEvent::AllInputsClosed => Event::Stop(event::StopCause::AllInputsClosed),
                NodeEvent::ParamUpdate { key, value } => Event::ParamUpdate { key, value },
                NodeEvent::ParamDeleted { key } => Event::ParamDeleted { key },
                other => {
                    tracing::warn!("ignoring unrecognized NodeEvent variant: {other:?}");
                    Event::Error(format!("unrecognized node event: {other:?}"))
                }
            },

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

pub fn data_to_arrow_array(
    data: Option<DataMessage>,
    metadata: &adora_message::metadata::Metadata,
    drop_channel: flume::Sender<()>,
) -> eyre::Result<Arc<dyn arrow::array::Array>> {
    let data = match data {
        None => Ok(None),
        Some(DataMessage::Vec(v)) => Ok(Some(RawData::Vec(v))),
        Some(DataMessage::SharedMemory {
            shared_memory_id,
            len,
            drop_token: _, // handled in `event_stream_loop`
        }) => unsafe {
            MappedInputData::map(&shared_memory_id, len).map(|data| {
                Some(RawData::SharedMemory(SharedMemoryData {
                    data,
                    _drop: drop_channel,
                }))
            })
        },
    };

    let is_ipc = adora_message::metadata::get_string_param(
        &metadata.parameters,
        adora_message::metadata::FRAMING,
    ) == Some(adora_message::metadata::FRAMING_ARROW_IPC);

    data.and_then(|data| {
        let raw_data = data.unwrap_or(RawData::Empty);
        raw_data
            .into_arrow_array(&metadata.type_info, is_ipc)
            .map(arrow::array::make_array)
    })
}

impl Stream for EventStream {
    type Item = Event;

    fn poll_next(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Option<Self::Item>> {
        let poll = self
            .receiver
            .poll_next_unpin(cx)
            .map(|item| item.map(Self::convert_event_item));

        // Run first-message type check on the Stream path too.
        if let std::task::Poll::Ready(Some(Event::Input {
            ref id, ref data, ..
        })) = poll
            && let Some(expected) = self.input_type_checks.remove(id)
        {
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

        poll
    }
}

impl Drop for EventStream {
    fn drop(&mut self) {
        let request = Timestamped {
            inner: DaemonRequest::EventStreamDropped,
            timestamp: self.clock.new_timestamp(),
        };
        let result = self
            .close_channel
            .request(&request)
            .map_err(|e| eyre!(e))
            .wrap_err("failed to signal event stream closure to adora-daemon")
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
}

impl WriteEventsTo {
    fn write_out(self) -> eyre::Result<()> {
        let Self {
            node_id,
            file,
            events_buffer,
        } = self;
        let mut inputs_file = serde_json::Map::new();
        inputs_file.insert("id".into(), node_id.to_string().into());
        inputs_file.insert("events".into(), events_buffer.into());

        serde_json::to_writer_pretty(file, &inputs_file)
            .context("failed to write events to file")?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Create a dummy ack channel for testing event conversion.
    fn dummy_ack() -> flume::Sender<()> {
        let (tx, _rx) = flume::bounded(1);
        tx
    }

    #[test]
    fn convert_param_update() {
        let item = EventItem::NodeEvent {
            event: NodeEvent::ParamUpdate {
                key: "fps".into(),
                value: serde_json::json!(60),
            },
            ack_channel: dummy_ack(),
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

    #[test]
    fn convert_param_deleted() {
        let item = EventItem::NodeEvent {
            event: NodeEvent::ParamDeleted { key: "fps".into() },
            ack_channel: dummy_ack(),
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
            ack_channel: dummy_ack(),
        };
        let event = EventStream::convert_event_item(item);
        assert!(matches!(event, Event::Stop(StopCause::Manual)));
    }

    #[test]
    fn convert_all_inputs_closed() {
        let item = EventItem::NodeEvent {
            event: NodeEvent::AllInputsClosed,
            ack_channel: dummy_ack(),
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
            ack_channel: dummy_ack(),
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
            ack_channel: dummy_ack(),
        };
        let event = EventStream::convert_event_item(item);
        match event {
            Event::NodeRestarted { id } => assert_eq!(AsRef::<str>::as_ref(&id), "upstream"),
            other => panic!("expected NodeRestarted, got {other:?}"),
        }
    }

    // ---- dora-rs/adora#148: pattern-aware correlation classification ----

    use adora_arrow_convert::ArrowData;
    use adora_message::metadata::{
        ArrowTypeInfo, GOAL_ID, GOAL_STATUS, GOAL_STATUS_ABORTED, GOAL_STATUS_SUCCEEDED, Metadata,
        MetadataParameters, Parameter, REQUEST_ID,
    };
    use arrow::array::new_empty_array;
    use arrow::datatypes::DataType as ArrowDataType;

    fn make_metadata(params: MetadataParameters) -> Metadata {
        let type_info = ArrowTypeInfo {
            data_type: ArrowDataType::Null,
            len: 0,
            null_count: 0,
            validity: None,
            offset: 0,
            buffer_offsets: vec![],
            child_data: vec![],
            field_names: None,
            schema_hash: None,
        };
        Metadata::from_parameters(
            adora_core::uhlc::HLC::default().new_timestamp(),
            type_info,
            params,
        )
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
                adora_message::metadata::get_string_param(&metadata.parameters, REQUEST_ID)
                    == Some(needle)
            }
            _ => false,
        }
    }

    fn is_action_result_match(needle: &str) -> impl Fn(&Event) -> bool + '_ {
        move |event: &Event| match event {
            Event::Input { metadata, .. } => {
                let p = &metadata.parameters;
                adora_message::metadata::get_string_param(p, GOAL_ID) == Some(needle)
                    && matches!(
                        adora_message::metadata::get_string_param(p, GOAL_STATUS),
                        Some(GOAL_STATUS_SUCCEEDED)
                            | Some(GOAL_STATUS_ABORTED)
                            | Some(adora_message::metadata::GOAL_STATUS_CANCELED)
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
}
