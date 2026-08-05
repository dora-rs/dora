use std::{
    any::Any,
    collections::{BTreeMap, VecDeque},
    time::{Duration, Instant},
    vec,
};

use crate::ffi::MetadataValueType;

use chrono::DateTime;
#[cfg(any(feature = "ros2-bridge", test))]
use dora_node_api::merged::MergeExternal;
use dora_node_api::{
    self, Event, EventStream, ExpectedServers, Metadata as DoraMetadata,
    MetadataParameters as DoraMetadataParameters, Parameter as DoraParameter, PatternError,
    TryRecvError,
    arrow::array::{AsArray, UInt8Array},
    dora_core::config::NodeId,
    merged::MergedEvent,
};
use eyre::{Result as EyreResult, bail, eyre};
use serde::Serialize;
use serde_json::Value as JsonValue;

#[cfg(feature = "ros2-bridge")]
pub use prelude::*;
#[cfg(feature = "ros2-bridge")]
pub mod prelude {
    pub use dora_ros2_bridge::prelude::*;
}
use futures_lite::{Stream, StreamExt, stream};

#[cxx::bridge]
#[allow(clippy::needless_lifetimes)]
mod ffi {
    struct DoraNode {
        events: Box<Events>,
        send_output: Box<OutputSender>,
    }

    pub enum DoraEventType {
        Stop,
        Input,
        InputClosed,
        Error,
        Unknown,
        AllInputsClosed,
        /// Non-blocking poll (`try_next_event`) found no event ready,
        /// and the same value is returned by `drained_events_next` once
        /// a drained queue is exhausted. Distinct from `Timeout` —
        /// `Empty` is returned even when no timeout was set.
        Empty,
        /// `next_event_timeout` returned without an event because the
        /// caller-supplied deadline elapsed first.
        Timeout,
        /// An upstream node has failed. Use `event_as_node_failed` to
        /// extract the failed node id, the error message, and the list
        /// of downstream inputs that will stop receiving data.
        NodeFailed,
        /// Hot-reload notification for an operator. Operator id is
        /// available via the daemon's reload protocol; this variant
        /// exists so C++ nodes can react to reloads (e.g. flushing
        /// caches) rather than treating them as `Unknown`.
        Reload,
    }

    struct DoraInput {
        id: String,
        data: Vec<u8>,
    }

    /// `DoraInput` plus the input's metadata.
    ///
    /// The metadata carries the Service/Action correlation keys
    /// (`request_id`, `goal_id`, `goal_status`), so the server side of a
    /// request/reply exchange needs it to echo the correlation back.
    /// `event_as_arrow_input_with_info` also exposes metadata, but only
    /// by exporting the payload through the Arrow C Data Interface —
    /// which would force an Arrow C++ dependency on byte-payload nodes.
    struct DoraInputWithMetadata {
        id: String,
        data: Vec<u8>,
        metadata: Box<Metadata>,
    }

    struct DoraResult {
        error: String,
    }

    /// Terminal outcome of a pattern-aware wait
    /// (`recv_service_response` / `recv_action_result`).
    ///
    /// Mirrors `dora_node_api::PatternError`, plus `Matched` for the
    /// success case and `InvalidArgument` for arguments rejected before
    /// the wait even starts.
    ///
    /// The success variant is called `Matched` rather than the more
    /// obvious `None` or `Success` because X11's `X.h` defines *both*
    /// of those as macros (`#define None 0L`, `#define Success 0`). Any
    /// node pulling in X11 — directly or via OpenCV / Qt / GTK — would
    /// otherwise fail to compile against this header.
    enum DoraPatternStatus {
        /// No error: the `event` field holds the correlated reply.
        Matched,
        /// The deadline elapsed before a correlated reply arrived.
        Timeout,
        /// The expected server node restarted, so the in-flight
        /// correlation was orphaned. Retry against the new instance.
        ServerRestarted,
        /// The event stream ended (dataflow stopping) before the reply
        /// arrived. The terminal `Stop` event is still delivered to the
        /// caller's next `next_event`.
        StreamEnded,
        /// An upstream error event surfaced during the wait.
        StreamError,
        /// A caller-supplied argument was rejected (e.g. a malformed
        /// server node id). Nothing was awaited.
        InvalidArgument,
        /// Returned only by the `try_recv_*` polls: no correlated reply
        /// is available yet. Not an error — call again on a later
        /// iteration. Distinct from `Timeout`, which means a deadline
        /// actually elapsed.
        NotReady,
    }

    /// Result of `recv_service_response` / `recv_action_result`.
    ///
    /// On success `status == Matched`, `error` is empty and `event` is
    /// the correlated `Input` event. On failure `event` still carries a
    /// matching `DoraEventType` (`Timeout` / `AllInputsClosed` /
    /// `Empty`) so callers can branch on either field.
    struct DoraPatternResult {
        status: DoraPatternStatus,
        error: String,
        event: Box<DoraEvent>,
    }

    /// Result of `send_service_request` / `send_arrow_service_request`.
    ///
    /// On success `error` is empty and `request_id` holds the generated
    /// UUID v7 to pass to `recv_service_response`. On failure
    /// `request_id` is empty.
    struct DoraRequestId {
        request_id: String,
        error: String,
    }

    /// Payload of a `DoraEventType::NodeFailed` event.
    struct DoraNodeFailed {
        /// Inputs on this node that will stop receiving data because
        /// the upstream node failed.
        affected_input_ids: Vec<String>,
        /// Human-readable error message from the failed node.
        error: String,
        /// Id of the node that failed.
        source_node_id: String,
    }

    struct ArrowInputInfo {
        id: String,
        metadata: Box<Metadata>,
        error: String,
    }

    enum MetadataValueType {
        Bool,
        Integer,
        Float,
        String,
        ListInt,
        ListFloat,
        ListString,
        Timestamp,
    }

    pub struct CombinedEvents {
        events: Box<MergedEvents>,
    }

    pub struct CombinedEvent {
        event: Box<MergedDoraEvent>,
    }

    extern "Rust" {
        type Events;
        type OutputSender;
        type DoraEvent;
        type DrainedEvents;
        type MergedEvents;
        type MergedDoraEvent;
        type Metadata;

        fn init_dora_node() -> Result<DoraNode>;

        fn dora_events_into_combined(events: Box<Events>) -> CombinedEvents;
        fn empty_combined_events() -> CombinedEvents;
        fn next(self: &mut Events) -> Box<DoraEvent>;
        fn next_event(events: &mut Box<Events>) -> Box<DoraEvent>;
        /// Block up to `timeout_ms` milliseconds for the next event.
        /// Returns an event with `event_type == Timeout` if the deadline
        /// elapses before one arrives, or `AllInputsClosed` if the
        /// stream closed first.
        fn next_event_timeout(events: &mut Box<Events>, timeout_ms: u64) -> Box<DoraEvent>;
        /// Non-blocking poll. Returns an event with `event_type ==
        /// Empty` if no event is immediately available, or
        /// `AllInputsClosed` if the stream is closed.
        fn try_next_event(events: &mut Box<Events>) -> Box<DoraEvent>;
        /// True when the event queue is currently empty (no events
        /// buffered). Note this can race against the daemon producing a
        /// new event; treat it as a hint, not a guarantee.
        fn events_is_empty(events: &Box<Events>) -> bool;
        /// Take a snapshot of all currently-buffered events. Subsequent
        /// `next_event` / `try_next_event` calls see only events that
        /// arrive after this point.
        fn drain_events(events: &mut Box<Events>) -> Box<DrainedEvents>;
        fn drained_events_len(drained: &Box<DrainedEvents>) -> usize;
        /// Pop the next event from a drained snapshot. Returns
        /// `event_type == Empty` once the snapshot is exhausted.
        fn drained_events_next(drained: &mut Box<DrainedEvents>) -> Box<DoraEvent>;
        fn event_type(event: &Box<DoraEvent>) -> DoraEventType;
        fn event_as_input(event: Box<DoraEvent>) -> Result<DoraInput>;
        /// Like `event_as_input`, but also hands back the input's
        /// metadata. Required by the server side of the Service and
        /// Action patterns, which must read the incoming `request_id` /
        /// `goal_id` in order to correlate its reply.
        fn event_as_input_with_metadata(event: Box<DoraEvent>) -> Result<DoraInputWithMetadata>;
        /// Extract the failure payload from a `NodeFailed` event.
        fn event_as_node_failed(event: Box<DoraEvent>) -> Result<DoraNodeFailed>;
        /// Selectively close one or more of this node's outputs without
        /// shutting the whole node down. Subsequent downstream
        /// subscribers see the corresponding `InputClosed` event.
        fn close_outputs(
            output_sender: &mut Box<OutputSender>,
            output_ids: Vec<String>,
        ) -> DoraResult;
        /// Return this node's `NodeRunConfig` (inputs / outputs /
        /// metadata block from the dataflow descriptor) serialized as
        /// JSON. Useful for runtime introspection: the C++ node can
        /// reason about its own declared interface without re-parsing
        /// the dataflow yaml.
        fn node_config_json(output_sender: &Box<OutputSender>) -> Result<String>;
        /// Return the full `Descriptor` (the parsed dataflow yaml)
        /// serialized as JSON. Useful for introspecting peer nodes,
        /// listing all topics, etc. May fail if the daemon hasn't
        /// delivered the descriptor yet.
        ///
        /// **Caution:** the returned JSON includes any `env` blocks
        /// declared on nodes in the dataflow yaml. Inline literals
        /// (`API_KEY: "..."`) and host-environment substitutions
        /// (`API_KEY: "${HOST_VAR}"`, expanded at descriptor parse
        /// time per `libraries/message/src/descriptor.rs`'s
        /// `with_expand_envs`) both appear as plain strings in the
        /// output. Don't pipe this into shared logs or telemetry
        /// without sanitizing if your dataflow can carry secrets in
        /// env vars.
        fn dataflow_descriptor_json(output_sender: &Box<OutputSender>) -> Result<String>;
        fn send_output(
            output_sender: &mut Box<OutputSender>,
            id: String,
            data: &[u8],
        ) -> DoraResult;
        fn log_message(
            output_sender: &Box<OutputSender>,
            level: String,
            message: String,
        ) -> DoraResult;
        fn send_output_with_metadata(
            output_sender: &mut Box<OutputSender>,
            id: String,
            data: &[u8],
            metadata: Box<Metadata>,
        ) -> DoraResult;

        // -------------------------------------------------------------
        // Service (request/reply) and Action (goal/feedback/result)
        // -------------------------------------------------------------

        /// Generate a unique, time-ordered request id (UUID v7).
        /// Mirrors `dora_node_api::DoraNode::new_request_id`.
        fn new_request_id() -> String;
        /// Alias of `new_request_id` that reads better in action
        /// (goal/feedback/result) contexts.
        fn new_goal_id() -> String;

        /// The terminal `goal_status` value for a goal that completed
        /// successfully. Use with `Metadata::set_goal_status` instead of
        /// hardcoding the string.
        fn goal_status_succeeded() -> String;
        /// The terminal `goal_status` value for a goal the server gave
        /// up on.
        fn goal_status_aborted() -> String;
        /// The terminal `goal_status` value for a goal canceled by the
        /// client.
        fn goal_status_canceled() -> String;

        /// Send a service request, injecting a freshly generated
        /// `request_id` into `metadata`, and return that id. Any
        /// `request_id` already present in `metadata` is replaced.
        fn send_service_request(
            output_sender: &mut Box<OutputSender>,
            output_id: String,
            data: &[u8],
            metadata: Box<Metadata>,
        ) -> DoraRequestId;

        /// Arrow-payload variant of `send_service_request`. Consumes the
        /// Arrow C Data Interface structs behind `array_ptr` /
        /// `schema_ptr` exactly like `send_arrow_output` does.
        unsafe fn send_arrow_service_request(
            output_sender: &mut Box<OutputSender>,
            output_id: String,
            array_ptr: *mut u8,
            schema_ptr: *mut u8,
            metadata: Box<Metadata>,
        ) -> DoraRequestId;

        /// Send a service response. A semantic alias of
        /// `send_output_with_metadata`: the server is expected to pass
        /// the incoming request's metadata (which carries `request_id`)
        /// straight through so the client can correlate the reply.
        fn send_service_response(
            output_sender: &mut Box<OutputSender>,
            output_id: String,
            data: &[u8],
            metadata: Box<Metadata>,
        ) -> DoraResult;

        /// Block until the response carrying `request_id` arrives from
        /// `server_node_id`, or up to `timeout_ms`.
        ///
        /// Non-matching events that arrive during the wait are buffered
        /// and replayed by later `next_event` calls, so the caller's
        /// main event loop loses nothing. If `server_node_id` restarts
        /// while waiting, the wait ends early with `ServerRestarted`
        /// rather than hanging until the deadline.
        fn recv_service_response(
            events: &mut Box<Events>,
            request_id: &str,
            server_node_id: &str,
            timeout_ms: u64,
        ) -> DoraPatternResult;

        /// Block until a *terminal* result (`goal_status` one of
        /// `succeeded` / `aborted` / `canceled`) for `goal_id` arrives
        /// from `server_node_id`, or up to `timeout_ms`.
        ///
        /// Intermediate feedback messages (same `goal_id`, no terminal
        /// `goal_status`) are buffered for the caller's own event loop
        /// rather than returned here.
        fn recv_action_result(
            events: &mut Box<Events>,
            goal_id: &str,
            server_node_id: &str,
            timeout_ms: u64,
        ) -> DoraPatternResult;

        /// Non-blocking poll for the response carrying `request_id`.
        ///
        /// Returns `NotReady` when the reply has not arrived yet, so a
        /// single-threaded node can check several outstanding requests
        /// per event-loop iteration without ever stalling. Follows the
        /// same convention as `try_next_event`.
        ///
        /// Non-matching events are buffered for later `next_event`
        /// calls exactly as in `recv_service_response`, and correlation
        /// state carries across calls — so polling in a loop is
        /// equivalent to waiting, minus the blocking.
        ///
        /// `timeout_ms` is owned by the framework: the first poll
        /// carrying one registers a deadline for `request_id`, and a
        /// later poll past it returns `Timeout` exactly once. So a
        /// caller does not sweep deadlines itself — it passes the same
        /// `timeout_ms` every iteration and reacts to `Timeout` like
        /// any other status. Pass `0` for no deadline.
        ///
        /// The clock starts at that first poll rather than at send
        /// time, and the first deadline registered for an id wins.
        ///
        /// Pass an empty `server_node_id` to accept a reply from any
        /// node and skip restart correlation.
        fn try_recv_service_response(
            events: &mut Box<Events>,
            request_id: &str,
            server_node_id: &str,
            timeout_ms: u64,
        ) -> DoraPatternResult;

        /// Non-blocking poll for a *terminal* result for `goal_id`.
        ///
        /// Returns `NotReady` while the goal is still running. Feedback
        /// messages stay buffered for the caller's own event loop.
        ///
        /// `timeout_ms` behaves as in `try_recv_service_response`, but
        /// bounds the *whole goal* rather than the gap between feedback
        /// messages: a long goal that is making visible progress will
        /// still expire. Pass `0` for no deadline, and an empty
        /// `server_node_id` to accept any responder.
        fn try_recv_action_result(
            events: &mut Box<Events>,
            goal_id: &str,
            server_node_id: &str,
            timeout_ms: u64,
        ) -> DoraPatternResult;

        /// `recv_service_response` over a set of acceptable responders,
        /// for a request fanned out to several nodes under one
        /// `request_id` where the first reply wins.
        ///
        /// An empty `server_node_ids` accepts a reply from any node.
        /// The set only affects restart detection: which reply matches
        /// is decided by `request_id` alone. A restart of any listed
        /// node is reported as `ServerRestarted` naming that node — a
        /// notification, not a verdict, since the other candidates may
        /// still answer.
        fn recv_service_response_from(
            events: &mut Box<Events>,
            request_id: &str,
            server_node_ids: &Vec<String>,
            timeout_ms: u64,
        ) -> DoraPatternResult;

        /// `recv_action_result` over a set of acceptable responders.
        /// See `recv_service_response_from`.
        fn recv_action_result_from(
            events: &mut Box<Events>,
            goal_id: &str,
            server_node_ids: &Vec<String>,
            timeout_ms: u64,
        ) -> DoraPatternResult;

        /// Non-blocking `recv_service_response_from`: the fan-out poll.
        fn try_recv_service_response_from(
            events: &mut Box<Events>,
            request_id: &str,
            server_node_ids: &Vec<String>,
            timeout_ms: u64,
        ) -> DoraPatternResult;

        /// Non-blocking `recv_action_result_from`.
        fn try_recv_action_result_from(
            events: &mut Box<Events>,
            goal_id: &str,
            server_node_ids: &Vec<String>,
            timeout_ms: u64,
        ) -> DoraPatternResult;

        /// Send a service request under a caller-supplied `request_id`.
        ///
        /// `send_service_request` mints a fresh id per call, so it
        /// cannot express one logical request fanned out to several
        /// nodes. Use `new_request_id()` once, then send each copy with
        /// this, and await them with `recv_service_response_from` /
        /// `try_recv_service_response_from`.
        fn send_service_request_with_id(
            output_sender: &mut Box<OutputSender>,
            output_id: String,
            data: &[u8],
            metadata: Box<Metadata>,
            request_id: &str,
        ) -> DoraResult;

        fn next(self: &mut CombinedEvents) -> CombinedEvent;

        fn is_dora(self: &CombinedEvent) -> bool;
        fn downcast_dora(event: CombinedEvent) -> Result<Box<DoraEvent>>;

        unsafe fn send_arrow_output(
            output_sender: &mut Box<OutputSender>,
            id: String,
            array_ptr: *mut u8,
            schema_ptr: *mut u8,
        ) -> DoraResult;

        #[cxx_name = "send_arrow_output"]
        unsafe fn send_arrow_output_with_metadata(
            output_sender: &mut Box<OutputSender>,
            id: String,
            array_ptr: *mut u8,
            schema_ptr: *mut u8,
            metadata: Box<Metadata>,
        ) -> DoraResult;

        unsafe fn event_as_arrow_input(
            event: Box<DoraEvent>,
            out_array: *mut u8,
            out_schema: *mut u8,
        ) -> DoraResult;

        unsafe fn event_as_arrow_input_with_info(
            event: Box<DoraEvent>,
            out_array: *mut u8,
            out_schema: *mut u8,
        ) -> ArrowInputInfo;

        fn new_metadata() -> Box<Metadata>;
        fn timestamp(self: &Metadata) -> u64;
        fn get_bool(self: &Metadata, key: &str) -> Result<bool>;
        fn get_float(self: &Metadata, key: &str) -> Result<f64>;
        fn get_int(self: &Metadata, key: &str) -> Result<i64>;
        fn get_str(self: &Metadata, key: &str) -> Result<String>;
        fn get_list_int(self: &Metadata, key: &str) -> Result<Vec<i64>>;
        fn get_list_float(self: &Metadata, key: &str) -> Result<Vec<f64>>;
        fn get_list_string(self: &Metadata, key: &str) -> Result<Vec<String>>;
        fn get_timestamp(self: &Metadata, key: &str) -> Result<i64>;
        fn get_json(self: &Metadata, key: &str) -> Result<String>;
        fn to_json(self: &Metadata) -> String;
        fn list_keys(self: &Metadata) -> Vec<String>;
        fn set_bool(self: &mut Metadata, key: &str, value: bool) -> Result<()>;
        fn set_int(self: &mut Metadata, key: &str, value: i64) -> Result<()>;
        fn set_float(self: &mut Metadata, key: &str, value: f64) -> Result<()>;
        fn set_string(self: &mut Metadata, key: &str, value: String) -> Result<()>;
        fn set_list_int(self: &mut Metadata, key: &str, value: Vec<i64>) -> Result<()>;
        fn set_list_float(self: &mut Metadata, key: &str, value: Vec<f64>) -> Result<()>;
        fn set_list_string(self: &mut Metadata, key: &str, value: Vec<String>) -> Result<()>;
        fn set_timestamp(self: &mut Metadata, key: &str, value: i64) -> Result<()>;
        #[cxx_name = "type"]
        fn value_type(self: &Metadata, key: &str) -> Result<MetadataValueType>;

        /// Typed accessors for the reserved correlation keys used by the
        /// Service and Action patterns. Equivalent to `set_string` /
        /// `get_str` with the well-known key name, but they keep the key
        /// spelling out of user code — a typo there silently breaks
        /// correlation instead of failing loudly.
        fn set_request_id(self: &mut Metadata, value: String) -> Result<()>;
        fn request_id(self: &Metadata) -> Result<String>;
        fn set_goal_id(self: &mut Metadata, value: String) -> Result<()>;
        fn goal_id(self: &Metadata) -> Result<String>;
        fn set_goal_status(self: &mut Metadata, value: String) -> Result<()>;
        fn goal_status(self: &Metadata) -> Result<String>;
    }
}

#[cfg(feature = "ros2-bridge")]
pub mod ros2 {
    // pub use dora_ros2_bridge::*;
    include!(env!("ROS2_BINDINGS_PATH"));
}

fn init_dora_node() -> eyre::Result<ffi::DoraNode> {
    let (node, events) = dora_node_api::DoraNode::init_from_env()?;
    let events = Events(events);
    let send_output = OutputSender(node);

    Ok(ffi::DoraNode {
        events: Box::new(events),
        send_output: Box::new(send_output),
    })
}

pub struct Events(EventStream);

impl Events {
    fn next(&mut self) -> Box<DoraEvent> {
        Box::new(DoraEvent(match self.0.recv() {
            Some(e) => EventOrReason::Event(e),
            None => EventOrReason::Closed,
        }))
    }
}

fn next_event(events: &mut Box<Events>) -> Box<DoraEvent> {
    events.next()
}

/// Block up to `timeout_ms` for the next event. Polls `try_recv` with
/// an `Instant`-based deadline (and a short sleep between polls) so we
/// can distinguish "timed out" from "stream closed" structurally —
/// `EventStream::recv_timeout` returns `None` for both cases and
/// matching on a wrapped `Event::Error(msg.contains("timed out"))`
/// would be fragile (#1409 review feedback).
fn next_event_timeout(events: &mut Box<Events>, timeout_ms: u64) -> Box<DoraEvent> {
    // `timeout_ms` arrives across the cxx::bridge as `u64`, so a C++
    // caller can supply values that would panic the underlying
    // `Instant + Duration` arithmetic on platforms with a bounded
    // Instant range. Use `checked_add` and treat overflow as
    // "effectively no deadline" -- keep polling indefinitely until
    // an event arrives or the stream closes. Never returns
    // `TimedOut` in the overflow case, matching the caller's
    // intent ("wait as long as needed").
    let deadline = Instant::now().checked_add(Duration::from_millis(timeout_ms));
    // 1 ms keeps the busy-wait cost negligible while bounding overshoot
    // of the caller-supplied deadline to ~1 ms in the worst case.
    let poll_interval = Duration::from_millis(1);
    loop {
        match events.0.try_recv() {
            Ok(event) => return Box::new(DoraEvent(EventOrReason::Event(event))),
            Err(TryRecvError::Closed) => return Box::new(DoraEvent(EventOrReason::Closed)),
            Err(TryRecvError::Empty) => match deadline {
                Some(d) => {
                    let now = Instant::now();
                    if now >= d {
                        return Box::new(DoraEvent(EventOrReason::TimedOut));
                    }
                    std::thread::sleep(std::cmp::min(d - now, poll_interval));
                }
                None => std::thread::sleep(poll_interval),
            },
        }
    }
}

/// Non-blocking single poll. Returns `EventOrReason::Empty` (surfaced
/// as `DoraEventType::Empty` on the C++ side) when no event is
/// available; distinct from a real timeout because no timeout was
/// requested.
fn try_next_event(events: &mut Box<Events>) -> Box<DoraEvent> {
    Box::new(DoraEvent(match events.0.try_recv() {
        Ok(event) => EventOrReason::Event(event),
        Err(TryRecvError::Empty) => EventOrReason::Empty,
        Err(TryRecvError::Closed) => EventOrReason::Closed,
    }))
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn events_is_empty(events: &Box<Events>) -> bool {
    events.0.is_empty()
}

/// Snapshot of buffered events at a point in time. Subsequent
/// receives on the `Events` stream see only events that arrive after
/// this snapshot.
pub struct DrainedEvents(VecDeque<Event>);

fn drain_events(events: &mut Box<Events>) -> Box<DrainedEvents> {
    let drained = events.0.drain().unwrap_or_default();
    Box::new(DrainedEvents(drained.into()))
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn drained_events_len(drained: &Box<DrainedEvents>) -> usize {
    drained.0.len()
}

fn drained_events_next(drained: &mut Box<DrainedEvents>) -> Box<DoraEvent> {
    Box::new(DoraEvent(match drained.0.pop_front() {
        Some(e) => EventOrReason::Event(e),
        // Drained snapshots cannot transition from non-empty to closed
        // (the snapshot is frozen at `drain_events` time), so an
        // exhausted drain is `Empty`, not `Closed`.
        None => EventOrReason::Empty,
    }))
}

fn dora_events_into_combined(events: Box<Events>) -> ffi::CombinedEvents {
    let events = events.0.map(MergedEvent::Dora);
    ffi::CombinedEvents {
        events: Box::new(MergedEvents {
            events: Some(Box::new(events)),
            #[cfg(any(feature = "ros2-bridge", test))]
            next_id: 1,
        }),
    }
}

fn empty_combined_events() -> ffi::CombinedEvents {
    ffi::CombinedEvents {
        events: Box::new(MergedEvents {
            events: Some(Box::new(stream::empty())),
            #[cfg(any(feature = "ros2-bridge", test))]
            next_id: 1,
        }),
    }
}

pub struct DoraEvent(EventOrReason);

/// Internal representation that lets `try_next_event` /
/// `next_event_timeout` / `drained_events_next` report "no event"
/// outcomes to C++ without overloading `Option<Event>` (#1409 review).
//
// `Event` is much larger than the unit variants (Closed/Empty/TimedOut),
// but this enum is always wrapped in `Box<DoraEvent>` at the FFI boundary
// so the size delta isn't paid per-stack-slot. Boxing the `Event` variant
// would add a second allocation on every event delivery for no benefit.
#[allow(clippy::large_enum_variant)]
pub(crate) enum EventOrReason {
    Event(Event),
    /// Source stream closed; surfaces as `DoraEventType::AllInputsClosed`.
    Closed,
    /// Non-blocking poll found no event ready, or a drained snapshot
    /// was exhausted. Surfaces as `DoraEventType::Empty`.
    Empty,
    /// `next_event_timeout` deadline elapsed before an event arrived.
    /// Surfaces as `DoraEventType::Timeout`.
    TimedOut,
}

fn event_type(event: &DoraEvent) -> ffi::DoraEventType {
    match &event.0 {
        EventOrReason::Event(event) => match event {
            Event::Stop(_) => ffi::DoraEventType::Stop,
            Event::Input { .. } => ffi::DoraEventType::Input,
            Event::InputClosed { .. } => ffi::DoraEventType::InputClosed,
            Event::Error(_) => ffi::DoraEventType::Error,
            Event::NodeFailed { .. } => ffi::DoraEventType::NodeFailed,
            Event::Reload { .. } => ffi::DoraEventType::Reload,
            _ => ffi::DoraEventType::Unknown,
        },
        EventOrReason::Closed => ffi::DoraEventType::AllInputsClosed,
        EventOrReason::Empty => ffi::DoraEventType::Empty,
        EventOrReason::TimedOut => ffi::DoraEventType::Timeout,
    }
}

// `Box<DoraEvent>` is mandated by the cxx bridge signature (ownership transfer
// across the FFI boundary), so the `boxed_local` lint is a false positive.
#[allow(clippy::boxed_local)]
fn event_as_input(event: Box<DoraEvent>) -> eyre::Result<ffi::DoraInput> {
    let EventOrReason::Event(Event::Input { id, data, .. }) = event.0 else {
        bail!("not an input event");
    };

    Ok(ffi::DoraInput {
        id: id.into(),
        data: input_bytes(&data)?,
    })
}

#[allow(clippy::boxed_local)] // `Box<DoraEvent>` is mandated by the cxx bridge signature.
fn event_as_input_with_metadata(event: Box<DoraEvent>) -> eyre::Result<ffi::DoraInputWithMetadata> {
    let EventOrReason::Event(Event::Input { id, metadata, data }) = event.0 else {
        bail!("not an input event");
    };

    Ok(ffi::DoraInputWithMetadata {
        id: id.into(),
        data: input_bytes(&data)?,
        metadata: Box::new(Metadata::from_dora(metadata)?),
    })
}

/// Decode a byte-payload input. The payload arrives as a self-describing
/// Arrow IPC stream, so the type is read from the array itself rather
/// than assumed.
fn input_bytes(data: &dora_node_api::ArrowData) -> eyre::Result<Vec<u8>> {
    match data.data_type() {
        dora_node_api::arrow::datatypes::DataType::UInt8 => {
            let array: &UInt8Array = data.as_primitive();
            Ok(array.values().to_vec())
        }
        dora_node_api::arrow::datatypes::DataType::Null => Ok(vec![]),
        other => {
            bail!(
                "unsupported input arrow type {other:?}; \
                 use event_as_arrow_input for typed access"
            );
        }
    }
}

#[allow(clippy::boxed_local)] // `Box<DoraEvent>` is mandated by the cxx bridge signature.
fn event_as_node_failed(event: Box<DoraEvent>) -> eyre::Result<ffi::DoraNodeFailed> {
    let EventOrReason::Event(Event::NodeFailed {
        affected_input_ids,
        error,
        source_node_id,
    }) = event.0
    else {
        bail!("not a NodeFailed event");
    };
    Ok(ffi::DoraNodeFailed {
        affected_input_ids: affected_input_ids
            .into_iter()
            .map(|id| id.to_string())
            .collect(),
        error,
        source_node_id: source_node_id.to_string(),
    })
}

/// Parse a caller-supplied output id via `FromStr` instead of the panicking
/// `From<String>`.
///
/// `DataId::from(String)` is documented as panicking on invalid characters
/// (see `libraries/message/src/id.rs`, `# Panics`). Calling it on a
/// caller-supplied string would mean a typo in a C++ string literal aborts
/// the whole node by unwinding across the `cxx::bridge`. Returning the error
/// as a `DoraResult.error` instead lets the C++ caller handle it.
fn parse_output_id(id: &str) -> Result<dora_node_api::dora_core::config::DataId, ffi::DoraResult> {
    id.parse().map_err(|e| ffi::DoraResult {
        error: format!("invalid output id '{id}': {e}"),
    })
}

fn close_outputs(
    output_sender: &mut Box<OutputSender>,
    output_ids: Vec<String>,
) -> ffi::DoraResult {
    let mut ids = Vec::with_capacity(output_ids.len());
    for id in output_ids {
        match parse_output_id(&id) {
            Ok(parsed) => ids.push(parsed),
            Err(err) => return err,
        }
    }
    match output_sender.0.close_outputs(ids) {
        Ok(()) => ffi::DoraResult {
            error: String::new(),
        },
        Err(err) => ffi::DoraResult {
            error: format!("{err:?}"),
        },
    }
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn node_config_json(output_sender: &Box<OutputSender>) -> eyre::Result<String> {
    serde_json::to_string(output_sender.0.node_config())
        .map_err(|e| eyre!("failed to serialize node config: {e}"))
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn dataflow_descriptor_json(output_sender: &Box<OutputSender>) -> eyre::Result<String> {
    let desc = output_sender.0.dataflow_descriptor()?;
    serde_json::to_string(desc).map_err(|e| eyre!("failed to serialize dataflow descriptor: {e}"))
}

#[allow(clippy::boxed_local)] // `Box<DoraEvent>` is mandated by the cxx bridge signature.
unsafe fn event_as_arrow_input(
    event: Box<DoraEvent>,
    out_array: *mut u8,
    out_schema: *mut u8,
) -> ffi::DoraResult {
    // Cast to Arrow FFI types
    let out_array = out_array as *mut arrow::ffi::FFI_ArrowArray;
    let out_schema = out_schema as *mut arrow::ffi::FFI_ArrowSchema;

    let EventOrReason::Event(Event::Input {
        id: _,
        metadata: _,
        data,
    }) = event.0
    else {
        return ffi::DoraResult {
            error: "Not an input event".to_string(),
        };
    };

    if out_array.is_null() || out_schema.is_null() {
        return ffi::DoraResult {
            error: "Received null output pointer".to_string(),
        };
    }

    let array_data = data.to_data();

    match arrow::ffi::to_ffi(&array_data) {
        Ok((ffi_array, ffi_schema)) => {
            unsafe {
                std::ptr::write(out_array, ffi_array);
                std::ptr::write(out_schema, ffi_schema);
            }
            ffi::DoraResult {
                error: String::new(),
            }
        }
        Err(e) => ffi::DoraResult {
            error: format!("Error exporting Arrow array to C++: {e:?}"),
        },
    }
}

pub struct Metadata {
    timestamp: u64,
    parameters: BTreeMap<String, DoraParameter>,
}

impl Metadata {
    fn from_dora(metadata: DoraMetadata) -> EyreResult<Self> {
        Ok(Self {
            timestamp: metadata.timestamp().get_time().as_u64(),
            parameters: metadata.parameters,
        })
    }

    fn empty() -> Self {
        Self {
            timestamp: 0,
            parameters: BTreeMap::new(),
        }
    }

    fn parameter_type_name(parameter: &DoraParameter) -> &'static str {
        match parameter {
            DoraParameter::Bool(_) => "bool",
            DoraParameter::Integer(_) => "integer",
            DoraParameter::String(_) => "string",
            DoraParameter::Float(_) => "float",
            DoraParameter::ListInt(_) => "list<int>",
            DoraParameter::ListFloat(_) => "list<float>",
            DoraParameter::ListString(_) => "list<string>",
            DoraParameter::Timestamp(_) => "timestamp",
        }
    }

    fn expect_parameter<'a>(&'a self, key: &str) -> EyreResult<&'a DoraParameter> {
        self.parameters
            .get(key)
            .ok_or_else(|| eyre!("metadata missing key '{key}'"))
    }

    fn parameter_to_json(parameter: &DoraParameter, _key: &str) -> EyreResult<JsonValue> {
        match parameter {
            DoraParameter::Bool(value) => Ok(JsonValue::Bool(*value)),
            DoraParameter::Integer(value) => Ok(JsonValue::from(*value)),
            DoraParameter::Float(value) => Ok(JsonValue::from(*value)),
            DoraParameter::String(value) => Ok(JsonValue::String(value.clone())),
            DoraParameter::ListInt(values) => Ok(JsonValue::Array(
                values.iter().map(|value| JsonValue::from(*value)).collect(),
            )),
            DoraParameter::ListFloat(values) => Ok(JsonValue::Array(
                values.iter().map(|value| JsonValue::from(*value)).collect(),
            )),
            DoraParameter::ListString(values) => Ok(JsonValue::Array(
                values
                    .iter()
                    .map(|value| JsonValue::String(value.clone()))
                    .collect(),
            )),
            DoraParameter::Timestamp(dt) => {
                serde_json::to_value(dt).map_err(|e| eyre!("failed to serialize timestamp: {e}"))
            }
        }
    }

    pub fn timestamp(&self) -> u64 {
        self.timestamp
    }

    pub fn get_bool(&self, key: &str) -> EyreResult<bool> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            DoraParameter::Bool(value) => Ok(*value),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'bool'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_float(&self, key: &str) -> EyreResult<f64> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            DoraParameter::Float(value) => Ok(*value),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'float'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_int(&self, key: &str) -> EyreResult<i64> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            DoraParameter::Integer(value) => Ok(*value),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'integer'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_str(&self, key: &str) -> EyreResult<String> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            DoraParameter::String(value) => Ok(value.clone()),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'string'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_list_int(&self, key: &str) -> EyreResult<Vec<i64>> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            DoraParameter::ListInt(values) => Ok(values.clone()),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'list<int>'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_list_float(&self, key: &str) -> EyreResult<Vec<f64>> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            DoraParameter::ListFloat(values) => Ok(values.clone()),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'list<float>'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_list_string(&self, key: &str) -> EyreResult<Vec<String>> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            DoraParameter::ListString(values) => Ok(values.clone()),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'list<string>'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_timestamp(&self, key: &str) -> EyreResult<i64> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            DoraParameter::Timestamp(dt) => {
                // Convert chrono::DateTime<Utc> to nanoseconds since Unix epoch
                dt.timestamp_nanos_opt()
                    .ok_or_else(|| eyre!("Timestamp out of range for conversion to nanoseconds"))
            }
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'timestamp'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_json(&self, key: &str) -> EyreResult<String> {
        let parameter = self.expect_parameter(key)?;
        let json_value = Metadata::parameter_to_json(parameter, key)?;
        serde_json::to_string(&json_value)
            .map_err(|err| eyre!("failed to serialize metadata value '{key}' to JSON: {err}"))
    }

    pub fn to_json(&self) -> String {
        #[derive(Serialize)]
        struct MetadataJson<'a> {
            timestamp: u64,
            parameters: &'a BTreeMap<String, DoraParameter>,
        }

        serde_json::to_string(&MetadataJson {
            timestamp: self.timestamp,
            parameters: &self.parameters,
        })
        .expect("failed to serialize metadata to JSON")
    }

    pub fn list_keys(&self) -> Vec<String> {
        self.parameters.keys().cloned().collect()
    }

    pub fn set_bool(&mut self, key: &str, value: bool) -> EyreResult<()> {
        self.insert_parameter(key, DoraParameter::Bool(value))
    }

    pub fn set_int(&mut self, key: &str, value: i64) -> EyreResult<()> {
        self.insert_parameter(key, DoraParameter::Integer(value))
    }

    pub fn set_float(&mut self, key: &str, value: f64) -> EyreResult<()> {
        self.insert_parameter(key, DoraParameter::Float(value))
    }

    pub fn set_string(&mut self, key: &str, value: String) -> EyreResult<()> {
        self.insert_parameter(key, DoraParameter::String(value))
    }

    pub fn set_list_int(&mut self, key: &str, value: Vec<i64>) -> EyreResult<()> {
        self.insert_parameter(key, DoraParameter::ListInt(value))
    }

    pub fn set_list_float(&mut self, key: &str, value: Vec<f64>) -> EyreResult<()> {
        self.insert_parameter(key, DoraParameter::ListFloat(value))
    }

    pub fn set_list_string(&mut self, key: &str, value: Vec<String>) -> EyreResult<()> {
        self.insert_parameter(key, DoraParameter::ListString(value))
    }

    pub fn set_timestamp(&mut self, key: &str, value: i64) -> EyreResult<()> {
        // Convert nanoseconds since Unix epoch to chrono::DateTime<Utc>.
        //
        // Use Euclidean division so pre-epoch (negative) timestamps split
        // correctly. Plain `/` and `%` truncate toward zero, yielding a
        // *negative* remainder for negative inputs; casting that to `u32`
        // wraps it to a huge value and makes `from_timestamp` reject the
        // (otherwise valid) instant. `div_euclid`/`rem_euclid` keep the
        // remainder in `0..1_000_000_000`, matching how `from_timestamp`
        // interprets `(secs, subsec_nanos)` and round-tripping with
        // `get_timestamp` for negative values.
        let secs = value.div_euclid(1_000_000_000);
        let subsec_nanos = value.rem_euclid(1_000_000_000) as u32;

        let dt = DateTime::from_timestamp(secs, subsec_nanos)
            .ok_or_else(|| eyre!("Invalid timestamp: out of range (nanos: {value})"))?;

        self.insert_parameter(key, DoraParameter::Timestamp(dt))
    }

    pub fn value_type(&self, key: &str) -> EyreResult<MetadataValueType> {
        let parameter = self.expect_parameter(key)?;
        let value_type = match parameter {
            DoraParameter::Bool(_) => MetadataValueType::Bool,
            DoraParameter::Integer(_) => MetadataValueType::Integer,
            DoraParameter::Float(_) => MetadataValueType::Float,
            DoraParameter::String(_) => MetadataValueType::String,
            DoraParameter::ListInt(_) => MetadataValueType::ListInt,
            DoraParameter::ListFloat(_) => MetadataValueType::ListFloat,
            DoraParameter::ListString(_) => MetadataValueType::ListString,
            DoraParameter::Timestamp(_) => MetadataValueType::Timestamp,
        };
        Ok(value_type)
    }

    pub fn set_request_id(&mut self, value: String) -> EyreResult<()> {
        self.set_string(dora_node_api::REQUEST_ID, value)
    }

    pub fn request_id(&self) -> EyreResult<String> {
        self.get_str(dora_node_api::REQUEST_ID)
    }

    pub fn set_goal_id(&mut self, value: String) -> EyreResult<()> {
        self.set_string(dora_node_api::GOAL_ID, value)
    }

    pub fn goal_id(&self) -> EyreResult<String> {
        self.get_str(dora_node_api::GOAL_ID)
    }

    pub fn set_goal_status(&mut self, value: String) -> EyreResult<()> {
        self.set_string(dora_node_api::GOAL_STATUS, value)
    }

    pub fn goal_status(&self) -> EyreResult<String> {
        self.get_str(dora_node_api::GOAL_STATUS)
    }

    fn into_parameters(self) -> DoraMetadataParameters {
        self.parameters
    }

    fn insert_parameter(&mut self, key: &str, parameter: DoraParameter) -> EyreResult<()> {
        self.parameters.insert(key.to_string(), parameter);
        Ok(())
    }
}

#[allow(clippy::boxed_local)] // `Box<DoraEvent>` is mandated by the cxx bridge signature.
unsafe fn event_as_arrow_input_with_info(
    event: Box<DoraEvent>,
    out_array: *mut u8,
    out_schema: *mut u8,
) -> ffi::ArrowInputInfo {
    // Cast to Arrow FFI types
    let out_array = out_array as *mut arrow::ffi::FFI_ArrowArray;
    let out_schema = out_schema as *mut arrow::ffi::FFI_ArrowSchema;

    let EventOrReason::Event(Event::Input { id, metadata, data }) = event.0 else {
        return ffi::ArrowInputInfo {
            id: String::new(),
            metadata: Box::new(Metadata::empty()),
            error: "Not an input event".to_string(),
        };
    };

    if out_array.is_null() || out_schema.is_null() {
        return ffi::ArrowInputInfo {
            id: id.to_string(),
            metadata: Box::new(Metadata::empty()),
            error: "Received null output pointer".to_string(),
        };
    }

    let prepared_metadata = match Metadata::from_dora(metadata) {
        Ok(metadata) => metadata,
        Err(err) => {
            return ffi::ArrowInputInfo {
                id: id.to_string(),
                metadata: Box::new(Metadata::empty()),
                error: format!("Error preparing metadata: {err:?}"),
            };
        }
    };

    let array_data = data.to_data();

    match arrow::ffi::to_ffi(&array_data) {
        Ok((ffi_array, ffi_schema)) => {
            unsafe {
                std::ptr::write(out_array, ffi_array);
                std::ptr::write(out_schema, ffi_schema);
            }
            ffi::ArrowInputInfo {
                id: id.to_string(),
                metadata: Box::new(prepared_metadata),
                error: String::new(),
            }
        }
        Err(e) => ffi::ArrowInputInfo {
            id: id.to_string(),
            metadata: Box::new(Metadata::empty()),
            error: format!("Error exporting Arrow array to C++: {e:?}"),
        },
    }
}

pub struct OutputSender(dora_node_api::DoraNode);

fn send_output(sender: &mut Box<OutputSender>, id: String, data: &[u8]) -> ffi::DoraResult {
    send_output_internal(sender, id, data, Default::default())
}

#[allow(clippy::borrowed_box)]
fn log_message(sender: &Box<OutputSender>, level: String, message: String) -> ffi::DoraResult {
    sender.0.log(&level, &message, None);
    ffi::DoraResult {
        error: String::new(),
    }
}

#[allow(clippy::boxed_local)]
fn send_output_with_metadata(
    sender: &mut Box<OutputSender>,
    id: String,
    data: &[u8],
    metadata: Box<Metadata>,
) -> ffi::DoraResult {
    let metadata = *metadata;
    let parameters = metadata.into_parameters();
    send_output_internal(sender, id, data, parameters)
}

fn send_output_internal(
    sender: &mut Box<OutputSender>,
    id: String,
    data: &[u8],
    metadata: DoraMetadataParameters,
) -> ffi::DoraResult {
    let output_id = match parse_output_id(&id) {
        Ok(parsed) => parsed,
        Err(err) => return err,
    };
    let result = sender
        .0
        .send_output_raw(output_id, metadata, data.len(), |out| {
            out.copy_from_slice(data)
        });
    let error = match result {
        Ok(()) => String::new(),
        Err(err) => format!("{err:?}"),
    };
    ffi::DoraResult { error }
}

// ---------------------------------------------------------------------
// Service (request/reply) and Action (goal/feedback/result)
// ---------------------------------------------------------------------

fn new_request_id() -> String {
    dora_node_api::DoraNode::new_request_id()
}

fn new_goal_id() -> String {
    dora_node_api::DoraNode::new_goal_id()
}

fn goal_status_succeeded() -> String {
    dora_node_api::GOAL_STATUS_SUCCEEDED.to_string()
}

fn goal_status_aborted() -> String {
    dora_node_api::GOAL_STATUS_ABORTED.to_string()
}

fn goal_status_canceled() -> String {
    dora_node_api::GOAL_STATUS_CANCELED.to_string()
}

#[allow(clippy::boxed_local)] // `Box<Metadata>` is mandated by the cxx bridge signature.
fn send_service_request(
    sender: &mut Box<OutputSender>,
    output_id: String,
    data: &[u8],
    metadata: Box<Metadata>,
) -> ffi::DoraRequestId {
    let mut parameters = (*metadata).into_parameters();
    let request_id = insert_request_id(&mut parameters);

    let result = send_output_internal(sender, output_id, data, parameters);
    finish_request(request_id, result)
}

unsafe fn send_arrow_service_request(
    sender: &mut Box<OutputSender>,
    output_id: String,
    array_ptr: *mut u8,
    schema_ptr: *mut u8,
    mut metadata: Box<Metadata>,
) -> ffi::DoraRequestId {
    let request_id = insert_request_id(&mut metadata.parameters);

    let result =
        unsafe { send_arrow_output_impl(sender, output_id, array_ptr, schema_ptr, Some(metadata)) };
    finish_request(request_id, result)
}

/// Overwrite `parameters`'s `request_id` with a fresh UUID v7 and return
/// it, matching `DoraNode::send_service_request`. Callers must not rely
/// on a self-supplied id surviving: the whole point of the helper is
/// that the framework owns the correlation key.
fn insert_request_id(parameters: &mut DoraMetadataParameters) -> String {
    let request_id = dora_node_api::DoraNode::new_request_id();
    parameters.insert(
        dora_node_api::REQUEST_ID.to_string(),
        DoraParameter::String(request_id.clone()),
    );
    request_id
}

/// Pair the generated request id with the send outcome. The id is only
/// meaningful if the send succeeded, so an error clears it — otherwise a
/// caller that skipped the `error` check would wait on a correlation that
/// was never put on the wire.
fn finish_request(request_id: String, result: ffi::DoraResult) -> ffi::DoraRequestId {
    if result.error.is_empty() {
        ffi::DoraRequestId {
            request_id,
            error: String::new(),
        }
    } else {
        ffi::DoraRequestId {
            request_id: String::new(),
            error: result.error,
        }
    }
}

/// Semantic alias of `send_output_with_metadata`, mirroring
/// `DoraNode::send_service_response`. It exists so the server side of an
/// exchange reads as a reply rather than an unrelated output; the
/// correlation itself travels in the passed-through `metadata`.
fn send_service_response(
    sender: &mut Box<OutputSender>,
    output_id: String,
    data: &[u8],
    metadata: Box<Metadata>,
) -> ffi::DoraResult {
    send_output_with_metadata(sender, output_id, data, metadata)
}

fn recv_service_response(
    events: &mut Box<Events>,
    request_id: &str,
    server_node_id: &str,
    timeout_ms: u64,
) -> ffi::DoraPatternResult {
    let (server, timeout) = match pattern_wait_args(server_node_id, timeout_ms) {
        Ok(args) => args,
        Err(result) => return result,
    };
    pattern_result(futures_lite::future::block_on(
        events.0.recv_service_response(request_id, &server, timeout),
    ))
}

fn recv_action_result(
    events: &mut Box<Events>,
    goal_id: &str,
    server_node_id: &str,
    timeout_ms: u64,
) -> ffi::DoraPatternResult {
    let (server, timeout) = match pattern_wait_args(server_node_id, timeout_ms) {
        Ok(args) => args,
        Err(result) => return result,
    };
    pattern_result(futures_lite::future::block_on(
        events.0.recv_action_result(goal_id, &server, timeout),
    ))
}

fn try_recv_service_response(
    events: &mut Box<Events>,
    request_id: &str,
    server_node_id: &str,
    timeout_ms: u64,
) -> ffi::DoraPatternResult {
    let servers = match parse_server_list(std::slice::from_ref(&server_node_id)) {
        Ok(servers) => servers,
        Err(result) => return result,
    };
    try_pattern_result(events.0.try_recv_service_response(
        request_id,
        servers.as_ref(),
        poll_timeout(timeout_ms),
    ))
}

fn try_recv_action_result(
    events: &mut Box<Events>,
    goal_id: &str,
    server_node_id: &str,
    timeout_ms: u64,
) -> ffi::DoraPatternResult {
    let servers = match parse_server_list(std::slice::from_ref(&server_node_id)) {
        Ok(servers) => servers,
        Err(result) => return result,
    };
    try_pattern_result(events.0.try_recv_action_result(
        goal_id,
        servers.as_ref(),
        poll_timeout(timeout_ms),
    ))
}

/// `&Vec<String>` rather than `&[String]`: cxx maps `Vec<String>` to
/// `rust::Vec<rust::String>` and has no slice-of-String equivalent, so
/// the signature is dictated by the bridge (the same reason the
/// `borrowed_box` allow appears elsewhere in this file).
#[allow(clippy::ptr_arg)]
fn recv_service_response_from(
    events: &mut Box<Events>,
    request_id: &str,
    server_node_ids: &Vec<String>,
    timeout_ms: u64,
) -> ffi::DoraPatternResult {
    let servers = match parse_server_strings(server_node_ids) {
        Ok(servers) => servers,
        Err(result) => return result,
    };
    pattern_result(futures_lite::future::block_on(
        events.0.recv_service_response_from(
            request_id,
            servers.as_ref(),
            clamp_pattern_timeout(timeout_ms),
        ),
    ))
}

/// `&Vec<String>` rather than `&[String]`: cxx maps `Vec<String>` to
/// `rust::Vec<rust::String>` and has no slice-of-String equivalent, so
/// the signature is dictated by the bridge (the same reason the
/// `borrowed_box` allow appears elsewhere in this file).
#[allow(clippy::ptr_arg)]
fn recv_action_result_from(
    events: &mut Box<Events>,
    goal_id: &str,
    server_node_ids: &Vec<String>,
    timeout_ms: u64,
) -> ffi::DoraPatternResult {
    let servers = match parse_server_strings(server_node_ids) {
        Ok(servers) => servers,
        Err(result) => return result,
    };
    pattern_result(futures_lite::future::block_on(
        events.0.recv_action_result_from(
            goal_id,
            servers.as_ref(),
            clamp_pattern_timeout(timeout_ms),
        ),
    ))
}

/// `&Vec<String>` rather than `&[String]`: cxx maps `Vec<String>` to
/// `rust::Vec<rust::String>` and has no slice-of-String equivalent, so
/// the signature is dictated by the bridge (the same reason the
/// `borrowed_box` allow appears elsewhere in this file).
#[allow(clippy::ptr_arg)]
fn try_recv_service_response_from(
    events: &mut Box<Events>,
    request_id: &str,
    server_node_ids: &Vec<String>,
    timeout_ms: u64,
) -> ffi::DoraPatternResult {
    let servers = match parse_server_strings(server_node_ids) {
        Ok(servers) => servers,
        Err(result) => return result,
    };
    try_pattern_result(events.0.try_recv_service_response(
        request_id,
        servers.as_ref(),
        poll_timeout(timeout_ms),
    ))
}

/// `&Vec<String>` rather than `&[String]`: cxx maps `Vec<String>` to
/// `rust::Vec<rust::String>` and has no slice-of-String equivalent, so
/// the signature is dictated by the bridge (the same reason the
/// `borrowed_box` allow appears elsewhere in this file).
#[allow(clippy::ptr_arg)]
fn try_recv_action_result_from(
    events: &mut Box<Events>,
    goal_id: &str,
    server_node_ids: &Vec<String>,
    timeout_ms: u64,
) -> ffi::DoraPatternResult {
    let servers = match parse_server_strings(server_node_ids) {
        Ok(servers) => servers,
        Err(result) => return result,
    };
    try_pattern_result(events.0.try_recv_action_result(
        goal_id,
        servers.as_ref(),
        poll_timeout(timeout_ms),
    ))
}

/// Owned form of [`ExpectedServers`], which borrows.
///
/// The parsed `NodeId`s must outlive the borrow handed to the receive,
/// so they are held here and lent out via [`Self::as_ref`].
enum OwnedServers {
    /// No node ids were supplied — accept any responder.
    Any,
    /// One or more parsed node ids.
    Some(Vec<NodeId>),
}

impl OwnedServers {
    fn as_ref(&self) -> ExpectedServers<'_> {
        match self {
            Self::Any => ExpectedServers::Any,
            Self::Some(ids) => ExpectedServers::AnyOf(ids),
        }
    }
}

/// Parse a C++-supplied list of server node ids.
///
/// An empty list — or a single empty string, which is how the
/// single-server polls spell "anyone" — means [`OwnedServers::Any`].
/// Empty entries in a non-trivial list are rejected rather than silently
/// widening the set to "any", which would be a surprising way for a
/// stray `""` to disable restart detection.
fn parse_server_strings(ids: &[String]) -> Result<OwnedServers, ffi::DoraPatternResult> {
    let borrowed: Vec<&str> = ids.iter().map(String::as_str).collect();
    parse_server_list(&borrowed)
}

fn parse_server_list(ids: &[&str]) -> Result<OwnedServers, ffi::DoraPatternResult> {
    if ids.is_empty() || (ids.len() == 1 && ids[0].is_empty()) {
        return Ok(OwnedServers::Any);
    }
    let mut parsed = Vec::with_capacity(ids.len());
    for id in ids {
        parsed.push(parse_server_node_id(id)?);
    }
    Ok(OwnedServers::Some(parsed))
}

/// Map a C++ `timeout_ms` onto the Rust poll's optional deadline.
///
/// `0` means "no deadline" — the poll then behaves exactly as it did
/// before deadlines existed. Any other value is clamped the same way
/// `clamp_pattern_timeout` clamps the blocking waits, because it
/// crosses the bridge as an unbounded `u64` and ends up in an
/// `Instant + Duration`.
fn poll_timeout(timeout_ms: u64) -> Option<Duration> {
    if timeout_ms == 0 {
        None
    } else {
        Some(clamp_pattern_timeout(timeout_ms))
    }
}

/// Map a non-blocking correlated receive onto the C++ result struct.
///
/// `Ok(None)` becomes `NotReady` with an `Empty` event, mirroring how
/// `try_next_event` reports "nothing available" — deliberately distinct
/// from `Timeout`, since no deadline was involved.
fn try_pattern_result(outcome: Result<Option<Event>, PatternError>) -> ffi::DoraPatternResult {
    match outcome {
        Ok(Some(event)) => pattern_result(Ok(event)),
        Ok(None) => pattern_failure(
            ffi::DoraPatternStatus::NotReady,
            String::new(),
            EventOrReason::Empty,
        ),
        Err(err) => pattern_result(Err(err)),
    }
}

#[allow(clippy::boxed_local)] // `Box<Metadata>` is mandated by the cxx bridge signature.
fn send_service_request_with_id(
    sender: &mut Box<OutputSender>,
    output_id: String,
    data: &[u8],
    metadata: Box<Metadata>,
    request_id: &str,
) -> ffi::DoraResult {
    let mut parameters = (*metadata).into_parameters();
    set_request_id(&mut parameters, request_id);
    send_output_internal(sender, output_id, data, parameters)
}

/// Pin `parameters` to a caller-supplied `request_id`.
///
/// The counterpart of [`insert_request_id`], which always mints a fresh
/// one. Split out so the "the caller's id survives" contract can be
/// tested without a live daemon connection.
fn set_request_id(parameters: &mut DoraMetadataParameters, request_id: &str) {
    parameters.insert(
        dora_node_api::REQUEST_ID.to_string(),
        DoraParameter::String(request_id.to_owned()),
    );
}

/// Validate and normalise the arguments shared by both pattern-aware
/// waits, before anything is awaited.
///
/// The node id is parsed via `FromStr` rather than `From<String>`: the
/// latter is documented as panicking on invalid characters
/// (`libraries/message/src/id.rs`), so a typo in a C++ string literal
/// would abort the whole node instead of returning an error.
fn pattern_wait_args(
    server_node_id: &str,
    timeout_ms: u64,
) -> Result<(NodeId, Duration), ffi::DoraPatternResult> {
    Ok((
        parse_server_node_id(server_node_id)?,
        clamp_pattern_timeout(timeout_ms),
    ))
}

/// Parse one server node id, reporting a malformed one as
/// `InvalidArgument` instead of panicking.
///
/// `FromStr` rather than `From<String>`: the latter is documented as
/// panicking on invalid characters (`libraries/message/src/id.rs`), so a
/// typo in a C++ string literal would abort the whole node.
fn parse_server_node_id(server_node_id: &str) -> Result<NodeId, ffi::DoraPatternResult> {
    server_node_id.parse::<NodeId>().map_err(|e| {
        pattern_failure(
            ffi::DoraPatternStatus::InvalidArgument,
            format!("invalid server node id '{server_node_id}': {e}"),
            EventOrReason::Empty,
        )
    })
}

/// Convert `timeout_ms` into a `Duration` that `Instant::now() + dur`
/// can represent.
///
/// `timeout_ms` crosses the bridge as an unbounded `u64`, and the
/// pattern helpers compute `Instant::now() + timeout` internally, which
/// panics on overflow. Halving until the sum is representable keeps a
/// bogus `UINT64_MAX` from a C++ caller behaving like "wait a very long
/// time" instead of aborting the process — the same defensive stance
/// `next_event_timeout` takes.
fn clamp_pattern_timeout(timeout_ms: u64) -> Duration {
    let mut timeout = Duration::from_millis(timeout_ms);
    let now = Instant::now();
    while !timeout.is_zero() && now.checked_add(timeout).is_none() {
        timeout /= 2;
    }
    timeout
}

fn pattern_result(outcome: Result<Event, PatternError>) -> ffi::DoraPatternResult {
    match outcome {
        Ok(event) => ffi::DoraPatternResult {
            status: ffi::DoraPatternStatus::Matched,
            error: String::new(),
            event: Box::new(DoraEvent(EventOrReason::Event(event))),
        },
        // Each failure also gets an `EventOrReason` whose `event_type`
        // matches the status, so C++ code that only inspects the event
        // (the pre-existing idiom) still sees something sensible.
        Err(err @ PatternError::Timeout) => pattern_failure(
            ffi::DoraPatternStatus::Timeout,
            err.to_string(),
            EventOrReason::TimedOut,
        ),
        Err(err @ PatternError::ServerRestarted(_)) => pattern_failure(
            ffi::DoraPatternStatus::ServerRestarted,
            err.to_string(),
            EventOrReason::Empty,
        ),
        Err(err @ PatternError::StreamEnded) => pattern_failure(
            ffi::DoraPatternStatus::StreamEnded,
            err.to_string(),
            EventOrReason::Closed,
        ),
        Err(err @ PatternError::StreamError(_)) => pattern_failure(
            ffi::DoraPatternStatus::StreamError,
            err.to_string(),
            EventOrReason::Empty,
        ),
    }
}

fn pattern_failure(
    status: ffi::DoraPatternStatus,
    error: String,
    event: EventOrReason,
) -> ffi::DoraPatternResult {
    ffi::DoraPatternResult {
        status,
        error,
        event: Box::new(DoraEvent(event)),
    }
}

pub struct MergedEvents {
    events: Option<Box<dyn Stream<Item = MergedEvent<ExternalEvent>> + Unpin>>,
    #[cfg(any(feature = "ros2-bridge", test))]
    next_id: u32,
}

fn new_metadata() -> Box<Metadata> {
    Box::new(Metadata::empty())
}
unsafe fn send_arrow_output(
    sender: &mut Box<OutputSender>,
    id: String,
    array_ptr: *mut u8,
    schema_ptr: *mut u8,
) -> ffi::DoraResult {
    unsafe { send_arrow_output_impl(sender, id, array_ptr, schema_ptr, None) }
}

unsafe fn send_arrow_output_with_metadata(
    sender: &mut Box<OutputSender>,
    id: String,
    array_ptr: *mut u8,
    schema_ptr: *mut u8,
    metadata: Box<Metadata>,
) -> ffi::DoraResult {
    unsafe { send_arrow_output_impl(sender, id, array_ptr, schema_ptr, Some(metadata)) }
}

unsafe fn send_arrow_output_impl(
    sender: &mut Box<OutputSender>,
    id: String,
    array_ptr: *mut u8,
    schema_ptr: *mut u8,
    metadata: Option<Box<Metadata>>,
) -> ffi::DoraResult {
    let array_ptr = array_ptr as *mut arrow::ffi::FFI_ArrowArray;
    let schema_ptr = schema_ptr as *mut arrow::ffi::FFI_ArrowSchema;

    if array_ptr.is_null() || schema_ptr.is_null() {
        return ffi::DoraResult {
            error: "Received null Arrow array or schema pointer".to_string(),
        };
    }

    let array = unsafe { std::ptr::read(array_ptr) };
    let schema = unsafe { std::ptr::read(schema_ptr) };

    unsafe {
        std::ptr::write(array_ptr, std::mem::zeroed());
        std::ptr::write(schema_ptr, std::mem::zeroed());
    }

    match unsafe { arrow::ffi::from_ffi(array, &schema) } {
        Ok(array_data) => {
            let arrow_array = arrow::array::make_array(array_data);
            let parameters: DoraMetadataParameters = metadata
                .as_ref()
                .map(|metadata| metadata.parameters.clone())
                .unwrap_or_default();
            let output_id = match parse_output_id(&id) {
                Ok(parsed) => parsed,
                Err(err) => return err,
            };
            let result = sender.0.send_output(output_id, parameters, arrow_array);
            match result {
                Ok(()) => ffi::DoraResult {
                    error: String::new(),
                },
                Err(err) => ffi::DoraResult {
                    error: format!("{err:?}"),
                },
            }
        }
        Err(e) => ffi::DoraResult {
            error: format!("Error importing array from C++: {e:?}"),
        },
    }
}

impl MergedEvents {
    #[cfg(any(feature = "ros2-bridge", test))]
    fn merge(&mut self, events: impl Stream<Item = Box<dyn Any>> + Unpin + 'static) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        let events = Box::pin(events.map(move |event| ExternalEvent { event, id }));

        let inner = self.events.take().unwrap();
        let merged: Box<dyn Stream<Item = _> + Unpin + 'static> =
            Box::new(inner.merge_external(events).map(|event| match event {
                MergedEvent::Dora(event) => MergedEvent::Dora(event),
                MergedEvent::External(event) => MergedEvent::External(event.flatten()),
            }));
        self.events = Some(merged);

        id
    }

    fn next(&mut self) -> MergedDoraEvent {
        let event = futures_lite::future::block_on(self.events.as_mut().unwrap().next());
        MergedDoraEvent(event)
    }
}

impl ffi::CombinedEvents {
    fn next(&mut self) -> ffi::CombinedEvent {
        ffi::CombinedEvent {
            event: Box::new(self.events.next()),
        }
    }
}

pub struct MergedDoraEvent(Option<MergedEvent<ExternalEvent>>);

pub struct ExternalEvent {
    pub event: Box<dyn Any>,
    pub id: u32,
}

impl ffi::CombinedEvent {
    fn is_dora(&self) -> bool {
        matches!(&self.event.0, Some(MergedEvent::Dora(_)))
    }
}

fn downcast_dora(event: ffi::CombinedEvent) -> eyre::Result<Box<DoraEvent>> {
    match event.event.0 {
        Some(MergedEvent::Dora(event)) => Ok(Box::new(DoraEvent(EventOrReason::Event(event)))),
        // None means the merged stream closed; pass that through as Closed
        // so C++ sees `DoraEventType::AllInputsClosed` rather than a
        // bail-out error.
        None => Ok(Box::new(DoraEvent(EventOrReason::Closed))),
        _ => eyre::bail!("not a dora event"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_node_api::{
        ArrowData,
        arrow::array::{ArrayRef, Int32Array},
        uhlc::HLC,
    };
    use std::sync::Arc;

    /// Regression test for #2030: a non-UInt8 input (e.g. Int32 from another
    /// node) must return an `Err` from `event_as_input` instead of aborting
    /// the process via `todo!()`.
    #[test]
    fn event_as_input_non_uint8_returns_err() {
        let array: ArrayRef = Arc::new(Int32Array::from(vec![1, 2, 3]));
        let event = Box::new(DoraEvent(EventOrReason::Event(Event::Input {
            id: "my_input".into(),
            metadata: DoraMetadata::new(HLC::default().new_timestamp()),
            data: ArrowData(array),
        })));

        let result = event_as_input(event);
        assert!(result.is_err(), "expected Err for non-UInt8 input, got Ok");
    }

    #[test]
    fn merged_events_assigns_ids_to_external_streams() {
        let mut events = MergedEvents {
            events: Some(Box::new(stream::empty())),
            #[cfg(any(feature = "ros2-bridge", test))]
            next_id: 1,
        };

        let first_id = events.merge(stream::once(Box::new("first") as Box<dyn Any>));
        let second_id = events.merge(stream::once(Box::new("second") as Box<dyn Any>));

        assert_eq!(first_id, 1);
        assert_eq!(second_id, 2);

        let mut seen = Vec::new();
        for _ in 0..2 {
            if let Some(MergedEvent::External(event)) = events.next().0 {
                seen.push(event.id);
            }
        }
        seen.sort_unstable();
        assert_eq!(seen, vec![1, 2]);
    }

    /// Regression test: a caller-supplied output id with invalid characters
    /// (e.g. a typo containing a space) must surface as a `DoraResult.error`
    /// rather than panicking across the `cxx::bridge` and aborting the node.
    /// The `send_output`/`send_arrow_output`/`close_outputs` paths all route
    /// through `parse_output_id`.
    #[test]
    fn parse_output_id_rejects_invalid_without_panicking() {
        let ok = parse_output_id("cmd_vel");
        assert!(ok.is_ok(), "a valid id must parse");

        for bad in ["cmd vel", "cmd!", "señal"] {
            let err = parse_output_id(bad).expect_err("invalid id must return an error");
            assert!(
                err.error.contains("invalid output id"),
                "error must describe the failure, got {:?}",
                err.error
            );
            assert!(!err.error.is_empty());
        }
    }

    // ---- #2686: Service / Action parity for the C++ binding ----

    /// The server side of a Service exchange reads `request_id` off the
    /// incoming request through this accessor, so it must surface both
    /// the payload bytes and the metadata.
    #[test]
    fn event_as_input_with_metadata_exposes_payload_and_correlation() {
        let mut metadata = DoraMetadata::new(HLC::default().new_timestamp());
        metadata.parameters.insert(
            dora_node_api::REQUEST_ID.to_string(),
            DoraParameter::String("req-1".into()),
        );
        let array: ArrayRef = Arc::new(UInt8Array::from(vec![7u8, 9u8]));

        let input =
            event_as_input_with_metadata(Box::new(DoraEvent(EventOrReason::Event(Event::Input {
                id: "request".into(),
                metadata,
                data: ArrowData(array),
            }))))
            .expect("input event should decode");

        assert_eq!(input.id, "request");
        assert_eq!(input.data, vec![7u8, 9u8]);
        assert_eq!(input.metadata.request_id().unwrap(), "req-1");
    }

    /// It must reject non-input events and non-byte payloads the same
    /// way `event_as_input` does, rather than aborting (cf. #2030).
    #[test]
    fn event_as_input_with_metadata_rejects_bad_input() {
        let not_input = event_as_input_with_metadata(Box::new(DoraEvent(EventOrReason::Empty)));
        assert!(not_input.is_err(), "a non-input event must return Err");

        let array: ArrayRef = Arc::new(Int32Array::from(vec![1, 2, 3]));
        let wrong_type =
            event_as_input_with_metadata(Box::new(DoraEvent(EventOrReason::Event(Event::Input {
                id: "input".into(),
                metadata: DoraMetadata::new(HLC::default().new_timestamp()),
                data: ArrowData(array),
            }))));
        assert!(wrong_type.is_err(), "a non-UInt8 payload must return Err");
    }

    /// The typed correlation setters must write the *exact* reserved keys
    /// the framework matches on. If these drift from
    /// `dora_message::metadata`, a C++ node's requests would look
    /// well-formed but never correlate.
    #[test]
    fn correlation_setters_use_the_reserved_keys() {
        let mut meta = Metadata::empty();
        meta.set_request_id("req-1".into()).unwrap();
        meta.set_goal_id("goal-1".into()).unwrap();
        meta.set_goal_status("succeeded".into()).unwrap();

        assert_eq!(meta.get_str(dora_node_api::REQUEST_ID).unwrap(), "req-1");
        assert_eq!(meta.get_str(dora_node_api::GOAL_ID).unwrap(), "goal-1");
        assert_eq!(
            meta.get_str(dora_node_api::GOAL_STATUS).unwrap(),
            "succeeded"
        );

        assert_eq!(meta.request_id().unwrap(), "req-1");
        assert_eq!(meta.goal_id().unwrap(), "goal-1");
        assert_eq!(meta.goal_status().unwrap(), "succeeded");
    }

    #[test]
    fn correlation_getters_error_when_unset() {
        let meta = Metadata::empty();
        assert!(meta.request_id().is_err());
        assert!(meta.goal_id().is_err());
        assert!(meta.goal_status().is_err());
    }

    /// The exported terminal statuses must equal the framework
    /// constants; `recv_action_result` only treats these three as
    /// terminal, so a typo here would make waits hang until timeout.
    #[test]
    fn exported_goal_statuses_match_framework_constants() {
        assert_eq!(
            goal_status_succeeded(),
            dora_node_api::GOAL_STATUS_SUCCEEDED
        );
        assert_eq!(goal_status_aborted(), dora_node_api::GOAL_STATUS_ABORTED);
        assert_eq!(goal_status_canceled(), dora_node_api::GOAL_STATUS_CANCELED);
    }

    #[test]
    fn new_request_id_is_a_unique_uuid() {
        let first = new_request_id();
        let second = new_request_id();
        dora_node_api::uuid::Uuid::parse_str(&first).expect("request id should be a valid UUID");
        assert_ne!(first, second, "successive request ids must differ");
        // `new_goal_id` is documented as an alias, so it must draw from
        // the same generator rather than being a distinct id space.
        dora_node_api::uuid::Uuid::parse_str(&new_goal_id()).expect("goal id should be a UUID");
    }

    /// A caller-supplied `request_id` must be replaced, matching
    /// `DoraNode::send_service_request`. Silently honouring it would let
    /// two in-flight requests share a correlation key.
    #[test]
    // ---- dora-rs/dora#3046 ----

    /// The whole point of the `_with_id` variant: the caller's id must
    /// survive, or a fan-out cannot share one correlation.
    #[test]
    fn send_service_request_with_id_preserves_the_caller_id() {
        let mut parameters = DoraMetadataParameters::default();
        parameters.insert(
            dora_node_api::REQUEST_ID.to_string(),
            DoraParameter::String("stale".into()),
        );

        set_request_id(&mut parameters, "caller-supplied");

        assert_eq!(
            parameters.get(dora_node_api::REQUEST_ID),
            Some(&DoraParameter::String("caller-supplied".into())),
            "the caller's id must replace whatever the metadata carried"
        );
    }

    /// The two id paths must stay distinguishable: one mints, one obeys.
    #[test]
    fn set_request_id_and_insert_request_id_differ() {
        let mut minted = DoraMetadataParameters::default();
        let generated = insert_request_id(&mut minted);

        let mut pinned = DoraMetadataParameters::default();
        set_request_id(&mut pinned, "req-fixed");

        assert_ne!(generated, "req-fixed");
        assert_eq!(
            pinned.get(dora_node_api::REQUEST_ID),
            Some(&DoraParameter::String("req-fixed".into()))
        );
    }

    #[test]
    fn zero_timeout_means_no_deadline() {
        // `0` has to keep meaning "poll forever", or a caller that
        // never wanted a deadline would start getting Timeout.
        assert!(poll_timeout(0).is_none());
    }

    #[test]
    fn nonzero_timeout_is_clamped_like_the_blocking_waits() {
        let clamped = poll_timeout(u64::MAX).expect("a non-zero timeout is a deadline");
        assert_eq!(clamped, clamp_pattern_timeout(u64::MAX));

        let ordinary = poll_timeout(5_000).expect("a non-zero timeout is a deadline");
        assert_eq!(ordinary, Duration::from_millis(5_000));
    }

    #[test]
    fn empty_server_list_means_any() {
        let Ok(parsed) = parse_server_list(&[]) else {
            panic!("an empty list is valid and means Any");
        };
        assert!(matches!(parsed, OwnedServers::Any));
    }

    #[test]
    fn single_empty_server_id_means_any() {
        // How the single-server polls spell "accept any responder".
        let Ok(parsed) = parse_server_list(&[""]) else {
            panic!("an empty id is the 'any' spelling and must be accepted");
        };
        assert!(matches!(parsed, OwnedServers::Any));
    }

    #[test]
    fn server_list_parses_every_entry() {
        let Ok(parsed) = parse_server_list(&["cam-eo", "cam-ir"]) else {
            panic!("both ids are valid");
        };
        match parsed {
            OwnedServers::Some(ids) => {
                assert_eq!(ids.len(), 2);
                assert_eq!(ids[0].as_ref(), "cam-eo");
                assert_eq!(ids[1].as_ref(), "cam-ir");
            }
            OwnedServers::Any => panic!("a non-empty list must not widen to Any"),
        }
    }

    #[test]
    fn empty_entry_inside_a_list_is_rejected() {
        // Silently treating this as "any" would let one stray "" switch
        // off restart detection for the whole set.
        let result = parse_server_list(&["cam-eo", ""]);
        match result {
            Err(failure) => assert!(matches!(
                failure.status,
                ffi::DoraPatternStatus::InvalidArgument
            )),
            Ok(_) => panic!("an empty entry in a real list must be rejected"),
        }
    }

    #[test]
    fn invalid_server_id_in_a_list_is_reported_not_panicked() {
        let result = parse_server_list(&["ok", "bad id/with slash"]);
        match result {
            Err(failure) => {
                assert!(matches!(
                    failure.status,
                    ffi::DoraPatternStatus::InvalidArgument
                ));
                assert!(failure.error.contains("bad id/with slash"));
            }
            Ok(_) => panic!("a malformed node id must be rejected"),
        }
    }

    #[test]
    fn not_ready_is_distinct_from_timeout_and_carries_no_error() {
        let not_ready = try_pattern_result(Ok(None));
        assert!(matches!(not_ready.status, ffi::DoraPatternStatus::NotReady));
        assert!(
            not_ready.error.is_empty(),
            "NotReady is not a failure, so it must not carry an error string: {}",
            not_ready.error
        );
        assert!(matches!(
            event_type(&not_ready.event),
            ffi::DoraEventType::Empty
        ));
    }

    #[test]
    fn try_pattern_result_passes_errors_through_unchanged() {
        let timeout = try_pattern_result(Err(PatternError::Timeout));
        assert!(matches!(timeout.status, ffi::DoraPatternStatus::Timeout));

        let restarted = try_pattern_result(Err(PatternError::ServerRestarted("cam-ir".into())));
        assert!(matches!(
            restarted.status,
            ffi::DoraPatternStatus::ServerRestarted
        ));
        assert!(
            restarted.error.contains("cam-ir"),
            "the restarted node's id must reach the C++ caller: {}",
            restarted.error
        );
    }

    #[test]
    fn insert_request_id_overwrites_caller_value() {
        let mut parameters = DoraMetadataParameters::default();
        parameters.insert(
            dora_node_api::REQUEST_ID.to_string(),
            DoraParameter::String("caller-supplied".into()),
        );

        let generated = insert_request_id(&mut parameters);

        assert_ne!(generated, "caller-supplied");
        assert_eq!(
            parameters.get(dora_node_api::REQUEST_ID),
            Some(&DoraParameter::String(generated))
        );
    }

    /// A failed send must not hand back a request id: a caller that
    /// skipped the `error` check would otherwise block in
    /// `recv_service_response` on a correlation never put on the wire.
    #[test]
    fn finish_request_clears_id_on_send_failure() {
        let ok = finish_request(
            "req-1".into(),
            ffi::DoraResult {
                error: String::new(),
            },
        );
        assert_eq!(ok.request_id, "req-1");
        assert!(ok.error.is_empty());

        let failed = finish_request(
            "req-1".into(),
            ffi::DoraResult {
                error: "send failed".into(),
            },
        );
        assert!(
            failed.request_id.is_empty(),
            "a failed send must not return a usable request id"
        );
        assert_eq!(failed.error, "send failed");
    }

    /// `timeout_ms` is an unbounded `u64` at the FFI boundary while the
    /// pattern helpers compute `Instant::now() + timeout` internally.
    /// A bogus `UINT64_MAX` from C++ must clamp, not panic.
    #[test]
    fn clamp_pattern_timeout_survives_extreme_values() {
        assert_eq!(clamp_pattern_timeout(0), Duration::ZERO);
        assert_eq!(clamp_pattern_timeout(5_000), Duration::from_millis(5_000));

        let clamped = clamp_pattern_timeout(u64::MAX);
        assert!(
            !clamped.is_zero(),
            "an overflowing timeout must clamp to a long wait, not to zero"
        );
        Instant::now()
            .checked_add(clamped)
            .expect("clamped timeout must be representable as a deadline");
    }

    /// An invalid node id must surface as `InvalidArgument` rather than
    /// panicking across the bridge (`NodeId::from(String)` panics), and
    /// it must be rejected *before* any waiting happens.
    #[test]
    fn invalid_server_node_id_is_reported_not_panicked() {
        let Err(err) = pattern_wait_args("bad id/with slash", 1_000) else {
            panic!("malformed node id must be rejected");
        };

        assert!(matches!(
            err.status,
            ffi::DoraPatternStatus::InvalidArgument
        ));
        assert!(err.error.contains("invalid server node id"));
        assert!(matches!(event_type(&err.event), ffi::DoraEventType::Empty));

        // `DoraPatternResult` (the error type here) is a cxx shared
        // struct and so has no `Debug`; destructure instead of unwrapping.
        let Ok((server, timeout)) = pattern_wait_args("service-server", 1_000) else {
            panic!("a valid node id must parse");
        };
        assert_eq!(server.to_string(), "service-server");
        assert_eq!(timeout, Duration::from_millis(1_000));
    }

    /// Every `PatternError` must map to a distinct status *and* to an
    /// event whose `event_type` tells the same story, so C++ code can
    /// branch on either field.
    #[test]
    fn pattern_errors_map_to_matching_kind_and_event_type() {
        let timeout = pattern_result(Err(PatternError::Timeout));
        assert!(matches!(timeout.status, ffi::DoraPatternStatus::Timeout));
        assert!(matches!(
            event_type(&timeout.event),
            ffi::DoraEventType::Timeout
        ));

        let restarted = pattern_result(Err(PatternError::ServerRestarted("server".into())));
        assert!(matches!(
            restarted.status,
            ffi::DoraPatternStatus::ServerRestarted
        ));
        assert!(
            restarted.error.contains("server"),
            "the restarted server's id must reach the C++ caller: {}",
            restarted.error
        );

        let ended = pattern_result(Err(PatternError::StreamEnded));
        assert!(matches!(ended.status, ffi::DoraPatternStatus::StreamEnded));
        assert!(matches!(
            event_type(&ended.event),
            ffi::DoraEventType::AllInputsClosed
        ));

        let stream_error = pattern_result(Err(PatternError::StreamError("boom".into())));
        assert!(matches!(
            stream_error.status,
            ffi::DoraPatternStatus::StreamError
        ));
        assert!(stream_error.error.contains("boom"));
    }

    /// On success the correlated input must come through unchanged, with
    /// no error set.
    #[test]
    fn pattern_result_passes_the_matched_input_through() {
        let mut metadata = DoraMetadata::new(HLC::default().new_timestamp());
        metadata.parameters.insert(
            dora_node_api::REQUEST_ID.to_string(),
            DoraParameter::String("req-1".into()),
        );
        let array: ArrayRef = Arc::new(Int32Array::from(vec![7]));

        let result = pattern_result(Ok(Event::Input {
            id: "response".into(),
            metadata,
            data: ArrowData(array),
        }));

        assert!(matches!(result.status, ffi::DoraPatternStatus::Matched));
        assert!(result.error.is_empty());
        assert!(matches!(
            event_type(&result.event),
            ffi::DoraEventType::Input
        ));
    }

    /// Regression test: `set_timestamp` must accept and correctly represent
    /// pre-epoch (negative) nanosecond timestamps. The previous truncating
    /// `value % 1_000_000_000` produced a negative remainder that wrapped when
    /// cast to `u32`, making `from_timestamp` reject valid instants. The value
    /// must also round-trip through `get_timestamp`.
    #[test]
    fn set_timestamp_roundtrips_negative_values() {
        for value in [
            -1_i64,
            -500_000_000,
            -1_000_000_000,
            -1_500_000_000,
            -1_000_000_001,
            0,
            1,
            1_500_000_000,
        ] {
            let mut meta = Metadata::empty();
            meta.set_timestamp("ts", value)
                .unwrap_or_else(|e| panic!("set_timestamp({value}) failed: {e}"));
            let got = meta
                .get_timestamp("ts")
                .unwrap_or_else(|e| panic!("get_timestamp after set({value}) failed: {e}"));
            assert_eq!(got, value, "timestamp {value} did not round-trip");
        }
    }
}
