use std::{
    any::Any,
    collections::{BTreeMap, VecDeque},
    time::{Duration, Instant},
    vec,
};

use crate::ffi::MetadataValueType;

use chrono::DateTime;
use dora_node_api::{
    self, Event, EventStream, Metadata as DoraMetadata,
    MetadataParameters as DoraMetadataParameters, Parameter as DoraParameter, TryRecvError,
    arrow::array::{AsArray, UInt8Array},
    merged::{MergeExternal, MergedEvent},
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

    struct DoraResult {
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
            next_id: 1,
        }),
    }
}

fn empty_combined_events() -> ffi::CombinedEvents {
    ffi::CombinedEvents {
        events: Box::new(MergedEvents {
            events: Some(Box::new(stream::empty())),
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

fn event_as_input(event: Box<DoraEvent>) -> eyre::Result<ffi::DoraInput> {
    let EventOrReason::Event(Event::Input { id, metadata, data }) = event.0 else {
        bail!("not an input event");
    };
    let data = match metadata.type_info.data_type {
        dora_node_api::arrow::datatypes::DataType::UInt8 => {
            let array: &UInt8Array = data.as_primitive();
            array.values().to_vec()
        }
        dora_node_api::arrow::datatypes::DataType::Null => {
            vec![]
        }
        other => {
            bail!(
                "unsupported input arrow type {other:?}; \
                 use event_as_arrow_input for typed access"
            );
        }
    };

    Ok(ffi::DoraInput {
        id: id.into(),
        data,
    })
}

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

fn close_outputs(
    output_sender: &mut Box<OutputSender>,
    output_ids: Vec<String>,
) -> ffi::DoraResult {
    // Parse via `FromStr` instead of `From<String>` so invalid IDs
    // surface as a `DoraResult.error` rather than panicking across
    // the cxx::bridge. `DataId::from(String)` is documented as
    // panicking on invalid characters (see
    // `libraries/message/src/id.rs:186`, `# Panics`); calling it on
    // caller-supplied input would mean a typo in a C++ string literal
    // aborts the whole node.
    let mut ids = Vec::with_capacity(output_ids.len());
    for id in output_ids {
        match id.parse::<dora_node_api::dora_core::config::DataId>() {
            Ok(parsed) => ids.push(parsed),
            Err(e) => {
                return ffi::DoraResult {
                    error: format!("invalid output id '{id}': {e}"),
                };
            }
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

    match arrow::ffi::to_ffi(&array_data.clone()) {
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
        // Convert nanoseconds since Unix epoch to chrono::DateTime<Utc>
        let secs = value / 1_000_000_000;
        let subsec_nanos = (value % 1_000_000_000) as u32;

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

    fn into_parameters(self) -> DoraMetadataParameters {
        self.parameters
    }

    fn insert_parameter(&mut self, key: &str, parameter: DoraParameter) -> EyreResult<()> {
        self.parameters.insert(key.to_string(), parameter);
        Ok(())
    }
}

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
    let result = sender
        .0
        .send_output_raw(id.into(), metadata, data.len(), |out| {
            out.copy_from_slice(data)
        });
    let error = match result {
        Ok(()) => String::new(),
        Err(err) => format!("{err:?}"),
    };
    ffi::DoraResult { error }
}

pub struct MergedEvents {
    events: Option<Box<dyn Stream<Item = MergedEvent<ExternalEvent>> + Unpin>>,
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
            let result = sender.0.send_output(id.into(), parameters, arrow_array);
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
    fn next(&mut self) -> MergedDoraEvent {
        let event = futures_lite::future::block_on(self.events.as_mut().unwrap().next());
        MergedDoraEvent(event)
    }

    pub fn merge(&mut self, events: impl Stream<Item = Box<dyn Any>> + Unpin + 'static) -> u32 {
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
        arrow::{
            array::{ArrayRef, Int32Array},
            datatypes::DataType,
        },
        metadata::ArrowTypeInfo,
        uhlc::HLC,
    };
    use std::sync::Arc;

    fn type_info(data_type: DataType) -> ArrowTypeInfo {
        ArrowTypeInfo {
            data_type,
            len: 0,
            null_count: 0,
            validity: None,
            offset: 0,
            buffer_offsets: vec![],
            child_data: vec![],
            field_names: None,
            schema_hash: None,
        }
    }

    /// Regression test for #2030: a non-UInt8 input (e.g. Int32 from another
    /// node) must return an `Err` from `event_as_input` instead of aborting
    /// the process via `todo!()`.
    #[test]
    fn event_as_input_non_uint8_returns_err() {
        let array: ArrayRef = Arc::new(Int32Array::from(vec![1, 2, 3]));
        let event = Box::new(DoraEvent(EventOrReason::Event(Event::Input {
            id: "my_input".into(),
            metadata: DoraMetadata::new(HLC::default().new_timestamp(), type_info(DataType::Int32)),
            data: ArrowData(array),
        })));

        let result = event_as_input(event);
        assert!(result.is_err(), "expected Err for non-UInt8 input, got Ok");
    }
}
