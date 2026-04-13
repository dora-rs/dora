use std::{any::Any, collections::BTreeMap, time::Duration, vec};

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
        Timeout,
        NodeFailed,
        Reload,
    }

    struct DoraInput {
        id: String,
        data: Vec<u8>,
    }

    struct DoraResult {
        error: String,
    }

    struct DoraNodeFailed {
        affected_input_ids: Vec<String>,
        error: String,
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
        type MergedEvents;
        type MergedDoraEvent;
        type Metadata;
        type DataSampleHandle;
        type DrainedEvents;

        fn init_dora_node() -> Result<DoraNode>;
        fn init_dora_node_from_id(node_id: String) -> Result<DoraNode>;
        fn init_dora_node_flexible(node_id: String) -> Result<DoraNode>;

        fn node_id(output_sender: &Box<OutputSender>) -> String;
        fn dataflow_id(output_sender: &Box<OutputSender>) -> String;

        fn dora_events_into_combined(events: Box<Events>) -> CombinedEvents;
        fn empty_combined_events() -> CombinedEvents;
        fn next(self: &mut Events) -> Box<DoraEvent>;
        fn next_event(events: &mut Box<Events>) -> Box<DoraEvent>;
        fn next_event_timeout(events: &mut Box<Events>, timeout_ms: u64) -> Box<DoraEvent>;
        fn try_next_event(events: &mut Box<Events>) -> Box<DoraEvent>;
        fn events_is_empty(events: &Box<Events>) -> bool;
        fn event_type(event: &Box<DoraEvent>) -> DoraEventType;

        fn drain_events(events: &mut Box<Events>) -> Box<DrainedEvents>;
        fn drained_events_len(drained: &Box<DrainedEvents>) -> usize;
        fn drained_events_next(drained: &mut Box<DrainedEvents>) -> Box<DoraEvent>;
        fn event_as_input(event: Box<DoraEvent>) -> Result<DoraInput>;
        fn event_as_node_failed(event: Box<DoraEvent>) -> Result<DoraNodeFailed>;
        fn close_outputs(
            output_sender: &mut Box<OutputSender>,
            output_ids: Vec<String>,
        ) -> DoraResult;
        fn send_output(
            output_sender: &mut Box<OutputSender>,
            id: String,
            data: &[u8],
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

        // Zero-copy output API
        fn allocate_data_sample(
            output_sender: &mut Box<OutputSender>,
            len: usize,
        ) -> Result<Box<DataSampleHandle>>;
        unsafe fn data_sample_as_ptr(handle: &mut Box<DataSampleHandle>) -> *mut u8;
        fn data_sample_len(handle: &Box<DataSampleHandle>) -> usize;
        fn send_data_sample(
            output_sender: &mut Box<OutputSender>,
            id: String,
            sample: Box<DataSampleHandle>,
        ) -> DoraResult;
        fn send_data_sample_with_metadata(
            output_sender: &mut Box<OutputSender>,
            id: String,
            sample: Box<DataSampleHandle>,
            metadata: Box<Metadata>,
        ) -> DoraResult;
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

fn init_dora_node_from_id(node_id: String) -> eyre::Result<ffi::DoraNode> {
    let (node, events) = dora_node_api::DoraNode::init_from_node_id(
        dora_node_api::dora_core::config::NodeId::from(node_id),
    )?;
    let events = Events(events);
    let send_output = OutputSender(node);

    Ok(ffi::DoraNode {
        events: Box::new(events),
        send_output: Box::new(send_output),
    })
}

fn init_dora_node_flexible(node_id: String) -> eyre::Result<ffi::DoraNode> {
    let (node, events) = dora_node_api::DoraNode::init_flexible(
        dora_node_api::dora_core::config::NodeId::from(node_id),
    )?;
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
        Box::new(DoraEvent {
            event: self.0.recv(),
            timed_out: false,
        })
    }
}

fn next_event(events: &mut Box<Events>) -> Box<DoraEvent> {
    events.next()
}

fn next_event_timeout(events: &mut Box<Events>, timeout_ms: u64) -> Box<DoraEvent> {
    let dur = Duration::from_millis(timeout_ms);
    match events.0.recv_timeout(dur) {
        Some(event) => {
            let timed_out = matches!(&event, Event::Error(msg) if msg.contains("timed out"));
            if timed_out {
                Box::new(DoraEvent {
                    event: None,
                    timed_out: true,
                })
            } else {
                Box::new(DoraEvent {
                    event: Some(event),
                    timed_out: false,
                })
            }
        }
        None => Box::new(DoraEvent {
            event: None,
            timed_out: false,
        }),
    }
}

fn try_next_event(events: &mut Box<Events>) -> Box<DoraEvent> {
    match events.0.try_recv() {
        Ok(event) => Box::new(DoraEvent {
            event: Some(event),
            timed_out: false,
        }),
        Err(TryRecvError::Empty) => Box::new(DoraEvent {
            event: None,
            timed_out: true,
        }),
        Err(TryRecvError::Closed) => Box::new(DoraEvent {
            event: None,
            timed_out: false,
        }),
    }
}

fn events_is_empty(events: &Box<Events>) -> bool {
    events.0.is_empty()
}

pub struct DrainedEvents(std::collections::VecDeque<Event>);

fn drain_events(events: &mut Box<Events>) -> Box<DrainedEvents> {
    let evts = events.0.drain().unwrap_or_default();
    Box::new(DrainedEvents(evts.into()))
}

fn drained_events_len(drained: &Box<DrainedEvents>) -> usize {
    drained.0.len()
}

fn drained_events_next(drained: &mut Box<DrainedEvents>) -> Box<DoraEvent> {
    match drained.0.pop_front() {
        Some(e) => Box::new(DoraEvent {
            event: Some(e),
            timed_out: false,
        }),
        None => Box::new(DoraEvent {
            event: None,
            timed_out: true,
        }),
    }
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

pub struct DoraEvent {
    event: Option<Event>,
    timed_out: bool,
}

fn event_type(event: &DoraEvent) -> ffi::DoraEventType {
    if event.timed_out {
        return ffi::DoraEventType::Timeout;
    }
    match &event.event {
        Some(event) => match event {
            Event::Stop(_) => ffi::DoraEventType::Stop,
            Event::Input { .. } => ffi::DoraEventType::Input,
            Event::InputClosed { .. } => ffi::DoraEventType::InputClosed,
            Event::Error(_) => ffi::DoraEventType::Error,
            Event::NodeFailed { .. } => ffi::DoraEventType::NodeFailed,
            Event::Reload { .. } => ffi::DoraEventType::Reload,
            _ => ffi::DoraEventType::Unknown,
        },
        None => ffi::DoraEventType::AllInputsClosed,
    }
}

fn event_as_input(event: Box<DoraEvent>) -> eyre::Result<ffi::DoraInput> {
    let Some(Event::Input { id, metadata, data }) = event.event else {
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
        _ => {
            todo!("dora C++ Node does not yet support higher level type of arrow. Only UInt8.
                The ultimate solution should be based on arrow FFI interface. Feel free to contribute :)")
        }
    };

    Ok(ffi::DoraInput {
        id: id.into(),
        data,
    })
}

fn event_as_node_failed(event: Box<DoraEvent>) -> eyre::Result<ffi::DoraNodeFailed> {
    let Some(Event::NodeFailed {
        affected_input_ids,
        error,
        source_node_id,
    }) = event.event
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

fn close_outputs(sender: &mut Box<OutputSender>, output_ids: Vec<String>) -> ffi::DoraResult {
    let ids: Vec<dora_node_api::dora_core::config::DataId> =
        output_ids.into_iter().map(|s| s.into()).collect();
    match sender.0.close_outputs(ids) {
        Ok(()) => ffi::DoraResult {
            error: String::new(),
        },
        Err(err) => ffi::DoraResult {
            error: format!("{err:?}"),
        },
    }
}

unsafe fn event_as_arrow_input(
    event: Box<DoraEvent>,
    out_array: *mut u8,
    out_schema: *mut u8,
) -> ffi::DoraResult {
    // Cast to Arrow FFI types
    let out_array = out_array as *mut arrow::ffi::FFI_ArrowArray;
    let out_schema = out_schema as *mut arrow::ffi::FFI_ArrowSchema;

    let Some(Event::Input {
        id: _,
        metadata: _,
        data,
    }) = event.event
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

    let Some(Event::Input { id, metadata, data }) = event.event else {
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

fn node_id(output_sender: &Box<OutputSender>) -> String {
    output_sender.0.id().to_string()
}

fn dataflow_id(output_sender: &Box<OutputSender>) -> String {
    output_sender.0.dataflow_id().to_string()
}

fn send_output(sender: &mut Box<OutputSender>, id: String, data: &[u8]) -> ffi::DoraResult {
    send_output_internal(sender, id, data, Default::default())
}

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

/// Opaque handle to a pre-allocated data sample.
///
/// The sample may be backed by shared memory (for buffers >= 4096 bytes) or
/// a regular aligned allocation (for smaller buffers). Use
/// [`data_sample_as_ptr`] to obtain a raw pointer for writing, then pass the
/// handle to [`send_data_sample`] to send it.
///
/// **Limitation:** The current zero-copy path sends data as a flat byte array
/// (`UInt8` Arrow type). It does not support structured Arrow data types.
/// For typed Arrow output, use [`send_arrow_output`] instead (which copies
/// the data into the sample internally).
pub struct DataSampleHandle(dora_node_api::DataSample);

/// Allocate a data sample of `len` bytes.
///
/// Uses shared memory when `len >= 4096` to enable zero-copy transfer to
/// subscribers. The returned handle owns the allocation; write into it via
/// [`data_sample_as_ptr`] and send it via [`send_data_sample`].
///
/// **Note:** The sample is typed as a flat `UInt8` byte array. For
/// structured Arrow types, use [`send_arrow_output`] instead.
fn allocate_data_sample(
    sender: &mut Box<OutputSender>,
    len: usize,
) -> eyre::Result<Box<DataSampleHandle>> {
    let sample = sender.0.allocate_data_sample(len)?;
    Ok(Box::new(DataSampleHandle(sample)))
}

/// Returns a mutable raw pointer to the sample's underlying buffer.
///
/// # Safety
///
/// - The pointer is valid only as long as the `DataSampleHandle` is alive
///   and has not been moved (i.e., not yet passed to [`send_data_sample`]).
/// - The caller must not write beyond `data_sample_len(handle)` bytes.
/// - After calling [`send_data_sample`], the handle is consumed and any
///   previously obtained pointer is **invalid** — do not dereference it.
unsafe fn data_sample_as_ptr(handle: &mut Box<DataSampleHandle>) -> *mut u8 {
    handle.0.as_mut_ptr()
}

/// Returns the size of the sample buffer in bytes.
fn data_sample_len(handle: &Box<DataSampleHandle>) -> usize {
    handle.0.len()
}

/// Send a pre-filled data sample as output (zero-copy for shared-memory
/// backed samples).
///
/// Consumes the `DataSampleHandle`. Any raw pointer previously obtained
/// from [`data_sample_as_ptr`] becomes invalid after this call.
///
/// The data is sent as a flat `UInt8` byte array.
fn send_data_sample(
    sender: &mut Box<OutputSender>,
    id: String,
    sample: Box<DataSampleHandle>,
) -> ffi::DoraResult {
    send_data_sample_internal(sender, id, sample, Default::default())
}

fn send_data_sample_with_metadata(
    sender: &mut Box<OutputSender>,
    id: String,
    sample: Box<DataSampleHandle>,
    metadata: Box<Metadata>,
) -> ffi::DoraResult {
    let metadata = *metadata;
    let parameters = metadata.into_parameters();
    send_data_sample_internal(sender, id, sample, parameters)
}

fn send_data_sample_internal(
    sender: &mut Box<OutputSender>,
    id: String,
    sample: Box<DataSampleHandle>,
    metadata: DoraMetadataParameters,
) -> ffi::DoraResult {
    use dora_node_api::dora_core::metadata::ArrowTypeInfoExt;
    let data_len = sample.0.len();
    let type_info = dora_message::metadata::ArrowTypeInfo::byte_array(data_len);
    let result = sender
        .0
        .send_output_sample(id.into(), type_info, metadata, Some(sample.0));
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
        let merged_stream = inner.merge_external(events).map(|event| match event {
            MergedEvent::Dora(event) => MergedEvent::Dora(event),
            MergedEvent::External(event) => MergedEvent::External(event.flatten()),
        });
        let merged: Box<dyn Stream<Item = _> + Unpin + 'static> = Box::new(merged_stream);
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
        Some(MergedEvent::Dora(event)) => Ok(Box::new(DoraEvent {
            event: Some(event),
            timed_out: false,
        })),
        _ => eyre::bail!("not an external event"),
    }
}
