use std::{any::Any, collections::BTreeMap, vec};

use crate::ffi::MetadataValueType;

use adora_node_api::{
    self, Event, EventStream, Metadata as AdoraMetadata,
    MetadataParameters as AdoraMetadataParameters, Parameter as AdoraParameter,
    arrow::array::{AsArray, UInt8Array},
    merged::{MergeExternal, MergedEvent},
};
use chrono::DateTime;
use eyre::{Result as EyreResult, bail, eyre};
use serde::Serialize;
use serde_json::Value as JsonValue;

#[cfg(feature = "ros2-bridge")]
pub use prelude::*;
#[cfg(feature = "ros2-bridge")]
pub mod prelude {
    pub use adora_ros2_bridge::prelude::*;
}
use futures_lite::{Stream, StreamExt, stream};

#[cxx::bridge]
#[allow(clippy::needless_lifetimes)]
mod ffi {
    struct AdoraNode {
        events: Box<Events>,
        send_output: Box<OutputSender>,
    }

    pub enum AdoraEventType {
        Stop,
        Input,
        InputClosed,
        Error,
        Unknown,
        AllInputsClosed,
    }

    struct AdoraInput {
        id: String,
        data: Vec<u8>,
    }

    struct AdoraResult {
        error: String,
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
        event: Box<MergedAdoraEvent>,
    }

    extern "Rust" {
        type Events;
        type OutputSender;
        type AdoraEvent;
        type MergedEvents;
        type MergedAdoraEvent;
        type Metadata;

        fn init_adora_node() -> Result<AdoraNode>;

        fn adora_events_into_combined(events: Box<Events>) -> CombinedEvents;
        fn empty_combined_events() -> CombinedEvents;
        fn next(self: &mut Events) -> Box<AdoraEvent>;
        fn next_event(events: &mut Box<Events>) -> Box<AdoraEvent>;
        fn event_type(event: &Box<AdoraEvent>) -> AdoraEventType;
        fn event_as_input(event: Box<AdoraEvent>) -> Result<AdoraInput>;
        fn send_output(
            output_sender: &mut Box<OutputSender>,
            id: String,
            data: &[u8],
        ) -> AdoraResult;
        fn log_message(
            output_sender: &Box<OutputSender>,
            level: String,
            message: String,
        ) -> AdoraResult;
        fn send_output_with_metadata(
            output_sender: &mut Box<OutputSender>,
            id: String,
            data: &[u8],
            metadata: Box<Metadata>,
        ) -> AdoraResult;

        fn next(self: &mut CombinedEvents) -> CombinedEvent;

        fn is_adora(self: &CombinedEvent) -> bool;
        fn downcast_adora(event: CombinedEvent) -> Result<Box<AdoraEvent>>;

        unsafe fn send_arrow_output(
            output_sender: &mut Box<OutputSender>,
            id: String,
            array_ptr: *mut u8,
            schema_ptr: *mut u8,
        ) -> AdoraResult;

        #[cxx_name = "send_arrow_output"]
        unsafe fn send_arrow_output_with_metadata(
            output_sender: &mut Box<OutputSender>,
            id: String,
            array_ptr: *mut u8,
            schema_ptr: *mut u8,
            metadata: Box<Metadata>,
        ) -> AdoraResult;

        unsafe fn event_as_arrow_input(
            event: Box<AdoraEvent>,
            out_array: *mut u8,
            out_schema: *mut u8,
        ) -> AdoraResult;

        unsafe fn event_as_arrow_input_with_info(
            event: Box<AdoraEvent>,
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
    // pub use adora_ros2_bridge::*;
    include!(env!("ROS2_BINDINGS_PATH"));
}

fn init_adora_node() -> eyre::Result<ffi::AdoraNode> {
    let (node, events) = adora_node_api::AdoraNode::init_from_env()?;
    let events = Events(events);
    let send_output = OutputSender(node);

    Ok(ffi::AdoraNode {
        events: Box::new(events),
        send_output: Box::new(send_output),
    })
}

pub struct Events(EventStream);

impl Events {
    fn next(&mut self) -> Box<AdoraEvent> {
        Box::new(AdoraEvent(self.0.recv()))
    }
}

fn next_event(events: &mut Box<Events>) -> Box<AdoraEvent> {
    events.next()
}

fn adora_events_into_combined(events: Box<Events>) -> ffi::CombinedEvents {
    let events = events.0.map(MergedEvent::Adora);
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

pub struct AdoraEvent(Option<Event>);

fn event_type(event: &AdoraEvent) -> ffi::AdoraEventType {
    match &event.0 {
        Some(event) => match event {
            Event::Stop(_) => ffi::AdoraEventType::Stop,
            Event::Input { .. } => ffi::AdoraEventType::Input,
            Event::InputClosed { .. } => ffi::AdoraEventType::InputClosed,
            Event::Error(_) => ffi::AdoraEventType::Error,
            _ => ffi::AdoraEventType::Unknown,
        },
        None => ffi::AdoraEventType::AllInputsClosed,
    }
}

fn event_as_input(event: Box<AdoraEvent>) -> eyre::Result<ffi::AdoraInput> {
    let Some(Event::Input { id, metadata, data }) = event.0 else {
        bail!("not an input event");
    };
    let data = match metadata.type_info.data_type {
        adora_node_api::arrow::datatypes::DataType::UInt8 => {
            let array: &UInt8Array = data.as_primitive();
            array.values().to_vec()
        }
        adora_node_api::arrow::datatypes::DataType::Null => {
            vec![]
        }
        _ => {
            todo!("adora C++ Node does not yet support higher level type of arrow. Only UInt8.
                The ultimate solution should be based on arrow FFI interface. Feel free to contribute :)")
        }
    };

    Ok(ffi::AdoraInput {
        id: id.into(),
        data,
    })
}

unsafe fn event_as_arrow_input(
    event: Box<AdoraEvent>,
    out_array: *mut u8,
    out_schema: *mut u8,
) -> ffi::AdoraResult {
    // Cast to Arrow FFI types
    let out_array = out_array as *mut arrow::ffi::FFI_ArrowArray;
    let out_schema = out_schema as *mut arrow::ffi::FFI_ArrowSchema;

    let Some(Event::Input {
        id: _,
        metadata: _,
        data,
    }) = event.0
    else {
        return ffi::AdoraResult {
            error: "Not an input event".to_string(),
        };
    };

    if out_array.is_null() || out_schema.is_null() {
        return ffi::AdoraResult {
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
            ffi::AdoraResult {
                error: String::new(),
            }
        }
        Err(e) => ffi::AdoraResult {
            error: format!("Error exporting Arrow array to C++: {e:?}"),
        },
    }
}

pub struct Metadata {
    timestamp: u64,
    parameters: BTreeMap<String, AdoraParameter>,
}

impl Metadata {
    fn from_adora(metadata: AdoraMetadata) -> EyreResult<Self> {
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

    fn parameter_type_name(parameter: &AdoraParameter) -> &'static str {
        match parameter {
            AdoraParameter::Bool(_) => "bool",
            AdoraParameter::Integer(_) => "integer",
            AdoraParameter::String(_) => "string",
            AdoraParameter::Float(_) => "float",
            AdoraParameter::ListInt(_) => "list<int>",
            AdoraParameter::ListFloat(_) => "list<float>",
            AdoraParameter::ListString(_) => "list<string>",
            AdoraParameter::Timestamp(_) => "timestamp",
        }
    }

    fn expect_parameter<'a>(&'a self, key: &str) -> EyreResult<&'a AdoraParameter> {
        self.parameters
            .get(key)
            .ok_or_else(|| eyre!("metadata missing key '{key}'"))
    }

    fn parameter_to_json(parameter: &AdoraParameter, _key: &str) -> EyreResult<JsonValue> {
        match parameter {
            AdoraParameter::Bool(value) => Ok(JsonValue::Bool(*value)),
            AdoraParameter::Integer(value) => Ok(JsonValue::from(*value)),
            AdoraParameter::Float(value) => Ok(JsonValue::from(*value)),
            AdoraParameter::String(value) => Ok(JsonValue::String(value.clone())),
            AdoraParameter::ListInt(values) => Ok(JsonValue::Array(
                values.iter().map(|value| JsonValue::from(*value)).collect(),
            )),
            AdoraParameter::ListFloat(values) => Ok(JsonValue::Array(
                values.iter().map(|value| JsonValue::from(*value)).collect(),
            )),
            AdoraParameter::ListString(values) => Ok(JsonValue::Array(
                values
                    .iter()
                    .map(|value| JsonValue::String(value.clone()))
                    .collect(),
            )),
            AdoraParameter::Timestamp(dt) => {
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
            AdoraParameter::Bool(value) => Ok(*value),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'bool'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_float(&self, key: &str) -> EyreResult<f64> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            AdoraParameter::Float(value) => Ok(*value),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'float'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_int(&self, key: &str) -> EyreResult<i64> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            AdoraParameter::Integer(value) => Ok(*value),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'integer'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_str(&self, key: &str) -> EyreResult<String> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            AdoraParameter::String(value) => Ok(value.clone()),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'string'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_list_int(&self, key: &str) -> EyreResult<Vec<i64>> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            AdoraParameter::ListInt(values) => Ok(values.clone()),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'list<int>'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_list_float(&self, key: &str) -> EyreResult<Vec<f64>> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            AdoraParameter::ListFloat(values) => Ok(values.clone()),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'list<float>'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_list_string(&self, key: &str) -> EyreResult<Vec<String>> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            AdoraParameter::ListString(values) => Ok(values.clone()),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'list<string>'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_timestamp(&self, key: &str) -> EyreResult<i64> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            AdoraParameter::Timestamp(dt) => {
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
            parameters: &'a BTreeMap<String, AdoraParameter>,
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
        self.insert_parameter(key, AdoraParameter::Bool(value))
    }

    pub fn set_int(&mut self, key: &str, value: i64) -> EyreResult<()> {
        self.insert_parameter(key, AdoraParameter::Integer(value))
    }

    pub fn set_float(&mut self, key: &str, value: f64) -> EyreResult<()> {
        self.insert_parameter(key, AdoraParameter::Float(value))
    }

    pub fn set_string(&mut self, key: &str, value: String) -> EyreResult<()> {
        self.insert_parameter(key, AdoraParameter::String(value))
    }

    pub fn set_list_int(&mut self, key: &str, value: Vec<i64>) -> EyreResult<()> {
        self.insert_parameter(key, AdoraParameter::ListInt(value))
    }

    pub fn set_list_float(&mut self, key: &str, value: Vec<f64>) -> EyreResult<()> {
        self.insert_parameter(key, AdoraParameter::ListFloat(value))
    }

    pub fn set_list_string(&mut self, key: &str, value: Vec<String>) -> EyreResult<()> {
        self.insert_parameter(key, AdoraParameter::ListString(value))
    }

    pub fn set_timestamp(&mut self, key: &str, value: i64) -> EyreResult<()> {
        // Convert nanoseconds since Unix epoch to chrono::DateTime<Utc>
        let secs = value / 1_000_000_000;
        let subsec_nanos = (value % 1_000_000_000) as u32;

        let dt = DateTime::from_timestamp(secs, subsec_nanos)
            .ok_or_else(|| eyre!("Invalid timestamp: out of range (nanos: {value})"))?;

        self.insert_parameter(key, AdoraParameter::Timestamp(dt))
    }

    pub fn value_type(&self, key: &str) -> EyreResult<MetadataValueType> {
        let parameter = self.expect_parameter(key)?;
        let value_type = match parameter {
            AdoraParameter::Bool(_) => MetadataValueType::Bool,
            AdoraParameter::Integer(_) => MetadataValueType::Integer,
            AdoraParameter::Float(_) => MetadataValueType::Float,
            AdoraParameter::String(_) => MetadataValueType::String,
            AdoraParameter::ListInt(_) => MetadataValueType::ListInt,
            AdoraParameter::ListFloat(_) => MetadataValueType::ListFloat,
            AdoraParameter::ListString(_) => MetadataValueType::ListString,
            AdoraParameter::Timestamp(_) => MetadataValueType::Timestamp,
        };
        Ok(value_type)
    }

    fn into_parameters(self) -> AdoraMetadataParameters {
        self.parameters
    }

    fn insert_parameter(&mut self, key: &str, parameter: AdoraParameter) -> EyreResult<()> {
        self.parameters.insert(key.to_string(), parameter);
        Ok(())
    }
}

unsafe fn event_as_arrow_input_with_info(
    event: Box<AdoraEvent>,
    out_array: *mut u8,
    out_schema: *mut u8,
) -> ffi::ArrowInputInfo {
    // Cast to Arrow FFI types
    let out_array = out_array as *mut arrow::ffi::FFI_ArrowArray;
    let out_schema = out_schema as *mut arrow::ffi::FFI_ArrowSchema;

    let Some(Event::Input { id, metadata, data }) = event.0 else {
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

    let prepared_metadata = match Metadata::from_adora(metadata) {
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

pub struct OutputSender(adora_node_api::AdoraNode);

fn send_output(sender: &mut Box<OutputSender>, id: String, data: &[u8]) -> ffi::AdoraResult {
    send_output_internal(sender, id, data, Default::default())
}

fn log_message(sender: &Box<OutputSender>, level: String, message: String) -> ffi::AdoraResult {
    sender.0.log(&level, &message, None);
    ffi::AdoraResult {
        error: String::new(),
    }
}

fn send_output_with_metadata(
    sender: &mut Box<OutputSender>,
    id: String,
    data: &[u8],
    metadata: Box<Metadata>,
) -> ffi::AdoraResult {
    let metadata = *metadata;
    let parameters = metadata.into_parameters();
    send_output_internal(sender, id, data, parameters)
}

fn send_output_internal(
    sender: &mut Box<OutputSender>,
    id: String,
    data: &[u8],
    metadata: AdoraMetadataParameters,
) -> ffi::AdoraResult {
    let result = sender
        .0
        .send_output_raw(id.into(), metadata, data.len(), |out| {
            out.copy_from_slice(data)
        });
    let error = match result {
        Ok(()) => String::new(),
        Err(err) => format!("{err:?}"),
    };
    ffi::AdoraResult { error }
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
) -> ffi::AdoraResult {
    unsafe { send_arrow_output_impl(sender, id, array_ptr, schema_ptr, None) }
}

unsafe fn send_arrow_output_with_metadata(
    sender: &mut Box<OutputSender>,
    id: String,
    array_ptr: *mut u8,
    schema_ptr: *mut u8,
    metadata: Box<Metadata>,
) -> ffi::AdoraResult {
    unsafe { send_arrow_output_impl(sender, id, array_ptr, schema_ptr, Some(metadata)) }
}

unsafe fn send_arrow_output_impl(
    sender: &mut Box<OutputSender>,
    id: String,
    array_ptr: *mut u8,
    schema_ptr: *mut u8,
    metadata: Option<Box<Metadata>>,
) -> ffi::AdoraResult {
    let array_ptr = array_ptr as *mut arrow::ffi::FFI_ArrowArray;
    let schema_ptr = schema_ptr as *mut arrow::ffi::FFI_ArrowSchema;

    if array_ptr.is_null() || schema_ptr.is_null() {
        return ffi::AdoraResult {
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
            let parameters: AdoraMetadataParameters = metadata
                .as_ref()
                .map(|metadata| metadata.parameters.clone())
                .unwrap_or_default();
            let result = sender.0.send_output(id.into(), parameters, arrow_array);
            match result {
                Ok(()) => ffi::AdoraResult {
                    error: String::new(),
                },
                Err(err) => ffi::AdoraResult {
                    error: format!("{err:?}"),
                },
            }
        }
        Err(e) => ffi::AdoraResult {
            error: format!("Error importing array from C++: {e:?}"),
        },
    }
}

impl MergedEvents {
    fn next(&mut self) -> MergedAdoraEvent {
        let event = futures_lite::future::block_on(self.events.as_mut().unwrap().next());
        MergedAdoraEvent(event)
    }

    pub fn merge(&mut self, events: impl Stream<Item = Box<dyn Any>> + Unpin + 'static) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        let events = Box::pin(events.map(move |event| ExternalEvent { event, id }));

        let inner = self.events.take().unwrap();
        let merged: Box<dyn Stream<Item = _> + Unpin + 'static> =
            Box::new(inner.merge_external(events).map(|event| match event {
                MergedEvent::Adora(event) => MergedEvent::Adora(event),
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

pub struct MergedAdoraEvent(Option<MergedEvent<ExternalEvent>>);

pub struct ExternalEvent {
    pub event: Box<dyn Any>,
    pub id: u32,
}

impl ffi::CombinedEvent {
    fn is_adora(&self) -> bool {
        matches!(&self.event.0, Some(MergedEvent::Adora(_)))
    }
}

fn downcast_adora(event: ffi::CombinedEvent) -> eyre::Result<Box<AdoraEvent>> {
    match event.event.0 {
        Some(MergedEvent::Adora(event)) => Ok(Box::new(AdoraEvent(Some(event)))),
        _ => eyre::bail!("not an external event"),
    }
}
