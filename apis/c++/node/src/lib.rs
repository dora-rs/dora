use std::{any::Any, vec};

use dora_node_api::{
    self,
    arrow::array::{AsArray, UInt8Array},
    merged::{MergeExternal, MergedEvent},
    Event, EventStream,
};
use eyre::bail;

#[cfg(feature = "ros2-bridge")]
use dora_ros2_bridge::{_core, ros2_client};
use futures_lite::{stream, Stream, StreamExt};

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
    }

    struct DoraInput {
        id: String,
        data: Vec<u8>,
    }

    struct DoraResult {
        error: String,
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

        fn init_dora_node() -> Result<DoraNode>;

        fn dora_events_into_combined(events: Box<Events>) -> CombinedEvents;
        fn empty_combined_events() -> CombinedEvents;
        fn next(self: &mut Events) -> Box<DoraEvent>;
        fn next_event(events: &mut Box<Events>) -> Box<DoraEvent>;
        fn event_type(event: &Box<DoraEvent>) -> DoraEventType;
        fn event_as_input(event: Box<DoraEvent>) -> Result<DoraInput>;
        fn send_output(
            output_sender: &mut Box<OutputSender>,
            id: String,
            data: &[u8],
        ) -> DoraResult;

        fn next(self: &mut CombinedEvents) -> CombinedEvent;

        fn is_dora(self: &CombinedEvent) -> bool;
        fn downcast_dora(event: CombinedEvent) -> Result<Box<DoraEvent>>;
    }
}

#[cfg(feature = "ros2-bridge")]
pub mod ros2 {
    pub use dora_ros2_bridge::*;
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
        Box::new(DoraEvent(self.0.recv()))
    }
}

fn next_event(events: &mut Box<Events>) -> Box<DoraEvent> {
    events.next()
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

pub struct DoraEvent(Option<Event>);

fn event_type(event: &DoraEvent) -> ffi::DoraEventType {
    match &event.0 {
        Some(event) => match event {
            Event::Stop => ffi::DoraEventType::Stop,
            Event::Input { .. } => ffi::DoraEventType::Input,
            Event::InputClosed { .. } => ffi::DoraEventType::InputClosed,
            Event::Error(_) => ffi::DoraEventType::Error,
            _ => ffi::DoraEventType::Unknown,
        },
        None => ffi::DoraEventType::AllInputsClosed,
    }
}

use dora_node_api::arrow::array::{
    Array, UInt8Array, UInt16Array, UInt32Array, Int32Array, Float32Array, Float64Array, StringArray, BooleanArray,
};
use dora_node_api::arrow::datatypes::DataType;
use eyre::{bail, eyre, Result};
use some_module::{DoraEvent, Event, ffi};  // यह मानकर चल रहे हैं कि ये दूसरे मॉड्यूल से आ रहे हैं

fn event_as_input(event: Box<DoraEvent>) -> Result<ffi::DoraInput> {
    let Some(Event::Input { id, metadata, data }) = event.0 else {
        bail!("Not an input event");
    };

    let extracted_data: Vec<u8> = match metadata.type_info.data_type {
        DataType::UInt8 => {
            let array = data.as_any().downcast_ref::<UInt8Array>()
                .ok_or_else(|| eyre!("Expected UInt8Array"))?;
            array.values().to_vec()
        }
        DataType::UInt16 => {
            let array = data.as_any().downcast_ref::<UInt16Array>()
                .ok_or_else(|| eyre!("Expected UInt16Array"))?;
            array.values().iter().flat_map(|&x| x.to_le_bytes()).collect()
        }
        DataType::UInt32 => {
            let array = data.as_any().downcast_ref::<UInt32Array>()
                .ok_or_else(|| eyre!("Expected UInt32Array"))?;
            array.values().iter().flat_map(|&x| x.to_le_bytes()).collect()
        }
        DataType::Int32 => {
            let array = data.as_any().downcast_ref::<Int32Array>()
                .ok_or_else(|| eyre!("Expected Int32Array"))?;
            array.values().iter().flat_map(|&x| x.to_le_bytes()).collect()
        }
        DataType::Float32 => {
            let array = data.as_any().downcast_ref::<Float32Array>()
                .ok_or_else(|| eyre!("Expected Float32Array"))?;
            array.values().iter().flat_map(|&x| x.to_bits().to_le_bytes()).collect()
        }
        DataType::Float64 => {
            let array = data.as_any().downcast_ref::<Float64Array>()
                .ok_or_else(|| eyre!("Expected Float64Array"))?;
            array.values().iter().flat_map(|&x| x.to_bits().to_le_bytes()).collect()
        }
        DataType::Utf8 => {
            let array = data.as_any().downcast_ref::<StringArray>()
                .ok_or_else(|| eyre!("Expected StringArray"))?;
            array.iter().filter_map(|s| s.map(|s| s.as_bytes().to_vec())).flatten().collect()
        }
        DataType::Boolean => {
            let array = data.as_any().downcast_ref::<BooleanArray>()
                .ok_or_else(|| eyre!("Expected BooleanArray"))?;
            array.iter().map(|x| x.unwrap_or(false) as u8).collect()
        }
        DataType::Null => vec![],
        _ => bail!("Unsupported Arrow data type: {:?}", metadata.type_info.data_type),
    };

    Ok(ffi::DoraInput {
        id: id.into(),
        data: extracted_data,
    })
}

pub struct OutputSender(dora_node_api::DoraNode);

fn send_output(sender: &mut Box<OutputSender>, id: String, data: &[u8]) -> ffi::DoraResult {
    let result = sender
        .0
        .send_output_raw(id.into(), Default::default(), data.len(), |out| {
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
        Some(MergedEvent::Dora(event)) => Ok(Box::new(DoraEvent(Some(event)))),
        _ => eyre::bail!("not an external event"),
    }
}
