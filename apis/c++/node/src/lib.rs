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

        unsafe fn send_arrow_output(
            output_sender: &mut Box<OutputSender>,
            id: String,
            array_ptr: *mut u8,  
            schema_ptr: *mut u8, 
        ) -> DoraResult;
        
        unsafe fn event_as_arrow_input(
            event: Box<DoraEvent>, 
            out_array: *mut u8,  
            out_schema: *mut u8,
        ) -> DoraResult;
    }
}

mod arrow_ffi {
    pub use arrow::ffi::{FFI_ArrowArray, FFI_ArrowSchema};
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

fn event_as_input(event: Box<DoraEvent>) -> eyre::Result<ffi::DoraInput> {
    let Some(Event::Input { id, metadata, data }) = event.0 else {
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

unsafe fn event_as_arrow_input(
    event: Box<DoraEvent>, 
    out_array: *mut u8,
    out_schema: *mut u8,
) -> ffi::DoraResult {
    // Cast to Arrow FFI types
    let out_array = out_array as *mut arrow::ffi::FFI_ArrowArray;
    let out_schema = out_schema as *mut arrow::ffi::FFI_ArrowSchema;

    let Some(Event::Input { id: _, metadata: _, data }) = event.0 else {
        return ffi::DoraResult { error: "Not an input event".to_string() };
    };
    
    if out_array.is_null() || out_schema.is_null() {
        return ffi::DoraResult { 
            error: "Received null output pointer".to_string() 
        };
    }
    
    let array_data = data.to_data();
    
    match arrow::ffi::to_ffi(&array_data.clone()) {
        Ok((ffi_array, ffi_schema)) => {
            std::ptr::write(out_array, ffi_array);
            std::ptr::write(out_schema, ffi_schema);
            ffi::DoraResult { error: String::new() }
        },
        Err(e) => {
            ffi::DoraResult {
                error: format!("Error exporting Arrow array to C++: {:?}", e)
            }
        }
    }
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
unsafe fn send_arrow_output(
    sender: &mut Box<OutputSender>,
    id: String,
    array_ptr: *mut u8, 
    schema_ptr: *mut u8
) -> ffi::DoraResult {
    let array_ptr = array_ptr as *mut arrow::ffi::FFI_ArrowArray;
    let schema_ptr = schema_ptr as *mut arrow::ffi::FFI_ArrowSchema;

    if array_ptr.is_null() || schema_ptr.is_null() {
        return ffi::DoraResult { 
            error: "Received null Arrow array or schema pointer".to_string()
        };
    }
    
    let array = std::ptr::read(array_ptr);
    let schema = std::ptr::read(schema_ptr);
    
    std::ptr::write(array_ptr, std::mem::zeroed());
    std::ptr::write(schema_ptr, std::mem::zeroed());
    
    match arrow::ffi::from_ffi(array, &schema) {
        Ok(array_data) => {
            let arrow_array = arrow::array::make_array(array_data);
            let result = sender.0.send_output(id.into(), Default::default(), arrow_array);
            match result {
                Ok(()) => ffi::DoraResult { error: String::new() },
                Err(err) => ffi::DoraResult { error: format!("{err:?}") },
            }
        },
        Err(e) => {
            ffi::DoraResult { 
                error: format!("Error importing array from C++: {:?}", e) 
            }
        }
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
        Some(MergedEvent::Dora(event)) => Ok(Box::new(DoraEvent(Some(event)))),
        _ => eyre::bail!("not an external event"),
    }
}
