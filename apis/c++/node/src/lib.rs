use std::any::Any;

use dora_node_api::{
    self,
    arrow::array::{AsArray, BinaryArray},
    merged::MergedEvent,
    Event, EventStream,
};
use eyre::bail;

#[cfg(feature = "ros2-bridge")]
use dora_ros2_bridge::_core;

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

    extern "C++" {
        #[allow(dead_code)]
        type ExternalEvents = crate::ros2::ExternalEvents;
        #[allow(dead_code)]
        type Ros2Event = crate::ros2::Ros2Event;
    }

    extern "Rust" {
        type Events;
        type MergedEvents;
        type OutputSender;
        type DoraEvent;
        type MergedDoraEvent;

        fn init_dora_node() -> Result<DoraNode>;

        fn next(self: &mut Events) -> Box<DoraEvent>;
        fn event_type(event: &Box<DoraEvent>) -> DoraEventType;
        fn event_as_input(event: Box<DoraEvent>) -> Result<DoraInput>;
        fn send_output(
            output_sender: &mut Box<OutputSender>,
            id: String,
            data: &[u8],
        ) -> DoraResult;

        fn merge_events(dora: Box<Events>, external: Box<ExternalEvents>) -> Box<MergedEvents>;
        fn next(self: &mut MergedEvents) -> Box<MergedDoraEvent>;

        fn is_ros2(event: &Box<MergedDoraEvent>) -> bool;
        fn downcast_ros2(event: Box<MergedDoraEvent>) -> Result<Box<Ros2Event>>;
        fn is_dora(event: &Box<MergedDoraEvent>) -> bool;
        fn downcast_dora(event: Box<MergedDoraEvent>) -> Result<Box<DoraEvent>>;
    }
}

#[cfg(feature = "ros2-bridge")]
pub mod ros2 {
    pub use dora_ros2_bridge::*;
    include!(env!("ROS2_BINDINGS_PATH"));
}

/// Dummy placeholder.
#[cfg(not(feature = "ros2-bridge"))]
#[cxx::bridge]
#[allow(clippy::needless_lifetimes)]
mod ros2 {
    pub struct ExternalEvents {
        dummy: u8,
    }
    pub struct Ros2Event {
        dummy: u8,
    }
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
    let Some(Event::Input { id, metadata: _, data }) = event.0 else {
        bail!("not an input event");
    };
    let data: Option<&BinaryArray> = data.as_binary_opt();
    Ok(ffi::DoraInput {
        id: id.into(),
        data: data.map(|d| d.value(0).to_owned()).unwrap_or_default(),
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

#[cfg(feature = "ros2-bridge")]
#[allow(clippy::boxed_local)]
pub fn merge_events(
    dora_events: Box<Events>,
    external: Box<ros2::ExternalEvents>,
) -> Box<MergedEvents> {
    use dora_node_api::merged::MergeExternal;

    let merge_external = dora_events
        .0
        .merge_external(external.events.0.as_event_stream());
    Box::new(MergedEvents(Box::new(futures_lite::stream::block_on(
        merge_external,
    ))))
}

/// Dummy
#[cfg(not(feature = "ros2-bridge"))]
#[allow(clippy::boxed_local)]
pub fn merge_events(
    dora_events: Box<Events>,
    _external: Box<ros2::ExternalEvents>,
) -> Box<MergedEvents> {
    use dora_node_api::merged::MergeExternal;

    let merge_external = dora_events.0.merge_external(futures_lite::stream::empty());
    Box::new(MergedEvents(Box::new(futures_lite::stream::block_on(
        merge_external,
    ))))
}

pub struct MergedEvents(Box<dyn Iterator<Item = MergedEvent<Box<dyn Any>>> + Unpin>);

impl MergedEvents {
    fn next(&mut self) -> Box<MergedDoraEvent> {
        let event = self.0.next();
        Box::new(MergedDoraEvent(event))
    }
}

pub struct MergedDoraEvent(Option<MergedEvent<Box<dyn Any>>>);

fn is_ros2(event: &Box<MergedDoraEvent>) -> bool {
    match event.0 {
        Some(MergedEvent::External(_)) => true,
        _ => false,
    }
}

fn downcast_ros2(event: Box<MergedDoraEvent>) -> eyre::Result<Box<ros2::Ros2Event>> {
    match event.0 {
        Some(MergedEvent::External(event)) => Ok(Box::new(ros2::Ros2Event {
            event: Box::new(ros2::ExternalRos2Event(event)),
        })),
        _ => eyre::bail!("not an external event"),
    }
}

fn is_dora(event: &Box<MergedDoraEvent>) -> bool {
    match event.0 {
        Some(MergedEvent::Dora(_)) => true,
        _ => false,
    }
}

fn downcast_dora(event: Box<MergedDoraEvent>) -> eyre::Result<Box<DoraEvent>> {
    match event.0 {
        Some(MergedEvent::Dora(event)) => Ok(Box::new(DoraEvent(Some(event)))),
        _ => eyre::bail!("not an external event"),
    }
}
