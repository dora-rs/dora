use std::any::Any;

use dora_node_api::{
    self,
    arrow::array::{AsArray, BinaryArray},
    merged::{MergeExternal, MergedEvent},
    Event, EventStream,
};
use eyre::bail;
use futures_lite::Stream;

#[cfg(feature = "ros2-bridge")]
use dora_ros2_bridge::_core;
#[cfg(feature = "ros2-bridge")]
pub use ros2::ExternalEvents;

#[cfg(feature = "ros2-bridge")]
pub mod ros2 {
    pub use dora_ros2_bridge::*;
    include!(env!("ROS2_BINDINGS_PATH"));
}

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
        #[cfg(feature = "ros2-bridge")]
        type ExternalEvents = crate::ros2::ExternalEvents;
    }

    extern "Rust" {
        type Events;
        // type ExternalEvents;
        type MergedEvents;
        type OutputSender;
        type DoraEvent;

        fn init_dora_node() -> Result<DoraNode>;

        fn next(self: &mut Events) -> Box<DoraEvent>;
        fn event_type(event: &Box<DoraEvent>) -> DoraEventType;
        fn event_as_input(event: Box<DoraEvent>) -> Result<DoraInput>;
        fn send_output(
            output_sender: &mut Box<OutputSender>,
            id: String,
            data: &[u8],
        ) -> DoraResult;

        #[cfg(feature = "ros2-bridge")]
        fn merge_events(dora: Box<Events>, external: Box<ExternalEvents>) -> Box<MergedEvents>;
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

#[cfg(feature = "ros2-bridge")]
#[allow(clippy::boxed_local)]
pub fn merge_events(
    dora_events: Box<Events>,
    external: Box<ros2::ExternalEvents>,
) -> Box<MergedEvents> {
    let merge_external = dora_events
        .0
        .merge_external(external.events.0.as_event_stream());
    Box::new(MergedEvents(merge_external))
}

pub struct MergedEvents(Box<dyn Stream<Item = MergedEvent<Box<dyn Any>>> + Unpin>);

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
