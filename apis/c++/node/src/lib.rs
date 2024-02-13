use std::any::Any;

use dora_node_api::{
    self,
    arrow::array::{AsArray, BinaryArray},
    merged::{MergeExternal, MergedEvent},
    Event, EventStream,
};
use eyre::bail;

#[cfg(feature = "ros2-bridge")]
use dora_ros2_bridge::_core;
use futures_lite::{Stream, StreamExt};

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
        fn next(self: &mut Events) -> Box<DoraEvent>;
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

fn dora_events_into_combined(events: Box<Events>) -> ffi::CombinedEvents {
    let events = events.0.map(MergedEvent::Dora);
    ffi::CombinedEvents {
        events: Box::new(MergedEvents {
            events: Some(Box::new(events)),
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
