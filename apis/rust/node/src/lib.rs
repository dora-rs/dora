//! This crate enables you to create nodes for the [Adora] dataflow framework.
//!
//! [Adora]: https://adora-rs.ai/
//!
//! ## The Adora Framework
//!
//! Adora is a dataflow frame work that models applications as a directed graph, with nodes
//! representing operations and edges representing data transfer.
//! The layout of the dataflow graph is defined through a YAML file in Adora.
//! For details, see our [Dataflow Specification](https://adora-rs.ai/docs/api/dataflow-config/)
//! chapter.
//!
//! Adora nodes are typically spawned by the Adora framework, instead of spawning them manually.
//! If you want to spawn a node manually, define it as a [_dynamic_ node](#dynamic-nodes).
//!
//! ## Normal Usage
//!
//! In order to connect your executable to Adora, you need to initialize a [`AdoraNode`].
//! For standard nodes, the recommended initialization function is [`init_from_env`][`AdoraNode::init_from_env`].
//! This function will return two values, a [`AdoraNode`] instance and an [`EventStream`]:
//!
//! ```no_run
//! use adora_node_api::AdoraNode;
//!
//! let (mut node, mut events) = AdoraNode::init_from_env()?;
//! # Ok::<(), eyre::Report>(())
//! ```
//!
//! You can use the `node` instance to send outputs and retrieve information about the node and
//! the dataflow. The `events` stream yields the inputs that the node defines in the dataflow
//! YAML file and other incoming events.
//!
//! ### Sending Outputs
//!
//! The [`AdoraNode`] instance enables you to send outputs in different formats.
//! For best performance, use the [Arrow](https://arrow.apache.org/docs/index.html) data format
//! and one of the output functions that utilizes shared memory.
//!
//! ### Receiving Events
//!
//! The [`EventStream`] is an [`AsyncIterator`][std::async_iter::AsyncIterator] that yields the incoming [`Event`]s.
//!
//! Nodes should iterate over this event stream and react to events that they are interested in.
//! Typically, the most important event type is [`Event::Input`].
//! You don't need to handle all events, it's fine to ignore events that are not relevant to your node.
//!
//! The event stream will close itself after a [`Event::Stop`] was received.
//! A manual `break` on [`Event::Stop`] is typically not needed.
//! _(You probably do need to use a manual `break` on stop events when using the
//! [`StreamExt::merge`][`futures_concurrency::stream::StreamExt::merge`] implementation on
//! [`EventStream`] to combine the stream with an external one.)_
//!
//! Once the event stream finished, nodes should exit.
//! Note that Adora kills nodes that don't exit quickly after a [`Event::Stop`] of type
//! [`StopCause::Manual`] was received.
//!
//!
//!
//! ## Dynamic Nodes
//!
//! <div class="warning">
//!
//! Dynamic nodes have certain [limitations](#limitations). Use with care.
//!
//! </div>
//!
//! Nodes can be defined as `dynamic` by setting their `path` attribute to `dynamic` in the
//! dataflow YAML file. Dynamic nodes are not spawned by the Adora framework and need to be started
//! manually.
//!
//! Dynamic nodes cannot use the [`AdoraNode::init_from_env`] function for initialization.
//! Instead, they can be initialized through the [`AdoraNode::init_from_node_id`] function.
//!
//! ### Limitations
//!
//! - Dynamic nodes **don't work with `adora run`**.
//! - As dynamic nodes are identified by their node ID, this **ID must be unique**
//!   across all running dataflows.
//! - For distributed dataflows, nodes need to be manually spawned on the correct machine.
//!
//!
//! ## Node Integration Testing
//!
//! Adora provides built-in support for integration testing of nodes. See the [integration_testing]
//! module for details.

#![warn(missing_docs)]

pub use adora_arrow_convert::*;
pub use adora_core::{self, uhlc};
pub use adora_message::{
    DataflowId,
    metadata::{
        self, GOAL_ID, GOAL_STATUS, GOAL_STATUS_ABORTED, GOAL_STATUS_CANCELED,
        GOAL_STATUS_SUCCEEDED, Metadata, MetadataParameters, Parameter, REQUEST_ID,
    },
};
use adora_message::{
    common::Timestamped,
    daemon_to_node::{DaemonCommunication, DaemonReply},
    node_to_daemon::DaemonRequest,
};
pub use arrow;
pub use event_stream::{
    Event, EventScheduler, EventStream, StopCause, TryRecvError,
    input_tracker::{InputState, InputTracker},
    merged,
};
pub use flume;
pub use flume::Receiver;
pub use futures;
#[cfg(feature = "tracing")]
pub use node::init_tracing;
pub use node::{AdoraNode, DataSample, ZERO_COPY_THRESHOLD, arrow_utils};
pub use uuid;

pub use serde_json;
use tokio::sync::oneshot;

mod daemon_connection;
mod error;
mod event_stream;
pub mod integration_testing;
mod node;

pub use error::{NodeError, NodeResult};

#[derive(Debug)]
enum DaemonCommunicationWrapper {
    Standard(DaemonCommunication),
    Testing {
        channel:
            tokio::sync::mpsc::Sender<(Timestamped<DaemonRequest>, oneshot::Sender<DaemonReply>)>,
    },
}

impl From<DaemonCommunication> for DaemonCommunicationWrapper {
    fn from(value: DaemonCommunication) -> Self {
        DaemonCommunicationWrapper::Standard(value)
    }
}
