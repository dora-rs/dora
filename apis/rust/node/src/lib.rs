//! This crate enables you to create nodes for the [Dora] dataflow framework.
//!
//! [Dora]: https://dora-rs.ai/
//!
//! ## The Dora Framework
//!
//! Dora is a dataflow frame work that models applications as a directed graph, with nodes
//! representing operations and edges representing data transfer.
//! The layout of the dataflow graph is defined through a YAML file in Dora.
//! For details, see our [Dataflow Specification](https://dora-rs.ai/docs/api/dataflow-config/)
//! chapter.
//!
//! Dora nodes are typically spawned by the Dora framework, instead of spawning them manually.
//! If you want to spawn a node manually, define it as a [_dynamic_ node](#dynamic-nodes).
//!
//! ## Normal Usage
//!
//! In order to connect your executable to Dora, you need to initialize a [`DoraNode`].
//! For standard nodes, the recommended initialization function is [`init_from_env`][`DoraNode::init_from_env`].
//! This function will return two values, a [`DoraNode`] instance and an [`EventStream`]:
//!
//! ```no_run
//! use dora_node_api::DoraNode;
//!
//! let (mut node, mut events) = DoraNode::init_from_env()?;
//! # Ok::<(), eyre::Report>(())
//! ```
//!
//! You can use the `node` instance to send outputs and retrieve information about the node and
//! the dataflow. The `events` stream yields the inputs that the node defines in the dataflow
//! YAML file and other incoming events.
//!
//! ### Sending Outputs
//!
//! The [`DoraNode`] instance enables you to send outputs in different formats.
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
//! Note that Dora kills nodes that don't exit quickly after a [`Event::Stop`] of type
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
//! dataflow YAML file. Dynamic nodes are not spawned by the Dora framework and need to be started
//! manually.
//!
//! Dynamic nodes cannot use the [`DoraNode::init_from_env`] function for initialization.
//! Instead, they can be initialized through the [`DoraNode::init_from_node_id`] function.
//!
//! ### Limitations
//!
//! - Dynamic nodes **don't work with `dora run`**.
//! - As dynamic nodes are identified by their node ID, this **ID must be unique**
//!   across all running dataflows.
//! - For distributed dataflows, nodes need to be manually spawned on the correct machine.

#![warn(missing_docs)]

pub use arrow;
pub use dora_arrow_convert::*;
pub use dora_core::{self, uhlc};
pub use dora_message::{
    DataflowId,
    metadata::{Metadata, MetadataParameters, Parameter},
};
pub use event_stream::{Event, EventScheduler, EventStream, StopCause, merged};
pub use flume::Receiver;
pub use futures;
pub use node::{DataSample, DoraNode, ZERO_COPY_THRESHOLD, arrow_utils};

mod daemon_connection;
mod event_stream;
mod node;
