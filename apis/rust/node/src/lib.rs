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
//!
//!
//! ## Node Integration Testing
//!
//! Dora provides built-in support for integration testing of nodes. See the [integration_testing]
//! module for details.

#![warn(missing_docs)]

/// Apache Arrow 59, dora's current **internal** major.
///
/// Enabled by the `arrow-v59` feature, which pulls no extra dependency: this
/// is the same copy of Arrow 59 dora itself links, so
/// [`DoraArray::as_array`] hands it back for free and arrays built with it
/// need no conversion.
///
/// This is deliberately not `pub use arrow;`. A bare `arrow` re-export changes
/// meaning silently when dora bumps its internal major; `arrow_v59` either
/// exists and means Arrow 59, or it is visibly gone. See
/// `docs/plan-arrow-version-decoupling.md`.
#[cfg(feature = "arrow-v59")]
pub use arrow as arrow_v59;
/// Apache Arrow 58, for nodes that name that major.
///
/// Enabled by the `arrow-v58` feature. Arrow 58 is **not** dora's internal
/// major, so payloads cross a C Data Interface hop
/// ([`DoraArray::from_arrow_v58`] / [`DoraArray::to_arrow_v58`]). The hop does
/// not copy buffers.
#[cfg(feature = "arrow-v58")]
pub use arrow58 as arrow_v58;
// Deliberately *not* a glob re-export: `dora_arrow_convert::internal` is an
// Arrow-typed, semver-exempt seam for dora's own crates and must not become
// reachable through the frozen `dora-node-api` surface.
pub use dora_arrow_convert::{DoraArray, IntoArrow, into_vec};
pub use dora_core::{self, uhlc};
pub use dora_message::{
    DataflowId,
    metadata::{
        self, FIN, FLUSH, GOAL_ID, GOAL_STATUS, GOAL_STATUS_ABORTED, GOAL_STATUS_CANCELED,
        GOAL_STATUS_SUCCEEDED, Metadata, MetadataParameters, Parameter, REQUEST_ID, SEGMENT_ID,
        SEQ, SESSION_ID, get_bool_param, get_integer_param, get_string_param,
    },
};
use dora_message::{
    common::Timestamped,
    daemon_to_node::{DaemonCommunication, DaemonReply},
    node_to_daemon::DaemonRequest,
};
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
pub use node::{
    DataSample, DoraNode, DoraNodeBuilder, EncodedSample, SampleAllocator, StreamSegment,
    ZERO_COPY_THRESHOLD, arrow_utils,
};
pub use uuid;

pub use serde_json;
use tokio::sync::oneshot;

mod daemon_connection;
mod error;
/// Asynchronous event stream and daemon communication.
pub mod event_stream;
pub mod integration_testing;
mod node;
mod orphan_guard;

pub use error::{NodeError, NodeResult, PatternError};

/// Backward-compatible alias for [`Event`], so code using the old `DoraEvent`
/// name still compiles.
pub use Event as DoraEvent;

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
