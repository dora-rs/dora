//! The custom node API allow you to integrate `dora` into your application.
//! It allows you to retrieve input and send output in any fashion you want.                                                 
//!
//! Try it out with:
//!
//! ```bash
//! dora new node --kind node
//! ```
//!
//! You can also generate a dora rust project with
//!
//! ```bash
//! dora new project_xyz --kind dataflow
//! ```
//!
pub use arrow;
pub use dora_arrow_convert::*;
pub use dora_core::{self, uhlc};
pub use dora_message::{
    metadata::{Metadata, MetadataParameters, Parameter},
    DataflowId,
};
pub use event_stream::{merged, Event, EventStream, MappedInputData, RawData};
pub use flume::Receiver;
pub use node::{arrow_utils, DataSample, DoraNode, ZERO_COPY_THRESHOLD};

mod daemon_connection;
mod event_stream;
mod node;
