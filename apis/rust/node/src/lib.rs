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
pub use dora_core;
pub use dora_core::message::{uhlc, Metadata, MetadataParameters};
pub use event_stream::{merged, Data, Event, EventStream, MappedInputData};
pub use flume::Receiver;
pub use node::{DataSample, DoraNode};

mod daemon_connection;
mod event_stream;
mod node;
