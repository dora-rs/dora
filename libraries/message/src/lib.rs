//! Enable serialisation and deserialisation of capnproto messages
//!

#![allow(clippy::missing_safety_doc)]

pub mod common;
pub mod metadata;

pub mod coordinator_to_daemon;
pub mod daemon_to_coordinator;

pub mod daemon_to_daemon;

pub mod daemon_to_node;
pub mod node_to_daemon;

pub mod cli_to_coordinator;
pub mod coordinator_to_cli;

pub type DataflowId = uuid::Uuid;
