//! Enable serialisation and deserialisation of capnproto messages
//!

#![allow(clippy::missing_safety_doc)]
#![warn(missing_docs)]

/// The version of the dora-message crate
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

pub use tarpc;
pub use uhlc;

/// Common message types and utilities
pub mod common;
/// Configuration structures for dataflow nodes and operators
pub mod config;
/// Message descriptor definitions for dataflow configuration
pub mod descriptor;
/// Node, operator, and data identifiers
pub mod id;
/// Message metadata structures
pub mod metadata;

/// Protocol messages from coordinator to daemon
pub mod coordinator_to_daemon;
/// Protocol messages from daemon to coordinator
pub mod daemon_to_coordinator;

/// Protocol messages between daemons
pub mod daemon_to_daemon;

/// Protocol messages from daemon to node
pub mod daemon_to_node;
/// Protocol messages from node to daemon
pub mod node_to_daemon;

/// Protocol messages from CLI to coordinator
pub mod cli_to_coordinator;
/// Protocol messages from coordinator to CLI
pub mod coordinator_to_cli;

pub mod integration_testing_format;

pub use arrow_data;
pub use arrow_schema;
use uuid::{Timestamp, Uuid};

/// Unique identifier for a dataflow instance.
///
/// Dora assigns each dataflow instance a unique ID on start.
pub type DataflowId = uuid::Uuid;

#[derive(
    Debug, Clone, Copy, serde::Serialize, serde::Deserialize, PartialEq, Eq, PartialOrd, Ord, Hash,
)]
pub struct SessionId(uuid::Uuid);

impl SessionId {
    pub fn generate() -> Self {
        Self(Uuid::new_v7(Timestamp::now(uuid::NoContext)))
    }

    pub fn uuid(&self) -> uuid::Uuid {
        self.0
    }
}

#[derive(
    Debug, Clone, Copy, serde::Serialize, serde::Deserialize, PartialEq, Eq, PartialOrd, Ord, Hash,
)]
pub struct BuildId(uuid::Uuid);

impl BuildId {
    pub fn generate() -> Self {
        Self(Uuid::new_v7(Timestamp::now(uuid::NoContext)))
    }
}

impl std::fmt::Display for BuildId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "BuildId({})", self.0)
    }
}

/// Check whether a remote version string is compatible with this crate's version.
pub fn check_version_compatibility(remote_version: &str) -> eyre::Result<()> {
    let crate_version = current_crate_version();
    let specified_version = semver::Version::parse(remote_version)
        .map_err(|e| eyre::eyre!("failed to parse remote version `{remote_version}`: {e}"))?;
    let compatible =
        versions_compatible(&crate_version, &specified_version).map_err(|e| eyre::eyre!(e))?;
    if compatible {
        Ok(())
    } else {
        Err(eyre::eyre!(
            "version mismatch: remote message format v{specified_version} is not compatible \
            with local message format v{crate_version}"
        ))
    }
}

pub(crate) fn current_crate_version() -> semver::Version {
    let crate_version_raw = env!("CARGO_PKG_VERSION");

    semver::Version::parse(crate_version_raw).unwrap()
}

pub(crate) fn versions_compatible(
    crate_version: &semver::Version,
    specified_version: &semver::Version,
) -> Result<bool, String> {
    let req = semver::VersionReq::parse(&crate_version.to_string()).map_err(|error| {
        format!("failed to parse crate version `{crate_version}` as `VersionReq`: {error}")
    })?;
    let specified_dora_req = semver::VersionReq::parse(&specified_version.to_string())
        .map_err(|error| {
            format!(
                "failed to parse specified dora version `{specified_version}` as `VersionReq`: {error}",
            )
        })?;
    let matches = req.matches(specified_version) || specified_dora_req.matches(crate_version);
    Ok(matches)
}
