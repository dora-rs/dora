//! Enable serialisation and deserialisation of capnproto messages
//!

#![allow(clippy::missing_safety_doc)]

pub use uhlc;

pub mod common;
pub mod config;
pub mod descriptor;
pub mod id;
pub mod metadata;

pub mod coordinator_to_daemon;
pub mod daemon_to_coordinator;

pub mod daemon_to_daemon;

pub mod daemon_to_node;
pub mod node_to_daemon;

pub mod cli_to_coordinator;
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

fn current_crate_version() -> semver::Version {
    let crate_version_raw = env!("CARGO_PKG_VERSION");

    semver::Version::parse(crate_version_raw).unwrap()
}

fn versions_compatible(
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
