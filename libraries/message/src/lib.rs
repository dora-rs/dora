//! Enable serialisation and deserialisation of capnproto messages
//!

#![allow(clippy::missing_safety_doc)]

use std::fmt::format;

use eyre::Context;

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

fn current_crate_version() -> semver::Version {
    let crate_version_raw = env!("CARGO_PKG_VERSION");
    let crate_version = semver::Version::parse(crate_version_raw).unwrap();
    crate_version
}

fn versions_compatible(
    crate_version: &semver::Version,
    specified_version: &semver::Version,
) -> Result<bool, String> {
    let current_version = semver::Version::parse(env!("CARGO_PKG_VERSION"))
        .map_err(|error| format!("failed to parse current version: {error}"))?;
    let req = semver::VersionReq::parse(&format!(
        "~{}.{}.0",
        current_version.major, current_version.minor
    ))
    .map_err(|error| format!("failed to set allowing patch version of messages: {error}"))?;
    let specified_dora_req = semver::VersionReq::parse(&specified_version.to_string())
        .map_err(|error| {
            format!(
                "failed to parse specified dora version `{specified_version}` as `VersionReq`: {error}",
            )
        })?;
    let matches = req.matches(&specified_version) || specified_dora_req.matches(crate_version);
    Ok(matches)
}
