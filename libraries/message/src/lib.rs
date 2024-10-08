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

fn current_crate_version() -> semver::Version {
    let crate_version_raw = env!("CARGO_PKG_VERSION");
    let crate_version = semver::Version::parse(crate_version_raw).unwrap();
    crate_version
}

// Current version compatibility will makes dora message accept
// all messages from the same major and minor version and ignore patch version.
fn versions_compatible(
    crate_version: &semver::Version,
    specified_version: &semver::Version,
) -> Result<bool, String> {
    let req = semver::VersionReq::parse(&format!(
        "~{}.{}.0",
        crate_version.major, crate_version.minor
    ))
    .unwrap();
    let matches = req.matches(&specified_version);
    Ok(matches)
}
