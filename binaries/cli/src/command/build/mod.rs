//! Provides the `dora build` command.
//!
//! The `dora build` command works like this:
//!
//! - Dataflows can specify a `build` command for each node in their YAML definition
//! - Dora will run the `build` command when `dora build` is invoked
//! - If the dataflow is distributed across multiple machines, each `build` command will be run the target machine of the corresponding node.
//!     - i.e. the machine specified under the `deploy` key
//!     - this requires a connection to the dora coordinator, so you need to specify the coordinator IP/port for this
//!     - to run the build commands of all nodes _locally_, you can use `dora build --local`
//! - If the build command does not specify any `deploy` keys, all build commands will be run locally (i.e. `dora build` behaves like `dora build --local`)
//!
//! #### Git Source
//!
//! - Nodes can have a git repository as source
//!     - set the `git` config key to the URL of the repository
//!     - by default, the default branch is used
//!     - you can also specify a specific `branch` name
//!     - alternatively, you can specify a `tag` name or a `rev` key with the commit hash
//!     - you can only specify one of `branch`, `tag`, and `rev`, otherwise an error will occur
//! - Dora will automatically clone and checkout the requested branch/tag/commit on `dora build`
//!     - the `build` command will be run after cloning
//!     - for distributed dataflows, the clone/checkout will happen on the target machine
//! - subsequent `dora build` command will automatically fetch the latest changes for nodes
//!     - not when using `tag` or `rev`, because these are not expected to change
//! - after fetching changes, the `build` command will be executed again
//!     - _tip:_ use a build tool that supports incremental builds (e.g. `cargo`) to make this rebuild faster
//!
//! The **working directory** will be set to the git repository.
//! This means that both the `build` and `path` keys will be run from this folder.
//! This allows you to use relative paths.
//!
//! #### Example
//!
//! ```yml
//! nodes:
//!   - id: rust-node
//!     # URL of your repository
//!     git: https://github.com/dora-rs/dora.git
//!     # the build command that should be invoked after cloning
//!     build: cargo build -p rust-dataflow-example-node
//!     # path to the executable that should be run on start
//!     path: target/debug/rust-dataflow-example-node
//!     inputs:
//!       tick: dora/timer/millis/10
//!     outputs:
//!       - random
//! ```

use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_core::{
    descriptor::{CoreNodeKind, CustomNode, Descriptor, DescriptorExt},
    topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST},
};
use dora_message::{BuildId, descriptor::NodeSource};
use eyre::Context;
use std::{collections::BTreeMap, net::IpAddr};

use super::{Executable, default_tracing};
use crate::{
    common::{connect_to_coordinator, local_working_dir, resolve_dataflow},
    session::DataflowSession,
};

use distributed::{build_distributed_dataflow, wait_until_dataflow_built};
use local::build_dataflow_locally;

mod distributed;
mod git;
mod local;

#[derive(Debug, clap::Args)]
/// Run build commands provided in the given dataflow.
pub struct Build {
    /// Path to the dataflow descriptor file
    #[clap(value_name = "PATH")]
    dataflow: String,
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP")]
    coordinator_addr: Option<IpAddr>,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT")]
    coordinator_port: Option<u16>,
    // Use UV to build nodes.
    #[clap(long, action)]
    uv: bool,
    // Run build on local machine
    #[clap(long, action)]
    local: bool,
}

impl Executable for Build {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        build(
            self.dataflow,
            self.coordinator_addr,
            self.coordinator_port,
            self.uv,
            self.local,
        )
    }
}

pub fn build(
    dataflow: String,
    coordinator_addr: Option<IpAddr>,
    coordinator_port: Option<u16>,
    uv: bool,
    force_local: bool,
) -> eyre::Result<()> {
    let dataflow_path = resolve_dataflow(dataflow).context("could not resolve dataflow")?;
    let dataflow_descriptor =
        Descriptor::blocking_read(&dataflow_path).wrap_err("Failed to read yaml dataflow")?;
    let mut dataflow_session =
        DataflowSession::read_session(&dataflow_path).context("failed to read DataflowSession")?;

    let mut git_sources = BTreeMap::new();
    let resolved_nodes = dataflow_descriptor
        .resolve_aliases_and_set_defaults()
        .context("failed to resolve nodes")?;
    for (node_id, node) in resolved_nodes {
        if let CoreNodeKind::Custom(CustomNode {
            source: NodeSource::GitBranch { repo, rev },
            ..
        }) = node.kind
        {
            let source = git::fetch_commit_hash(repo, rev)
                .with_context(|| format!("failed to find commit hash for `{node_id}`"))?;
            git_sources.insert(node_id, source);
        }
    }

    let session = || connect_to_coordinator_with_defaults(coordinator_addr, coordinator_port);

    let build_kind = if force_local {
        log::info!("Building locally, as requested through `--force-local`");
        BuildKind::Local
    } else if dataflow_descriptor.nodes.iter().all(|n| n.deploy.is_none()) {
        log::info!("Building locally because dataflow does not contain any `deploy` sections");
        BuildKind::Local
    } else if coordinator_addr.is_some() || coordinator_port.is_some() {
        log::info!("Building through coordinator, using the given coordinator socket information");
        // explicit coordinator address or port set -> there should be a coordinator running
        BuildKind::ThroughCoordinator {
            coordinator_session: session().context("failed to connect to coordinator")?,
        }
    } else {
        match session() {
            Ok(coordinator_session) => {
                // we found a local coordinator instance at default port -> use it for building
                log::info!("Found local dora coordinator instance -> building through coordinator");
                BuildKind::ThroughCoordinator {
                    coordinator_session,
                }
            }
            Err(_) => {
                log::warn!("No dora coordinator instance found -> trying a local build");
                // no coordinator instance found -> do a local build
                BuildKind::Local
            }
        }
    };

    match build_kind {
        BuildKind::Local => {
            log::info!("running local build");
            // use dataflow dir as base working dir
            let local_working_dir = dunce::canonicalize(&dataflow_path)
                .context("failed to canonicalize dataflow path")?
                .parent()
                .ok_or_else(|| eyre::eyre!("dataflow path has no parent dir"))?
                .to_owned();
            let build_info = build_dataflow_locally(
                dataflow_descriptor,
                &git_sources,
                &dataflow_session,
                local_working_dir,
                uv,
            )?;

            dataflow_session.git_sources = git_sources;
            // generate a random BuildId and store the associated build info
            dataflow_session.build_id = Some(BuildId::generate());
            dataflow_session.local_build = Some(build_info);
            dataflow_session
                .write_out_for_dataflow(&dataflow_path)
                .context("failed to write out dataflow session file")?;
        }
        BuildKind::ThroughCoordinator {
            mut coordinator_session,
        } => {
            let local_working_dir = local_working_dir(
                &dataflow_path,
                &dataflow_descriptor,
                &mut *coordinator_session,
            )?;
            let build_id = build_distributed_dataflow(
                &mut *coordinator_session,
                dataflow_descriptor,
                &git_sources,
                &dataflow_session,
                local_working_dir,
                uv,
            )?;

            dataflow_session.git_sources = git_sources;
            dataflow_session
                .write_out_for_dataflow(&dataflow_path)
                .context("failed to write out dataflow session file")?;

            // wait until dataflow build is finished

            wait_until_dataflow_built(
                build_id,
                &mut *coordinator_session,
                coordinator_socket(coordinator_addr, coordinator_port),
                log::LevelFilter::Info,
            )?;

            dataflow_session.build_id = Some(build_id);
            dataflow_session.local_build = None;
            dataflow_session
                .write_out_for_dataflow(&dataflow_path)
                .context("failed to write out dataflow session file")?;
        }
    };

    Ok(())
}

enum BuildKind {
    Local,
    ThroughCoordinator {
        coordinator_session: Box<TcpRequestReplyConnection>,
    },
}

fn connect_to_coordinator_with_defaults(
    coordinator_addr: Option<std::net::IpAddr>,
    coordinator_port: Option<u16>,
) -> std::io::Result<Box<TcpRequestReplyConnection>> {
    let coordinator_socket = coordinator_socket(coordinator_addr, coordinator_port);
    connect_to_coordinator(coordinator_socket)
}

fn coordinator_socket(
    coordinator_addr: Option<std::net::IpAddr>,
    coordinator_port: Option<u16>,
) -> std::net::SocketAddr {
    let coordinator_addr = coordinator_addr.unwrap_or(LOCALHOST);
    let coordinator_port = coordinator_port.unwrap_or(DORA_COORDINATOR_PORT_CONTROL_DEFAULT);
    (coordinator_addr, coordinator_port).into()
}
