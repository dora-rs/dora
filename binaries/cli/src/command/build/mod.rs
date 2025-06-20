use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_core::{
    descriptor::{CoreNodeKind, CustomNode, Descriptor, DescriptorExt},
    topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST},
};
use dora_message::{descriptor::NodeSource, BuildId};
use eyre::Context;
use std::collections::BTreeMap;

use crate::{connect_to_coordinator, resolve_dataflow, session::DataflowSession};

use distributed::{build_distributed_dataflow, wait_until_dataflow_built};
use local::build_dataflow_locally;

mod distributed;
mod git;
mod local;

pub fn build(
    dataflow: String,
    coordinator_addr: Option<std::net::IpAddr>,
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
        log::info!("Building through coordinator, using the given cooridnator socket information");
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
            let local_working_dir = super::local_working_dir(
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
