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

use dora_core::{
    descriptor::{CoreNodeKind, CustomNode, Descriptor, DescriptorExt},
    topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST},
};
use dora_message::{BuildId, cli_to_coordinator::CoordinatorControlClient, descriptor::NodeSource};
use eyre::Context;
use std::{collections::BTreeMap, net::IpAddr};

use super::{Executable, default_tracing};
use crate::{
    common::{connect_and_check_version, local_working_dir, resolve_dataflow},
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
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        build_async(
            self.dataflow,
            self.coordinator_addr,
            self.coordinator_port,
            self.uv,
            self.local,
        )
        .await
    }
}

pub fn build(
    dataflow: String,
    coordinator_addr: Option<IpAddr>,
    coordinator_port: Option<u16>,
    uv: bool,
    force_local: bool,
) -> eyre::Result<()> {
    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .context("tokio runtime failed")?;
    rt.block_on(build_async(
        dataflow,
        coordinator_addr,
        coordinator_port,
        uv,
        force_local,
    ))
}

pub async fn build_async(
    dataflow: String,
    coordinator_addr: Option<IpAddr>,
    coordinator_port: Option<u16>,
    uv: bool,
    force_local: bool,
) -> eyre::Result<()> {
    let dataflow_path = resolve_dataflow(dataflow)
        .await
        .context("could not resolve dataflow")?;
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

    let session = || connect_to_coordinator_rpc_with_defaults(coordinator_addr, coordinator_port);

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
            coordinator_client: session()
                .await
                .context("failed to connect to coordinator")?,
        }
    } else {
        match session().await {
            Ok(coordinator_client) => {
                // we found a local coordinator instance at default port -> use it for building
                log::info!("Found local dora coordinator instance -> building through coordinator");
                BuildKind::ThroughCoordinator { coordinator_client }
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
            )
            .await?;

            dataflow_session.git_sources = git_sources;
            // generate a random BuildId and store the associated build info
            dataflow_session.build_id = Some(BuildId::generate());
            dataflow_session.local_build = Some(build_info);
            dataflow_session
                .write_out_for_dataflow(&dataflow_path)
                .context("failed to write out dataflow session file")?;
        }
        BuildKind::ThroughCoordinator { coordinator_client } => {
            let local_working_dir =
                local_working_dir(&dataflow_path, &dataflow_descriptor, &coordinator_client)
                    .await?;
            let build_id = build_distributed_dataflow(
                &coordinator_client,
                dataflow_descriptor,
                &git_sources,
                &dataflow_session,
                local_working_dir,
                uv,
            )
            .await?;
            let wait_result = wait_until_dataflow_built(
                build_id,
                &coordinator_client,
                coordinator_socket(coordinator_addr, coordinator_port),
                log::LevelFilter::Info,
            )
            .await;
            finalize_distributed_build_session(
                &mut dataflow_session,
                &dataflow_path,
                git_sources,
                wait_result,
            )?;
        }
    };

    Ok(())
}

enum BuildKind {
    Local,
    ThroughCoordinator {
        coordinator_client: CoordinatorControlClient,
    },
}

async fn connect_to_coordinator_rpc_with_defaults(
    coordinator_addr: Option<std::net::IpAddr>,
    coordinator_port: Option<u16>,
) -> eyre::Result<CoordinatorControlClient> {
    let addr = coordinator_addr.unwrap_or(LOCALHOST);
    let control_port = coordinator_port.unwrap_or(DORA_COORDINATOR_PORT_CONTROL_DEFAULT);
    connect_and_check_version(addr, control_port).await
}

fn coordinator_socket(
    coordinator_addr: Option<std::net::IpAddr>,
    coordinator_port: Option<u16>,
) -> std::net::SocketAddr {
    let coordinator_addr = coordinator_addr.unwrap_or(LOCALHOST);
    let coordinator_port = coordinator_port.unwrap_or(DORA_COORDINATOR_PORT_CONTROL_DEFAULT);
    (coordinator_addr, coordinator_port).into()
}

fn finalize_distributed_build_session(
    dataflow_session: &mut DataflowSession,
    dataflow_path: &std::path::Path,
    git_sources: BTreeMap<dora_message::id::NodeId, dora_message::common::GitSource>,
    wait_result: eyre::Result<BuildId>,
) -> eyre::Result<()> {
    let build_id = wait_result?;
    dataflow_session.git_sources = git_sources;
    dataflow_session.build_id = Some(build_id);
    dataflow_session.local_build = None;
    dataflow_session
        .write_out_for_dataflow(dataflow_path)
        .context("failed to write out dataflow session file")
}

#[cfg(test)]
mod tests {
    use std::{collections::BTreeMap, fs, path::PathBuf};

    use dora_message::{BuildId, SessionId, common::GitSource, id::NodeId};

    use super::finalize_distributed_build_session;
    use crate::session::DataflowSession;

    fn test_session_file_path(dataflow_path: &std::path::Path) -> PathBuf {
        let stem = dataflow_path
            .file_stem()
            .expect("dataflow path should have file stem")
            .to_string_lossy();
        dataflow_path
            .with_file_name("out")
            .join(format!("{stem}.dora-session.yaml"))
    }

    #[test]
    fn distributed_build_failure_does_not_persist_new_git_sources() {
        let temp_root =
            std::env::temp_dir().join(format!("dora-cli-build-test-{}", uuid::Uuid::new_v4()));
        fs::create_dir_all(&temp_root).expect("failed to create temp test dir");
        let dataflow_path = temp_root.join("dataflow.yml");
        fs::write(&dataflow_path, "nodes: []\n").expect("failed to write test dataflow");

        let old_source = GitSource {
            repo: "https://example.com/old.git".to_string(),
            commit_hash: "1111111".to_string(),
        };
        let old_sources = BTreeMap::from([(NodeId::from("node-a".to_string()), old_source)]);
        let mut session = DataflowSession {
            build_id: Some(BuildId::generate()),
            session_id: SessionId::generate(),
            git_sources: old_sources.clone(),
            local_build: None,
        };
        session
            .write_out_for_dataflow(&dataflow_path)
            .expect("failed to write initial session");

        let session_file = test_session_file_path(&dataflow_path);
        let before =
            fs::read_to_string(&session_file).expect("failed to read initial session file");

        let new_source = GitSource {
            repo: "https://example.com/new.git".to_string(),
            commit_hash: "2222222".to_string(),
        };
        let new_sources = BTreeMap::from([(NodeId::from("node-a".to_string()), new_source)]);

        let result = finalize_distributed_build_session(
            &mut session,
            &dataflow_path,
            new_sources,
            Err(eyre::eyre!("remote build failed")),
        );
        assert!(result.is_err(), "expected failure to be propagated");
        assert_eq!(
            session.git_sources, old_sources,
            "in-memory session should not be mutated on failed distributed build"
        );

        let after =
            fs::read_to_string(&session_file).expect("failed to read session file after failure");
        assert_eq!(
            after, before,
            "session file should remain unchanged on failed distributed build"
        );

        let _ = fs::remove_dir_all(&temp_root);
    }
}
