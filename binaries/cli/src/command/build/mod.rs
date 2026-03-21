//! Provides the `adora build` command.
//!
//! The `adora build` command works like this:
//!
//! - Dataflows can specify a `build` command for each node in their YAML definition
//! - Adora will run the `build` command when `adora build` is invoked
//! - If the dataflow is distributed across multiple machines, each `build` command will be run the target machine of the corresponding node.
//!     - i.e. the machine specified under the `deploy` key
//!     - this requires a connection to the adora coordinator, so you need to specify the coordinator IP/port for this
//!     - to run the build commands of all nodes _locally_, you can use `adora build --local`
//! - If the build command does not specify any `deploy` keys, all build commands will be run locally (i.e. `adora build` behaves like `adora build --local`)
//!
//! #### Git Source
//!
//! - Nodes can have a git repository as source
//!     - set the `git` config key to the URL of the repository
//!     - by default, the default branch is used
//!     - you can also specify a specific `branch` name
//!     - alternatively, you can specify a `tag` name or a `rev` key with the commit hash
//!     - you can only specify one of `branch`, `tag`, and `rev`, otherwise an error will occur
//! - Adora will automatically clone and checkout the requested branch/tag/commit on `adora build`
//!     - the `build` command will be run after cloning
//!     - for distributed dataflows, the clone/checkout will happen on the target machine
//! - subsequent `adora build` command will automatically fetch the latest changes for nodes
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
//!     git: https://github.com/dora-rs/adora.git
//!     # the build command that should be invoked after cloning
//!     build: cargo build -p rust-dataflow-example-node
//!     # path to the executable that should be run on start
//!     path: target/debug/rust-dataflow-example-node
//!     inputs:
//!       tick: adora/timer/millis/10
//!     outputs:
//!       - random
//! ```

use adora_core::{
    descriptor::{CoreNodeKind, CustomNode, Descriptor, DescriptorExt},
    topics::{ADORA_COORDINATOR_PORT_WS_DEFAULT, LOCALHOST},
    types::TypeRegistry,
};
use adora_message::{BuildId, descriptor::NodeSource};
use eyre::Context;
use std::{collections::BTreeMap, net::IpAddr, path::PathBuf};

use crate::ws_client::WsSession;

use super::{Executable, default_tracing};
use crate::{
    common::{connect_to_coordinator, local_working_dir, resolve_dataflow},
    session::DataflowSession,
};

use distributed::{build_distributed_dataflow, wait_until_dataflow_built};
use local::build_dataflow_locally;
use lockfile::BuildLockfile;

mod distributed;
mod git;
mod local;
mod lockfile;

#[derive(Debug, clap::Args)]
/// Run build commands provided in the given dataflow.
pub struct Build {
    /// Path to the dataflow descriptor file
    #[clap(value_name = "PATH")]
    dataflow: String,
    /// Address of the adora coordinator
    #[clap(long, value_name = "IP", env = "ADORA_COORDINATOR_ADDR")]
    coordinator_addr: Option<IpAddr>,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", env = "ADORA_COORDINATOR_PORT")]
    coordinator_port: Option<u16>,
    // Use UV to build nodes.
    #[clap(long, action)]
    uv: bool,
    // Run build on local machine
    #[clap(long, action)]
    local: bool,
    /// Treat type warnings as errors
    #[clap(long, action)]
    strict_types: bool,
    /// Use pinned git source commits from a lockfile.
    #[clap(long, action)]
    locked: bool,
    /// Write resolved git source commits to a lockfile.
    #[clap(long, action)]
    write_lockfile: bool,
    /// Path to build lockfile (defaults to `<dataflow-stem>.adora-lock.yaml`).
    #[clap(long, value_name = "PATH")]
    lockfile: Option<PathBuf>,
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
            self.strict_types,
            self.locked,
            self.write_lockfile,
            self.lockfile,
        )
    }
}

pub fn build(
    dataflow: String,
    coordinator_addr: Option<IpAddr>,
    coordinator_port: Option<u16>,
    uv: bool,
    force_local: bool,
    strict_types: bool,
    locked: bool,
    write_lockfile: bool,
    lockfile_override: Option<PathBuf>,
) -> eyre::Result<()> {
    let dataflow_path = resolve_dataflow(dataflow).context("could not resolve dataflow")?;
    if lockfile_override.is_some() && !(locked || write_lockfile) {
        eyre::bail!("`--lockfile` requires either `--locked` or `--write-lockfile`");
    }
    let working_dir = dataflow_path
        .parent()
        .unwrap_or_else(|| std::path::Path::new("."));
    let dataflow_descriptor = Descriptor::blocking_read(&dataflow_path)
        .wrap_err_with(|| {
            format!(
                "failed to read dataflow at `{}`\n\n  \
                 hint: check the file exists and is valid YAML",
                dataflow_path.display()
            )
        })?
        .expand(working_dir)
        .wrap_err("failed to expand modules in dataflow descriptor")?;

    // --- Type checking (Phase 1) ---
    let strict = strict_types || dataflow_descriptor.strict_types.unwrap_or(false);
    let mut registry = TypeRegistry::new();
    let types_dir = working_dir.join("types");
    if types_dir.is_dir() {
        match registry.load_from_dir(&types_dir) {
            Ok(count) if count > 0 => {
                log::info!("Loaded {count} user-defined type(s) from types/");
            }
            Err(e) => {
                eyre::bail!("failed to load user types: {e}");
            }
            _ => {}
        }
    }
    let type_result = adora_core::descriptor::validate::check_type_annotations_full(
        &dataflow_descriptor,
        &registry,
        strict,
    );
    for inf in &type_result.inferences {
        println!("  {inf}");
    }
    if !type_result.warnings.is_empty() {
        for w in &type_result.warnings {
            eprintln!("  warning: {w}");
        }
        let count = type_result.warnings.len();
        if strict {
            eyre::bail!("{count} type error(s) found (strict mode)");
        } else {
            eprintln!(
                "{count} type warning(s) found.\n  \
                 hint: use --strict-types to fail on type warnings"
            );
        }
    }

    let mut dataflow_session =
        DataflowSession::read_session(&dataflow_path).context("failed to read DataflowSession")?;

    let lockfile_path = BuildLockfile::path_for_dataflow(&dataflow_path, lockfile_override);
    let build_lockfile = if locked {
        Some(BuildLockfile::read_from(&lockfile_path).with_context(|| {
            format!(
                "failed to read build lockfile at `{}`",
                lockfile_path.display()
            )
        })?)
    } else {
        None
    };

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
            let source = match &build_lockfile {
                Some(lockfile) => lockfile
                    .get_source(&node_id, &repo)
                    .with_context(|| format!("failed to resolve locked git source `{node_id}`"))?,
                None => git::fetch_commit_hash(repo, rev)
                    .with_context(|| format!("failed to find commit hash for `{node_id}`"))?,
            };
            git_sources.insert(node_id, source);
        }
    }
    if write_lockfile {
        let lockfile = BuildLockfile::from_git_sources(git_sources.clone());
        lockfile.write_to(&lockfile_path).with_context(|| {
            format!(
                "failed to write build lockfile to `{}`",
                lockfile_path.display()
            )
        })?;
        log::info!("wrote build lockfile to {}", lockfile_path.display());
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
                log::info!(
                    "Found local adora coordinator instance -> building through coordinator"
                );
                BuildKind::ThroughCoordinator {
                    coordinator_session,
                }
            }
            Err(_) => {
                log::warn!("No adora coordinator instance found -> trying a local build");
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
            coordinator_session,
        } => {
            let local_working_dir =
                local_working_dir(&dataflow_path, &dataflow_descriptor, &coordinator_session)?;
            let build_id = build_distributed_dataflow(
                &coordinator_session,
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

            wait_until_dataflow_built(build_id, &coordinator_session, log::LevelFilter::Info)?;

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
    ThroughCoordinator { coordinator_session: WsSession },
}

fn connect_to_coordinator_with_defaults(
    coordinator_addr: Option<std::net::IpAddr>,
    coordinator_port: Option<u16>,
) -> eyre::Result<WsSession> {
    let coordinator_addr = coordinator_addr.unwrap_or(LOCALHOST);
    let coordinator_port = coordinator_port.unwrap_or(ADORA_COORDINATOR_PORT_WS_DEFAULT);
    connect_to_coordinator((coordinator_addr, coordinator_port).into())
}
