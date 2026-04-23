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
    topics::{DORA_COORDINATOR_PORT_WS_DEFAULT, LOCALHOST},
    types::TypeRegistry,
};
use dora_message::{BuildId, descriptor::NodeSource};
use eyre::Context;
use std::{collections::BTreeMap, net::IpAddr, path::PathBuf};

use crate::ws_client::WsSession;

use super::{Executable, default_tracing};
use crate::{
    common::{
        canonicalize_working_dir, connect_to_coordinator, local_working_dir, resolve_dataflow,
        working_dir_or_parent,
    },
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
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", env = "DORA_COORDINATOR_ADDR")]
    coordinator_addr: Option<IpAddr>,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", env = "DORA_COORDINATOR_PORT")]
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
    #[clap(long, action, conflicts_with = "write_lockfile")]
    locked: bool,
    /// Write resolved git source commits to a lockfile.
    #[clap(long, action)]
    write_lockfile: bool,
    /// Path to build lockfile (defaults to `<dataflow-stem>.dora-lock.yaml`).
    #[clap(long, value_name = "PATH")]
    lockfile: Option<PathBuf>,
    /// Build nodes concurrently (faster on multi-core machines).
    #[clap(long, action)]
    parallel: bool,
}

impl Executable for Build {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        build(BuildConfig {
            dataflow: self.dataflow,
            coordinator_addr: self.coordinator_addr,
            coordinator_port: self.coordinator_port,
            uv: self.uv,
            force_local: self.local,
            strict_types: self.strict_types,
            locked: self.locked,
            write_lockfile: self.write_lockfile,
            lockfile_override: self.lockfile,
            parallel: self.parallel,
            ..Default::default()
        })
    }
}

/// Configuration for a [`build`] invocation. Set `dataflow` and
/// override only the fields you care about using struct-update syntax:
///
/// ```ignore
/// build(BuildConfig {
///     dataflow: path,
///     uv: true,
///     force_local: true,
///     ..Default::default()
/// })?;
/// ```
#[derive(Debug, Clone, Default)]
pub struct BuildConfig {
    pub dataflow: String,
    pub coordinator_addr: Option<IpAddr>,
    pub coordinator_port: Option<u16>,
    pub uv: bool,
    pub force_local: bool,
    pub strict_types: bool,
    pub locked: bool,
    pub write_lockfile: bool,
    pub lockfile_override: Option<PathBuf>,
    pub parallel: bool,
    /// Overrides the working directory for cargo invocations and
    /// module expansion. Needed when the dataflow path points at a
    /// rewritten copy (e.g. a tempfile) whose parent can't resolve the
    /// original `build:` directives or relative node binaries.
    pub working_dir_override: Option<PathBuf>,
}

impl BuildConfig {
    /// Shared constructor for callers that marshal arguments as
    /// strings (`coordinator_addr: Option<String>`) and optional
    /// booleans (`uv: Option<bool>`) rather than as pre-parsed
    /// `IpAddr` / `bool`. Keeps the `addr.parse()` + `unwrap_or_default`
    /// glue in one place instead of duplicating it in every binding
    /// that exposes a simplified build API (PyO3 today, potentially
    /// a C FFI or WASM shim later).
    pub fn from_str_args(
        dataflow: String,
        uv: Option<bool>,
        coordinator_addr: Option<String>,
        coordinator_port: Option<u16>,
        force_local: bool,
    ) -> eyre::Result<Self> {
        Ok(Self {
            dataflow,
            coordinator_addr: coordinator_addr
                .map(|addr| addr.parse())
                .transpose()
                .wrap_err("invalid coordinator_addr")?,
            coordinator_port,
            uv: uv.unwrap_or_default(),
            force_local,
            ..Default::default()
        })
    }
}

pub fn build(cfg: BuildConfig) -> eyre::Result<()> {
    let BuildConfig {
        dataflow,
        coordinator_addr,
        coordinator_port,
        uv,
        force_local,
        strict_types,
        locked,
        write_lockfile,
        lockfile_override,
        parallel,
        working_dir_override,
    } = cfg;
    // `BuildConfig` derives `Default` so `..Default::default()` works at
    // call sites, but that gives `dataflow: String::new()` which would
    // fail late with a confusing "failed to read ``" error. Catch it up
    // front with a clear message.
    if dataflow.is_empty() {
        eyre::bail!(
            "BuildConfig::dataflow is empty — set it to a YAML path or URL before calling build()"
        );
    }
    let dataflow_path = resolve_dataflow(dataflow).context("could not resolve dataflow")?;
    if lockfile_override.is_some() && !(locked || write_lockfile) {
        eyre::bail!("`--lockfile` requires either `--locked` or `--write-lockfile`");
    }
    let working_dir = working_dir_or_parent(working_dir_override.as_deref(), &dataflow_path);
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
    let type_result = dora_core::descriptor::validate::check_type_annotations_full(
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
    let mut descriptor_git_sources = BTreeMap::new();
    let resolved_nodes = dataflow_descriptor
        .resolve_aliases_and_set_defaults()
        .context("failed to resolve nodes")?;
    // Pass 1 (fail-fast): derive descriptor git-source fingerprint and validate lockfile
    // provenance before any per-node locked-source lookups.
    for (node_id, node) in &resolved_nodes {
        if let CoreNodeKind::Custom(CustomNode {
            source: NodeSource::GitBranch { repo, rev },
            ..
        }) = &node.kind
        {
            descriptor_git_sources.insert(
                node_id.clone(),
                NodeSource::GitBranch {
                    repo: repo.clone(),
                    rev: rev.clone(),
                },
            );
        }
    }
    let descriptor_fingerprint =
        BuildLockfile::fingerprint_descriptor_git_sources(&descriptor_git_sources);
    if let Some(lockfile) = &build_lockfile {
        lockfile
            .ensure_descriptor_fingerprint_matches(&descriptor_fingerprint)
            .with_context(|| {
                format!(
                    "failed to validate lockfile against descriptor at `{}`",
                    dataflow_path.display()
                )
            })?;
    }
    // Pass 2: resolve each node's concrete source, now that lockfile provenance
    // has been validated (when `--locked` is enabled).
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
        BuildLockfile::write_git_sources(&lockfile_path, &git_sources, &descriptor_fingerprint)
            .with_context(|| {
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
            let local_working_dir =
                canonicalize_working_dir(working_dir_override.as_deref(), &dataflow_path)?;
            let build_info = build_dataflow_locally(
                dataflow_descriptor,
                &git_sources,
                &dataflow_session,
                local_working_dir,
                uv,
                parallel,
            )?;

            dataflow_session.git_sources = git_sources;
            // Reuse existing build_id if git sources are unchanged and
            // a prior build already exists. This preserves the
            // association between the build_id and the git clone
            // directories so `dora run`'s internal rebuild doesn't
            // orphan them.
            if dataflow_session.build_id.is_none() {
                dataflow_session.build_id = Some(BuildId::generate());
            }
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
    let coordinator_port = coordinator_port.unwrap_or(DORA_COORDINATOR_PORT_WS_DEFAULT);
    connect_to_coordinator((coordinator_addr, coordinator_port).into())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_str_args_parses_valid_coordinator_addr() {
        let cfg = BuildConfig::from_str_args(
            "dataflow.yml".into(),
            None,
            Some("127.0.0.1".into()),
            None,
            false,
        )
        .expect("valid IP should parse");
        assert_eq!(
            cfg.coordinator_addr,
            Some("127.0.0.1".parse::<IpAddr>().unwrap())
        );
    }

    #[test]
    fn from_str_args_accepts_none_addr() {
        let cfg = BuildConfig::from_str_args("dataflow.yml".into(), None, None, None, false)
            .expect("None addr should be fine");
        assert!(cfg.coordinator_addr.is_none());
    }

    #[test]
    fn from_str_args_errors_on_invalid_addr() {
        let err = BuildConfig::from_str_args(
            "dataflow.yml".into(),
            None,
            Some("not-an-ip".into()),
            None,
            false,
        )
        .expect_err("malformed addr should error");
        assert!(
            err.to_string().contains("invalid coordinator_addr"),
            "error should carry context: {err}"
        );
    }

    #[test]
    fn from_str_args_unwraps_uv_default_to_false() {
        let cfg =
            BuildConfig::from_str_args("dataflow.yml".into(), None, None, None, false).unwrap();
        assert!(!cfg.uv, "uv should default to false when None");
    }
}
