//! The `dora run` command runs a dataflow locally in isolation: it
//! spawns nodes in-process via `dora_daemon::Daemon::run_dataflow`
//! without binding any coordinator port. As a result, `dora run` does
//! NOT integrate with the CLI monitoring commands (`dora list`,
//! `dora stop`, `dora logs`, `dora top`, ...). Use `dora up` + `dora
//! start` instead when you need to attach those tools.
//!
//! Running isolated means:
//!   - `dora run` can be invoked while `dora up` is already running
//!     (the coordinator port is not contested).
//!   - Multiple `dora run` calls can execute in parallel.

use super::Executable;
use crate::common::parse_duration;
use crate::{
    BuildConfig, build as build_dataflow,
    common::{handle_dataflow_result, resolve_dataflow, write_events_to},
    output::{
        LogFormat, LogOutputConfig, parse_log_filter, parse_log_level_str, print_log_message,
    },
    session::DataflowSession,
};
use dora_core::build::LogLevelOrStdout;
use dora_core::descriptor::{Descriptor, DescriptorExt};
use dora_daemon::{Daemon, LogDestination, RunDataflowOptions, flume};
use eyre::Context;
use std::{path::PathBuf, time::Duration};
use tokio::runtime::Builder;

#[derive(Debug, clap::Args)]
/// Run a dataflow locally in isolation.
///
/// Spawns nodes in-process without a coordinator. CLI monitoring
/// commands (`dora list`, `dora stop`, `dora logs`, ...) do NOT attach
/// to a `dora run` execution. Use `dora up` + `dora start` instead when
/// you need to attach those tools or run multiple coordinated
/// dataflows.
pub struct Run {
    /// Path to the dataflow descriptor file
    #[clap(value_name = "PATH")]
    pub dataflow: String,
    // Use UV to run nodes.
    #[clap(long, action)]
    pub uv: bool,
    /// Automatically stop the dataflow after the given duration
    ///
    /// The command will send a stop message after the specified time has elapsed,
    /// similar to pressing Ctrl-C. This gracefully stops all nodes in the dataflow.
    ///
    /// Examples:
    ///   --stop-after 30      # 30 seconds
    ///   --stop-after 30s     # 30 seconds
    ///   --stop-after 5m      # 5 minutes
    #[clap(long, value_name = "DURATION", verbatim_doc_comment)]
    #[arg(value_parser = parse_duration)]
    pub stop_after: Option<Duration>,
    /// Minimum log level to display
    ///
    /// Levels: error, warn, info, debug, trace, stdout (default).
    /// "stdout" shows everything including raw stdout from nodes.
    #[clap(long, default_value = "stdout", env = "DORA_LOG_LEVEL")]
    #[arg(value_parser = parse_log_level_str)]
    pub log_level: LogLevelOrStdout,
    /// Output format for log messages
    #[clap(long, default_value = "pretty", env = "DORA_LOG_FORMAT")]
    pub log_format: LogFormat,
    /// Per-node log level filter
    ///
    /// Format: "node1=level,node2=level". Overrides --log-level for matched nodes.
    ///
    /// Examples:
    ///   --log-filter "sensor=debug,processor=warn"
    #[clap(
        long,
        value_name = "FILTER",
        env = "DORA_LOG_FILTER",
        verbatim_doc_comment
    )]
    pub log_filter: Option<String>,
    /// Allow shell nodes to execute arbitrary commands.
    ///
    /// Shell nodes are disabled by default for security reasons. This flag
    /// sets the DORA_ALLOW_SHELL_NODES environment variable.
    #[clap(long)]
    pub allow_shell_nodes: bool,
    /// Enable debug mode (publishes all messages to Zenoh for topic echo/hz/info)
    #[clap(long, action)]
    pub debug: bool,
    /// Use pinned git source commits from a lockfile during pre-run build.
    #[clap(long, action, conflicts_with = "write_lockfile")]
    pub locked: bool,
    /// Write resolved git source commits to a lockfile during pre-run build.
    #[clap(long, action)]
    pub write_lockfile: bool,
    /// Path to build lockfile (defaults to `<dataflow-stem>.dora-lock.yaml`).
    #[clap(long, value_name = "PATH")]
    pub lockfile: Option<PathBuf>,
    /// Decouples the working dir used for descriptor-relative paths
    /// from the dataflow file's parent. Needed when the path passed in
    /// is a rewritten copy (e.g. `dora record`'s tempfile) whose parent
    /// can't resolve the original `build:` / relative-path references.
    #[clap(skip)]
    pub working_dir: Option<PathBuf>,
    /// Substitute a local checkout for a hub package (UC11 inner loop):
    /// `--hub-override <namespace>/<name>=<path>`. Same as `dora build
    /// --hub-override`; `dora run` is always local, so it applies directly.
    #[clap(long = "hub-override", value_name = "PKG=PATH")]
    pub hub_override: Vec<String>,
    /// Set an environment variable for every node of this dataflow
    /// (repeatable). Merges into the dataflow-level `env:` block;
    /// node-level `env:` entries still win on conflict.
    ///
    /// Under `dora run` nodes also inherit this process's environment,
    /// but `--env` is the portable spelling that behaves identically
    /// under `dora start` (where nodes inherit the DAEMON's environment
    /// instead). Applies at spawn time only; `build:` commands are
    /// unaffected.
    /// Values must survive the descriptor encoding verbatim: a literal
    /// `$` is refused (the receiving process would expand it) and so are
    /// numeric-looking values that would be coerced.
    #[clap(long = "env", value_name = "KEY=VALUE")]
    pub env: Vec<String>,
    /// Exit once every node has finished, treating `dora/timer/...`
    /// inputs as a clock rather than as work.
    ///
    /// By default a node is only told its inputs are closed when ALL of
    /// them are, and a timer input never closes — so a graph where any
    /// node consumes a timer cannot finish on its own, even after every
    /// worker has done its work. With this flag a node is finished once
    /// its DATA inputs have closed, which makes "run N items and exit"
    /// scriptable without wrapping the command in a timeout.
    ///
    /// Nodes whose inputs are all timers are unaffected: they have no
    /// data dependency that could finish, so they are treated as
    /// sources, exactly as a node with no inputs is.
    ///
    /// Overrides `exit_when_nodes_finish:` in the dataflow YAML in
    /// either direction: pass the flag to force it on, or
    /// `--exit-when-nodes-finish=false` to force it off for a descriptor
    /// that asks for it. Omit it entirely and the descriptor decides.
    #[clap(
        long,
        num_args = 0..=1,
        require_equals = true,
        default_missing_value = "true",
        value_name = "BOOL"
    )]
    pub exit_when_nodes_finish: Option<bool>,
}

impl Run {
    pub fn new(dataflow: String) -> Self {
        Self {
            dataflow,
            uv: false,
            stop_after: None,
            log_level: LogLevelOrStdout::Stdout,
            log_format: LogFormat::Pretty,
            log_filter: None,
            allow_shell_nodes: false,
            debug: false,
            locked: false,
            write_lockfile: false,
            lockfile: None,
            working_dir: None,
            hub_override: Vec::new(),
            env: Vec::new(),
            exit_when_nodes_finish: None,
        }
    }

    pub fn with_working_dir(mut self, working_dir: PathBuf) -> Self {
        self.working_dir = Some(working_dir);
        self
    }
}

pub fn run(dataflow: String, uv: bool) -> eyre::Result<()> {
    let mut run = Run::new(dataflow);
    run.uv = uv;
    run.execute()
}

impl Executable for Run {
    fn execute(self) -> eyre::Result<()> {
        if self.allow_shell_nodes {
            // SAFETY: Called before spawning any threads (tokio runtime not yet built),
            // so there are no concurrent reads of environment variables.
            unsafe { std::env::set_var("DORA_ALLOW_SHELL_NODES", "true") };
        }

        let rt = Builder::new_multi_thread()
            .enable_all()
            .build()
            .context("tokio runtime failed")?;

        #[cfg(feature = "tracing")]
        let _guard = {
            let _enter = rt.enter();
            let env_log = std::env::var("RUST_LOG").unwrap_or("info".to_string());
            dora_tracing::init_tracing_subscriber(
                "dora-run",
                Some(&env_log),
                None,
                tracing::metadata::LevelFilter::INFO,
            )
            .context("failed to initialize tracing")?
        };

        let dataflow_path =
            resolve_dataflow(self.dataflow.clone()).context("could not resolve dataflow")?;
        // Validate `--env` BEFORE building: a typo should fail in
        // milliseconds, not after a full dataflow build.
        let env_overrides = crate::env_overrides::parse_env_overrides(&self.env)?;

        build_dataflow(BuildConfig {
            dataflow: dataflow_path.to_string_lossy().into_owned(),
            uv: self.uv,
            force_local: true,
            locked: self.locked,
            write_lockfile: self.write_lockfile,
            lockfile_override: self.lockfile.clone(),
            working_dir_override: self.working_dir.clone(),
            hub_overrides: self.hub_override.clone(),
            ..Default::default()
        })
        .context("failed to build dataflow before run")?;
        let dataflow_session = DataflowSession::read_session(&dataflow_path)
            .context("failed to read DataflowSession")?;

        // `--env` merges into the dataflow-level `env:` of the descriptor
        // handed to the in-process daemon. The hub-resolved descriptor
        // (when present) is the mandatory base — the on-disk YAML still
        // has unresolved `hub:` references. Without `--env`, pass the
        // session state through unchanged.
        let descriptor_override = if env_overrides.is_empty() {
            dataflow_session.resolved_dataflow.clone()
        } else {
            let mut descriptor = match dataflow_session.resolved_dataflow.clone() {
                Some(resolved) => resolved,
                None => Descriptor::blocking_read(&dataflow_path)
                    .context("failed to read dataflow descriptor for --env")?,
            };
            crate::env_overrides::apply_env_overrides(&mut descriptor, env_overrides);
            Some(descriptor)
        };

        let node_filters = match &self.log_filter {
            Some(filter) => parse_log_filter(filter).map_err(|e| eyre::eyre!(e))?,
            None => Default::default(),
        };

        let log_config = LogOutputConfig {
            min_level: self.log_level,
            format: self.log_format,
            node_filters,
            print_dataflow_id: false,
            print_daemon_name: false,
        };

        let (log_tx, log_rx) = flume::bounded(100);
        std::thread::spawn(move || {
            for message in log_rx {
                print_log_message(message, &log_config);
            }
        });

        // Drive `Daemon::run_dataflow` on a tokio worker thread, not the
        // calling main thread. On Windows the main thread's default stack
        // is 1 MiB, which the daemon's deeply-nested async state machine
        // (zenoh + arrow + flume + coordinator I/O) overflows in debug
        // builds — `target\debug\examples\rust-dataflow.exe` crashed with
        // STATUS_STACK_OVERFLOW in the 05-28 nightly (#1964). tokio worker
        // threads default to 2 MiB, which clears the overflow. Linux and
        // macOS were unaffected because their main-thread stacks are
        // multi-MiB by default. Regression from #1962.
        //
        // `run_dataflow` takes `&Path`, so the returned future borrows
        // non-`'static`; move an owned `PathBuf` (plus the other Run
        // fields) into the spawned task so the future is `'static`.
        let dataflow_path_for_daemon = dataflow_path.clone();
        let uv = self.uv;
        let stop_after = self.stop_after;
        let debug = self.debug;
        let working_dir_override = self.working_dir.clone();
        let exit_when_nodes_finish = self.exit_when_nodes_finish;
        let handle = rt.spawn(async move {
            Daemon::run_dataflow_with(
                &dataflow_path_for_daemon,
                dataflow_session.build_id,
                dataflow_session.local_build,
                dataflow_session.session_id,
                uv,
                LogDestination::Channel { sender: log_tx },
                write_events_to(),
                stop_after,
                debug,
                working_dir_override,
                // hub-resolved descriptor and/or `--env` merge — see above
                descriptor_override,
                // Left unset the descriptor decides; given, it overrides.
                match exit_when_nodes_finish {
                    Some(v) => RunDataflowOptions::default().exit_when_nodes_finish(v),
                    None => RunDataflowOptions::default(),
                },
            )
            .await
        });
        let result = rt
            .block_on(handle)
            .context("dora-run daemon task panicked")??;
        // Bound runtime shutdown to prevent hanging on blocking Drop impls
        // (e.g. zenoh::Session::drop blocks tokio workers on macOS during
        // TCP teardown). Without this, `rt` drops implicitly at end of scope
        // and waits indefinitely for all worker threads to exit (#2287).
        rt.shutdown_timeout(Duration::from_secs(10));
        handle_dataflow_result(result, None)
    }
}
