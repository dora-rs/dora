//! The `dora start` command is used to spawn a dataflow in a pre-existing _dora network_. To create a dora network, spawn a `dora coordinator` and one or multiple `dora daemon` instances.
//!
//! The `dora start` command does not run any build commands, nor update git dependencies or similar. Use `dora build` for that.

use super::{Executable, default_tracing};
use crate::{
    command::start::attach::attach_dataflow,
    common::{
        CoordinatorOptions, connect_to_coordinator, error_indicates_dataflow_finished,
        expect_reply, local_working_dir, resolve_dataflow, send_control_request, write_events_to,
    },
    output::{LogOutputConfig, print_log_message},
    session::DataflowSession,
    ws_client::WsSession,
};
use dora_core::descriptor::{Descriptor, DescriptorExt};
use dora_message::{cli_to_coordinator::ControlRequest, common::LogMessage, descriptor::EnvValue};
use eyre::Context;
use std::{collections::BTreeMap, io::IsTerminal, net::SocketAddr, path::PathBuf};
use uuid::Uuid;

mod attach;

#[derive(Debug, clap::Args)]
/// Start the given dataflow path. Attach a name to the running dataflow by using --name.
pub struct Start {
    /// Path to the dataflow descriptor file
    #[clap(value_name = "PATH")]
    dataflow: String,
    /// Assign a name to the dataflow
    #[clap(long, short = 'n')]
    name: Option<String>,
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
    /// Attach to the dataflow and wait for its completion
    #[clap(long, action, conflicts_with = "detach")]
    attach: bool,
    /// Run the dataflow in background
    #[clap(long, action)]
    detach: bool,
    /// Enable hot reloading (Python only)
    #[clap(long, action)]
    hot_reload: bool,
    // Use UV to run nodes.
    #[clap(long, action)]
    uv: bool,
    /// Enable debug mode (publishes all messages to Zenoh for topic echo/hz/info)
    #[clap(long, action)]
    debug: bool,
    /// Set an environment variable for every node of this dataflow
    /// (repeatable). Merges into the dataflow-level `env:` block;
    /// node-level `env:` entries still win on conflict.
    ///
    /// Nodes started via `dora start` are spawned by the DAEMON and do
    /// NOT inherit this CLI invocation's environment — `--env` is the
    /// supported way to parameterize a run without editing the YAML.
    /// Applies at spawn time only; `build:` commands are unaffected.
    /// Values must survive the descriptor encoding verbatim: a literal
    /// `$` is refused (the receiving process would expand it) and so are
    /// numeric-looking values that would be coerced. They are also
    /// persisted with the dataflow in the coordinator's state store and
    /// visible in `ps` — prefer a node `env:` block for secrets.
    /// Exit once every node has finished, treating `dora/timer/...`
    /// inputs as a clock rather than as work.
    ///
    /// A timer input never closes, so by default a dataflow in which any
    /// node consumes one cannot end on its own, even after every node
    /// doing real work has exited. With this flag a node is finished once
    /// its DATA inputs have closed.
    ///
    /// Note this is usually what you want only for batch-style runs: a
    /// long-lived dataflow is normally ended with `dora stop`, and there
    /// the timer is exactly what keeps it alive.
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
    #[clap(long = "env", value_name = "KEY=VALUE")]
    env: Vec<String>,
}

impl Executable for Start {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let coordinator_socket = self.coordinator.socket_addr();

        let env_overrides = crate::env_overrides::parse_env_overrides(&self.env)?;
        let (dataflow, dataflow_descriptor, session, dataflow_id) = start_dataflow(
            self.dataflow,
            self.name,
            coordinator_socket,
            self.uv,
            self.debug,
            env_overrides,
            self.exit_when_nodes_finish,
        )?;

        let attach = match (self.attach, self.detach) {
            (true, _) => true,
            (false, true) => false,
            (false, false) => {
                if std::io::stdin().is_terminal() {
                    eprintln!("attaching to dataflow (use `--detach` to run in background)");
                    true
                } else {
                    eprintln!("non-interactive mode: running in background");
                    false
                }
            }
        };

        if attach {
            let log_level = env_logger::Builder::new()
                .filter_level(log::LevelFilter::Info)
                .parse_default_env()
                .build()
                .filter();

            attach_dataflow(
                dataflow_descriptor,
                dataflow,
                dataflow_id,
                &session,
                self.hot_reload,
                log_level,
            )
        } else {
            let print_daemon_name = dataflow_descriptor.nodes.iter().any(|n| n.deploy.is_some());
            // wait until dataflow is started
            wait_until_dataflow_started(
                dataflow_id,
                &session,
                log::LevelFilter::Info,
                print_daemon_name,
            )
        }
    }
}

fn start_dataflow(
    dataflow: String,
    name: Option<String>,
    coordinator_socket: SocketAddr,
    uv: bool,
    debug: bool,
    env_overrides: BTreeMap<String, EnvValue>,
    exit_when_nodes_finish: Option<bool>,
) -> Result<(PathBuf, Descriptor, WsSession, Uuid), eyre::Error> {
    let dataflow = resolve_dataflow(dataflow).context("could not resolve dataflow")?;
    let working_dir = dataflow
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| std::path::Path::new("."));
    let mut dataflow_descriptor = Descriptor::blocking_read(&dataflow)
        .wrap_err_with(|| {
            format!(
                "failed to read dataflow at `{}`\n\n  \
                 hint: check the file exists, is valid YAML, and matches the dataflow schema (see details below)",
                dataflow.display()
            )
        })?
        .expand(working_dir)
        .wrap_err("failed to expand modules in dataflow descriptor")?;
    let mut dataflow_session =
        DataflowSession::read_session(&dataflow).context("failed to read DataflowSession")?;
    // `hub:` references are desugared by `dora build`, which stores the
    // resolved descriptor in the session — `dora start` requires that prior
    // build, exactly like git sources require a prior clone. Compare the
    // expanded descriptor against the digest taken at build time so *any*
    // on-disk edit (hub or not) since the build is caught, then substitute
    // the resolved form.
    if dataflow_descriptor.nodes.iter().any(|n| n.hub.is_some()) {
        let resolved = dataflow_session.resolved_dataflow.clone().ok_or_else(|| {
            eyre::eyre!("this dataflow uses `hub:` nodes — run `dora build` first")
        })?;
        let current = DataflowSession::fingerprint_source(&dataflow_descriptor);
        if current.is_none() || current != dataflow_session.source_fingerprint {
            eyre::bail!(
                "this dataflow changed since the last `dora build` — run `dora build` again \
                 (`dora start` cannot re-resolve `hub:` references)"
            );
        }
        dataflow_descriptor = resolved;
    }
    // Invalidate cached `build_id`/`local_build`/`git_sources` if the
    // descriptor's build-inputs (build command, source, env, cwd) changed
    // since the last `dora build`. Without this, `dora start` would send a
    // stale `build_id` to the coordinator and silently target the previous
    // build's artifacts (#1444).
    let resolved_for_fingerprint = dataflow_descriptor
        .resolve_aliases_and_set_defaults()
        .context("failed to resolve nodes for session fingerprint")?;
    if dataflow_session.invalidate_if_build_inputs_changed(&resolved_for_fingerprint) {
        dataflow_session
            .write_out_for_dataflow(&dataflow)
            .context("failed to persist invalidated dataflow session")?;
    }
    drop(resolved_for_fingerprint);

    // A local `dora build` produces artifacts that only exist on this machine.
    // A distributed (`deploy`) dataflow is started through the coordinator and
    // runs on remote daemons that cannot see that build, so the cached id is
    // useless there and `dora start` cannot rebuild. Refuse early with an
    // actionable message instead of letting the daemon fail confusingly (#1955).
    let has_deploy_nodes = dataflow_descriptor.nodes.iter().any(|n| n.deploy.is_some());
    if local_build_blocks_distributed_start(
        dataflow_session.local_build.is_some(),
        has_deploy_nodes,
    ) {
        eyre::bail!(
            "this dataflow was built locally, but it has `deploy` sections and is started \
             through the coordinator — remote daemons cannot use a local build.\n\n  \
             run `dora build` against the running coordinator (without `--local`) to build on \
             the target machines before `dora start`"
        );
    }

    if debug {
        dataflow_descriptor.debug.enable_debug_inspection = true;
    }

    // Merge `--env` LAST: the hub `fingerprint_source` comparison and
    // `invalidate_if_build_inputs_changed` above both hash the
    // descriptor (env included) against `dora build`-time state, and
    // `--env` is a spawn-time input, not a build input — merging
    // earlier would reject hub dataflows as "changed since build" and
    // invalidate the cached build id on every `--env` change.
    crate::env_overrides::apply_env_overrides(&mut dataflow_descriptor, env_overrides);

    // Fold the `--exit-when-nodes-finish` flag into the descriptor LAST, for
    // the same reason `--env` is merged last (see above): it is a spawn-time
    // override, not a build input, so it must not be part of the hub
    // `fingerprint_source` comparison. Applied before the hub block, a value
    // differing from the on-disk YAML changed the serialized
    // descriptor and its fingerprint, making a `hub:` dataflow bail with a
    // misleading "changed since the last `dora build`" error — and the hub
    // block's `dataflow_descriptor = resolved` then discarded the flag anyway
    // (#2996). Folding it in here targets the final (hub-resolved) descriptor,
    // which is what the daemon reads and the one copy that survives
    // auto-recovery, coordinator restart and `dora restart` (#2920). Given, it
    // overrides the descriptor in either direction; omitted, the descriptor's
    // own setting stands.
    dataflow_descriptor.apply_exit_when_nodes_finish(exit_when_nodes_finish);

    let session = connect_to_coordinator(coordinator_socket)?;

    let local_working_dir = local_working_dir(&dataflow, &dataflow_descriptor, &session)?;

    let dataflow_id = {
        let dataflow = dataflow_descriptor.clone();
        let reply = send_control_request(
            &session,
            &ControlRequest::Start {
                build_id: dataflow_session.build_id,
                session_id: dataflow_session.session_id,
                dataflow,
                name,
                local_working_dir,
                uv,
                write_events_to: write_events_to(),
            },
        )?;
        let uuid = expect_reply!(reply, DataflowStartTriggered { uuid })?;
        println!("dataflow start triggered: {uuid}");
        uuid
    };
    Ok((dataflow, dataflow_descriptor, session, dataflow_id))
}

fn wait_until_dataflow_started(
    dataflow_id: Uuid,
    session: &WsSession,
    log_level: log::LevelFilter,
    print_daemon_id: bool,
) -> eyre::Result<()> {
    // Subscribe to log messages. This can race against sub-second
    // dataflows: the coordinator drops the running-dataflow entry on
    // completion, so a LogSubscribe that arrives after the dataflow
    // finished fails with "no running dataflow with id X". That's a
    // valid state — the dataflow ran and exited — not an error for
    // the --detach caller. Warn and continue without a log stream.
    match session.subscribe_logs(
        &serde_json::to_vec(&ControlRequest::LogSubscribe {
            dataflow_id,
            level: log_level,
        })
        .wrap_err("failed to serialize message")?,
    ) {
        Ok(log_rx) => {
            std::thread::spawn(move || {
                while let Ok(Ok(raw)) = log_rx.recv() {
                    let parsed: eyre::Result<LogMessage> =
                        serde_json::from_slice(&raw).context("failed to parse log message");
                    match parsed {
                        Ok(log_message) => {
                            let config = LogOutputConfig {
                                print_daemon_name: print_daemon_id,
                                ..LogOutputConfig::default()
                            };
                            print_log_message(log_message, &config);
                        }
                        Err(err) => {
                            tracing::warn!("failed to parse log message: {err:?}")
                        }
                    }
                }
            });
        }
        Err(err) if error_indicates_dataflow_finished(&err.to_string()) => {
            tracing::debug!("dataflow {dataflow_id} completed before log subscribe arrived");
        }
        Err(err) => return Err(err).wrap_err("failed to subscribe to logs"),
    }

    // Same race on WaitForSpawn: a dataflow that exited before this
    // request landed is reported as "unknown dataflow X". The
    // DataflowStartTriggered reply we already observed is proof that
    // the coordinator accepted the spawn — treat "unknown" as evidence
    // the dataflow ran to completion, not as a startup failure.
    match send_control_request(session, &ControlRequest::WaitForSpawn { dataflow_id }) {
        Ok(reply) => {
            let uuid = expect_reply!(reply, DataflowSpawned { uuid })?;
            println!("dataflow started: {uuid}");
        }
        Err(err) if error_indicates_dataflow_finished(&err.to_string()) => {
            println!("dataflow started and finished: {dataflow_id}");
        }
        Err(err) => {
            return Err(err).wrap_err(
                "dataflow failed to start\n\n  \
                 hint: if nodes require building, run `dora build <dataflow.yml>` first",
            );
        }
    }
    Ok(())
}

/// Whether a cached **local** build makes a **distributed** `dora start`
/// unusable.
///
/// A local `dora build` records `local_build`; its artifacts live only on this
/// machine. A distributed dataflow (one with `deploy` sections) is started
/// through the coordinator and runs on remote daemons that cannot see that
/// build. The daemon resolves build metadata by id (`self.builds.get(id)`), so
/// the local id is unknown there and behaves exactly like no build: git-source
/// nodes fail with "no `dora build` was run yet" and other nodes silently spawn
/// with default working dirs/env. `dora start` does not build, so it cannot
/// recover — refuse early and tell the user to run a distributed `dora build`
/// (#1955). Local builds for non-distributed dataflows are fine and allowed.
fn local_build_blocks_distributed_start(has_local_build: bool, has_deploy_nodes: bool) -> bool {
    has_local_build && has_deploy_nodes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn local_build_blocks_distributed_start_only_when_deployed_and_local() {
        // local build on a `deploy` dataflow: remote daemons can't use it -> block.
        assert!(local_build_blocks_distributed_start(true, true));
        // distributed build (no local_build): the coordinator knows the id -> allow.
        assert!(!local_build_blocks_distributed_start(false, true));
        // single-machine dataflow with a local build: don't regress -> allow.
        assert!(!local_build_blocks_distributed_start(true, false));
        // nothing built: nothing to block -> allow.
        assert!(!local_build_blocks_distributed_start(false, false));
    }
}
