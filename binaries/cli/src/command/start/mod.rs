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
use dora_message::{cli_to_coordinator::ControlRequest, common::LogMessage};
use eyre::Context;
use std::{io::IsTerminal, net::SocketAddr, path::PathBuf};
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
}

impl Executable for Start {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let coordinator_socket = self.coordinator.socket_addr();

        let (dataflow, dataflow_descriptor, session, dataflow_id) = start_dataflow(
            self.dataflow,
            self.name,
            coordinator_socket,
            self.uv,
            self.debug,
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
                 hint: check the file exists and is valid YAML",
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
