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
    let dataflow_session =
        DataflowSession::read_session(&dataflow).context("failed to read DataflowSession")?;

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
