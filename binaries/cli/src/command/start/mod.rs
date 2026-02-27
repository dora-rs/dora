//! The `adora start` command is used to spawn a dataflow in a pre-existing _adora network_. To create a adora network, spawn a `adora coordinator` and one or multiple `adora daemon` instances.
//!
//! The `adora start` command does not run any build commands, nor update git dependencies or similar. Use `adora build` for that.

use super::{Executable, default_tracing};
use crate::{
    command::start::attach::attach_dataflow,
    common::{connect_to_coordinator, local_working_dir, resolve_dataflow, write_events_to},
    output::{LogOutputConfig, print_log_message},
    session::DataflowSession,
    ws_client::WsSession,
};
use adora_core::{
    descriptor::{Descriptor, DescriptorExt},
    topics::{ADORA_COORDINATOR_PORT_WS_DEFAULT, LOCALHOST},
};
use adora_message::{
    cli_to_coordinator::ControlRequest, common::LogMessage, coordinator_to_cli::ControlRequestReply,
};
use eyre::{Context, bail};
use std::{
    net::{IpAddr, SocketAddr},
    path::PathBuf,
};
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
    /// Address of the adora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST, env = "ADORA_COORDINATOR_ADDR")]
    coordinator_addr: IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = ADORA_COORDINATOR_PORT_WS_DEFAULT, env = "ADORA_COORDINATOR_PORT")]
    coordinator_port: u16,
    /// Attach to the dataflow and wait for its completion
    #[clap(long, action)]
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
}

impl Executable for Start {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let coordinator_socket = (self.coordinator_addr, self.coordinator_port).into();

        let (dataflow, dataflow_descriptor, session, dataflow_id) =
            start_dataflow(self.dataflow, self.name, coordinator_socket, self.uv)?;

        let attach = match (self.attach, self.detach) {
            (true, true) => eyre::bail!("both `--attach` and `--detach` are given"),
            (true, false) => true,
            (false, true) => false,
            (false, false) => {
                println!("attaching to dataflow (use `--detach` to run in background)");
                true
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
) -> Result<(PathBuf, Descriptor, WsSession, Uuid), eyre::Error> {
    let dataflow = resolve_dataflow(dataflow).context("could not resolve dataflow")?;
    let dataflow_descriptor =
        Descriptor::blocking_read(&dataflow).wrap_err("Failed to read yaml dataflow")?;
    let dataflow_session =
        DataflowSession::read_session(&dataflow).context("failed to read DataflowSession")?;

    let session = connect_to_coordinator(coordinator_socket)
        .wrap_err("failed to connect to adora coordinator")?;

    let local_working_dir = local_working_dir(&dataflow, &dataflow_descriptor, &session)?;

    let dataflow_id = {
        let dataflow = dataflow_descriptor.clone();
        let reply_raw = session
            .request(
                &serde_json::to_vec(&ControlRequest::Start {
                    build_id: dataflow_session.build_id,
                    session_id: dataflow_session.session_id,
                    dataflow,
                    name,
                    local_working_dir,
                    uv,
                    write_events_to: write_events_to(),
                })
                .unwrap(),
            )
            .wrap_err("failed to send start dataflow message")?;

        let result: ControlRequestReply =
            serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
        match result {
            ControlRequestReply::DataflowStartTriggered { uuid } => {
                eprintln!("dataflow start triggered: {uuid}");
                uuid
            }
            ControlRequestReply::Error(err) => bail!("{err}"),
            other => bail!("unexpected start dataflow reply: {other:?}"),
        }
    };
    Ok((dataflow, dataflow_descriptor, session, dataflow_id))
}

fn wait_until_dataflow_started(
    dataflow_id: Uuid,
    session: &WsSession,
    log_level: log::LevelFilter,
    print_daemon_id: bool,
) -> eyre::Result<()> {
    // subscribe to log messages
    let log_rx = session.subscribe_logs(
        &serde_json::to_vec(&ControlRequest::LogSubscribe {
            dataflow_id,
            level: log_level,
        })
        .wrap_err("failed to serialize message")?,
    )?;
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

    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::WaitForSpawn { dataflow_id }).unwrap())
        .wrap_err("failed to send start dataflow message")?;

    let result: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match result {
        ControlRequestReply::DataflowSpawned { uuid } => {
            eprintln!("dataflow started: {uuid}");
        }
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected start dataflow reply: {other:?}"),
    }
    Ok(())
}
