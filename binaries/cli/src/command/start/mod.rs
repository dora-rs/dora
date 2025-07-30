//! The `dora start` command is used to spawn a dataflow in a pre-existing _dora network_. To create a dora network, spawn a `dora coordinator` and one or multiple `dora daemon` instances.
//!
//! The `dora start` command does not run any build commands, nor update git dependencies or similar. Use `dora build` for that.

use super::{default_tracing, Executable};
use crate::{
    command::start::attach::attach_dataflow,
    common::{connect_to_coordinator, local_working_dir, resolve_dataflow},
    output::print_log_message,
    session::DataflowSession,
};
use communication_layer_request_reply::{TcpConnection, TcpRequestReplyConnection};
use dora_core::{
    descriptor::{Descriptor, DescriptorExt},
    topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST},
};
use dora_message::{
    cli_to_coordinator::ControlRequest, common::LogMessage, coordinator_to_cli::ControlRequestReply,
};
use eyre::{bail, Context};
use std::{
    net::{IpAddr, SocketAddr, TcpStream},
    path::PathBuf,
};
use uuid::Uuid;

mod attach;

#[derive(Debug, clap::Args)]
/// Start the given dataflow path. Attach a name to the running dataflow by using --name.
pub struct Start {
    /// Path to the dataflow descriptor file
    #[clap(value_name = "PATH")]
    pub dataflow: String,
    /// Assign a name to the dataflow
    #[clap(long)]
    pub name: Option<String>,
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    pub coordinator_addr: IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    pub coordinator_port: u16,
    /// Attach to the dataflow and wait for its completion
    #[clap(long, action)]
    pub attach: bool,
    /// Run the dataflow in background
    #[clap(long, action)]
    pub detach: bool,
    /// Enable hot reloading (Python only)
    #[clap(long, action)]
    pub hot_reload: bool,
    // Use UV to run nodes.
    #[clap(long, action)]
    pub uv: bool,
}

impl Executable for Start {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let coordinator_socket = (self.coordinator_addr, self.coordinator_port).into();

        let (dataflow, dataflow_descriptor, mut session, dataflow_id) =
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
                &mut *session,
                self.hot_reload,
                coordinator_socket,
                log_level,
            )
        } else {
            let print_daemon_name = dataflow_descriptor.nodes.iter().any(|n| n.deploy.is_some());
            // wait until dataflow is started
            wait_until_dataflow_started(
                dataflow_id,
                &mut session,
                coordinator_socket,
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
) -> Result<(PathBuf, Descriptor, Box<TcpRequestReplyConnection>, Uuid), eyre::Error> {
    let dataflow = resolve_dataflow(dataflow).context("could not resolve dataflow")?;
    let dataflow_descriptor =
        Descriptor::blocking_read(&dataflow).wrap_err("Failed to read yaml dataflow")?;
    let dataflow_session =
        DataflowSession::read_session(&dataflow).context("failed to read DataflowSession")?;

    let mut session = connect_to_coordinator(coordinator_socket)
        .wrap_err("failed to connect to dora coordinator")?;

    let local_working_dir = local_working_dir(&dataflow, &dataflow_descriptor, &mut *session)?;

    let dataflow_id = {
        let dataflow = dataflow_descriptor.clone();
        let session: &mut TcpRequestReplyConnection = &mut *session;
        let reply_raw = session
            .request(
                &serde_json::to_vec(&ControlRequest::Start {
                    build_id: dataflow_session.build_id,
                    session_id: dataflow_session.session_id,
                    dataflow,
                    name,
                    local_working_dir,
                    uv,
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
    session: &mut Box<TcpRequestReplyConnection>,
    coordinator_addr: SocketAddr,
    log_level: log::LevelFilter,
    print_daemon_id: bool,
) -> eyre::Result<()> {
    // subscribe to log messages
    let mut log_session = TcpConnection {
        stream: TcpStream::connect(coordinator_addr)
            .wrap_err("failed to connect to dora coordinator")?,
    };
    log_session
        .send(
            &serde_json::to_vec(&ControlRequest::LogSubscribe {
                dataflow_id,
                level: log_level,
            })
            .wrap_err("failed to serialize message")?,
        )
        .wrap_err("failed to send log subscribe request to coordinator")?;
    std::thread::spawn(move || {
        while let Ok(raw) = log_session.receive() {
            let parsed: eyre::Result<LogMessage> =
                serde_json::from_slice(&raw).context("failed to parse log message");
            match parsed {
                Ok(log_message) => {
                    print_log_message(log_message, false, print_daemon_id);
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
