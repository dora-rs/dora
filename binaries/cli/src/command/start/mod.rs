//! The `dora start` command is used to spawn a dataflow in a pre-existing _dora network_. To create a dora network, spawn a `dora coordinator` and one or multiple `dora daemon` instances.
//!
//! The `dora start` command does not run any build commands, nor update git dependencies or similar. Use `dora build` for that.

use super::{Executable, default_tracing};
use crate::{
    command::start::attach::attach_dataflow,
    common::{connect_to_coordinator, local_working_dir, resolve_dataflow, write_events_to},
    output::print_log_message,
    session::DataflowSession,
};
use communication_layer_request_reply::{Transport, transport::FramedTransport};
use dora_core::{
    descriptor::{Descriptor, DescriptorExt},
    topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST},
};
use dora_message::{
    cli_to_coordinator::{
        CliToCoordinatorClient, CliToCoordinatorEncoding, CliToCoordinatorRequest, StartReq,
    },
    common::LogMessage,
};
use eyre::Context;
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
    dataflow: String,
    /// Assign a name to the dataflow
    #[clap(long)]
    name: Option<String>,
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    coordinator_addr: IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
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
                &mut session,
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
) -> Result<(PathBuf, Descriptor, CliToCoordinatorClient, Uuid), eyre::Error> {
    let dataflow = resolve_dataflow(dataflow).context("could not resolve dataflow")?;
    let dataflow_descriptor =
        Descriptor::blocking_read(&dataflow).wrap_err("Failed to read yaml dataflow")?;
    let dataflow_session =
        DataflowSession::read_session(&dataflow).context("failed to read DataflowSession")?;

    let mut session = connect_to_coordinator(coordinator_socket)
        .wrap_err("failed to connect to dora coordinator")?;
    let mut coordinator_client = CliToCoordinatorClient::new_tcp(coordinator_socket)?;

    let local_working_dir =
        local_working_dir(&dataflow, &dataflow_descriptor, &mut coordinator_client)?;

    let dataflow_id = session.start(StartReq {
        build_id: dataflow_session.build_id,
        session_id: dataflow_session.session_id,
        dataflow: dataflow_descriptor.clone(),
        name,
        local_working_dir,
        uv,
        write_events_to: write_events_to(),
    })?;
    Ok((dataflow, dataflow_descriptor, session, dataflow_id))
}

fn wait_until_dataflow_started(
    dataflow_id: Uuid,
    session: &mut CliToCoordinatorClient,
    coordinator_addr: SocketAddr,
    log_level: log::LevelFilter,
    print_daemon_id: bool,
) -> eyre::Result<()> {
    // subscribe to log messages
    let mut log_session = FramedTransport::new(
        TcpStream::connect(coordinator_addr).wrap_err("failed to connect to dora coordinator")?,
    )
    .with_encoding::<_, CliToCoordinatorRequest, LogMessage>(CliToCoordinatorEncoding);
    log_session
        .send(&CliToCoordinatorRequest::LogSubscribe {
            dataflow_id,
            level: log_level,
        })
        .wrap_err("failed to send log subscribe request to coordinator")?;
    std::thread::spawn(move || {
        while let Ok(Some(message)) = log_session.receive() {
            print_log_message(message, false, print_daemon_id);
        }
    });

    let uuid = session.wait_for_spawn(dataflow_id)?;
    eprintln!("dataflow started: {uuid}");
    Ok(())
}
