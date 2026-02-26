//! The `dora start` command is used to spawn a dataflow in a pre-existing _dora network_. To create a dora network, spawn a `dora coordinator` and one or multiple `dora daemon` instances.
//!
//! The `dora start` command does not run any build commands, nor update git dependencies or similar. Use `dora build` for that.

use super::{Executable, default_tracing};
use crate::tcp::AsyncTcpConnection;
use crate::{
    command::start::attach::attach_dataflow,
    common::{
        connect_to_coordinator_rpc, local_working_dir, long_context, resolve_dataflow, rpc,
        write_events_to,
    },
    output::print_log_message,
    session::DataflowSession,
};
use dora_core::{
    descriptor::{Descriptor, DescriptorExt},
    topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT,
};
use dora_message::{
    cli_to_coordinator::{CliControlClient, LegacyControlRequest, StartRequest},
    common::LogMessage,
    tarpc,
};
use eyre::Context;
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
    #[clap(long)]
    name: Option<String>,
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP")]
    coordinator_addr: Option<IpAddr>,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT")]
    coordinator_port: Option<u16>,
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
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        use crate::common::resolve_coordinator_addr;
        let (addr, port) = resolve_coordinator_addr(
            self.coordinator_addr,
            self.coordinator_port,
            DORA_COORDINATOR_PORT_CONTROL_DEFAULT,
        );
        let coordinator_socket = (addr, port).into();

        let (dataflow, dataflow_descriptor, client, dataflow_id) =
            start_dataflow(self.dataflow, self.name, coordinator_socket, self.uv).await?;

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
                &client,
                self.hot_reload,
                coordinator_socket,
                log_level,
            )
            .await
        } else {
            let print_daemon_name = dataflow_descriptor.nodes.iter().any(|n| n.deploy.is_some());
            // wait until dataflow is started
            wait_until_dataflow_started(
                dataflow_id,
                &client,
                coordinator_socket,
                log::LevelFilter::Info,
                print_daemon_name,
            )
            .await
        }
    }
}

async fn start_dataflow(
    dataflow: String,
    name: Option<String>,
    coordinator_socket: SocketAddr,
    uv: bool,
) -> Result<(PathBuf, Descriptor, CliControlClient, Uuid), eyre::Error> {
    let dataflow = resolve_dataflow(dataflow)
        .await
        .context("could not resolve dataflow")?;
    let dataflow_descriptor =
        Descriptor::blocking_read(&dataflow).wrap_err("Failed to read yaml dataflow")?;
    let dataflow_session =
        DataflowSession::read_session(&dataflow).context("failed to read DataflowSession")?;

    let client = connect_to_coordinator_rpc(coordinator_socket.ip(), coordinator_socket.port())
        .await
        .wrap_err("failed to connect to dora coordinator")?;

    let local_working_dir = local_working_dir(&dataflow, &dataflow_descriptor, &client).await?;

    let dataflow_id = rpc(
        "start dataflow",
        client.start(
            tarpc::context::current(),
            StartRequest {
                build_id: dataflow_session.build_id,
                session_id: dataflow_session.session_id,
                dataflow: dataflow_descriptor.clone(),
                name,
                local_working_dir,
                uv,
                write_events_to: write_events_to(),
            },
        ),
    )
    .await?;
    eprintln!("dataflow start triggered: {dataflow_id}");

    Ok((dataflow, dataflow_descriptor, client, dataflow_id))
}

async fn wait_until_dataflow_started(
    dataflow_id: Uuid,
    client: &CliControlClient,
    coordinator_addr: SocketAddr,
    log_level: log::LevelFilter,
    print_daemon_id: bool,
) -> eyre::Result<()> {
    // subscribe to log messages (TCP streaming)
    let mut log_session = AsyncTcpConnection {
        stream: tokio::net::TcpStream::connect(coordinator_addr)
            .await
            .wrap_err("failed to connect to dora coordinator")?,
    };
    log_session
        .send(
            &serde_json::to_vec(&LegacyControlRequest::LogSubscribe {
                dataflow_id,
                level: log_level,
            })
            .wrap_err("failed to serialize message")?,
        )
        .await
        .wrap_err("failed to send log subscribe request to coordinator")?;
    tokio::spawn(async move {
        while let Ok(raw) = log_session.receive().await {
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

    rpc(
        "wait for dataflow spawn",
        client.wait_for_spawn(long_context(), dataflow_id),
    )
    .await?;
    eprintln!("dataflow started: {dataflow_id}");

    Ok(())
}
