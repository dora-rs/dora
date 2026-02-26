//! The `dora start` command is used to spawn a dataflow in a pre-existing _dora network_. To create a dora network, spawn a `dora coordinator` and one or multiple `dora daemon` instances.
//!
//! The `dora start` command does not run any build commands, nor update git dependencies or similar. Use `dora build` for that.

use super::{Executable, default_tracing};
use crate::{
    command::start::attach::attach_dataflow,
    common::{
        connect_and_check_version, local_working_dir, long_context, resolve_dataflow, rpc,
        write_events_to,
    },
    output::print_log_message,
    session::DataflowSession,
};
use dora_core::{
    descriptor::{Descriptor, DescriptorExt},
    topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST, zenoh_log_topic_for_dataflow},
};
use dora_message::{
    cli_to_coordinator::{CoordinatorControlClient, StartRequest},
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
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let coordinator_socket: SocketAddr = (self.coordinator_addr, self.coordinator_port).into();

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
                self.coordinator_addr,
                log_level,
            )
            .await
        } else {
            let print_daemon_name = dataflow_descriptor.nodes.iter().any(|n| n.deploy.is_some());
            // wait until dataflow is started
            wait_until_dataflow_started(
                dataflow_id,
                &client,
                self.coordinator_addr,
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
) -> Result<(PathBuf, Descriptor, CoordinatorControlClient, Uuid), eyre::Error> {
    let dataflow = resolve_dataflow(dataflow)
        .await
        .context("could not resolve dataflow")?;
    let dataflow_descriptor =
        Descriptor::blocking_read(&dataflow).wrap_err("Failed to read yaml dataflow")?;
    let dataflow_session =
        DataflowSession::read_session(&dataflow).context("failed to read DataflowSession")?;

    let client = connect_and_check_version(coordinator_socket.ip(), coordinator_socket.port())
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
    client: &CoordinatorControlClient,
    coordinator_addr: IpAddr,
    log_level: log::LevelFilter,
    print_daemon_id: bool,
) -> eyre::Result<()> {
    // Subscribe to log messages via zenoh
    let zenoh_session = dora_core::topics::open_zenoh_session(Some(coordinator_addr))
        .await
        .wrap_err("failed to open zenoh session for log subscription")?;
    let log_topic = zenoh_log_topic_for_dataflow(dataflow_id);
    let subscriber = zenoh_session
        .declare_subscriber(&log_topic)
        .await
        .map_err(|e| eyre::eyre!(e))
        .wrap_err("failed to subscribe to log topic")?;
    tokio::spawn(async move {
        loop {
            match subscriber.recv_async().await {
                Ok(sample) => {
                    let payload = sample.payload().to_bytes();
                    let parsed: eyre::Result<LogMessage> = serde_json::from_slice(&payload)
                        .context("failed to parse log message from zenoh");
                    match parsed {
                        Ok(log_message) => {
                            if should_display(&log_message, log_level) {
                                print_log_message(log_message, false, print_daemon_id);
                            }
                        }
                        Err(err) => {
                            tracing::warn!("failed to parse log message: {err:?}")
                        }
                    }
                }
                Err(_) => break,
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

/// Check whether a log message should be displayed given the log level filter.
fn should_display(message: &LogMessage, filter: log::LevelFilter) -> bool {
    match &message.level {
        dora_core::build::LogLevelOrStdout::Stdout => true,
        dora_core::build::LogLevelOrStdout::LogLevel(level) => *level <= filter,
    }
}
