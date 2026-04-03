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
    output::{abort_log_task_with_grace, subscribe_and_print_logs},
    session::DataflowSession,
};
use dora_core::{
    descriptor::{Descriptor, DescriptorExt},
    topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST},
};
use dora_message::{
    cli_to_coordinator::{CoordinatorControlClient, StartRequest},
    tarpc,
};
use eyre::Context;
use std::{
    net::{IpAddr, SocketAddr},
    path::Path,
};
use tokio::task::JoinHandle;
use uuid::{NoContext, Timestamp, Uuid};

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

        // Generate the dataflow ID on the CLI side so we can subscribe to
        // zenoh log messages *before* triggering the start RPC, ensuring
        // no early log messages are missed.
        let dataflow_id = Uuid::new_v7(Timestamp::now(NoContext));

        let attach = match (self.attach, self.detach) {
            (true, true) => eyre::bail!("both `--attach` and `--detach` are given"),
            (true, false) => true,
            (false, true) => false,
            (false, false) => {
                println!("attaching to dataflow (use `--detach` to run in background)");
                true
            }
        };

        let dataflow_path = resolve_dataflow(self.dataflow)
            .await
            .context("could not resolve dataflow")?;
        let dataflow_descriptor =
            Descriptor::blocking_read(&dataflow_path).wrap_err("Failed to read yaml dataflow")?;
        let dataflow_session = DataflowSession::read_session(&dataflow_path)
            .context("failed to read DataflowSession")?;
        let client = connect_and_check_version(coordinator_socket.ip(), coordinator_socket.port())
            .await
            .wrap_err("failed to connect to dora coordinator")?;

        let print_daemon_name = dataflow_descriptor.nodes.iter().any(|n| n.deploy.is_some());
        let log_level = if attach {
            env_logger::Builder::new()
                .filter_level(log::LevelFilter::Info)
                .parse_default_env()
                .build()
                .filter()
        } else {
            log::LevelFilter::Info
        };

        // Open the zenoh session and subscribe to logs *before* the start
        // RPC so that no early log messages are missed.
        let zenoh_session = dora_core::topics::open_zenoh_session(Some(self.coordinator_addr))
            .await
            .wrap_err("failed to open zenoh session for log subscription")?;
        let base_log_topic = dora_core::topics::zenoh_log_base_topic_for_dataflow(dataflow_id);
        let log_task = subscribe_and_print_logs(
            &zenoh_session,
            &base_log_topic,
            log_level,
            false,
            print_daemon_name,
        )
        .await?;

        // Now send the start RPC — the subscription is already active.
        send_start_rpc(
            &dataflow_path,
            &dataflow_descriptor,
            &dataflow_session,
            &client,
            self.name,
            self.uv,
            dataflow_id,
        )
        .await?;

        let result = if attach {
            attach_dataflow(
                dataflow_descriptor,
                dataflow_path,
                dataflow_id,
                &client,
                self.hot_reload,
                log_task,
            )
            .await
        } else {
            wait_until_dataflow_started(dataflow_id, &client, log_task).await
        };
        // Close the zenoh session explicitly for clean shutdown.
        // Note: zenoh 1.8.0 may still emit "Unable to publish transport
        // event: session closed" — see https://github.com/eclipse-zenoh/zenoh/issues/2492
        let _ = zenoh_session.close().await;
        result
    }
}

/// Send the start RPC to the coordinator.  The caller should have already
/// subscribed to zenoh log topics before calling this.
async fn send_start_rpc(
    dataflow: &Path,
    dataflow_descriptor: &Descriptor,
    dataflow_session: &DataflowSession,
    client: &CoordinatorControlClient,
    name: Option<String>,
    uv: bool,
    dataflow_id: Uuid,
) -> Result<(), eyre::Error> {
    let local_working_dir = local_working_dir(dataflow, dataflow_descriptor, client).await?;

    let returned_id = rpc(
        "start dataflow",
        client.start(
            tarpc::context::current(),
            StartRequest {
                dataflow_id: Some(dataflow_id),
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
    eprintln!("dataflow start triggered: {returned_id}");

    Ok(())
}

async fn wait_until_dataflow_started(
    dataflow_id: Uuid,
    client: &CoordinatorControlClient,
    log_task: JoinHandle<()>,
) -> eyre::Result<()> {
    let result = rpc(
        "wait for dataflow spawn",
        client.wait_for_spawn(long_context(), dataflow_id),
    )
    .await;
    abort_log_task_with_grace(log_task).await;
    result?;
    eprintln!("dataflow started: {dataflow_id}");

    Ok(())
}
