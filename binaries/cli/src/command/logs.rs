use std::io::Write;

use chrono::Utc;

use super::{Executable, default_tracing};
use crate::{
    common::{
        connect_and_check_version, long_context, resolve_dataflow_identifier_interactive, rpc,
    },
    output::subscribe_and_print_logs,
};
use clap::Args;
use dora_core::topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST};
use dora_message::cli_to_coordinator::CoordinatorControlClient;
use eyre::{Context, Result};
use uuid::Uuid;

#[derive(Debug, Args)]
/// Show logs of a given dataflow and node.
pub struct LogsArgs {
    /// Identifier of the dataflow
    #[clap(value_name = "UUID_OR_NAME")]
    pub dataflow: Option<String>,
    /// Show logs for the given node
    #[clap(value_name = "NAME")]
    pub node: dora_message::id::NodeId,
    /// Number of lines to show from the end of the logs
    #[clap(long, short = 'n')]
    pub tail: Option<usize>,
    /// Follow log output
    #[clap(long, short)]
    pub follow: bool,
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    pub coordinator_addr: std::net::IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    pub coordinator_port: u16,
}

impl Executable for LogsArgs {
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let client = connect_and_check_version(self.coordinator_addr, self.coordinator_port)
            .await
            .wrap_err("failed to connect to dora coordinator")?;
        let uuid =
            resolve_dataflow_identifier_interactive(&client, self.dataflow.as_deref()).await?;
        logs(
            &client,
            uuid,
            self.node,
            self.tail,
            self.follow,
            self.coordinator_addr,
        )
        .await
    }
}

pub async fn logs(
    client: &CoordinatorControlClient,
    uuid: Uuid,
    node: dora_message::id::NodeId,
    tail: Option<usize>,
    follow: bool,
    coordinator_addr: std::net::IpAddr,
) -> Result<()> {
    // When following, subscribe to zenoh *before* fetching historical logs
    // so that messages published during the RPC are buffered.  We record
    // the current time as a dedup cutoff: the subscriber will silently
    // drop messages with a timestamp before this point, since they are
    // already covered by the historical fetch.
    let log_task = if follow {
        let log_level = env_logger::Builder::new()
            .filter_level(log::LevelFilter::Info)
            .parse_default_env()
            .build()
            .filter();

        let zenoh_session = dora_core::topics::open_zenoh_session(Some(coordinator_addr))
            .await
            .wrap_err("failed to open zenoh session for log subscription")?;
        let base_topic = dora_core::topics::zenoh_log_base_topic_for_dataflow_node(uuid, &node);
        let drop_before = Utc::now();
        Some(
            subscribe_and_print_logs(
                &zenoh_session,
                &base_topic,
                log_level,
                false,
                false,
                Some(drop_before),
            )
            .await?,
        )
    } else {
        None
    };

    let logs = rpc(
        "retrieve logs",
        client.logs(long_context(), Some(uuid), None, node.to_string(), tail),
    )
    .await?;

    std::io::stdout()
        .write_all(&logs)
        .expect("failed to write logs to stdout");

    if let Some(log_task) = log_task {
        // Block until the task ends (subscriber closes).
        let _ = log_task.await;
    }

    Ok(())
}
