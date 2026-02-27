use std::io::Write;

use super::{Executable, default_tracing};
use crate::{
    common::{
        connect_and_check_version, long_context, resolve_dataflow_identifier_interactive, rpc,
    },
    output::print_log_message,
};
use clap::Args;
use dora_core::topics::{
    DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST, zenoh_log_subscribe_all_for_dataflow,
};
use dora_message::{cli_to_coordinator::CoordinatorControlClient, common::LogMessage};
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
    let logs = rpc(
        "retrieve logs",
        client.logs(long_context(), Some(uuid), None, node.to_string(), tail),
    )
    .await?;

    std::io::stdout()
        .write_all(&logs)
        .expect("failed to write logs to stdout");

    if !follow {
        return Ok(());
    }
    let log_level = env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .build()
        .filter();

    // Subscribe to log messages via zenoh
    let zenoh_session = dora_core::topics::open_zenoh_session(Some(coordinator_addr))
        .await
        .wrap_err("failed to open zenoh session for log subscription")?;
    let log_topic = zenoh_log_subscribe_all_for_dataflow(uuid);
    let subscriber = zenoh_session
        .declare_subscriber(&log_topic)
        .await
        .map_err(|e| eyre::eyre!(e))
        .wrap_err("failed to subscribe to log topic")?;
    loop {
        match subscriber.recv_async().await {
            Ok(sample) => {
                let payload = sample.payload().to_bytes();
                let parsed: eyre::Result<LogMessage> = serde_json::from_slice(&payload)
                    .context("failed to parse log message from zenoh");
                match parsed {
                    Ok(log_message) => {
                        if should_display(&log_message, log_level) {
                            print_log_message(log_message, false, false);
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

    Ok(())
}

/// Check whether a log message should be displayed given the log level filter.
fn should_display(message: &LogMessage, filter: log::LevelFilter) -> bool {
    match &message.level {
        dora_core::build::LogLevelOrStdout::Stdout => true,
        dora_core::build::LogLevelOrStdout::LogLevel(level) => *level <= filter,
    }
}
