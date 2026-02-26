use std::{io::Write, net::SocketAddr};

use super::{Executable, default_tracing};
use crate::{
    common::{
        connect_to_coordinator_rpc, long_context, resolve_dataflow_identifier_interactive, rpc,
    },
    output::print_log_message,
    tcp::AsyncTcpConnection,
};
use clap::Args;
use dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use dora_message::{
    cli_to_coordinator::{CliControlClient, LegacyControlRequest},
    common::LogMessage,
};
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
    #[clap(long, value_name = "IP")]
    pub coordinator_addr: Option<std::net::IpAddr>,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT")]
    pub coordinator_port: Option<u16>,
}

impl Executable for LogsArgs {
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        // Resolve coordinator address and port from CLI args, config, or defaults
        use crate::common::resolve_coordinator_addr;
        let (addr, port) = resolve_coordinator_addr(
            self.coordinator_addr,
            self.coordinator_port,
            DORA_COORDINATOR_PORT_CONTROL_DEFAULT,
        );

        let client = connect_to_coordinator_rpc(addr, port)
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
            (addr, port).into(),
        )
        .await
    }
}

pub async fn logs(
    client: &CliControlClient,
    uuid: Uuid,
    node: dora_message::id::NodeId,
    tail: Option<usize>,
    follow: bool,
    coordinator_addr: SocketAddr,
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

    // subscribe to log messages
    let mut log_session = AsyncTcpConnection {
        stream: tokio::net::TcpStream::connect(coordinator_addr)
            .await
            .wrap_err("failed to connect to dora coordinator")?,
    };
    log_session
        .send(
            &serde_json::to_vec(&LegacyControlRequest::LogSubscribe {
                dataflow_id: uuid,
                level: log_level,
            })
            .wrap_err("failed to serialize message")?,
        )
        .await
        .wrap_err("failed to send log subscribe request to coordinator")?;
    while let Ok(raw) = log_session.receive().await {
        let parsed: eyre::Result<LogMessage> =
            serde_json::from_slice(&raw).context("failed to parse log message");
        match parsed {
            Ok(log_message) => {
                print_log_message(log_message, false, false);
            }
            Err(err) => {
                tracing::warn!("failed to parse log message: {err:?}")
            }
        }
    }

    Ok(())
}
