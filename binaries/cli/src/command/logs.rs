use std::{
    io::Write,
    net::{SocketAddr, TcpStream},
};

use super::{Executable, default_tracing};
use crate::{
    common::{connect_to_coordinator, resolve_dataflow_identifier_interactive},
    output::print_log_message,
};
use clap::Args;
use communication_layer_request_reply::{TcpConnection, TcpRequestReplyConnection};
use dora_core::topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST};
use dora_message::{
    cli_to_coordinator::ControlRequest, common::LogMessage,
    coordinator_to_cli::ControlRequestReply, id::NodeId,
};
use eyre::{Context, Result, bail};
use uuid::Uuid;

#[derive(Debug, Args)]
/// Show logs of a given dataflow and node.
pub struct LogsArgs {
    /// Identifier of the dataflow
    #[clap(value_name = "UUID_OR_NAME")]
    pub dataflow: Option<String>,
    /// Show logs for the given node
    #[clap(value_name = "NAME")]
    pub node: NodeId,
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
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        let mut session =
            connect_to_coordinator((self.coordinator_addr, self.coordinator_port).into())
                .wrap_err("failed to connect to dora coordinator")?;
        let uuid =
            resolve_dataflow_identifier_interactive(&mut *session, self.dataflow.as_deref())?;
        logs(
            &mut *session,
            uuid,
            self.node,
            self.tail,
            self.follow,
            (self.coordinator_addr, self.coordinator_port).into(),
        )
    }
}

pub fn logs(
    session: &mut TcpRequestReplyConnection,
    uuid: Uuid,
    node: NodeId,
    tail: Option<usize>,
    follow: bool,
    coordinator_addr: SocketAddr,
) -> Result<()> {
    let logs = {
        let reply_raw = session
            .request(
                &serde_json::to_vec(&ControlRequest::Logs {
                    uuid: Some(uuid),
                    name: None,
                    node: node.to_string(),
                    tail,
                })
                .wrap_err("")?,
            )
            .wrap_err("failed to send Logs request message")?;

        let reply = serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
        match reply {
            ControlRequestReply::Logs(data) => data,
            other => bail!("unexpected reply to daemon logs: {other:?}"),
        }
    };

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
    let mut log_session = TcpConnection {
        stream: TcpStream::connect(coordinator_addr)
            .wrap_err("failed to connect to dora coordinator")?,
    };
    log_session
        .send(
            &serde_json::to_vec(&ControlRequest::LogSubscribe {
                dataflow_id: uuid,
                level: log_level,
            })
            .wrap_err("failed to serialize message")?,
        )
        .wrap_err("failed to send log subscribe request to coordinator")?;
    while let Ok(raw) = log_session.receive() {
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
