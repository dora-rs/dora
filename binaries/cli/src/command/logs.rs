use std::{
    io::Write,
    net::{SocketAddr, TcpStream},
};

use super::{default_tracing, Executable};
use crate::{common::{connect_to_coordinator, query_running_dataflows}, output::print_log_message};
use clap::Args;
use communication_layer_request_reply::{TcpConnection, TcpRequestReplyConnection};
use dora_core::topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST};
use dora_message::{cli_to_coordinator::ControlRequest, common::LogMessage, coordinator_to_cli::ControlRequestReply};
use eyre::{bail, Context, Result};
use uuid::Uuid;

#[derive(Debug, Args)]
/// Show logs of a given dataflow and node.
pub struct LogsArgs {
    /// Identifier of the dataflow
    #[clap(value_name = "UUID_OR_NAME")]
    pub dataflow: Option<String>,
    /// Show logs for the given node
    #[clap(value_name = "NAME")]
    pub node: String,
    /// Number of lines to show from the end of the logs
    ///
    /// Default (`0`) is to show all lines.
    #[clap(long, short = 'n', default_value_t = 0)]
    pub tail: usize,
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
        let list =
            query_running_dataflows(&mut *session).wrap_err("failed to query running dataflows")?;
        let (uuid, name) = match self.dataflow.as_ref().map(|it| Uuid::parse_str(it)) {
            Some(Ok(uuid)) => (Some(uuid), None),
            Some(Err(_)) => (None, self.dataflow.clone()),
            None => {
                let active = list.get_active();
                let uuid = match &active[..] {
                    [] => bail!("No dataflows are running"),
                    [uuid] => uuid.uuid,
                    _ => {
                        let uuid = inquire::Select::new("Choose dataflow to show logs:", active)
                            .prompt()?;
                        uuid.uuid
                    }
                };
                (Some(uuid), None)
            }
        };
        logs(
            &mut *session,
            uuid,
            name,
            self.node,
            self.tail,
            self.follow,
            (self.coordinator_addr, self.coordinator_port).into(),
        )
    }
}

pub fn logs(
    session: &mut TcpRequestReplyConnection,
    uuid: Option<Uuid>,
    name: Option<String>,
    node: String,
    tail: usize,
    follow: bool,
    coordinator_addr: SocketAddr,
) -> Result<()> {
    let (uuid, logs) = {
        let reply_raw = session
            .request(
                &serde_json::to_vec(&ControlRequest::Logs {
                    uuid,
                    name,
                    node: node.clone(),
                    tail,
                })
                .wrap_err("")?,
            )
            .wrap_err("failed to send Logs request message")?;

        let reply = serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
        match reply {
            ControlRequestReply::Logs { uuid, data } => (uuid, data),
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
