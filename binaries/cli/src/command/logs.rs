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
use communication_layer_request_reply::{
    Transport, encoding::JsonEncoding, transport::FramedTransport,
};
use dora_core::topics::{DORA_COORDINATOR_PORT_CONTROL_DEFAULT, LOCALHOST};
use dora_message::{
    cli_to_coordinator::{CliToCoordinatorClient, CliToCoordinatorRequest},
    common::LogMessage,
    id::NodeId,
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
        let uuid = resolve_dataflow_identifier_interactive(&mut session, self.dataflow.as_deref())?;
        logs(
            &mut session,
            uuid,
            self.node,
            self.tail,
            self.follow,
            (self.coordinator_addr, self.coordinator_port).into(),
        )
    }
}

pub fn logs(
    session: &mut CliToCoordinatorClient,
    uuid: Uuid,
    node: NodeId,
    tail: Option<usize>,
    follow: bool,
    coordinator_addr: SocketAddr,
) -> Result<()> {
    let logs = session.logs(Some(uuid), None, node.to_string(), tail)?;

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
    let mut log_session = FramedTransport::new(
        TcpStream::connect(coordinator_addr).wrap_err("failed to connect to dora coordinator")?,
    )
    .with_encoding::<_, CliToCoordinatorRequest, LogMessage>(JsonEncoding);
    log_session.send(&CliToCoordinatorRequest::LogSubscribe {
        dataflow_id: uuid,
        level: log_level,
    })?;

    while let Ok(Some(message)) = log_session.receive() {
        print_log_message(message, false, false);
    }

    Ok(())
}
