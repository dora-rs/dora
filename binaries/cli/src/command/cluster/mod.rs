use adora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{ControlRequestReply, DaemonInfo},
};
use eyre::{Context, bail};

use crate::{command::Executable, ws_client::WsSession};

pub mod config;
mod down;
mod status;
mod up;

/// Manage a multi-machine cluster.
#[derive(Debug, clap::Subcommand)]
pub enum Cluster {
    Up(up::Up),
    Status(status::Status),
    Down(down::Down),
}

impl Executable for Cluster {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Cluster::Up(cmd) => cmd.execute(),
            Cluster::Status(cmd) => cmd.execute(),
            Cluster::Down(cmd) => cmd.execute(),
        }
    }
}

pub(crate) fn query_connected_daemons(session: &WsSession) -> eyre::Result<Vec<DaemonInfo>> {
    let reply_raw = session
        .request(&serde_json::to_vec(&ControlRequest::ConnectedMachines).unwrap())
        .wrap_err("failed to send ConnectedMachines request")?;
    let reply: ControlRequestReply =
        serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
    match reply {
        ControlRequestReply::ConnectedDaemons(daemons) => Ok(daemons),
        ControlRequestReply::Error(err) => bail!("{err}"),
        other => bail!("unexpected reply: {other:?}"),
    }
}
