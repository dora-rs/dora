use adora_message::id::{DataId, NodeId};
use crate::command::Executable;

mod add;
mod connect;
mod disconnect;

/// Parse "node_id/port_id" format used by connect/disconnect commands.
pub(crate) fn parse_node_port(s: &str) -> eyre::Result<(NodeId, DataId)> {
    let parts: Vec<&str> = s.splitn(2, '/').collect();
    if parts.len() != 2 || parts[0].is_empty() || parts[1].is_empty() {
        eyre::bail!("expected 'node_id/port_id', got '{s}'");
    }
    let node: NodeId = parts[0].parse().map_err(|e| eyre::eyre!("invalid node ID: {e}"))?;
    let port: DataId = parts[1].parse().map_err(|e| eyre::eyre!("invalid port ID: {e}"))?;
    Ok((node, port))
}
mod info;
mod list;
mod remove;
mod restart;
mod stop;

pub use add::Add;
pub use connect::Connect;
pub use disconnect::Disconnect;
pub use info::Info;
pub use list::List;
pub use remove::Remove;
pub use restart::Restart;
pub use stop::Stop;

/// Manage and inspect dataflow nodes.
#[derive(Debug, clap::Subcommand)]
pub enum Node {
    List(List),
    Info(Info),
    Add(Add),
    Remove(Remove),
    Connect(Connect),
    Disconnect(Disconnect),
    Restart(Restart),
    Stop(Stop),
}

impl Executable for Node {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Node::List(cmd) => cmd.execute(),
            Node::Info(cmd) => cmd.execute(),
            Node::Add(cmd) => cmd.execute(),
            Node::Remove(cmd) => cmd.execute(),
            Node::Connect(cmd) => cmd.execute(),
            Node::Disconnect(cmd) => cmd.execute(),
            Node::Restart(cmd) => cmd.execute(),
            Node::Stop(cmd) => cmd.execute(),
        }
    }
}
