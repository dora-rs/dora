use crate::command::Executable;

mod info;
mod list;

pub use info::Info;
pub use list::List;

/// Manage and inspect dataflow nodes.
#[derive(Debug, clap::Subcommand)]
pub enum Node {
    Info(Info),
    List(List),
}

impl Executable for Node {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Node::Info(cmd) => cmd.execute(),
            Node::List(cmd) => cmd.execute(),
        }
    }
}
