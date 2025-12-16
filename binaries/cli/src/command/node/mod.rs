use crate::command::Executable;

mod list;

pub use list::List;

/// Manage and inspect dataflow nodes.
#[derive(Debug, clap::Subcommand)]
pub enum Node {
    List(List),
}

impl Executable for Node {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Node::List(cmd) => cmd.execute(),
        }
    }
}
