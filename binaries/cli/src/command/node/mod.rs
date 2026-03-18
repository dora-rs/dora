use crate::command::Executable;

mod inspect_registry;
mod list;

pub use inspect_registry::InspectRegistry;
pub use list::List;

/// Manage and inspect dataflow nodes.
#[derive(Debug, clap::Subcommand)]
pub enum Node {
    List(List),
    InspectRegistry(InspectRegistry),
}

impl Executable for Node {
    async fn execute(self) -> eyre::Result<()> {
        match self {
            Node::List(cmd) => cmd.execute().await,
            Node::InspectRegistry(cmd) => cmd.execute().await,
        }
    }
}
