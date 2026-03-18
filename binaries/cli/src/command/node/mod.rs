use crate::command::Executable;

mod list;
mod validate_metadata;

pub use list::List;
pub use validate_metadata::ValidateMetadata;

/// Manage and inspect dataflow nodes.
#[derive(Debug, clap::Subcommand)]
pub enum Node {
    List(List),
    ValidateMetadata(ValidateMetadata),
}

impl Executable for Node {
    async fn execute(self) -> eyre::Result<()> {
        match self {
            Node::List(cmd) => cmd.execute().await,
            Node::ValidateMetadata(cmd) => cmd.execute().await,
        }
    }
}
