mod delete;
mod get;
mod list;
mod set;

use crate::command::Executable;

/// Manage runtime parameters on running nodes
#[derive(Debug, clap::Subcommand)]
pub enum Param {
    /// List all parameters for a node
    List(list::List),
    /// Get a parameter value
    Get(get::Get),
    /// Set a parameter value
    Set(set::Set),
    /// Delete a parameter
    Delete(delete::Delete),
}

impl Executable for Param {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Param::List(args) => args.execute(),
            Param::Get(args) => args.execute(),
            Param::Set(args) => args.execute(),
            Param::Delete(args) => args.execute(),
        }
    }
}
