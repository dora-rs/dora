mod export;

use crate::command::Executable;

/// Manage dataflow recordings.
#[derive(Debug, clap::Subcommand)]
pub enum Recording {
    /// Export a recording to an MCAP file
    Export(export::Export),
}

impl Executable for Recording {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Recording::Export(cmd) => cmd.execute(),
        }
    }
}
