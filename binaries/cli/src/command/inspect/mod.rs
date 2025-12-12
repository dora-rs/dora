mod top;

use clap::Subcommand;

use super::Executable;

#[derive(Debug, Subcommand)]
pub enum Inspect {
    /// Real-time monitor node resource usage (similar to Linux top)
    Top(top::Top),
}

impl Executable for Inspect {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Inspect::Top(args) => args.execute(),
        }
    }
}
