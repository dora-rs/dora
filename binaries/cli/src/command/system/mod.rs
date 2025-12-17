pub mod status;

pub use status::check_environment;

use super::Executable;
use status::Status;

/// System management commands
#[derive(Debug, clap::Subcommand)]
pub enum System {
    Status(Status),
}

impl Executable for System {
    fn execute(self) -> eyre::Result<()> {
        match self {
            System::Status(args) => args.execute(),
        }
    }
}
