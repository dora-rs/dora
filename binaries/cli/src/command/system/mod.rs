pub mod doctor;
pub mod status;

use super::Executable;
use doctor::Doctor;
use status::Status;

/// System management commands
#[derive(Debug, clap::Subcommand)]
pub enum System {
    Doctor(Doctor),
    Status(Status),
}

impl Executable for System {
    async fn execute(self) -> eyre::Result<()> {
        match self {
            System::Doctor(args) => args.execute().await,
            System::Status(args) => args.execute().await,
        }
    }
}
