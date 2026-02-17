pub mod start;
pub mod status;
pub mod stop;

pub use status::check_environment;

use super::Executable;
use start::Start;
use status::Status;
use stop::Stop;

/// System management commands
#[derive(Debug, clap::Subcommand)]
pub enum System {
    /// Show system status
    Status(Status),
    /// Start coordinator and daemon
    Start(Start),
    /// Stop coordinator and daemon
    Stop(Stop),
}

impl Executable for System {
    fn execute(self) -> eyre::Result<()> {
        match self {
            System::Status(args) => args.execute(),
            System::Start(args) => args.execute(),
            System::Stop(args) => args.execute(),
        }
    }
}
