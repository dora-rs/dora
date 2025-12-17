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
    /// Start coordinator and daemon
    Start(Start),
    /// Stop all dataflows, coordinator and daemon
    Stop(Stop),
    /// Show system status
    Status(Status),
}

impl Executable for System {
    fn execute(self) -> eyre::Result<()> {
        match self {
            System::Start(args) => args.execute(),
            System::Stop(args) => args.execute(),
            System::Status(args) => args.execute(),
        }
    }
}
