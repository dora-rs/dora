use crate::command::{
    Executable,
    ros::{bridge::Bridge, import::Import, status::Status},
};

mod bridge;
mod import;
mod status;

/// ROS integration commands for message format compatibility and bridging.
#[derive(Debug, clap::Subcommand)]
pub enum Ros {
    /// Import ROS message types and generate Dora bindings
    Import(Import),
    /// Start ROS bridge node for bidirectional communication
    Bridge(Bridge),
    /// View ROS bridge status and active connections
    Status(Status),
}

impl Executable for Ros {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Ros::Import(cmd) => cmd.execute(),
            Ros::Bridge(cmd) => cmd.execute(),
            Ros::Status(cmd) => cmd.execute(),
        }
    }
}
