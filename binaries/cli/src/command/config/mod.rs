mod config_struct;
mod get;
mod list;
mod set;
mod unset;

pub use config_struct::DoraConfig;

use clap::Subcommand;

use super::Executable;

/// Manage dora configuration
///
/// Configuration files:
///   Global: ~/.dora/config.toml
///   Project: ./dora.toml (higher priority)
///
/// Supported configuration keys:
///   coordinator.addr - Coordinator IP address
///   coordinator.port - Coordinator port number
#[derive(Debug, Subcommand)]
#[clap(verbatim_doc_comment)]
pub enum Config {
    /// List all configuration values
    List(list::List),

    /// Get a configuration value
    Get(get::Get),

    /// Set a configuration value
    Set(set::Set),

    /// Remove a configuration value
    Unset(unset::Unset),
}

impl Executable for Config {
    async fn execute(self) -> eyre::Result<()> {
        match self {
            Config::List(cmd) => cmd.execute().await,
            Config::Get(cmd) => cmd.execute().await,
            Config::Set(cmd) => cmd.execute().await,
            Config::Unset(cmd) => cmd.execute().await,
        }
    }
}
