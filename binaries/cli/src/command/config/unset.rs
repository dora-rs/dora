use clap::Args;
use colored::Colorize;

use super::config_struct::DoraConfig;
use crate::command::Executable;

/// Remove a configuration value
///
/// Examples:
///
/// Remove coordinator address from global config:
///   dora config unset coordinator.addr
///
/// Remove coordinator port from project config:
///   dora config unset --local coordinator.port
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Unset {
    /// Configuration key to remove
    pub key: String,

    /// Remove from project config (./dora.toml) instead of global config
    #[clap(long)]
    pub local: bool,
}

impl Executable for Unset {
    async fn execute(self) -> eyre::Result<()> {
        // Load the appropriate config
        let mut config = DoraConfig::load()?;

        // Unset the value
        config.unset_value(&self.key)?;

        // Save to appropriate location
        if self.local {
            config.save_project()?;
        } else {
            config.save_global()?;
        }

        let location = if self.local {
            "project config (./dora.toml)"
        } else {
            "global config (~/.dora/config.toml)"
        };

        println!("Removed {} from {}", self.key.green(), location);
        Ok(())
    }
}
