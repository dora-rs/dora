use clap::Args;

use crate::command::Executable;

use super::manager::ConfigManager;

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
    fn execute(self) -> eyre::Result<()> {
        let manager = ConfigManager::new()?;
        manager.unset(&self.key, self.local)?;

        let location = if self.local {
            "project config (./dora.toml)"
        } else {
            "global config (~/.dora/config.toml)"
        };

        println!("Removed {} from {}", self.key, location);
        Ok(())
    }
}
