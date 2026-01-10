use clap::Args;
use colored::Colorize;

use crate::command::Executable;

use super::manager::ConfigManager;

/// Set a configuration value
///
/// Examples:
///
/// Set coordinator address globally:
///   dora config set coordinator.addr 192.168.1.100
///
/// Set coordinator port for current project:
///   dora config set --local coordinator.port 8080
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Set {
    /// Configuration key to set
    pub key: String,

    /// Value to set
    pub value: String,

    /// Write to project config (./dora.toml) instead of global config
    #[clap(long)]
    pub local: bool,
}

impl Executable for Set {
    fn execute(self) -> eyre::Result<()> {
        let manager = ConfigManager::new()?;
        manager.set(&self.key, &self.value, self.local)?;

        let location = if self.local {
            "project config (./dora.toml)"
        } else {
            "global config (~/.dora/config.toml)"
        };

        println!("Set {} = \"{}\" in {}", self.key.green(), self.value, location);
        Ok(())
    }
}
