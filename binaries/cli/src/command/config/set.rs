use clap::Args;
use colored::Colorize;

use super::config_struct::DoraConfig;
use crate::command::Executable;

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
    async fn execute(self) -> eyre::Result<()> {
        // Load existing config
        let mut config = if self.local {
            // For local, load existing project config or start fresh
            let project_path = DoraConfig::project_config_path();
            if project_path.exists() {
                DoraConfig::load_from_file(&project_path)?
            } else {
                DoraConfig::default()
            }
        } else {
            // For global, load existing global config or start fresh
            let global_path = DoraConfig::global_config_path()?;
            if global_path.exists() {
                DoraConfig::load_from_file(&global_path)?
            } else {
                DoraConfig::default()
            }
        };

        // Set the value (this validates the input)
        config.set_value(&self.key, &self.value)?;

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

        println!(
            "Set {} = \"{}\" in {}",
            self.key.green(),
            self.value,
            location
        );
        Ok(())
    }
}
