use clap::Args;

use crate::command::Executable;

use super::manager::ConfigManager;

/// Get a configuration value
///
/// Examples:
///
/// Get coordinator address:
///   dora config get coordinator.addr
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Get {
    /// Configuration key to retrieve
    pub key: String,
}

impl Executable for Get {
    fn execute(self) -> eyre::Result<()> {
        let manager = ConfigManager::new()?;
        let value = manager.get(&self.key)?;
        println!("{}", value);
        Ok(())
    }
}
