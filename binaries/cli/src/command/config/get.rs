use clap::Args;

use super::config_struct::DoraConfig;
use crate::command::Executable;

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
    async fn execute(self) -> eyre::Result<()> {
        let config = DoraConfig::load()?;
        let value = config.get_value(&self.key)?;
        println!("{}", value);
        Ok(())
    }
}
