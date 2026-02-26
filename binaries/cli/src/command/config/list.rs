use clap::Args;
use colored::Colorize;

use super::config_struct::DoraConfig;
use crate::command::Executable;

/// List all configuration values
///
/// Examples:
///
/// List all configuration:
///   dora config list
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct List {}

impl Executable for List {
    async fn execute(self) -> eyre::Result<()> {
        let config = DoraConfig::load()?;
        let items = config.list_values();

        if items.is_empty() {
            println!("No configuration values set");
        } else {
            for (key, value) in items {
                println!("{} = {}", key.cyan(), value);
            }
        }

        Ok(())
    }
}
