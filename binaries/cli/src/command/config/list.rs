use clap::Args;

use crate::command::Executable;

use super::manager::ConfigManager;

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
    fn execute(self) -> eyre::Result<()> {
        let manager = ConfigManager::new()?;
        let items = manager.list()?;

        if items.is_empty() {
            println!("No configuration values set");
        } else {
            for (key, value) in items {
                println!("{} = \"{}\"", key, value);
            }
        }

        Ok(())
    }
}
