//! `dora hub logout` — remove the locally saved registry token.

use dora_hub_client::registry;
use eyre::Context;

use crate::command::Executable;

#[derive(Debug, clap::Args)]
/// Remove the saved registry token.
pub struct Logout {
    /// Registry base URL. Defaults to DORA_HUB_REGISTRY_URL or https://dora.pub.
    #[clap(long, env = "DORA_HUB_REGISTRY_URL", value_name = "URL")]
    registry: Option<String>,
}

impl Executable for Logout {
    fn execute(self) -> eyre::Result<()> {
        let registry_url = self.registry.unwrap_or_else(registry::registry_url);
        let path = registry::token_file_for(&registry_url);
        registry::clear_saved_token_for(&registry_url)
            .context("failed to remove the saved token")?;
        println!("logged out (removed {})", path.display());
        Ok(())
    }
}
