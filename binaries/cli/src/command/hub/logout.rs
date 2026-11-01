//! `dora hub logout` — remove the locally saved registry token.

use dora_hub_client::registry;
use eyre::Context;

use crate::command::Executable;

#[derive(Debug, clap::Args)]
/// Remove the saved registry token.
pub struct Logout;

impl Executable for Logout {
    fn execute(self) -> eyre::Result<()> {
        let path = registry::token_file();
        registry::clear_saved_token().context("failed to remove the saved token")?;
        println!("logged out (removed {})", path.display());
        Ok(())
    }
}
