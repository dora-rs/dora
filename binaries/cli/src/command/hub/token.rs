//! `dora hub token` — inspect or clear the locally saved registry token.

use dora_hub_client::registry;
use eyre::Context;

use crate::command::Executable;

#[derive(Debug, clap::Args)]
/// Show or clear the saved registry token.
pub struct Token {
    /// Print the full token (use with care).
    #[clap(long, action)]
    show: bool,
    /// Delete the saved token.
    #[clap(long, action)]
    clear: bool,
}

impl Executable for Token {
    fn execute(self) -> eyre::Result<()> {
        let path = registry::token_file();
        if self.clear {
            registry::clear_saved_token().context("failed to clear the saved token")?;
            println!("cleared {}", path.display());
            return Ok(());
        }
        match registry::read_saved_token()? {
            Some(token) => {
                if self.show {
                    println!("{token}");
                } else {
                    let prefix: String = token.chars().take(18).collect();
                    println!("{prefix}… ({} chars)", token.chars().count());
                    println!("token file: {}", path.display());
                    println!("pass --show to print the full token");
                }
            }
            None => {
                let tokens_url = format!(
                    "{}/account/tokens",
                    registry::registry_url().trim_end_matches('/')
                );
                println!("no saved token at {}", path.display());
                println!("run `dora hub login` to save a token from {tokens_url}");
            }
        }
        Ok(())
    }
}
