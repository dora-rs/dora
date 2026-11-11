//! `dora hub login` — save a registry API token created on the Hub website.
//!
//! This follows `cargo login` rather than an in-CLI username/password sign-in:
//! you create a scoped token at <https://dora.pub/account/tokens>, paste it at
//! the prompt (or pass it as an argument), and the CLI stores it. The token is
//! written to `~/.config/dora/hub-token` (0600, shared with the standalone
//! `dora-hub` client).
//!
//! The registry defaults to <https://dora.pub>; `DORA_HUB_REGISTRY_URL`
//! overrides it (local tests, mirrors). It selects which website the prompt
//! points at; the token file itself is registry-independent, exactly as before.

use std::io::{self, BufRead, IsTerminal};

use dora_hub_client::registry;
use eyre::{Context, bail};

use crate::command::Executable;

#[derive(Debug, clap::Args)]
/// Save a registry API token created on the Hub website.
pub struct Login {
    /// Registry base URL. Defaults to DORA_HUB_REGISTRY_URL or https://dora.pub.
    #[clap(long, env = "DORA_HUB_REGISTRY_URL", value_name = "URL")]
    registry: Option<String>,
    /// API token created on the Hub website (default:
    /// https://dora.pub/account/tokens). Omit to paste it at the prompt, which
    /// keeps it out of shell history.
    #[clap(value_name = "TOKEN")]
    token: Option<String>,
}

impl Executable for Login {
    fn execute(self) -> eyre::Result<()> {
        let registry_url = self.registry.unwrap_or_else(registry::registry_url);
        let tokens_url = format!("{}/account/tokens", registry_url.trim_end_matches('/'));

        let token = match self.token {
            Some(token) => token,
            None => prompt_for_token(&tokens_url)?,
        };
        let token = token.trim();
        if token.is_empty() {
            bail!("the token must not be empty\n  create one at {tokens_url}");
        }

        let path = registry::save_token(token).context("failed to save the token")?;
        println!("token saved to {}", path.display());
        Ok(())
    }
}

/// Ask for the token on a terminal (hidden input), or read one line from stdin
/// so `echo "$DORA_TOKEN" | dora hub login` works from scripts.
fn prompt_for_token(tokens_url: &str) -> eyre::Result<String> {
    println!("please paste the token found on {tokens_url} below");
    if io::stdin().is_terminal() {
        return inquire::Password::new("Token")
            .without_confirmation()
            .prompt()
            .context("token prompt failed");
    }

    let mut line = String::new();
    io::stdin()
        .lock()
        .read_line(&mut line)
        .context("failed to read the token from stdin")?;
    Ok(line)
}
