//! `dora hub login` — sign in to the HTTP registry and store a publish token.
//!
//! The registry defaults to <https://dora.pub>; `DORA_HUB_REGISTRY_URL`
//! overrides it (local tests, mirrors). The token is written to
//! `~/.config/dora/hub-token` (0600, shared with the standalone `dora-hub`
//! client) unless `--no-save` is given.

use dora_hub_client::registry::{self, RegistryClient};
use eyre::Context;

use crate::command::Executable;

#[derive(Debug, clap::Args)]
/// Sign in to a registry and create a publish token.
pub struct Login {
    /// Registry base URL. Defaults to DORA_HUB_REGISTRY_URL or https://dora.pub.
    #[clap(long, env = "DORA_HUB_REGISTRY_URL", value_name = "URL")]
    registry: Option<String>,
    /// Registry login (account name).
    #[clap(long, env = "DORA_HUB_USERNAME", value_name = "LOGIN")]
    username: Option<String>,
    /// Password; prompted for if omitted.
    #[clap(
        long,
        env = "DORA_HUB_PASSWORD",
        value_name = "PASSWORD",
        hide_env_values = true
    )]
    password: Option<String>,
    /// Name recorded for the created token.
    #[clap(long, default_value = "cli", value_name = "NAME")]
    token_name: String,
    /// Comma-separated scopes for the token.
    #[clap(
        long,
        value_delimiter = ',',
        default_value = "publish-new,publish-update",
        value_name = "SCOPE"
    )]
    scopes: Vec<String>,
    /// Print the token instead of saving it to the token file.
    #[clap(long, action)]
    no_save: bool,
}

impl Executable for Login {
    fn execute(self) -> eyre::Result<()> {
        let registry_url = self.registry.unwrap_or_else(registry::registry_url);
        let username = match self.username {
            Some(username) if !username.trim().is_empty() => username,
            _ => inquire::Text::new("Registry login")
                .prompt()
                .context("login cancelled")?,
        };
        let password = match self.password {
            Some(password) if !password.is_empty() => password,
            _ => inquire::Password::new("Password")
                .without_confirmation()
                .prompt()
                .context("password prompt failed")?,
        };

        let client = RegistryClient::with_url(&registry_url, false)
            .context("invalid registry configuration")?;
        let session = client
            .login(username.trim(), &password)
            .with_context(|| format!("sign-in to {registry_url} failed"))?;
        let token = client
            .create_token(&session, &self.token_name, &self.scopes)
            .context("failed to create a publish token")?;

        if self.no_save {
            println!("{token}");
        } else {
            let path = registry::save_token(&token).context("failed to save the publish token")?;
            println!(
                "signed in to {registry_url}\ntoken `{}` saved to {} (scope: {})",
                self.token_name,
                path.display(),
                self.scopes.join(", ")
            );
        }
        Ok(())
    }
}
