use clap::Args;
use eyre::Context;
use std::path::PathBuf;

use crate::{
    command::{Executable, default_tracing},
    dora_toml,
};

/// Validate Dora package metadata (`Dora.toml`).
///
/// This checks schema and semantic constraints for:
/// - `[package]` (`name`, `version`, `entrypoint`)
/// - `[dependencies]` source declaration (`version` | `path` | `git`)
///
/// Examples:
///
/// Validate the default file in current directory:
///   dora node validate-metadata
///
/// Validate a custom path:
///   dora node validate-metadata --path ./nodes/camera/Dora.toml
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct ValidateMetadata {
    /// Path to metadata file
    #[clap(long, value_name = "PATH", default_value = "Dora.toml")]
    path: PathBuf,
}

impl Executable for ValidateMetadata {
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        dora_toml::read_and_validate(&self.path)
            .with_context(|| format!("metadata validation failed for `{}`", self.path.display()))?;
        println!("metadata validation passed: {}", self.path.display());
        Ok(())
    }
}
