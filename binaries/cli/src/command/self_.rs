use std::io::IsTerminal;

use super::{Executable, default_tracing};
use clap::Subcommand;
use eyre::{Context, bail};

#[derive(Debug, Subcommand)]
/// Adora CLI self-management commands
pub enum SelfSubCommand {
    /// Check for updates or update the CLI
    Update {
        /// Only check for updates without installing
        #[clap(long)]
        check_only: bool,
    },
    /// Remove The Adora CLI from the system
    Uninstall {
        /// Force uninstallation without confirmation
        #[clap(long)]
        force: bool,
    },
}

impl Executable for SelfSubCommand {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        match self {
            SelfSubCommand::Update { check_only } => {
                println!("Checking for updates...");

                #[cfg(target_os = "linux")]
                let bin_path_in_archive = format!("adora-cli-{}/adora", env!("TARGET"));
                #[cfg(target_os = "macos")]
                let bin_path_in_archive = format!("adora-cli-{}/adora", env!("TARGET"));
                #[cfg(target_os = "windows")]
                let bin_path_in_archive = String::from("adora.exe");

                let status = self_update::backends::github::Update::configure()
                    .repo_owner("adora-rs")
                    .repo_name("adora")
                    .bin_path_in_archive(&bin_path_in_archive)
                    .bin_name("adora")
                    .show_download_progress(true)
                    .current_version(env!("CARGO_PKG_VERSION"))
                    .build()?;

                if check_only {
                    // Only check if an update is available
                    match status.get_latest_release() {
                        Ok(release) => {
                            let current_version = self_update::cargo_crate_version!();
                            if current_version != release.version {
                                println!(
                                    "An update is available: {}. Run 'adora self update' to update",
                                    release.version
                                );
                            } else {
                                println!(
                                    "Adora CLI is already at the latest version: {current_version}"
                                );
                            }
                        }
                        Err(e) => bail!("failed to check for updates: {e}"),
                    }
                } else {
                    // Perform the actual update
                    match status.update() {
                        Ok(update_status) => match update_status {
                            self_update::Status::UpToDate(version) => {
                                println!("Adora CLI is already at the latest version: {version}");
                            }
                            self_update::Status::Updated(version) => {
                                println!("Successfully updated Adora CLI to version: {version}");
                            }
                        },
                        Err(e) => bail!("failed to update: {e}"),
                    }
                }
            }
            SelfSubCommand::Uninstall { force } => {
                if !force && !std::io::stdin().is_terminal() {
                    bail!("use --force for non-interactive uninstall");
                }
                if !force {
                    let confirmed =
                        inquire::Confirm::new("Are you sure you want to uninstall Adora CLI?")
                            .with_default(false)
                            .prompt()
                            .wrap_err("Uninstallation cancelled")?;

                    if !confirmed {
                        println!("Uninstallation cancelled");
                        return Ok(());
                    }
                }

                println!("Uninstalling Adora CLI...");
                #[cfg(feature = "python")]
                {
                    println!("Detected Python installation...");

                    // Try uv pip uninstall first
                    let uv_status = std::process::Command::new("uv")
                        .args(["pip", "uninstall", "adora-rs-cli"])
                        .status();

                    if let Ok(status) = uv_status
                        && status.success()
                    {
                        println!("Adora CLI has been successfully uninstalled via uv pip.");
                        return Ok(());
                    }

                    // Fall back to regular pip uninstall
                    println!("Trying with pip...");
                    let status = std::process::Command::new("pip")
                        .args(["uninstall", "-y", "adora-rs-cli"])
                        .status()
                        .wrap_err("Failed to run pip uninstall")?;

                    if status.success() {
                        println!("Adora CLI has been successfully uninstalled via pip.");
                    } else {
                        bail!("Failed to uninstall Adora CLI via pip.");
                    }
                }
                #[cfg(not(feature = "python"))]
                {
                    match self_replace::self_delete() {
                        Ok(_) => {
                            println!("Adora CLI has been successfully uninstalled.");
                        }
                        Err(e) => {
                            bail!("Failed to uninstall Adora CLI: {}", e);
                        }
                    }
                }
            }
        }
        Ok(())
    }
}
