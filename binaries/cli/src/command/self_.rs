use super::{default_tracing, Executable};
use clap::Subcommand;
use eyre::{bail, Context};

#[derive(Debug, Subcommand)]
/// Dora CLI self-management commands
pub enum SelfSubCommand {
    /// Check for updates or update the CLI
    Update {
        /// Only check for updates without installing
        #[clap(long)]
        check_only: bool,
    },
    /// Remove The Dora CLI from the system
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
                let bin_path_in_archive = format!("dora-cli-{}/dora", env!("TARGET"));
                #[cfg(target_os = "macos")]
                let bin_path_in_archive = format!("dora-cli-{}/dora", env!("TARGET"));
                #[cfg(target_os = "windows")]
                let bin_path_in_archive = String::from("dora.exe");

                let status = self_update::backends::github::Update::configure()
                    .repo_owner("dora-rs")
                    .repo_name("dora")
                    .bin_path_in_archive(&bin_path_in_archive)
                    .bin_name("dora")
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
                                    "An update is available: {}. Run 'dora self update' to update",
                                    release.version
                                );
                            } else {
                                println!(
                                    "Dora CLI is already at the latest version: {}",
                                    current_version
                                );
                            }
                        }
                        Err(e) => println!("Failed to check for updates: {}", e),
                    }
                } else {
                    // Perform the actual update
                    match status.update() {
                        Ok(update_status) => match update_status {
                            self_update::Status::UpToDate(version) => {
                                println!("Dora CLI is already at the latest version: {}", version);
                            }
                            self_update::Status::Updated(version) => {
                                println!("Successfully updated Dora CLI to version: {}", version);
                            }
                        },
                        Err(e) => println!("Failed to update: {}", e),
                    }
                }
            }
            SelfSubCommand::Uninstall { force } => {
                if !force {
                    let confirmed =
                        inquire::Confirm::new("Are you sure you want to uninstall Dora CLI?")
                            .with_default(false)
                            .prompt()
                            .wrap_err("Uninstallation cancelled")?;

                    if !confirmed {
                        println!("Uninstallation cancelled");
                        return Ok(());
                    }
                }

                println!("Uninstalling Dora CLI...");
                #[cfg(feature = "python")]
                {
                    println!("Detected Python installation...");

                    // Try uv pip uninstall first
                    let uv_status = std::process::Command::new("uv")
                        .args(["pip", "uninstall", "dora-rs-cli"])
                        .status();

                    if let Ok(status) = uv_status {
                        if status.success() {
                            println!("Dora CLI has been successfully uninstalled via uv pip.");
                            return Ok(());
                        }
                    }

                    // Fall back to regular pip uninstall
                    println!("Trying with pip...");
                    let status = std::process::Command::new("pip")
                        .args(["uninstall", "-y", "dora-rs-cli"])
                        .status()
                        .wrap_err("Failed to run pip uninstall")?;

                    if status.success() {
                        println!("Dora CLI has been successfully uninstalled via pip.");
                    } else {
                        bail!("Failed to uninstall Dora CLI via pip.");
                    }
                }
                #[cfg(not(feature = "python"))]
                {
                    match self_replace::self_delete() {
                        Ok(_) => {
                            println!("Dora CLI has been successfully uninstalled.");
                        }
                        Err(e) => {
                            bail!("Failed to uninstall Dora CLI: {}", e);
                        }
                    }
                }
            }
        }
        Ok(())
    }
}
