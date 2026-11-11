use std::io::IsTerminal;

use super::{Executable, default_tracing};
use clap::Subcommand;
use eyre::{Context, bail};

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

/// Selects the CLI archive among a release's assets. The target triple alone
/// also matches the C and C++ library archives.
const CLI_ASSET_IDENTIFIER: &str = "dora-cli-";

/// The updater `dora self update` runs: the CLI archive of the latest
/// `dora-rs/dora` release for this target.
fn cli_updater() -> eyre::Result<self_update::backends::github::Update> {
    #[cfg(target_os = "linux")]
    let bin_path_in_archive = format!("dora-cli-{}/dora", env!("TARGET"));
    #[cfg(target_os = "macos")]
    let bin_path_in_archive = format!("dora-cli-{}/dora", env!("TARGET"));
    #[cfg(target_os = "windows")]
    let bin_path_in_archive = String::from("dora.exe");

    Ok(self_update::backends::github::Update::configure()
        .repo_owner("dora-rs")
        .repo_name("dora")
        .bin_path_in_archive(&bin_path_in_archive)
        .bin_name("dora")
        .asset_identifier(CLI_ASSET_IDENTIFIER)
        .show_download_progress(true)
        .current_version(env!("CARGO_PKG_VERSION"))
        .build()?)
}

impl Executable for SelfSubCommand {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        match self {
            SelfSubCommand::Update { check_only } => {
                println!("Checking for updates...");

                let status = cli_updater()?;

                if check_only {
                    // Only check if an update is available
                    match status.get_latest_release() {
                        Ok(releases) => {
                            let Some(release) = releases.latest() else {
                                bail!("failed to check for updates: no release found");
                            };
                            let release_version = release.version();
                            let current_version = self_update::cargo_crate_version!();
                            // Semver comparison — a dev/pre-release build that is
                            // ahead of the latest published release must not be
                            // offered a downgrade.
                            let newer = self_update::version::bump_is_greater(
                                current_version,
                                release_version,
                            )
                            .unwrap_or(current_version != release_version);
                            if newer {
                                println!(
                                    "An update is available: {release_version}. Run 'dora self update' to update"
                                );
                            } else {
                                println!(
                                    "Dora CLI is up to date (current: {current_version}, latest release: {release_version})"
                                );
                            }
                        }
                        Err(e) => bail!("failed to check for updates: {e}"),
                    }
                } else {
                    // Perform the actual update
                    match status.update() {
                        Ok(update_status) => {
                            let version = update_status.version();
                            if update_status.is_updated() {
                                println!("Successfully updated Dora CLI to version: {version}");
                            } else {
                                println!("Dora CLI is already at the latest version: {version}");
                            }
                        }
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

                    if let Ok(status) = uv_status
                        && status.success()
                    {
                        println!("Dora CLI has been successfully uninstalled via uv pip.");
                        return Ok(());
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

#[cfg(test)]
mod tests {
    use std::io::Write;

    /// A release's asset names (v1.1.0), in the order the GitHub API lists
    /// them; the library archives come before the CLI for every target.
    const V1_1_0_ASSETS: &[&str] = &[
        "dora-c-libraries-aarch64-apple-darwin.tar.gz",
        "dora-c-libraries-aarch64-unknown-linux-gnu.tar.gz",
        "dora-c-libraries-x86_64-pc-windows-msvc.zip",
        "dora-c-libraries-x86_64-unknown-linux-gnu.tar.gz",
        "dora-cli-aarch64-apple-darwin.tar.gz",
        "dora-cli-aarch64-unknown-linux-gnu.tar.gz",
        "dora-cli-installer.ps1",
        "dora-cli-installer.sh",
        "dora-cli-x86_64-pc-windows-msvc.zip",
        "dora-cli-x86_64-unknown-linux-gnu.tar.gz",
        "dora-cpp-libraries-aarch64-apple-darwin.tar.gz",
        "dora-cpp-libraries-aarch64-unknown-linux-gnu.tar.gz",
        "dora-cpp-libraries-x86_64-pc-windows-msvc.zip",
        "dora-cpp-libraries-x86_64-unknown-linux-gnu.tar.gz",
    ];

    /// The updater `dora self update` builds selects the CLI archive: without
    /// the identifier, `self_update` takes the first asset matching the
    /// target, which can be one of the library archives.
    #[test]
    fn the_cli_updater_selects_the_cli_archive() {
        use self_update::UpdateConfig;
        let updater = super::cli_updater().unwrap();
        assert_eq!(
            updater.asset_identifier(),
            Some(super::CLI_ASSET_IDENTIFIER)
        );
    }

    #[test]
    fn self_update_picks_the_cli_archive() {
        let release = self_update::update::Release::builder()
            .name("v1.1.0")
            .version("1.1.0")
            .assets(V1_1_0_ASSETS.iter().map(|name| {
                self_update::update::ReleaseAsset::new(*name, format!("https://host/{name}"))
            }))
            .build()
            .unwrap();
        for target in [
            "aarch64-apple-darwin",
            "aarch64-unknown-linux-gnu",
            "x86_64-pc-windows-msvc",
            "x86_64-unknown-linux-gnu",
        ] {
            let without = release.asset_for(target, None).unwrap();
            assert!(without.name().starts_with("dora-c-libraries-"), "{target}");
            let with = release
                .asset_for(target, Some(super::CLI_ASSET_IDENTIFIER))
                .unwrap();
            assert!(
                with.name().starts_with(&format!("dora-cli-{target}")),
                "{target}"
            );
        }
    }

    /// The Windows release asset is made with PowerShell `Compress-Archive`,
    /// which always writes Deflate entries. `self_update` only links a Deflate
    /// decoder when `compression-zip-deflate` is enabled, so without it every
    /// Windows `dora self update` fails after the download.
    #[test]
    fn self_update_extracts_a_deflate_zip() {
        let dir = tempfile::tempdir().unwrap();
        let archive = dir.path().join("dora-cli-x86_64-pc-windows-msvc.zip");
        let payload = b"MZ fake dora.exe payload ".repeat(64);

        let mut writer = zip::ZipWriter::new(std::fs::File::create(&archive).unwrap());
        let options = zip::write::SimpleFileOptions::default()
            .compression_method(zip::CompressionMethod::Deflated);
        writer
            .start_file("dora.exe", options)
            .expect("Deflate support must be compiled into zip");
        writer.write_all(&payload).unwrap();
        writer.finish().unwrap();

        let out = dir.path().join("out");
        std::fs::create_dir(&out).unwrap();
        self_update::Extract::from_source(&archive)
            .archive(self_update::ArchiveKind::Zip)
            .extract_file(&out, "dora.exe")
            .expect("self_update must extract a Deflate zip entry");
        assert_eq!(std::fs::read(out.join("dora.exe")).unwrap(), payload);
    }
}
