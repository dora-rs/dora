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

/// Whether `release` ships a CLI archive built for `target`.
///
/// `self_update` falls back to the first asset that only contains the
/// identifier, so on a target without a prebuilt CLI (e.g.
/// `x86_64-apple-darwin` or a musl build) it would download another
/// platform's archive and then fail with a misleading "path not found in
/// archive" error. Checking up front lets us say what is actually wrong.
fn has_prebuilt_cli(release: &self_update::update::Release, target: &str) -> bool {
    release
        .asset_for(target, Some(CLI_ASSET_IDENTIFIER))
        .is_some_and(|asset| {
            asset
                .name()
                .starts_with(&format!("{CLI_ASSET_IDENTIFIER}{target}."))
        })
}

/// Whether `release_version` is newer than `current_version`. Semver
/// comparison: a dev/pre-release build that is ahead of the latest published
/// release must not be offered a downgrade.
fn is_newer(current_version: &str, release_version: &str) -> bool {
    self_update::version::bump_is_greater(current_version, release_version)
        .unwrap_or(current_version != release_version)
}

/// Whether `dora self update` must refuse: `release` is newer than this build
/// but has no CLI archive for `target`. When there is nothing newer, the
/// updater reports "already at the latest version" on any target.
fn lacks_prebuilt_update(
    release: &self_update::update::Release,
    current_version: &str,
    target: &str,
) -> bool {
    is_newer(current_version, release.version()) && !has_prebuilt_cli(release, target)
}

/// How to upgrade a CLI installed from the `dora-rs-cli` wheel. Printed
/// rather than run: only the user knows which installer and environment the
/// `dora` on their `PATH` came from, and each one upgrades differently.
const WHEEL_UPDATE_HINT: &str = "upgrade the `dora-rs-cli` package with the installer you used: \
     `pip install -U dora-rs-cli`, `uv pip install -U dora-rs-cli`, \
     `uv tool upgrade dora-rs-cli` or `pipx upgrade dora-rs-cli`";

/// Why `dora self update` must not replace the running binary, if it must not.
///
/// `wheel` is whether this CLI was built with the `python` feature, which
/// marks the build shipped in the `dora-rs-cli` wheel. There the running
/// executable is the Python interpreter and the installer owns the files, so
/// replacing `current_exe()` would overwrite the interpreter and leave the
/// installer's record of the package stale (#4172). `--check-only` never
/// replaces anything, so it always proceeds.
fn update_refusal(wheel: bool, check_only: bool) -> Option<String> {
    (wheel && !check_only).then(|| {
        format!(
            "this dora CLI was installed from the `dora-rs-cli` Python package; {WHEEL_UPDATE_HINT}"
        )
    })
}

/// What `--check-only` tells the user to do when an update is available.
fn check_only_update_hint(wheel: bool) -> String {
    if wheel {
        format!("To update, {WHEEL_UPDATE_HINT}")
    } else {
        "Run `dora self update` to update".to_owned()
    }
}

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
                let wheel = cfg!(feature = "python");
                if let Some(refusal) = update_refusal(wheel, check_only) {
                    bail!(refusal);
                }

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
                            if is_newer(current_version, release_version) {
                                println!(
                                    "An update is available: {release_version}. {}",
                                    check_only_update_hint(wheel)
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
                    // Refuse up front when the latest release has no CLI
                    // archive for this target, instead of downloading another
                    // platform's archive and failing inside it (#4151).
                    let releases = status
                        .get_latest_release()
                        .map_err(|e| eyre::eyre!("failed to check for updates: {e}"))?;
                    if let Some(release) = releases.latest()
                        && lacks_prebuilt_update(
                            release,
                            self_update::cargo_crate_version!(),
                            env!("TARGET"),
                        )
                    {
                        bail!(
                            "no prebuilt dora CLI for {} in release {}; install it with \
                             `cargo install dora-cli --locked` instead",
                            env!("TARGET"),
                            release.version()
                        );
                    }

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

    fn v1_1_0_release() -> self_update::update::Release {
        self_update::update::Release::builder()
            .name("v1.1.0")
            .version("1.1.0")
            .assets(V1_1_0_ASSETS.iter().map(|name| {
                self_update::update::ReleaseAsset::new(*name, format!("https://host/{name}"))
            }))
            .build()
            .unwrap()
    }

    #[test]
    fn self_update_picks_the_cli_archive() {
        let release = v1_1_0_release();
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

    /// Targets without a CLI archive in the release are refused instead of
    /// falling back to another platform's archive (#4151).
    #[test]
    fn only_targets_with_a_cli_archive_have_a_prebuilt_cli() {
        let release = v1_1_0_release();
        for target in [
            "aarch64-apple-darwin",
            "aarch64-unknown-linux-gnu",
            "x86_64-pc-windows-msvc",
            "x86_64-unknown-linux-gnu",
        ] {
            assert!(super::has_prebuilt_cli(&release, target), "{target}");
        }
        for target in [
            "x86_64-apple-darwin",
            "aarch64-pc-windows-msvc",
            "armv7-unknown-linux-gnueabihf",
            "i686-unknown-linux-gnu",
            "x86_64-unknown-linux-musl",
        ] {
            // `self_update` itself would still pick some `dora-cli-*` asset.
            assert!(
                release
                    .asset_for(target, Some(super::CLI_ASSET_IDENTIFIER))
                    .is_some(),
                "{target}"
            );
            assert!(!super::has_prebuilt_cli(&release, target), "{target}");
        }
    }

    /// An up-to-date (or newer dev) build on a target without a prebuilt CLI
    /// is not refused: the updater reports "already at the latest version"
    /// (#4169 review). Only an actual update to a release without an archive
    /// for the target is refused.
    #[test]
    fn only_an_actual_update_needs_a_prebuilt_cli() {
        let release = v1_1_0_release();
        let musl = "x86_64-unknown-linux-musl";
        assert!(super::lacks_prebuilt_update(&release, "1.0.0", musl));
        assert!(!super::lacks_prebuilt_update(&release, "1.1.0", musl));
        assert!(!super::lacks_prebuilt_update(&release, "1.2.0-dev", musl));
        let gnu = "x86_64-unknown-linux-gnu";
        assert!(!super::lacks_prebuilt_update(&release, "1.0.0", gnu));
    }

    /// A wheel-installed CLI refuses to replace the running executable, which
    /// is the Python interpreter, and points at the package installers instead
    /// (#4172). `--check-only` and a standalone CLI proceed.
    #[test]
    fn a_wheel_build_refuses_the_binary_swap() {
        let refusal = super::update_refusal(true, false).expect("a wheel update must be refused");
        for command in [
            "`pip install -U dora-rs-cli`",
            "`uv pip install -U dora-rs-cli`",
            "`uv tool upgrade dora-rs-cli`",
            "`pipx upgrade dora-rs-cli`",
        ] {
            assert!(refusal.contains(command), "{refusal:?} lacks {command}");
        }

        assert_eq!(super::update_refusal(true, true), None);
        assert_eq!(super::update_refusal(false, false), None);
        assert_eq!(super::update_refusal(false, true), None);
    }

    /// The `--check-only` hint names the command that actually updates this
    /// build; the nightly smoke greps for "update is available" around it.
    #[test]
    fn check_only_names_the_matching_update_command() {
        let wheel = super::check_only_update_hint(true);
        assert!(wheel.contains("`pipx upgrade dora-rs-cli`"));
        assert!(!wheel.contains("dora self update"));
        assert_eq!(
            super::check_only_update_hint(false),
            "Run `dora self update` to update"
        );
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
