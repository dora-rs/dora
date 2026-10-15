use std::io::{IsTerminal, Write};
use std::path::Path;

use super::{Executable, default_tracing};
use clap::Subcommand;
use eyre::{Context, bail};

/// Owner and name of the repository the CLI is released from.
const REPO_OWNER: &str = "dora-rs";
const REPO_NAME: &str = "dora";

/// Node binaries shipped in the CLI archive next to `dora`. `dora record` and
/// `dora replay` resolve them through `node_binary::search()`, which looks in
/// the `dora` executable's own directory first — so an update that replaces
/// only `dora` leaves the old nodes in place and keeps using them.
const BUNDLED_NODE_BINARIES: [&str; 2] = ["dora-record-node", "dora-replay-node"];

/// Path of `name` inside the release archive for `target`. The unix tarballs
/// nest the binaries under `dora-cli-<target>/`; the Windows zip keeps them
/// flat at the archive root. Mirrors the packaging in `release.yml` and what
/// `install.sh` / `install.ps1` expect.
fn archive_member(name: &str, target: &str, exe_suffix: &str) -> String {
    if target.contains("windows") {
        format!("{name}{exe_suffix}")
    } else {
        format!("dora-cli-{target}/{name}")
    }
}

/// Copy `source` over `dest`, staging the copy inside `dest`'s directory so the
/// final rename stays on one filesystem: the extracted archive lives in the
/// temp directory, which may be on another one.
fn replace_binary(source: &Path, dest: &Path) -> eyre::Result<()> {
    let file_name = dest
        .file_name()
        .and_then(|name| name.to_str())
        .ok_or_else(|| eyre::eyre!("invalid destination path: {dest:?}"))?;
    let staged = dest.with_file_name(format!("{file_name}.new"));

    std::fs::copy(source, &staged).wrap_err_with(|| {
        format!(
            "failed to copy {} to {}",
            source.display(),
            staged.display()
        )
    })?;
    std::fs::rename(&staged, dest)
        .wrap_err_with(|| format!("failed to replace {}", dest.display()))?;
    Ok(())
}

/// Replace the node binaries in `dest_dir` with the ones from the extracted
/// release at `root`. Members missing from the archive are skipped: releases
/// cut before the nodes shipped in it don't contain them.
fn install_siblings(
    root: &Path,
    dest_dir: &Path,
    target: &str,
    exe_suffix: &str,
) -> eyre::Result<()> {
    for name in BUNDLED_NODE_BINARIES {
        let source = root.join(archive_member(name, target, exe_suffix));
        if !source.exists() {
            continue;
        }
        println!("Replacing {name}...");
        replace_binary(&source, &dest_dir.join(format!("{name}{exe_suffix}")))?;
    }
    Ok(())
}

/// Install the newest release, including the node binaries its archive ships.
///
/// `self_update`'s own updater extracts only `bin_path_in_archive`, so it
/// replaces `dora` and leaves the siblings stale. Download the archive once,
/// unpack it, and install every binary it contains.
fn install_release(
    updater: &dyn self_update::update::ReleaseUpdate,
) -> eyre::Result<self_update::Status> {
    let current = env!("CARGO_PKG_VERSION");
    let releases = updater.get_latest_releases(current)?;

    // Same release selection as `self_update`'s updater: prefer the newest
    // release on the current compatibility line, otherwise the newest one.
    let release = releases
        .iter()
        .find(|release| {
            self_update::version::bump_is_compatible(current, &release.version).unwrap_or(false)
        })
        .or_else(|| releases.first())
        .cloned();
    let Some(release) = release else {
        return Ok(self_update::Status::UpToDate(current.to_owned()));
    };

    let target = self_update::get_target();
    let asset = release
        .asset_for(target, None)
        .ok_or_else(|| eyre::eyre!("release v{} has no asset for {target}", release.version))?;

    println!("\nDora CLI release status:");
    println!("  * Current version: v{current}");
    println!("  * New release: v{} ({})", release.version, asset.name);
    println!(
        "The new release will be downloaded and its binaries will replace the installed ones."
    );

    // A blank answer means yes, matching `self_update`'s prompt.
    print!("Do you want to continue? [Y/n] ");
    std::io::stdout().flush()?;
    let mut answer = String::new();
    std::io::stdin().read_line(&mut answer)?;
    let answer = answer.trim().to_lowercase();
    if !answer.is_empty() && answer != "y" {
        bail!("update aborted");
    }

    // `ReleaseAsset::download_url` is the API URL, which returns release
    // metadata rather than the file unless `Accept: application/octet-stream`
    // is set; `self_update` keeps its header types private. The public
    // `releases/download` URL serves the same file, and is what `install.sh`
    // uses too.
    let url = format!(
        "https://github.com/{REPO_OWNER}/{REPO_NAME}/releases/download/v{}/{}",
        release.version, asset.name
    );

    let tmp = self_update::TempDir::new()?;
    let archive = tmp.path().join(&asset.name);

    println!("Downloading {}...", asset.name);
    self_update::Download::from_url(&url)
        .show_progress(true)
        .download_to(std::fs::File::create(&archive)?)?;

    println!("Extracting archive...");
    self_update::Extract::from_source(&archive).extract_into(tmp.path())?;

    let exe = std::env::current_exe()?;
    let dest_dir = exe
        .parent()
        .ok_or_else(|| eyre::eyre!("could not determine the install directory"))?;
    let exe_suffix = std::env::consts::EXE_SUFFIX;

    println!("Replacing dora...");
    self_replace::self_replace(tmp.path().join(updater.bin_path_in_archive()))?;

    install_siblings(tmp.path(), dest_dir, target, exe_suffix)?;

    Ok(self_update::Status::Updated(release.version))
}

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

                let status = self_update::backends::github::Update::configure()
                    .repo_owner(REPO_OWNER)
                    .repo_name(REPO_NAME)
                    .bin_path_in_archive(&archive_member(
                        "dora",
                        env!("TARGET"),
                        std::env::consts::EXE_SUFFIX,
                    ))
                    .bin_name("dora")
                    .show_download_progress(true)
                    .current_version(env!("CARGO_PKG_VERSION"))
                    .build()?;

                if check_only {
                    // Only check if an update is available
                    match status.get_latest_release() {
                        Ok(release) => {
                            let current_version = self_update::cargo_crate_version!();
                            // Semver comparison — a dev/pre-release build that is
                            // ahead of the latest published release must not be
                            // offered a downgrade.
                            let newer = self_update::version::bump_is_greater(
                                current_version,
                                &release.version,
                            )
                            .unwrap_or(current_version != release.version);
                            if newer {
                                println!(
                                    "An update is available: {}. Run 'dora self update' to update",
                                    release.version
                                );
                            } else {
                                println!(
                                    "Dora CLI is up to date (current: {current_version}, latest release: {})",
                                    release.version
                                );
                            }
                        }
                        Err(e) => bail!("failed to check for updates: {e}"),
                    }
                } else {
                    // Perform the actual update
                    match install_release(&*status) {
                        Ok(update_status) => match update_status {
                            self_update::Status::UpToDate(version) => {
                                println!("Dora CLI is already at the latest version: {version}");
                            }
                            self_update::Status::Updated(version) => {
                                println!("Successfully updated Dora CLI to version: {version}");
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
    use super::{archive_member, install_siblings};

    /// The archive layout is fixed by `release.yml`: unix tarballs nest the
    /// binaries under `dora-cli-<target>/`, the Windows zip keeps them flat.
    /// Both `install.sh`/`install.ps1` and the update extraction rely on it.
    #[test]
    fn archive_member_matches_both_archive_layouts() {
        assert_eq!(
            archive_member("dora", "x86_64-unknown-linux-gnu", ""),
            "dora-cli-x86_64-unknown-linux-gnu/dora"
        );
        assert_eq!(
            archive_member("dora-record-node", "aarch64-apple-darwin", ""),
            "dora-cli-aarch64-apple-darwin/dora-record-node"
        );
        assert_eq!(
            archive_member("dora", "x86_64-pc-windows-msvc", ".exe"),
            "dora.exe"
        );
        assert_eq!(
            archive_member("dora-replay-node", "x86_64-pc-windows-msvc", ".exe"),
            "dora-replay-node.exe"
        );
    }

    /// `dora self update` has to replace the node binaries next to `dora`, or
    /// `node_binary::search()` keeps preferring the stale siblings. It also has
    /// to tolerate an archive that doesn't ship them (releases before they were
    /// added) and handle both archive layouts.
    #[test]
    fn siblings_are_installed_for_both_archive_layouts() {
        let base =
            std::env::temp_dir().join(format!("dora-self-update-test-{}", std::process::id()));

        // Nested unix tarball layout, with `dora-replay-node` missing.
        let unix_root = base.join("unix");
        let unix_bin = base.join("unix-bin");
        let unix_dir = unix_root.join("dora-cli-x86_64-unknown-linux-gnu");
        std::fs::create_dir_all(&unix_dir).unwrap();
        std::fs::create_dir_all(&unix_bin).unwrap();
        std::fs::write(unix_dir.join("dora-record-node"), b"new").unwrap();
        std::fs::write(unix_bin.join("dora-record-node"), b"old").unwrap();
        std::fs::write(unix_bin.join("dora-replay-node"), b"old").unwrap();

        install_siblings(&unix_root, &unix_bin, "x86_64-unknown-linux-gnu", "").unwrap();

        assert_eq!(
            std::fs::read(unix_bin.join("dora-record-node")).unwrap(),
            b"new"
        );
        assert_eq!(
            std::fs::read(unix_bin.join("dora-replay-node")).unwrap(),
            b"old",
            "a binary missing from the archive must be left alone"
        );

        // Flat Windows zip layout.
        let win_root = base.join("windows");
        let win_bin = base.join("windows-bin");
        std::fs::create_dir_all(&win_root).unwrap();
        std::fs::create_dir_all(&win_bin).unwrap();
        std::fs::write(win_root.join("dora-replay-node.exe"), b"new").unwrap();
        std::fs::write(win_bin.join("dora-replay-node.exe"), b"old").unwrap();

        install_siblings(&win_root, &win_bin, "x86_64-pc-windows-msvc", ".exe").unwrap();

        assert_eq!(
            std::fs::read(win_bin.join("dora-replay-node.exe")).unwrap(),
            b"new"
        );

        std::fs::remove_dir_all(&base).ok();
    }
}
