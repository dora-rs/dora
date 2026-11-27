use std::io::IsTerminal;
use std::path::{Path, PathBuf};

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

/// Where the executable `name` sits in this target's CLI archive: under
/// `dora-cli-<target>/` in the Unix tarballs, at the root of the Windows zip.
fn path_in_archive(target: &str, name: &str) -> String {
    if cfg!(windows) {
        format!("{name}.exe")
    } else {
        format!("dora-cli-{target}/{name}")
    }
}

/// The node binaries the CLI archive ships next to `dora`, and that
/// `node_binary::search` looks for next to the `dora` executable first.
const NODE_BINARIES: &[&str] = &["dora-record-node", "dora-replay-node"];

/// Extract the [`NODE_BINARIES`] from the downloaded CLI `archive` into
/// `staging`, at their paths in the archive. A binary the archive lacks
/// (a release from before they were shipped) or that fails to extract is
/// left out; [`refresh_node_binaries`] reports it.
fn stage_node_binaries(archive: &Path, staging: &Path, target: &str) {
    for name in NODE_BINARIES {
        let _ = self_update::Extract::from_source(archive)
            .extract_file(staging, path_in_archive(target, name));
    }
}

/// Bring the [`NODE_BINARIES`] in `exe_dir` up to the copies staged by
/// [`stage_node_binaries`], so the updated CLI does not go on running the
/// previous version's record and replay nodes (#4337). Returns a warning
/// for each node left stale.
///
/// A node missing from `exe_dir` is installed only when `install_missing`:
/// for the installers' `~/.dora/bin`, which held only `dora` before the
/// nodes were shipped. Elsewhere (e.g. `~/.cargo/bin`, which cargo
/// manages) it is left out and `node_binary::search` looks further.
fn refresh_node_binaries(
    staging: &Path,
    exe_dir: &Path,
    target: &str,
    install_missing: bool,
) -> Vec<String> {
    let mut warnings = Vec::new();
    for name in NODE_BINARIES {
        let file_name = format!("{name}{}", std::env::consts::EXE_SUFFIX);
        let installed = exe_dir.join(&file_name);
        let staged = staging.join(path_in_archive(target, name));
        let tmp = exe_dir.join(format!(".{file_name}.new"));
        if !installed.exists() {
            if install_missing
                && staged.is_file()
                && let Err(e) = install_file(&staged, &installed, &tmp, None)
            {
                warnings.push(format!("{} was not installed ({e:#})", installed.display()));
            }
            continue;
        }
        let result = if staged.is_file() {
            std::fs::metadata(&installed)
                .wrap_err("reading the installed file failed")
                .and_then(|m| install_file(&staged, &installed, &tmp, Some(m.permissions())))
        } else {
            Err(eyre::eyre!("the release archive does not contain it"))
        };
        if let Err(e) = result {
            warnings.push(format!(
                "{} was not updated ({e:#}); it is from the previous version, so delete it \
                 to stop `dora record`/`dora replay` from using it",
                installed.display()
            ));
        }
    }
    warnings
}

/// Whether `exe_dir` is where `install.sh` / `install.ps1` put `dora`.
fn is_installer_dir(exe_dir: &Path) -> bool {
    let installer_dir = super::coordinator::dora_dir().join("bin");
    match (
        dunce::canonicalize(exe_dir),
        dunce::canonicalize(installer_dir),
    ) {
        (Ok(a), Ok(b)) => a == b,
        _ => false,
    }
}

/// Put a copy of `src` at `dest`, through `tmp` in the same directory so
/// the swap is a rename and `dest` is never left half-written. The copy
/// gets `permissions` (an installed file's, since the installers mark the
/// nodes executable themselves rather than rely on the archive's modes),
/// or else is made executable.
fn install_file(
    src: &Path,
    dest: &Path,
    tmp: &Path,
    permissions: Option<std::fs::Permissions>,
) -> eyre::Result<()> {
    let result = std::fs::copy(src, tmp)
        .wrap_err("copy failed")
        .and_then(|_| {
            let permissions = match permissions {
                Some(permissions) => permissions,
                None => executable(std::fs::metadata(tmp)?.permissions()),
            };
            std::fs::set_permissions(tmp, permissions).wrap_err("chmod failed")
        })
        .and_then(|()| std::fs::rename(tmp, dest).wrap_err("replace failed"));
    if result.is_err() {
        let _ = std::fs::remove_file(tmp);
    }
    result
}

#[cfg(unix)]
fn executable(permissions: std::fs::Permissions) -> std::fs::Permissions {
    use std::os::unix::fs::PermissionsExt;
    std::fs::Permissions::from_mode(permissions.mode() | 0o755)
}

#[cfg(not(unix))]
fn executable(permissions: std::fs::Permissions) -> std::fs::Permissions {
    permissions
}

/// The updater `dora self update` runs: the CLI archive of the latest
/// `dora-rs/dora` release for this target. With `node_staging`, the
/// [`NODE_BINARIES`] are also extracted from the downloaded archive into
/// that directory, before the CLI is replaced.
fn cli_updater(
    node_staging: Option<PathBuf>,
) -> eyre::Result<self_update::backends::github::Update> {
    let target = env!("TARGET");
    let mut builder = self_update::backends::github::Update::configure();
    builder
        .repo_owner("dora-rs")
        .repo_name("dora")
        .bin_path_in_archive(path_in_archive(target, "dora"))
        .bin_name("dora")
        .asset_identifier(CLI_ASSET_IDENTIFIER)
        .show_download_progress(true)
        .current_version(env!("CARGO_PKG_VERSION"));
    if let Some(staging) = node_staging {
        // The archive hook is the one point that sees the downloaded
        // archive; it only reads it, so it never rejects the update.
        builder.verify_archive(move |archive| {
            stage_node_binaries(archive, &staging, target);
            Ok(())
        });
    }
    Ok(builder.build()?)
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

                // Before the update: once the running executable is
                // replaced, `current_exe()` may no longer name a real file.
                let exe_dir = std::env::current_exe()
                    .ok()
                    .and_then(|exe| exe.parent().map(Path::to_owned));
                let node_staging = if check_only {
                    None
                } else {
                    Some(tempfile::tempdir().wrap_err("failed to create a staging directory")?)
                };
                let status = cli_updater(node_staging.as_ref().map(|d| d.path().to_owned()))?;

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
                                if let (Some(staging), Some(exe_dir)) = (&node_staging, &exe_dir) {
                                    for warning in refresh_node_binaries(
                                        staging.path(),
                                        exe_dir,
                                        env!("TARGET"),
                                        is_installer_dir(exe_dir),
                                    ) {
                                        eprintln!("warning: {warning}");
                                    }
                                }
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
        let updater = super::cli_updater(None).unwrap();
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

    /// Write a CLI archive for `target` holding `files` (name, contents) at
    /// their [`super::path_in_archive`] paths.
    fn cli_archive(
        dir: &std::path::Path,
        target: &str,
        files: &[(&str, &[u8])],
    ) -> std::path::PathBuf {
        // A zip on every platform: `extract_file` detects the format from the
        // extension, and the entries' paths are what this exercises.
        let archive = dir.join(format!("dora-cli-{target}.zip"));
        let mut writer = zip::ZipWriter::new(std::fs::File::create(&archive).unwrap());
        let options = zip::write::SimpleFileOptions::default()
            .compression_method(zip::CompressionMethod::Deflated);
        for (name, contents) in files {
            writer
                .start_file(super::path_in_archive(target, name), options)
                .unwrap();
            writer.write_all(contents).unwrap();
        }
        writer.finish().unwrap();
        archive
    }

    fn node_file(name: &str) -> String {
        format!("{name}{}", std::env::consts::EXE_SUFFIX)
    }

    /// #4337: after `dora self update`, a record/replay node next to `dora`
    /// is replaced by the one in the same release archive, so the new CLI
    /// does not run the previous version's node, which `node_binary::search`
    /// prefers over every other location.
    #[test]
    fn self_update_replaces_the_node_binaries_next_to_dora() {
        let target = "x86_64-unknown-linux-gnu";
        let dir = tempfile::tempdir().unwrap();
        let archive = cli_archive(
            dir.path(),
            target,
            &[
                ("dora", b"new dora"),
                ("dora-record-node", b"new record"),
                ("dora-replay-node", b"new replay"),
            ],
        );
        let staging = dir.path().join("staging");
        super::stage_node_binaries(&archive, &staging, target);

        let exe_dir = dir.path().join("bin");
        std::fs::create_dir(&exe_dir).unwrap();
        for name in super::NODE_BINARIES {
            let installed = exe_dir.join(node_file(name));
            std::fs::write(&installed, b"old").unwrap();
            #[cfg(unix)]
            {
                use std::os::unix::fs::PermissionsExt;
                std::fs::set_permissions(&installed, std::fs::Permissions::from_mode(0o755))
                    .unwrap();
            }
        }

        let warnings = super::refresh_node_binaries(&staging, &exe_dir, target, false);
        assert!(warnings.is_empty(), "{warnings:?}");
        for (name, contents) in [
            ("dora-record-node", "new record"),
            ("dora-replay-node", "new replay"),
        ] {
            let installed = exe_dir.join(node_file(name));
            assert_eq!(std::fs::read_to_string(&installed).unwrap(), contents);
            #[cfg(unix)]
            {
                use std::os::unix::fs::PermissionsExt;
                let mode = std::fs::metadata(&installed).unwrap().permissions().mode();
                assert_eq!(mode & 0o777, 0o755, "{name} must stay executable");
            }
        }
        // Nothing but the two nodes is left behind in the install directory.
        assert_eq!(std::fs::read_dir(&exe_dir).unwrap().count(), 2);
    }

    /// A node the archive does not contain is reported, not silently kept,
    /// and one missing from both is not invented.
    #[test]
    fn self_update_warns_about_a_node_it_cannot_refresh() {
        let target = "x86_64-unknown-linux-gnu";
        let dir = tempfile::tempdir().unwrap();
        let archive = cli_archive(dir.path(), target, &[("dora", b"new dora")]);
        let staging = dir.path().join("staging");
        super::stage_node_binaries(&archive, &staging, target);

        let exe_dir = dir.path().join("bin");
        std::fs::create_dir(&exe_dir).unwrap();
        let record = exe_dir.join(node_file("dora-record-node"));
        std::fs::write(&record, b"old").unwrap();

        let warnings = super::refresh_node_binaries(&staging, &exe_dir, target, true);
        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(
            warnings[0].contains(&record.display().to_string())
                && warnings[0].contains("delete it"),
            "{warnings:?}"
        );
        assert_eq!(std::fs::read(&record).unwrap(), b"old");
        assert!(!exe_dir.join(node_file("dora-replay-node")).exists());
    }

    /// An installer directory from before the nodes were shipped holds only
    /// `dora`: the update installs both nodes there, executable, so
    /// `dora record`/`dora replay` work without a Rust toolchain. Any other
    /// directory (e.g. `~/.cargo/bin`) gets nothing it did not have.
    #[test]
    fn self_update_installs_missing_nodes_only_where_asked() {
        let target = "x86_64-unknown-linux-gnu";
        let dir = tempfile::tempdir().unwrap();
        let archive = cli_archive(
            dir.path(),
            target,
            &[
                ("dora", b"new dora"),
                ("dora-record-node", b"new record"),
                ("dora-replay-node", b"new replay"),
            ],
        );
        let staging = dir.path().join("staging");
        super::stage_node_binaries(&archive, &staging, target);

        let other = dir.path().join("other");
        std::fs::create_dir(&other).unwrap();
        let warnings = super::refresh_node_binaries(&staging, &other, target, false);
        assert!(warnings.is_empty(), "{warnings:?}");
        assert_eq!(std::fs::read_dir(&other).unwrap().count(), 0);

        let installer = dir.path().join("installer");
        std::fs::create_dir(&installer).unwrap();
        let warnings = super::refresh_node_binaries(&staging, &installer, target, true);
        assert!(warnings.is_empty(), "{warnings:?}");
        for (name, contents) in [
            ("dora-record-node", "new record"),
            ("dora-replay-node", "new replay"),
        ] {
            let installed = installer.join(node_file(name));
            assert_eq!(std::fs::read_to_string(&installed).unwrap(), contents);
            #[cfg(unix)]
            {
                use std::os::unix::fs::PermissionsExt;
                let mode = std::fs::metadata(&installed).unwrap().permissions().mode();
                assert_eq!(mode & 0o755, 0o755, "{name} must be executable");
            }
        }
        assert_eq!(std::fs::read_dir(&installer).unwrap().count(), 2);
    }
}
