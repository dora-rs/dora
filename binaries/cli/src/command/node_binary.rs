use std::path::{Path, PathBuf};
use std::process::Command;

use eyre::bail;

/// Search for a node binary by name, auto-building from the given crate if not found.
///
/// Tries `cargo build -p` first (works in the workspace during development),
/// then falls back to installing the crate from crates.io (works for
/// standalone installations).
pub fn find(binary_name: &str, crate_name: &str) -> eyre::Result<PathBuf> {
    if let Some(path) = search(binary_name) {
        return Ok(path);
    }

    eprintln!("{binary_name} not found, building...");

    // Try workspace build first (fast, works during development)
    let build_ok = Command::new("cargo")
        .args(["build", "-p", crate_name])
        .status()
        .is_ok_and(|s| s.success());

    if build_ok && let Some(path) = search(binary_name) {
        return Ok(path);
    }

    // Fall back to installing the crate version that matches this CLI.
    let version = env!("CARGO_PKG_VERSION");
    let root = install_root(version);
    let manual = manual_install(crate_name, version, &root);
    eprintln!(
        "{binary_name} not found in workspace, installing {crate_name} {version} into {}...",
        root.display()
    );
    let status = Command::new("cargo")
        .args(install_args(crate_name, version, &root))
        .status();
    match status {
        Ok(s) if s.success() => {}
        Ok(s) => bail!("failed to install {crate_name} (exit code: {s})"),
        Err(e) => bail!(
            "could not find `{binary_name}` and installation failed: {e}\n\
             Install it manually with: {manual}"
        ),
    }

    search(binary_name).ok_or_else(|| {
        eyre::eyre!(
            "installed {crate_name} but could not find the binary.\n\
             Try: {manual}"
        )
    })
}

/// The directory that [`find`] installs node binaries into for the given CLI
/// version: `~/.dora/nodes/<version>`. The binaries end up in its `bin/`
/// subdirectory, which [`search`] checks.
///
/// Using a directory of our own means the binary is found right after the
/// install, wherever cargo would install by default (`$CARGO_INSTALL_ROOT`,
/// `install.root`, or a `$CARGO_HOME/bin` that may not be on `PATH`). Using
/// one directory per version means a released CLI never picks up a node that
/// an older release installed this way. Builds from `main` carry the last
/// release's version, so they share that release's directory; a version
/// check of the found binary would close that gap.
fn install_root(version: &str) -> PathBuf {
    super::coordinator::dora_dir().join("nodes").join(version)
}

/// The `cargo install` arguments that install exactly `version` of the node
/// crate `crate_name` from crates.io into `root`.
///
/// `--locked` builds with the `Cargo.lock` published in the crate, so the
/// node uses the dependency versions the release was tested with, rather
/// than newer semver-compatible ones resolved at install time.
fn install_args(crate_name: &str, version: &str, root: &Path) -> Vec<String> {
    vec![
        "install".to_owned(),
        "--locked".to_owned(),
        "--root".to_owned(),
        root.display().to_string(),
        "--version".to_owned(),
        format!("={version}"),
        crate_name.to_owned(),
    ]
}

/// The [`install_args`] install as a shell command, for users to run by hand
/// when the automatic install fails.
fn manual_install(crate_name: &str, version: &str, root: &Path) -> String {
    let mut command = vec!["cargo".to_owned()];
    command.extend(install_args(crate_name, version, root));
    command.join(" ")
}

/// Look for `binary_name` in `dir`, also probing the platform executable
/// suffix (e.g. `.exe` on Windows) so a `cargo build` artifact is found.
fn probe_dir(dir: &Path, binary_name: &str) -> Option<PathBuf> {
    let candidate = dir.join(binary_name);
    if candidate.exists() {
        return Some(candidate);
    }
    let suffix = std::env::consts::EXE_SUFFIX;
    if !suffix.is_empty() {
        let candidate = dir.join(format!("{binary_name}{suffix}"));
        if candidate.exists() {
            return Some(candidate);
        }
    }
    None
}

fn search(binary_name: &str) -> Option<PathBuf> {
    // Check next to current executable first
    if let Ok(exe) = std::env::current_exe() {
        let dir = exe.parent().unwrap_or(Path::new("."));
        if let Some(candidate) = probe_dir(dir, binary_name) {
            return Some(candidate);
        }
    }

    // Check where the `cargo install` fallback in `find` puts it, before PATH
    // so that a node an older CLI installed elsewhere does not win.
    let fallback_bin = install_root(env!("CARGO_PKG_VERSION")).join("bin");
    if let Some(candidate) = probe_dir(&fallback_bin, binary_name) {
        return Some(candidate);
    }

    // Check PATH
    if let Ok(path) = which::which(binary_name) {
        return Some(path);
    }

    // Check cargo target directory (development)
    let cargo_target = std::env::var("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from("target"));
    for profile in ["debug", "release"] {
        if let Some(candidate) = probe_dir(&cargo_target.join(profile), binary_name) {
            // Fall back if canonicalization fails (e.g. a transient race against
            // the build, or a symlink/permission edge case). The other branches
            // above return the located path directly; discarding a binary we
            // just found — and thereby also skipping the remaining `release`
            // probe — would be worse. Make the fallback absolute so it still
            // resolves like the other branches' paths even if the working
            // directory changes before the binary is spawned (`cargo_target`
            // may be the relative default `target/`).
            return Some(
                dunce::canonicalize(&candidate)
                    .or_else(|_| std::path::absolute(&candidate))
                    .unwrap_or(candidate),
            );
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::{install_args, install_root, manual_install, probe_dir};

    /// The fallback installs the exact version that matches the CLI, with the
    /// published `Cargo.lock`, into the CLI's own install directory (#3800).
    #[test]
    fn node_installs_the_matching_version() {
        let root = Path::new("/home/u/.dora/nodes/1.2.0");
        assert_eq!(
            install_args("dora-record-node", "1.2.0", root),
            [
                "install",
                "--locked",
                "--root",
                "/home/u/.dora/nodes/1.2.0",
                "--version",
                "=1.2.0",
                "dora-record-node",
            ]
        );
        assert_eq!(
            manual_install("dora-record-node", "1.2.0", root),
            "cargo install --locked --root /home/u/.dora/nodes/1.2.0 --version =1.2.0 dora-record-node"
        );
    }

    /// The fallback installs into a root of its own, one per CLI version, so
    /// `search` finds the result no matter where cargo would install by
    /// default, and never a node another CLI version installed there.
    #[test]
    fn install_root_is_per_version_under_dora_dir() {
        let root = install_root("1.2.0");
        assert!(root.ends_with(Path::new(".dora").join("nodes").join("1.2.0")));
        assert_ne!(root, install_root("1.1.0"));
    }

    #[test]
    fn probe_dir_finds_binary_with_platform_suffix() {
        // Build a unique temp dir without external crates.
        let mut dir = std::env::temp_dir();
        dir.push(format!("dora-node-binary-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();

        // Write a file under the real executable name (`foo` on unix,
        // `foo.exe` on windows), mirroring what `cargo build` produces.
        let file_name = format!("foo{}", std::env::consts::EXE_SUFFIX);
        let file_path = dir.join(&file_name);
        std::fs::write(&file_path, b"binary").unwrap();

        // `probe_dir` must locate it from the bare name on every platform.
        let found = probe_dir(&dir, "foo");
        assert_eq!(found.as_deref(), Some(file_path.as_path()));

        // A name that does not exist yields None.
        assert_eq!(probe_dir(&dir, "does-not-exist"), None);

        std::fs::remove_dir_all(&dir).ok();
    }
}
