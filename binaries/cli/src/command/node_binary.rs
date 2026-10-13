use std::path::{Path, PathBuf};
use std::process::Command;

use eyre::bail;

/// The project the node binaries are built from. `dora-record-node` and
/// `dora-replay-node` are `publish = false` — they are delivered through the
/// `dora` CLI rather than crates.io (see their manifests and the CRATES list in
/// `.github/workflows/release.yml`) — so a registry install with a version pin
/// can never resolve them. They are built from the release tag that matches
/// this CLI's version, the same source the ROS 2 bridge node documents.
const REPO_URL: &str = "https://github.com/dora-rs/dora";

/// Search for a node binary by name, auto-building from the given crate if not found.
///
/// Tries `cargo build -p` first (works in the workspace during development),
/// then falls back to installing the crate from this project's release tag
/// (works for standalone installations).
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

    // Fall back to installing from the release tag (works for standalone
    // installations). It has to come from the git tag: the crate is
    // `publish = false`, so `cargo install <crate> --version <v>` always fails.
    let version = env!("CARGO_PKG_VERSION");
    let manual = manual_install(crate_name, version);
    eprintln!(
        "{binary_name} not found in workspace, installing from {REPO_URL} (tag v{version})..."
    );
    let status = Command::new("cargo")
        .args(install_args(crate_name, version))
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

/// `cargo install` arguments for a node crate that is not on crates.io.
fn install_args(crate_name: &str, version: &str) -> Vec<String> {
    vec![
        "install".to_owned(),
        "--git".to_owned(),
        REPO_URL.to_owned(),
        "--tag".to_owned(),
        format!("v{version}"),
        crate_name.to_owned(),
    ]
}

/// The same install as a copy-pasteable command for error messages.
fn manual_install(crate_name: &str, version: &str) -> String {
    let mut command = vec!["cargo".to_owned()];
    command.extend(install_args(crate_name, version));
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
    use super::{REPO_URL, install_args, manual_install, probe_dir};

    /// `dora-record-node` and `dora-replay-node` are `publish = false`, so the
    /// registry form (`cargo install <crate> --version <v>`) can never resolve
    /// them and `dora record` failed after a normal install (#3800). The
    /// install has to come from this project's release tag instead.
    #[test]
    fn unpublished_node_installs_from_the_release_tag() {
        assert_eq!(
            install_args("dora-record-node", "1.1.0"),
            [
                "install",
                "--git",
                REPO_URL,
                "--tag",
                "v1.1.0",
                "dora-record-node",
            ]
        );
        assert_eq!(
            manual_install("dora-record-node", "1.1.0"),
            format!("cargo install --git {REPO_URL} --tag v1.1.0 dora-record-node")
        );
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
