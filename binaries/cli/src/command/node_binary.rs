use std::path::{Path, PathBuf};

use eyre::bail;

/// Search for a node binary by name, auto-building from the given crate if not found.
///
/// Tries `cargo build -p` first (works in the workspace during development),
/// then falls back to `cargo install` (works for standalone installations).
pub fn find(binary_name: &str, crate_name: &str) -> eyre::Result<PathBuf> {
    if let Some(path) = search(binary_name) {
        return Ok(path);
    }

    eprintln!("{binary_name} not found, building...");

    // Try workspace build first (fast, works during development)
    let build_ok = std::process::Command::new("cargo")
        .args(["build", "-p", crate_name])
        .status()
        .is_ok_and(|s| s.success());

    if build_ok && let Some(path) = search(binary_name) {
        return Ok(path);
    }

    // Fall back to cargo install (works for standalone installations)
    eprintln!("{binary_name} not found in workspace, installing via cargo install...");
    let version = env!("CARGO_PKG_VERSION");
    let status = std::process::Command::new("cargo")
        .args(["install", crate_name, "--version", version])
        .status();
    match status {
        Ok(s) if s.success() => {}
        Ok(s) => bail!("failed to install {crate_name} (exit code: {s})"),
        Err(e) => bail!(
            "could not find `{binary_name}` and installation failed: {e}\n\
             Install it manually with: cargo install {crate_name}"
        ),
    }

    search(binary_name).ok_or_else(|| {
        eyre::eyre!(
            "installed {crate_name} but could not find the binary.\n\
             Try: cargo install {crate_name}"
        )
    })
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
            return dunce::canonicalize(candidate).ok();
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::probe_dir;

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
