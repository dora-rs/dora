use std::path::PathBuf;

use eyre::bail;

/// Search for a node binary by name, auto-building from the given crate if not found.
pub fn find(binary_name: &str, crate_name: &str) -> eyre::Result<PathBuf> {
    if let Some(path) = search(binary_name) {
        return Ok(path);
    }

    eprintln!("{binary_name} not found, building...");
    let status = std::process::Command::new("cargo")
        .args(["build", "-p", crate_name])
        .status();
    match status {
        Ok(s) if s.success() => {}
        Ok(s) => bail!("failed to build {crate_name} (exit code: {s})"),
        Err(e) => bail!(
            "could not find `{binary_name}` binary and `cargo build` failed: {e}\n\
             Build it manually with: cargo build -p {crate_name}"
        ),
    }

    search(binary_name).ok_or_else(|| {
        eyre::eyre!(
            "built {crate_name} but could not find the binary.\n\
             Try: cargo build -p {crate_name}"
        )
    })
}

fn search(binary_name: &str) -> Option<PathBuf> {
    // Check next to current executable first
    if let Ok(exe) = std::env::current_exe() {
        let dir = exe.parent().unwrap_or(std::path::Path::new("."));
        let candidate = dir.join(binary_name);
        if candidate.exists() {
            return Some(candidate);
        }
        #[cfg(target_os = "windows")]
        {
            let candidate = dir.join(format!("{binary_name}.exe"));
            if candidate.exists() {
                return Some(candidate);
            }
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
        let candidate = cargo_target.join(profile).join(binary_name);
        if candidate.exists() {
            return dunce::canonicalize(candidate).ok();
        }
    }

    None
}
