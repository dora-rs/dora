use eyre::{Context, bail, eyre};
use std::{
    env::consts::{DLL_PREFIX, DLL_SUFFIX},
    ffi::OsStr,
    path::Path,
};

pub use dora_message::{config, uhlc};

#[cfg(feature = "build")]
pub mod build;
pub mod descriptor;
pub mod metadata;
pub mod topics;

pub fn adjust_shared_library_path(path: &Path) -> Result<std::path::PathBuf, eyre::ErrReport> {
    let file_name = path
        .file_name()
        .ok_or_else(|| eyre!("shared library path has no file name"))?
        .to_str()
        .ok_or_else(|| eyre!("shared library file name is not valid UTF8"))?;
    if file_name.starts_with("lib") {
        bail!("Shared library file name must not start with `lib`, prefix is added automatically");
    }
    if path.extension().is_some() {
        bail!("Shared library file name must have no extension, it is added automatically");
    }

    let library_filename = format!("{DLL_PREFIX}{file_name}{DLL_SUFFIX}");

    let path = path.with_file_name(library_filename);
    Ok(path)
}

// Search for python binary.
// 1. If `uv` is available, use `uv python find` to get the Python path
// 2. Otherwise, try `python` and check it's not Python 2
// 3. Fall back to `python3` if `python` is Python 2
pub fn get_python_path() -> Result<std::path::PathBuf, eyre::ErrReport> {
    // First, try using uv if available
    if let Ok(uv_path) = get_uv_path() {
        let output = std::process::Command::new(&uv_path)
            .args(["python", "find"])
            .output();

        if let Ok(output) = output {
            if output.status.success() {
                let python_path = String::from_utf8_lossy(&output.stdout).trim().to_string();
                if !python_path.is_empty() {
                    let path = std::path::PathBuf::from(&python_path);
                    if path.exists() {
                        return Ok(path);
                    }
                }
            }
        }
    }

    // Fall back to finding python directly
    if let Ok(python) = which::which("python") {
        // Check if it's Python 2
        if !is_python2(&python) {
            return Ok(python);
        }
    }

    // Fall back to python3
    which::which("python3").context(
        "failed to find a valid Python 3 installation. Make sure that python3 is available.",
    )
}

fn is_python2(python_path: &std::path::Path) -> bool {
    let output = std::process::Command::new(python_path)
        .args(["--version"])
        .output();

    match output {
        Ok(output) => {
            // Python 2 prints version to stderr, Python 3 to stdout
            let version = if output.stdout.is_empty() {
                String::from_utf8_lossy(&output.stderr)
            } else {
                String::from_utf8_lossy(&output.stdout)
            };
            version.starts_with("Python 2")
        }
        Err(_) => false,
    }
}

// Search for pip binary.
// First search for `pip3` as for ubuntu <20, `pip` can resolves to `python2,7 -m pip`
// Then search for `pip`, this will resolve for windows to python3 -m pip.
pub fn get_pip_path() -> Result<std::path::PathBuf, eyre::ErrReport> {
    let python = match which::which("pip3") {
        Ok(python) => python,
        Err(_) => which::which("pip")
            .context("failed to find `pip3` or `pip`. Make sure that python is available.")?,
    };
    Ok(python)
}

// Search for uv binary.
pub fn get_uv_path() -> Result<std::path::PathBuf, eyre::ErrReport> {
    which::which("uv")
        .context("failed to find `uv`. Make sure to install it using: https://docs.astral.sh/uv/getting-started/installation/")
}

// Helper function to run a program
pub async fn run<S>(program: S, args: &[&str], pwd: Option<&Path>) -> eyre::Result<()>
where
    S: AsRef<OsStr>,
{
    let mut run = tokio::process::Command::new(program);
    run.args(args);

    if let Some(pwd) = pwd {
        run.current_dir(pwd);
    }
    if !run.status().await?.success() {
        eyre::bail!("failed to run {args:?}");
    };
    Ok(())
}
