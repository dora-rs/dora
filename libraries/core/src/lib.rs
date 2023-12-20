use eyre::{bail, eyre, Context};
use std::{
    env::consts::{DLL_PREFIX, DLL_SUFFIX},
    path::Path,
};

pub use dora_message as message;

pub mod config;
pub mod coordinator_messages;
pub mod daemon_messages;
pub mod descriptor;
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
// First search for `python3` of python as for ubuntu <20, `python` resolves to `python2,7`
// Then search for `python`, this will resolve for windows.
pub fn get_python_path() -> Result<std::path::PathBuf, eyre::ErrReport> {
    let python = match which::which("python3") {
                    Ok(python) => python,
                    Err(_) => which::which("python")
                        .context("failed to find `python` or `python3` in dora-daemon path. Make sure that python is available for the daemon.")?,
                };
    Ok(python)
}
