use colored::Colorize;
use std::{
    net::{IpAddr, Ipv4Addr},
    path::PathBuf,
};

mod command;
mod common;
mod formatting;
pub mod output;
pub mod session;
mod template;
mod ws_client;
pub use ws_client::WsSession;

pub use command::build;
pub use command::{Executable, Run as RunCommand, run};

/// Default address for *connecting* to a coordinator (client side).
const LOCALHOST: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
/// Default address for the coordinator to *listen* on (server side).
const LISTEN_DEFAULT: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));

#[derive(Debug, clap::Parser)]
#[clap(version = get_version_info())]
pub struct Args {
    #[clap(subcommand)]
    command: command::Command,
}

fn get_version_info() -> clap::builder::Str {
    build_version_string().into()
}

fn build_version_string() -> String {
    // Only return the CLI version for fast --version output.
    // Python version check moved to `adora self check` to avoid spawning
    // external processes on every --version invocation.
    env!("CARGO_PKG_VERSION").to_string()
}

/// Check if a Python adora-rs package is installed and return its version.
#[allow(dead_code)] // used in tests; intended for future `adora self check` command
pub(crate) fn get_python_adora_version() -> Option<String> {
    // Try with uv first
    if let Ok(output) = std::process::Command::new("uv")
        .args(["pip", "show", "adora-rs"])
        .output()
    {
        if output.status.success() {
            if let Some(version) = parse_version_from_pip_show(&output.stdout) {
                return Some(version);
            }
        }
    }

    // Try with regular pip
    if let Ok(output) = std::process::Command::new("pip")
        .args(["show", "adora-rs"])
        .output()
    {
        if output.status.success() {
            if let Some(version) = parse_version_from_pip_show(&output.stdout) {
                return Some(version);
            }
        }
    }

    None
}

#[allow(dead_code)]
pub(crate) fn parse_version_from_pip_show(output: &[u8]) -> Option<String> {
    let output_str = String::from_utf8_lossy(output);
    for line in output_str.lines() {
        if line.starts_with("Version:") {
            return line.split(':').nth(1).map(|s| s.trim().to_string());
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pip_show_valid_output() {
        let output = b"Name: adora-rs\nVersion: 0.1.0\nSummary: some desc\n";
        assert_eq!(
            parse_version_from_pip_show(output),
            Some("0.1.0".to_string())
        );
    }

    #[test]
    fn pip_show_missing_version_line() {
        let output = b"Name: adora-rs\nSummary: some desc\n";
        assert_eq!(parse_version_from_pip_show(output), None);
    }

    #[test]
    fn pip_show_empty() {
        assert_eq!(parse_version_from_pip_show(b""), None);
    }
}

#[derive(Debug, clap::Args)]
pub struct CommandNew {
    /// The entity that should be created
    #[clap(long, value_enum, default_value_t = Kind::Dataflow)]
    kind: Kind,
    /// The programming language that should be used
    #[clap(long, value_enum, default_value_t = Lang::Rust)]
    lang: Lang,
    /// Desired name of the entity
    name: String,
    /// Where to create the entity
    #[clap(hide = true)]
    path: Option<PathBuf>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Kind {
    Dataflow,
    Node,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Lang {
    Rust,
    Python,
    C,
    Cxx,
}

pub fn lib_main(args: Args) {
    if let Err(err) = args.command.execute() {
        eprintln!("\n\n{}", "[ERROR]".bold().red());
        eprintln!("{err:?}");
        std::process::exit(1);
    }
}

#[cfg(feature = "python")]
use clap::Parser;
#[cfg(feature = "python")]
use pyo3::{
    Bound, PyResult, Python, pyfunction, pymodule,
    types::{PyModule, PyModuleMethods},
    wrap_pyfunction,
};

#[cfg(feature = "python")]
#[pyfunction]
fn py_main(_py: Python) -> PyResult<()> {
    Python::initialize();
    // Skip first argument as it is a python call.
    let args = std::env::args_os().skip(1).collect::<Vec<_>>();

    match Args::try_parse_from(args) {
        Ok(args) => lib_main(args),
        Err(err) => {
            eprintln!("{err}");
        }
    }
    Ok(())
}

/// A Python module implemented in Rust.
#[cfg(feature = "python")]
#[pymodule]
fn adora_cli(_py: Python, m: Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(py_main, &m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
