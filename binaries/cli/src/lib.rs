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

pub(crate) mod tcp;

pub use command::{Executable, Run as RunCommand, run, run_func};
pub use command::{build, build_async};

const LOCALHOST: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
const LISTEN_WILDCARD: IpAddr = IpAddr::V4(Ipv4Addr::new(0, 0, 0, 0));

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
    let cli_version = env!("CARGO_PKG_VERSION");

    let mut version_output = format!("{}\n", cli_version);

    version_output.push_str(&format!("dora-message: {}\n", dora_message::VERSION));

    // Try to detect Python dora-rs version
    match get_python_dora_version() {
        Some(python_version) => {
            version_output.push_str(&format!("dora-rs (Python): {}\n", python_version));

            // Check for version mismatch
            if python_version != cli_version {
                version_output.push_str(&format!(
                    "\n⚠️  WARNING: Version mismatch detected!\n   CLI version ({}) differs from Python dora-rs version ({})\n",
                    cli_version,
                    python_version
                ));
            }
        }
        None => {
            version_output.push_str("dora-rs (Python): not found\n");
        }
    }

    version_output
}

pub(crate) fn get_python_dora_version() -> Option<String> {
    // Try with uv first
    if let Ok(output) = std::process::Command::new("uv")
        .args(["pip", "show", "dora-rs"])
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
        .args(["show", "dora-rs"])
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

fn parse_version_from_pip_show(output: &[u8]) -> Option<String> {
    let output_str = String::from_utf8_lossy(output);
    for line in output_str.lines() {
        if line.starts_with("Version:") {
            return line.split(':').nth(1).map(|s| s.trim().to_string());
        }
    }
    None
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

pub async fn lib_main(args: Args) {
    if let Err(err) = args.command.execute().await {
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
    pyo3::prepare_freethreaded_python();
    // Skip first argument as it is a python call.
    let args = std::env::args_os().skip(1).collect::<Vec<_>>();

    match Args::try_parse_from(args) {
        Ok(args) => {
            let rt = tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .expect("failed to create tokio runtime");
            rt.block_on(lib_main(args));
        }
        Err(err) => {
            eprintln!("{err}");
        }
    }
    Ok(())
}

/// A Python module implemented in Rust.
#[cfg(feature = "python")]
#[pymodule]
fn dora_cli(_py: Python, m: Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(py_main, &m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
