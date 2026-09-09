use colored::Colorize;
use std::{
    net::{IpAddr, Ipv4Addr},
    path::PathBuf,
};

mod command;
mod common;
mod env_overrides;
mod formatting;
pub mod output;
pub mod session;
mod template;
mod ws_client;
pub use ws_client::WsSession;

pub use command::{BuildConfig, build};
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
    env!("CARGO_PKG_VERSION").to_string()
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

/// Parse a command line and run it, the way both dora entry points need to.
///
/// `main.rs` and the `dora` console script in the `dora-rs-cli` wheel both land
/// here, so clap's exit behaviour -- usage errors to stderr with status 2,
/// `--help` / `--version` to stdout with status 0 -- is defined once instead of
/// being reimplemented per entry point.
pub fn lib_main_from_argv<I, T>(argv: I)
where
    I: IntoIterator<Item = T>,
    T: Into<std::ffi::OsString> + Clone,
{
    use clap::Parser as _;

    let argv: Vec<std::ffi::OsString> = argv.into_iter().map(Into::into).collect();

    // Record argv[0] -- the console-script/executable path -- so that, in the
    // `dora-rs-cli` wheel, `dora up` / `dora cluster up` re-spawn the real
    // `dora` binary via `sys.argv[0]` rather than a fixed `args_os()` index that
    // is wrong on Windows (#3327). `py_main` passes the normalized `sys.argv`
    // here, whose first element is that path on both Unix and Windows. Harmless
    // in the standalone binary, where `dora_executable_path` uses `current_exe`.
    if let Some(exe) = argv.first() {
        command::set_python_executable_path(exe.clone());
    }

    lib_main(Args::try_parse_from(argv).unwrap_or_else(|err| err.exit()))
}

pub fn lib_main(args: Args) {
    if let Err(err) = args.command.execute() {
        eprintln!("\n\n{}", "[ERROR]".bold().red());
        eprintln!("{err:?}");
        std::process::exit(1);
    }
}
