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

pub fn lib_main(args: Args) {
    if let Err(err) = args.command.execute() {
        eprintln!("\n\n{}", "[ERROR]".bold().red());
        eprintln!("{err:?}");
        std::process::exit(1);
    }
}
