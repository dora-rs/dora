mod build;
mod check;
mod coordinator;
mod daemon;
mod destroy;
mod graph;
mod list;
mod logs;
mod new;
mod run;
mod runtime;
mod self_;
mod start;
mod stop;
mod up;

use std::path::{Path, PathBuf};

use build::Build;
use check::Check;
use communication_layer_request_reply::TcpRequestReplyConnection;
use coordinator::Coordinator;
use daemon::Daemon;
use destroy::Destroy;
use dora_core::descriptor::Descriptor;
use eyre::{Context, ContextCompat};
use graph::Graph;
use list::ListArgs;
use logs::LogsArgs;
use new::NewArgs;
use run::Run;
use runtime::Runtime;
use self_::SelfSubCommand;
use start::Start;
use stop::Stop;
use up::Up;

use crate::common::cli_and_daemon_on_same_machine;

/// dora-rs cli client
#[derive(Debug, clap::Subcommand)]
pub enum Command {
    Check(Check),
    Graph(Graph),
    Build(Build),
    New(NewArgs),
    Run(Run),
    Up(Up),
    Destroy(Destroy),
    Start(Start),
    Stop(Stop),
    List(ListArgs),
    // Planned for future releases:
    // Dashboard,
    #[command(allow_missing_positional = true)]
    Logs(LogsArgs),
    // Metrics,
    // Stats,
    // Get,
    // Upgrade,
    Daemon(Daemon),
    Runtime(Runtime),
    Coordinator(Coordinator),

    Self_ {
        #[clap(subcommand)]
        command: SelfSubCommand,
    },
}

fn default_tracing() -> eyre::Result<()> {
    #[cfg(feature = "tracing")]
    {
        use dora_tracing::TracingBuilder;

        TracingBuilder::new("dora-cli")
            .with_stdout("warn")
            .build()
            .wrap_err("failed to set up tracing subscriber")?;
    }
    Ok(())
}

pub trait Executable {
    fn execute(self) -> eyre::Result<()>;
}

impl Executable for Command {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Command::Check(args) => args.execute(),
            Command::Coordinator(args) => args.execute(),
            Command::Graph(args) => args.execute(),
            Command::Build(args) => args.execute(),
            Command::New(args) => args.execute(),
            Command::Run(args) => args.execute(),
            Command::Up(args) => args.execute(),
            Command::Destroy(args) => args.execute(),
            Command::Start(args) => args.execute(),
            Command::Stop(args) => args.execute(),
            Command::List(args) => args.execute(),
            Command::Logs(args) => args.execute(),
            Command::Daemon(args) => args.execute(),
            Command::Self_ { command } => command.execute(),
            Command::Runtime(args) => args.execute(),
        }
    }
}
