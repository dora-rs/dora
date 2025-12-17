mod build;
mod completion;
mod coordinator;
mod daemon;
mod destroy;
mod graph;
mod inspect;
mod list;
mod logs;
mod new;
mod node;
mod run;
mod runtime;
mod self_;
mod start;
mod stop;
mod system;
mod topic;
mod up;

pub use build::build;
pub use run::{run, run_func};
pub use system::check_environment;

use build::Build;
use completion::Completion;
use coordinator::Coordinator;
use daemon::Daemon;
use destroy::Destroy;
use eyre::Context;
use graph::Graph;
use inspect::Inspect;
use list::ListArgs;
use logs::LogsArgs;
use new::NewArgs;
use node::Node;
use run::Run;
use runtime::Runtime;
use self_::SelfSubCommand;
use start::Start;
use stop::Stop;
use system::System;
use topic::Topic;
use up::Up;

/// dora-rs cli client
#[derive(Debug, clap::Subcommand)]
pub enum Command {
    #[clap(subcommand)]
    System(System),
    /// Alias for `system status`
    Check(system::status::Status),
    Graph(Graph),
    Build(Build),
    New(NewArgs),
    Run(Run),
    Up(Up),
    Destroy(Destroy),
    Start(Start),
    Stop(Stop),
    #[clap(alias = "ps")]
    List(ListArgs),
    // Planned for future releases:
    // Dashboard,
    #[command(allow_missing_positional = true)]
    Logs(LogsArgs),
    // Metrics,
    // Stats,
    // Get,
    // Upgrade,
    #[clap(subcommand)]
    Inspect(Inspect),
    Daemon(Daemon),
    Runtime(Runtime),
    Coordinator(Coordinator),
    #[clap(subcommand)]
    Topic(Topic),
    #[clap(subcommand)]
    Node(Node),

    Completion(Completion),
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
            .with_stdout("warn", false)
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
            Command::System(args) => args.execute(),
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
            Command::Inspect(args) => args.execute(),
            Command::Daemon(args) => args.execute(),
            Command::Self_ { command } => command.execute(),
            Command::Runtime(args) => args.execute(),
            Command::Topic(args) => args.execute(),
            Command::Node(args) => args.execute(),
            Command::Completion(args) => args.execute(),
        }
    }
}
