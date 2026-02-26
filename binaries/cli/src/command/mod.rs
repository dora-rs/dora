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
mod version;

pub use build::{build, build_async};
pub use run::{Run, run, run_func};

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
use runtime::Runtime;
use self_::SelfSubCommand;
use start::Start;
use stop::Stop;
use system::System;
use topic::Topic;
use up::Up;
use version::Version;

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
    Node(Node),
    #[clap(subcommand)]
    Topic(Topic),
    Version(Version),
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
    fn execute(self) -> impl std::future::Future<Output = eyre::Result<()>> + Send;
}

impl Executable for Command {
    async fn execute(self) -> eyre::Result<()> {
        match self {
            Command::System(args) => args.execute().await,
            Command::Check(args) => args.execute().await,
            Command::Coordinator(args) => args.execute().await,
            Command::Graph(args) => args.execute().await,
            Command::Build(args) => args.execute().await,
            Command::New(args) => args.execute().await,
            Command::Run(args) => args.execute().await,
            Command::Up(args) => args.execute().await,
            Command::Destroy(args) => args.execute().await,
            Command::Start(args) => args.execute().await,
            Command::Stop(args) => args.execute().await,
            Command::List(args) => args.execute().await,
            Command::Logs(args) => args.execute().await,
            Command::Inspect(args) => args.execute().await,
            Command::Daemon(args) => args.execute().await,
            Command::Self_ { command } => command.execute().await,
            Command::Runtime(args) => args.execute().await,
            Command::Topic(args) => args.execute().await,
            Command::Node(args) => args.execute().await,
            Command::Version(args) => args.execute().await,
            Command::Completion(args) => args.execute().await,
        }
    }
}
