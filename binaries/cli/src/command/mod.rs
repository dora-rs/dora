mod build;
mod completion;
mod coordinator;
mod daemon;
mod destroy;
mod graph;
pub mod inspect;
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
pub use run::{Run, run};

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

/// adora-rs cli client
#[derive(Debug, clap::Subcommand)]
pub enum Command {
    // -- Lifecycle --
    /// Run a dataflow locally without coordinator or daemon
    #[clap(display_order = 1)]
    Run(Run),
    /// Start coordinator and daemon in local mode
    #[clap(display_order = 2)]
    Up(Up),
    /// Tear down coordinator and daemon. Stops any running dataflows first.
    #[clap(name = "down", alias = "destroy", display_order = 3)]
    Destroy(Destroy),
    /// Run build commands provided in the given dataflow
    #[clap(display_order = 4)]
    Build(Build),
    /// Start a dataflow on a running coordinator
    #[clap(display_order = 5)]
    Start(Start),
    /// Stop a running dataflow
    #[clap(display_order = 6)]
    Stop(Stop),

    // -- Monitoring --
    /// List running dataflows
    #[clap(alias = "ps", display_order = 10)]
    List(ListArgs),
    /// Show logs of a given dataflow and node
    #[command(allow_missing_positional = true)]
    #[clap(display_order = 11)]
    Logs(LogsArgs),
    /// Inspect running dataflows in real-time
    #[clap(subcommand, display_order = 12)]
    Inspect(Inspect),
    /// Manage and inspect dataflow topics
    #[clap(subcommand, display_order = 13)]
    Topic(Topic),
    /// Manage and inspect dataflow nodes
    #[clap(subcommand, display_order = 14)]
    Node(Node),

    // -- Setup --
    /// Check system health
    #[clap(alias = "check", display_order = 20)]
    Status(system::status::Status),
    /// Generate a new project or node
    #[clap(display_order = 21)]
    New(NewArgs),
    /// Visualize a dataflow as a graph
    #[clap(display_order = 22)]
    Graph(Graph),
    /// System management commands
    #[clap(subcommand, display_order = 23)]
    System(System),

    // -- Utility --
    /// Generate shell completions
    #[clap(display_order = 30)]
    Completion(Completion),
    /// CLI self-management (update, uninstall)
    #[clap(display_order = 31)]
    Self_ {
        #[clap(subcommand)]
        command: SelfSubCommand,
    },

    // -- Hidden: internal / aliases --
    #[clap(hide = true)]
    Daemon(Daemon),
    #[clap(hide = true)]
    Runtime(Runtime),
    #[clap(hide = true)]
    Coordinator(Coordinator),
    /// Alias for `inspect top`
    #[clap(hide = true)]
    Top(inspect::top::Top),
}

fn default_tracing() -> eyre::Result<()> {
    #[cfg(feature = "tracing")]
    {
        use adora_tracing::TracingBuilder;

        TracingBuilder::new("adora-cli")
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
            Command::Run(args) => args.execute(),
            Command::Up(args) => args.execute(),
            Command::Destroy(args) => args.execute(),
            Command::Build(args) => args.execute(),
            Command::Start(args) => args.execute(),
            Command::Stop(args) => args.execute(),
            Command::List(args) => args.execute(),
            Command::Logs(args) => args.execute(),
            Command::Inspect(args) => args.execute(),
            Command::Topic(args) => args.execute(),
            Command::Node(args) => args.execute(),
            Command::Status(args) => args.execute(),
            Command::New(args) => args.execute(),
            Command::Graph(args) => args.execute(),
            Command::System(args) => args.execute(),
            Command::Completion(args) => args.execute(),
            Command::Self_ { command } => command.execute(),
            Command::Daemon(args) => args.execute(),
            Command::Runtime(args) => args.execute(),
            Command::Coordinator(args) => args.execute(),
            Command::Top(args) => args.execute(),
        }
    }
}
