mod build;
mod check;
mod coordinator;
mod daemon;
mod destroy;
mod graph;
mod inspect;
mod list;
mod logs;
mod new;
mod run;
mod runtime;
mod self_;
mod start;
mod stop;
mod up;

use enum_dispatch::enum_dispatch;
pub use run::run_func;

use build::Build;
use check::Check;
use coordinator::Coordinator;
use daemon::Daemon;
use destroy::Destroy;
use eyre::Context;
use graph::Graph;
use inspect::Inspect;
use list::ListArgs;
use logs::LogsArgs;
use new::NewArgs;
use run::Run;
use runtime::Runtime;
use self_::Self_;
use start::Start;
use stop::Stop;
use up::Up;

/// dora-rs cli client
#[enum_dispatch(Executable)]
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
    Inspect(Inspect),

    Self_(Self_),
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

#[enum_dispatch]
pub trait Executable {
    fn execute(self) -> eyre::Result<()>;
}
