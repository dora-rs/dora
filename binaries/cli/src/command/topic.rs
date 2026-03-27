use crate::command::{
    Executable,
    topic::{echo::Echo, hz::Hz, info::Info, list::List},
};

mod echo;
mod hz;
mod info;
mod list;
mod selector;

/// Manage and inspect dataflow topics.
#[derive(Debug, clap::Subcommand)]
pub enum Topic {
    List(List),
    Echo(Echo),
    Hz(Hz),
    Info(Info),
}

impl Executable for Topic {
    async fn execute(self) -> eyre::Result<()> {
        match self {
            Topic::List(cmd) => cmd.execute().await,
            Topic::Echo(cmd) => cmd.execute().await,
            Topic::Hz(cmd) => cmd.execute().await,
            Topic::Info(cmd) => cmd.execute().await,
        }
    }
}
