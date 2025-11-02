use crate::command::{
    Executable,
    topic::{echo::Echo, hz::Hz, list::List},
};

mod echo;
mod hz;
mod list;
mod selector;

/// Topic related commands
#[derive(Debug, clap::Subcommand)]
pub enum Topic {
    List(List),
    Echo(Echo),
    Hz(Hz),
}

impl Executable for Topic {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Topic::List(cmd) => cmd.execute(),
            Topic::Echo(cmd) => cmd.execute(),
            Topic::Hz(cmd) => cmd.execute(),
        }
    }
}
