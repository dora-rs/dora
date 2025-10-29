use crate::command::{
    Executable,
    topic::{hz::Hz, list::List, watch::Watch},
};

mod hz;
mod list;
mod selector;
mod watch;

/// Topic related commands
#[derive(Debug, clap::Subcommand)]
pub enum Topic {
    List(List),
    Watch(Watch),
    Hz(Hz),
}

impl Executable for Topic {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Topic::List(cmd) => cmd.execute(),
            Topic::Watch(cmd) => cmd.execute(),
            Topic::Hz(cmd) => cmd.execute(),
        }
    }
}
