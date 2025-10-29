use crate::command::{
    Executable,
    topic::{watch::Watch, hz::Hz},
};

mod watch;
mod hz;
pub mod selector;

/// Topic related commands
#[derive(Debug, clap::Subcommand)]
pub enum Topic {
    Watch(Watch),
    Hz(Hz),
}

impl Executable for Topic {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Topic::Watch(cmd) => cmd.execute(),
            Topic::Hz(cmd) => cmd.execute(),
        }
    }
}
