use crate::command::{
    inspect::{data::InspectData, hz::Hz},
    Executable,
};

mod data;
mod hz;
pub mod selector;

/// Inspect data in terminal.
#[derive(Debug, clap::Subcommand)]
pub enum Inspect {
    Hz(Hz),
    Data(InspectData),
}

impl Executable for Inspect {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Inspect::Data(cmd) => cmd.execute(),
            Inspect::Hz(cmd) => cmd.execute(),
        }
    }
}
