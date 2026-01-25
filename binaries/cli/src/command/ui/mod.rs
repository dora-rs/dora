use super::Executable;
use rerun::Rerun;

mod rerun;

/// UI visualization commands
#[derive(Debug, clap::Subcommand)]
pub enum Ui {
    /// Launch Rerun visualization viewer for Dora dataflows
    Rerun(Rerun),
}

impl Executable for Ui {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Ui::Rerun(args) => args.execute(),
        }
    }
}
