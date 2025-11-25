use clap::CommandFactory;

use crate::command::Executable;

#[derive(Debug, clap::Args)]
pub struct Completions {
    /// The shell to generate the script for
    #[arg(value_enum)]
    shell: clap_complete::Shell,
}
impl Executable for Completions {
    fn execute(self) -> eyre::Result<()> {
        clap_complete::generate(
            self.shell,
            &mut crate::Args::command(),
            "dora",
            &mut std::io::stdout(),
        );
        Ok(())
    }
}
