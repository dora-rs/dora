use clap::Args;

use super::{Executable, default_tracing};

#[derive(Debug, Args)]
/// Generate a new project or node. Choose the language between Rust, Python, C or C++.
pub struct NewArgs {
    #[clap(flatten)]
    // TODO!: better impl
    args: crate::CommandNew,
    /// Internal flag for creating with path dependencies
    #[clap(hide = true, long)]
    pub internal_create_with_path_dependencies: bool,
}

impl Executable for NewArgs {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        crate::template::create(self.args, self.internal_create_with_path_dependencies)
    }
}
