use eyre::Context;

use super::Executable;

#[derive(Debug, clap::Args)]
/// Run runtime
pub struct Runtime;

impl Executable for Runtime {
    fn execute(self) -> eyre::Result<()> {
        // No tracing: Do not set the runtime in the cli.
        // ref: 72b4be808122574fcfda69650954318e0355cc7b cli::run
        dora_runtime::main().context("Failed to run dora-runtime")
    }
}
