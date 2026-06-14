//! Deliberate error stub: there is no global install step (spec §9).

use crate::command::Executable;

/// Not a real command — hub packages are resolved per-dataflow
#[derive(Debug, clap::Args)]
pub struct Install {
    /// Ignored; present so the error message appears for any invocation
    #[clap(value_name = "ARGS", allow_hyphen_values = true, num_args = 0..)]
    _args: Vec<String>,
}

impl Executable for Install {
    fn execute(self) -> eyre::Result<()> {
        eyre::bail!(
            "`dora hub install` does not exist — hub packages are resolved \
             per-dataflow, cargo-style, not installed into a global store.\n\n  \
             hint: reference the package in your dataflow YAML:\n\n    \
             - id: my-node\n      hub: <name>@<version-req>\n\n  \
             then `dora build dataflow.yml` fetches and builds it.\n  \
             To pre-populate caches for offline use, see `dora hub fetch` (planned)."
        )
    }
}
