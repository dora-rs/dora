use super::Executable;
use dora_core::descriptor::{Descriptor, DescriptorExt, check_module_file};
use eyre::Context;
use std::path::PathBuf;

#[derive(Debug, clap::Args)]
/// Expand module references in a dataflow and print the flat result
pub struct Expand {
    /// Path to the dataflow descriptor file (or module file with --module)
    #[clap(value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    dataflow: PathBuf,
    /// Validate a standalone module file instead of a full dataflow
    #[clap(long, action)]
    module: bool,
}

impl Executable for Expand {
    fn execute(self) -> eyre::Result<()> {
        if self.module {
            check_module_file(&self.dataflow)?;
            println!("Module file is valid: {}", self.dataflow.display());
            return Ok(());
        }

        let working_dir = self
            .dataflow
            .parent()
            .unwrap_or_else(|| std::path::Path::new("."));
        let descriptor = Descriptor::blocking_read(&self.dataflow)
            .with_context(|| format!("failed to read dataflow at `{}`", self.dataflow.display()))?
            .expand(working_dir)
            .context("failed to expand modules")?;
        let yaml = serde_yaml::to_string(&descriptor)
            .context("failed to serialize expanded descriptor")?;
        print!("{yaml}");
        // serde_yaml includes a trailing newline; if not, ensure one for
        // shell pipeline composability
        if !yaml.ends_with('\n') {
            println!();
        }
        Ok(())
    }
}
