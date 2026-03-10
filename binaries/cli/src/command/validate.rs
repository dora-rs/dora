use super::Executable;
use adora_core::{
    descriptor::{Descriptor, DescriptorExt, validate::check_type_annotations},
    types::TypeRegistry,
};
use eyre::{Context, bail};
use std::path::PathBuf;

#[derive(Debug, clap::Args)]
/// Validate a dataflow YAML file and check type annotations
pub struct Validate {
    /// Path to the dataflow descriptor file
    #[clap(value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    dataflow: PathBuf,
    /// Treat warnings as errors (non-zero exit code)
    #[clap(long, action)]
    strict: bool,
}

impl Executable for Validate {
    fn execute(self) -> eyre::Result<()> {
        let working_dir = self
            .dataflow
            .parent()
            .unwrap_or_else(|| std::path::Path::new("."));

        println!("Validating {}...", self.dataflow.display());

        // Parse and expand modules (no runtime needed)
        let descriptor = Descriptor::blocking_read(&self.dataflow)
            .with_context(|| format!("failed to read dataflow at `{}`", self.dataflow.display()))?
            .expand(working_dir)
            .context("failed to expand modules")?;

        // Run type annotation checks
        let registry = TypeRegistry::new();
        let warnings = check_type_annotations(&descriptor, &registry);

        if warnings.is_empty() {
            println!("All type annotations OK.");
        } else {
            println!("Type warnings:");
            for w in &warnings {
                println!("  - {w}");
            }
            let count = warnings.len();
            println!("{count} type warning(s) found.");
            if self.strict {
                bail!("{count} type warning(s) found (--strict mode)");
            }
        }

        Ok(())
    }
}
