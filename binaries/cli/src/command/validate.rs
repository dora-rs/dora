use super::Executable;
use dora_core::{
    descriptor::{
        Descriptor, DescriptorExt,
        validate::{check_type_annotations_full, check_wiring},
    },
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
    /// Treat type warnings as errors (non-zero exit code)
    #[clap(long, action)]
    strict_types: bool,
}

impl Executable for Validate {
    fn execute(self) -> eyre::Result<()> {
        let working_dir = self
            .dataflow
            .parent()
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or_else(|| std::path::Path::new("."));

        println!("Validating {}...", self.dataflow.display());

        // Parse and expand modules (no runtime needed)
        let descriptor = Descriptor::blocking_read(&self.dataflow)
            .with_context(|| {
                format!(
                    "failed to read dataflow at `{}`\n\n  \
                     hint: check the file exists and is valid YAML",
                    self.dataflow.display()
                )
            })?
            .expand(working_dir)
            .context("failed to expand modules in dataflow descriptor")?;

        // Check input/output wiring (no build required)
        check_wiring(&descriptor).context("wiring check failed")?;
        println!("Input/output wiring OK.");

        let strict = self.strict_types || descriptor.strict_types.unwrap_or(false);

        // Load user types if a types/ directory exists
        let mut registry = TypeRegistry::new();
        let types_dir = working_dir.join("types");
        if types_dir.is_dir() {
            match registry.load_from_dir(&types_dir) {
                Ok(count) if count > 0 => {
                    println!("Loaded {count} user-defined type(s) from types/");
                }
                Err(e) => {
                    bail!("failed to load user types: {e}");
                }
                _ => {}
            }
        }

        // Run type annotation checks
        let result = check_type_annotations_full(&descriptor, &registry, strict);

        // Print inferences
        for inf in &result.inferences {
            println!("  {inf}");
        }

        if result.warnings.is_empty() {
            println!("All type annotations OK.");
        } else {
            println!("Type warnings:");
            for w in &result.warnings {
                println!("  - {w}");
            }
            let count = result.warnings.len();
            println!("{count} type warning(s) found.");
            if strict {
                bail!("{count} type warning(s) found (--strict mode)");
            }
        }

        Ok(())
    }
}
