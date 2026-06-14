use super::Executable;
use dora_core::{
    descriptor::{
        Descriptor, DescriptorExt,
        validate::{check_type_annotations_full, check_wiring},
    },
    manifest::NodeManifest,
    types::TypeRegistry,
};
use eyre::{Context, bail};
use std::path::{Path, PathBuf};

#[derive(Debug, clap::Args)]
/// Validate a dataflow YAML file (or a node manifest) and check type annotations
pub struct Validate {
    /// Path to the dataflow descriptor file
    #[clap(
        value_name = "PATH",
        value_hint = clap::ValueHint::FilePath,
        required_unless_present = "node_manifest",
        conflicts_with = "node_manifest"
    )]
    dataflow: Option<PathBuf>,
    /// Treat type warnings as errors (non-zero exit code)
    #[clap(long, action, conflicts_with = "node_manifest")]
    strict_types: bool,
    /// Validate a node manifest (dora-node.yml) instead of a dataflow
    #[clap(long, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    node_manifest: Option<PathBuf>,
}

impl Executable for Validate {
    fn execute(self) -> eyre::Result<()> {
        if let Some(manifest_path) = &self.node_manifest {
            return validate_node_manifest(manifest_path);
        }
        let dataflow = self
            .dataflow
            .expect("clap guarantees dataflow when --node-manifest is absent");
        validate_dataflow(&dataflow, self.strict_types)
    }
}

fn validate_node_manifest(path: &Path) -> eyre::Result<()> {
    println!("Validating node manifest {}...", path.display());
    let manifest = NodeManifest::read(path)?;

    // Built-in std/ types only; types shipped in the manifest's `types:`
    // block are resolved by validate() against the manifest itself.
    let registry = TypeRegistry::new();
    manifest.validate_strict(&registry)?;

    println!("Node manifest OK.");
    Ok(())
}

fn validate_dataflow(dataflow: &Path, strict_types: bool) -> eyre::Result<()> {
    let working_dir = dataflow
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| std::path::Path::new("."));

    println!("Validating {}...", dataflow.display());

    // Parse and expand modules (no runtime needed)
    let descriptor = Descriptor::blocking_read(dataflow)
        .with_context(|| {
            format!(
                "failed to read dataflow at `{}`\n\n  \
                     hint: check the file exists and is valid YAML",
                dataflow.display()
            )
        })?
        .expand(working_dir)
        .context("failed to expand modules in dataflow descriptor")?;

    // Check input/output wiring (no build required)
    check_wiring(&descriptor).context("wiring check failed")?;
    println!("Input/output wiring OK.");

    let strict = strict_types || descriptor.strict_types.unwrap_or(false);

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
