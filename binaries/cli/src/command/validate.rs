use super::Executable;
use dora_core::{
    descriptor::{
        Descriptor, DescriptorExt,
        validate::{check_type_annotations_full, check_wiring},
    },
    manifest::{NodeManifest, inject::inject_adjacent_manifests},
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
    /// Do not access the network for hub index refreshes (cache only)
    #[clap(long, action, conflicts_with = "node_manifest")]
    offline: bool,
}

impl Executable for Validate {
    fn execute(self) -> eyre::Result<()> {
        if let Some(manifest_path) = &self.node_manifest {
            return validate_node_manifest(manifest_path);
        }
        let dataflow = self
            .dataflow
            .expect("clap guarantees dataflow when --node-manifest is absent");
        validate_dataflow(&dataflow, self.strict_types, self.offline)
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

fn validate_dataflow(dataflow: &Path, strict_types: bool, offline: bool) -> eyre::Result<()> {
    let working_dir = dataflow
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| std::path::Path::new("."));

    println!("Validating {}...", dataflow.display());

    // Parse and expand modules (no runtime needed)
    let mut descriptor = Descriptor::blocking_read(dataflow)
        .with_context(|| {
            format!(
                "failed to read dataflow at `{}`\n\n  \
                     hint: check the file exists and is valid YAML",
                dataflow.display()
            )
        })?
        .expand(working_dir)
        .context("failed to expand modules in dataflow descriptor")?;

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

    // Resolve hub: references against the (cached) index so their contracts
    // take part in validation (spec §6.2 ordering note)
    let hub_resolution = super::build::hub::resolve_hub_nodes(
        &mut descriptor,
        &mut registry,
        offline,
        None,
        &std::collections::BTreeMap::<String, std::path::PathBuf>::new(),
    )?;
    for note in &hub_resolution.notes {
        println!("  {note}");
    }

    // Check input/output wiring (no build required)
    check_wiring(&descriptor).context("wiring check failed")?;
    println!("Input/output wiring OK.");

    // Inject contracts from node manifests adjacent to path: nodes (§6.2)
    let injection = inject_adjacent_manifests(&mut descriptor, working_dir, &mut registry);
    for note in &injection.notes {
        println!("  {note}");
    }

    // Run type annotation checks
    let result = check_type_annotations_full(&descriptor, &registry, strict);

    // Print inferences
    for inf in &result.inferences {
        println!("  {inf}");
    }

    let count = hub_resolution.warnings.len() + injection.warnings.len() + result.warnings.len();
    if count == 0 {
        println!("All type annotations OK.");
    } else {
        println!("Type warnings:");
        for w in &hub_resolution.warnings {
            println!("  - {w}");
        }
        for w in &injection.warnings {
            println!("  - {w}");
        }
        for w in &result.warnings {
            println!("  - {w}");
        }
        println!("{count} type warning(s) found.");
        if strict {
            bail!("{count} type warning(s) found (--strict mode)");
        }
    }

    Ok(())
}
