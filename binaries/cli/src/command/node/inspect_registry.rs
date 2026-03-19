use clap::Args;
use serde::Serialize;
use std::{io::Write, path::PathBuf};
use tabwriter::TabWriter;

use crate::{
    command::{Executable, default_tracing},
    formatting::OutputFormat,
    registry::{RegistryPackage, RegistrySource, resolve_package},
};

/// Inspect a local filesystem node registry index.
///
/// The registry path can point to:
/// - a directory containing `index.toml`
/// - an explicit `index.toml` file path
///
/// Examples:
///
/// List all packages in a registry:
///   dora node inspect-registry --registry ./registry
///
/// Resolve latest package by name:
///   dora node inspect-registry --registry ./registry --name camera_node
///
/// Resolve with version requirement:
///   dora node inspect-registry --registry ./registry --name camera_node --requirement "^0.2"
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct InspectRegistry {
    /// Registry directory or index.toml path
    #[clap(long, value_name = "PATH")]
    registry: PathBuf,
    /// Package name to resolve
    #[clap(long, value_name = "NAME")]
    name: Option<String>,
    /// Optional semver requirement for resolution
    #[clap(long, value_name = "REQ")]
    requirement: Option<String>,
    /// Output format
    #[clap(long, value_name = "FORMAT", default_value_t = OutputFormat::Table)]
    format: OutputFormat,
}

#[derive(Serialize)]
struct PackageRow<'a> {
    name: &'a str,
    version: &'a str,
    entrypoint: &'a str,
    #[serde(skip_serializing_if = "Option::is_none")]
    description: Option<&'a str>,
}

impl Executable for InspectRegistry {
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        if self.requirement.is_some() && self.name.is_none() {
            eyre::bail!("`--requirement` requires `--name`");
        }

        let source = RegistrySource::from_path(&self.registry);
        let index = source.read_index()?;

        match self.name {
            Some(name) => {
                let resolved = resolve_package(&index, &name, self.requirement.as_deref())?;
                match resolved {
                    Some(pkg) => print_packages(std::slice::from_ref(pkg), self.format)?,
                    None => eyre::bail!("no matching package found for `{name}`"),
                }
            }
            None => {
                print_packages(&index.packages, self.format)?;
            }
        }
        Ok(())
    }
}

fn print_packages(packages: &[RegistryPackage], format: OutputFormat) -> eyre::Result<()> {
    match format {
        OutputFormat::Json => {
            for pkg in packages {
                let row = PackageRow {
                    name: &pkg.name,
                    version: &pkg.version,
                    entrypoint: &pkg.entrypoint,
                    description: pkg.description.as_deref(),
                };
                println!("{}", serde_json::to_string(&row)?);
            }
        }
        OutputFormat::Table => {
            let mut tw = TabWriter::new(std::io::stdout().lock());
            tw.write_all(b"NAME\tVERSION\tENTRYPOINT\tDESCRIPTION\n")?;
            for pkg in packages {
                let description = pkg.description.as_deref().unwrap_or("-");
                tw.write_all(
                    format!(
                        "{}\t{}\t{}\t{}\n",
                        pkg.name, pkg.version, pkg.entrypoint, description
                    )
                    .as_bytes(),
                )?;
            }
            tw.flush()?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_requirement_without_name() {
        let args = InspectRegistry {
            registry: PathBuf::from("./registry"),
            name: None,
            requirement: Some("^0.1".to_owned()),
            format: OutputFormat::Json,
        };

        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        let err = rt.block_on(args.execute()).unwrap_err().to_string();
        assert!(err.contains("requires `--name`"));
    }

    #[test]
    fn can_build_registry_source_from_directory() {
        let base = PathBuf::from("./registry");
        let source = RegistrySource::from_path(base.as_path());
        assert_eq!(source.index_path, base.join("index.toml"));
    }

    #[test]
    fn can_build_registry_source_from_file() {
        let source = RegistrySource::from_path(PathBuf::from("./registry/index.toml").as_path());
        assert!(source.index_path.ends_with("registry/index.toml"));
    }

    #[test]
    fn parser_accepts_valid_index_shape() {
        use crate::registry::RegistryIndex;

        let raw = r#"
version = 1

[[packages]]
name = "camera_node"
version = "0.1.0"
entrypoint = "python -m camera_node"
"#;
        let index: RegistryIndex = toml::from_str(raw).unwrap();
        assert_eq!(index.packages.len(), 1);
    }
}
