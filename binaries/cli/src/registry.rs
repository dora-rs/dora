use crate::package_metadata::{
    DependencyRequirement, PackageDefinition, PackageIdentity, parse_version, parse_version_req,
    validate_entrypoint, validate_name,
};
use eyre::{Context, bail};
use semver::VersionReq;
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

const REGISTRY_INDEX_VERSION: u32 = 1;

#[derive(Debug, Clone)]
pub struct RegistryIndex {
    pub version: u32,
    pub packages: Vec<RegistryPackageRecord>,
}

#[derive(Debug, Clone)]
pub struct RegistryPackageRecord {
    pub package: PackageDefinition,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct RawRegistryIndex {
    pub version: u32,
    #[serde(default)]
    pub packages: Vec<RawRegistryPackage>,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct RawRegistryPackage {
    pub name: String,
    pub version: String,
    pub entrypoint: String,
    #[serde(default)]
    pub description: Option<String>,
    #[serde(default)]
    pub dependencies: BTreeMap<String, RawRegistryDependency>,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct RawRegistryDependency {
    pub version: String,
}

#[derive(Debug, Clone)]
pub struct RegistrySource {
    pub index_path: PathBuf,
}

impl RegistrySource {
    pub fn from_path(path: &Path) -> Self {
        let index_path = match path.extension().and_then(|ext| ext.to_str()) {
            Some("toml") => path.to_owned(),
            _ => path.join("index.toml"),
        };
        Self { index_path }
    }

    pub fn read_index(&self) -> eyre::Result<RegistryIndex> {
        let raw = std::fs::read_to_string(&self.index_path).with_context(|| {
            format!(
                "failed to read registry index at `{}`",
                self.index_path.display()
            )
        })?;
        let index: RawRegistryIndex = toml::from_str(&raw).with_context(|| {
            format!(
                "failed to parse registry index at `{}`",
                self.index_path.display()
            )
        })?;
        normalize_index(index)
    }
}

pub fn normalize_index(index: RawRegistryIndex) -> eyre::Result<RegistryIndex> {
    if index.version != REGISTRY_INDEX_VERSION {
        bail!(
            "unsupported registry index version `{}` (expected `{}`)",
            index.version,
            REGISTRY_INDEX_VERSION
        );
    }

    let packages = index
        .packages
        .into_iter()
        .map(normalize_package)
        .collect::<eyre::Result<Vec<_>>>()?;

    Ok(RegistryIndex {
        version: index.version,
        packages,
    })
}

pub fn resolve_package<'a>(
    index: &'a RegistryIndex,
    name: &str,
    requirement: Option<&str>,
) -> eyre::Result<Option<&'a RegistryPackageRecord>> {
    let version_req = match requirement {
        Some(req) => Some(
            VersionReq::parse(req)
                .with_context(|| format!("invalid version requirement `{req}`"))?,
        ),
        None => None,
    };

    let mut matching = index
        .packages
        .iter()
        .filter(|pkg| pkg.package.identity.name == name)
        .collect::<Vec<_>>();

    if let Some(version_req) = version_req {
        matching.retain(|pkg| version_req.matches(&pkg.package.identity.version));
    }

    matching.sort_by(|a, b| b.package.identity.version.cmp(&a.package.identity.version));
    Ok(matching.into_iter().next())
}

fn normalize_package(pkg: RawRegistryPackage) -> eyre::Result<RegistryPackageRecord> {
    validate_name(&pkg.name, "package.name")?;
    let version = parse_version(&pkg.version, "package.version")?;
    validate_entrypoint(&pkg.entrypoint, "package.entrypoint")?;

    let dependencies = pkg
        .dependencies
        .into_iter()
        .map(|(dep_name, dep_spec)| {
            validate_name(&dep_name, "dependency key")?;
            Ok((
                dep_name,
                DependencyRequirement::Version {
                    requirement: parse_version_req(
                        &dep_spec.version,
                        "registry dependency version",
                    )?,
                },
            ))
        })
        .collect::<eyre::Result<BTreeMap<_, _>>>()?;

    Ok(RegistryPackageRecord {
        package: PackageDefinition {
            identity: PackageIdentity {
                name: pkg.name,
                version,
            },
            entrypoint: pkg.entrypoint,
            description: pkg.description,
            dependencies,
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolves_latest_matching_version() {
        let index: RawRegistryIndex = toml::from_str(
            r#"
version = 1

[[packages]]
name = "camera_node"
version = "0.1.0"
entrypoint = "python -m camera_node"

[[packages]]
name = "camera_node"
version = "0.2.0"
entrypoint = "python -m camera_node"
"#,
        )
        .unwrap();

        let normalized = normalize_index(index).unwrap();
        let resolved = resolve_package(&normalized, "camera_node", Some(">=0.1.0,<1.0.0"))
            .unwrap()
            .unwrap();
        assert_eq!(resolved.package.identity.version.to_string(), "0.2.0");
    }

    #[test]
    fn resolves_none_when_no_match() {
        let index: RawRegistryIndex = toml::from_str(
            r#"
version = 1

[[packages]]
name = "camera_node"
version = "0.1.0"
entrypoint = "python -m camera_node"
"#,
        )
        .unwrap();

        let normalized = normalize_index(index).unwrap();
        let resolved = resolve_package(&normalized, "camera_node", Some(">=1.0.0")).unwrap();
        assert!(resolved.is_none());
    }

    #[test]
    fn invalid_entrypoint_fails_validation() {
        let index: RawRegistryIndex = toml::from_str(
            r#"
version = 1

[[packages]]
name = "camera_node"
version = "0.1.0"
entrypoint = "   "
"#,
        )
        .unwrap();

        let err = normalize_index(index).unwrap_err().to_string();
        assert!(err.contains("must not be empty"));
    }
}
