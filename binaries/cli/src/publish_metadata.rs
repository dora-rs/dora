use eyre::{Context, bail};
use semver::Version;
use std::{collections::BTreeMap, path::Path};

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct PublishedDependency {
    pub name: String,
    pub requirement: String,
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct PublishedPackageRecord {
    pub name: String,
    pub version: Version,
    #[serde(default)]
    pub dependencies: Vec<PublishedDependency>,
    pub checksum: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PublishManifest {
    pub name: String,
    pub version: Version,
    pub dependencies: Vec<PublishedDependency>,
}

#[derive(Debug, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct RawDoraToml {
    package: RawPackageSection,
    #[serde(default)]
    dependencies: BTreeMap<String, RawDependencySpec>,
}

#[derive(Debug, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct RawPackageSection {
    name: String,
    version: String,
}

#[derive(Debug, serde::Deserialize)]
#[serde(untagged)]
enum RawDependencySpec {
    Version(String),
    Detailed {
        version: Option<String>,
        path: Option<String>,
        git: Option<String>,
        rev: Option<String>,
    },
}

impl PublishManifest {
    pub fn from_dora_toml_path(path: &Path) -> eyre::Result<Self> {
        let raw = std::fs::read_to_string(path)
            .with_context(|| format!("failed to read Dora metadata at `{}`", path.display()))?;
        Self::from_dora_toml_str(&raw)
            .with_context(|| format!("failed to parse Dora metadata at `{}`", path.display()))
    }

    pub fn from_dora_toml_str(raw: &str) -> eyre::Result<Self> {
        let manifest: RawDoraToml = toml::from_str(raw)?;
        normalize_manifest(manifest)
    }
}

impl PublishedPackageRecord {
    pub fn from_manifest(manifest: PublishManifest, checksum: String) -> Self {
        Self {
            name: manifest.name,
            version: manifest.version,
            dependencies: manifest.dependencies,
            checksum,
        }
    }
}

fn normalize_manifest(manifest: RawDoraToml) -> eyre::Result<PublishManifest> {
    validate_package_name(&manifest.package.name)?;
    let version = Version::parse(&manifest.package.version).with_context(|| {
        format!(
            "package version `{}` is not valid semver",
            manifest.package.version
        )
    })?;

    let mut dependencies = Vec::new();
    for (name, spec) in manifest.dependencies {
        validate_package_name(&name)?;
        let requirement = match spec {
            RawDependencySpec::Version(requirement) => requirement,
            RawDependencySpec::Detailed {
                version: Some(requirement),
                path: None,
                git: None,
                rev: None,
            } => requirement,
            RawDependencySpec::Detailed { path: Some(_), .. } => bail!(
                "dependency `{name}` uses `path`, which is not publishable in registry metadata"
            ),
            RawDependencySpec::Detailed { git: Some(_), .. } => bail!(
                "dependency `{name}` uses `git`, which is not publishable in registry metadata"
            ),
            RawDependencySpec::Detailed { rev: Some(_), .. } => {
                bail!("dependency `{name}` sets `rev` without a publishable `version` source")
            }
            RawDependencySpec::Detailed { version: None, .. } => {
                bail!("dependency `{name}` must declare a version requirement to be published")
            }
        };

        semver::VersionReq::parse(&requirement).with_context(|| {
            format!("dependency `{name}` has invalid version requirement `{requirement}`")
        })?;

        dependencies.push(PublishedDependency { name, requirement });
    }

    Ok(PublishManifest {
        name: manifest.package.name,
        version,
        dependencies,
    })
}

fn validate_package_name(name: &str) -> eyre::Result<()> {
    if name.is_empty() {
        bail!("package/dependency name must not be empty");
    }
    if !name
        .chars()
        .all(|ch| ch.is_ascii_alphanumeric() || ch == '-' || ch == '_')
    {
        bail!("invalid package/dependency name `{name}`");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_publishable_manifest() {
        let manifest = PublishManifest::from_dora_toml_str(
            r#"
[package]
name = "camera_node"
version = "0.1.0"

[dependencies]
yolo = { version = "^1.2.0" }
math = ">=0.3.0,<1.0.0"
"#,
        )
        .unwrap();

        assert_eq!(manifest.name, "camera_node");
        assert_eq!(manifest.version, Version::parse("0.1.0").unwrap());
        assert_eq!(manifest.dependencies.len(), 2);
    }

    #[test]
    fn rejects_path_dependency() {
        let err = PublishManifest::from_dora_toml_str(
            r#"
[package]
name = "camera_node"
version = "0.1.0"

[dependencies]
vision = { path = "../vision-node" }
"#,
        )
        .unwrap_err()
        .to_string();

        assert!(err.contains("not publishable"));
    }

    #[test]
    fn rejects_invalid_version_requirement() {
        let err = PublishManifest::from_dora_toml_str(
            r#"
[package]
name = "camera_node"
version = "0.1.0"

[dependencies]
yolo = { version = "not-a-version" }
"#,
        )
        .unwrap_err()
        .to_string();

        assert!(err.contains("invalid version requirement"));
    }
}
