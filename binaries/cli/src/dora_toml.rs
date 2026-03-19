use crate::package_metadata::{
    DependencyRequirement, PackageDefinition, PackageIdentity, parse_version, parse_version_req,
    validate_entrypoint, validate_name,
};
use eyre::{Context, bail};
use std::{collections::BTreeMap, path::Path};

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct RawDoraToml {
    pub package: RawPackageMetadata,
    #[serde(default)]
    pub dependencies: BTreeMap<String, RawDependencySpec>,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct RawPackageMetadata {
    pub name: String,
    pub version: String,
    pub entrypoint: String,
    #[serde(default)]
    pub description: Option<String>,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct RawDependencySpec {
    pub version: Option<String>,
    pub path: Option<String>,
    pub git: Option<String>,
    pub rev: Option<String>,
}

pub fn read_and_normalize(path: &Path) -> eyre::Result<PackageDefinition> {
    let raw = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read metadata file `{}`", path.display()))?;
    let metadata: RawDoraToml = toml::from_str(&raw)
        .with_context(|| format!("failed to parse metadata file `{}`", path.display()))?;
    normalize(metadata)
}

pub fn normalize(metadata: RawDoraToml) -> eyre::Result<PackageDefinition> {
    validate_name(&metadata.package.name, "package.name")?;
    let version = parse_version(&metadata.package.version, "package.version")?;
    validate_entrypoint(&metadata.package.entrypoint, "package.entrypoint")?;

    let dependencies = metadata
        .dependencies
        .into_iter()
        .map(|(name, spec)| {
            validate_name(&name, "dependency key")?;
            let requirement = normalize_dependency_spec(&name, spec)?;
            Ok((name, requirement))
        })
        .collect::<eyre::Result<BTreeMap<_, _>>>()?;

    Ok(PackageDefinition {
        identity: PackageIdentity {
            name: metadata.package.name,
            version,
        },
        entrypoint: metadata.package.entrypoint,
        description: metadata.package.description,
        dependencies,
    })
}

fn normalize_dependency_spec(
    name: &str,
    spec: RawDependencySpec,
) -> eyre::Result<DependencyRequirement> {
    let mut source_count = 0;
    if spec.version.is_some() {
        source_count += 1;
    }
    if spec.path.is_some() {
        source_count += 1;
    }
    if spec.git.is_some() {
        source_count += 1;
    }
    if source_count == 0 {
        bail!("dependency `{name}` must specify one source via `version`, `path`, or `git`");
    }
    if source_count > 1 {
        bail!(
            "dependency `{name}` must not mix sources; choose exactly one of `version`, `path`, or `git`"
        );
    }

    if let Some(version) = spec.version {
        return Ok(DependencyRequirement::Version {
            requirement: parse_version_req(&version, &format!("dependency `{name}` version"))?,
        });
    }

    if let Some(path) = spec.path {
        if path.trim().is_empty() {
            bail!("dependency `{name}` has an empty `path`");
        }
        if spec.rev.is_some() {
            bail!("dependency `{name}` sets `rev` but does not set `git`");
        }
        return Ok(DependencyRequirement::Path { path });
    }

    if let Some(git) = spec.git {
        if git.trim().is_empty() {
            bail!("dependency `{name}` has an empty `git` URL");
        }
        return Ok(DependencyRequirement::Git {
            repo: git,
            rev: spec.rev,
        });
    }

    bail!("dependency `{name}` must specify one source");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valid_metadata_normalizes() {
        let metadata: RawDoraToml = toml::from_str(
            r#"
[package]
name = "camera_node"
version = "0.1.0"
entrypoint = "python -m camera_node"
description = "A camera node"

[dependencies]
yolo = { version = "^1.2.0" }
math = { path = "../math-node" }
vision = { git = "https://github.com/example/vision-node.git", rev = "main" }
"#,
        )
        .unwrap();

        let normalized = normalize(metadata).unwrap();
        assert_eq!(normalized.identity.name, "camera_node");
        assert_eq!(normalized.identity.version.to_string(), "0.1.0");
        assert_eq!(normalized.description.as_deref(), Some("A camera node"));
    }

    #[test]
    fn invalid_package_version_fails() {
        let metadata: RawDoraToml = toml::from_str(
            r#"
[package]
name = "camera_node"
version = "abc"
entrypoint = "python -m camera_node"
"#,
        )
        .unwrap();

        let err = normalize(metadata).unwrap_err().to_string();
        assert!(err.contains("not valid semver"));
    }

    #[test]
    fn dependency_with_rev_without_git_fails() {
        let metadata: RawDoraToml = toml::from_str(
            r#"
[package]
name = "camera_node"
version = "0.1.0"
entrypoint = "python -m camera_node"

[dependencies]
vision = { rev = "main", path = "../vision" }
"#,
        )
        .unwrap();

        let err = normalize(metadata).unwrap_err().to_string();
        assert!(err.contains("sets `rev` but does not set `git`"));
    }

    #[test]
    fn package_with_invalid_name_fails() {
        let metadata: RawDoraToml = toml::from_str(
            r#"
[package]
name = "bad name"
version = "0.1.0"
entrypoint = "python -m camera_node"
"#,
        )
        .unwrap();

        let err = normalize(metadata).unwrap_err().to_string();
        assert!(err.contains("invalid characters"));
    }
}
