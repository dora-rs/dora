use eyre::{Context, bail};
use semver::{Version, VersionReq};
use std::{collections::BTreeMap, path::Path};

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct DoraToml {
    pub package: PackageMetadata,
    #[serde(default)]
    pub dependencies: BTreeMap<String, DependencySpec>,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct PackageMetadata {
    pub name: String,
    pub version: String,
    pub entrypoint: String,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct DependencySpec {
    pub version: Option<String>,
    pub path: Option<String>,
    pub git: Option<String>,
    pub rev: Option<String>,
}

pub fn read_and_validate(path: &Path) -> eyre::Result<DoraToml> {
    let raw = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read metadata file `{}`", path.display()))?;
    let metadata: DoraToml = toml::from_str(&raw)
        .with_context(|| format!("failed to parse metadata file `{}`", path.display()))?;
    validate(&metadata)?;
    Ok(metadata)
}

pub fn validate(metadata: &DoraToml) -> eyre::Result<()> {
    validate_package(&metadata.package)?;
    for (name, spec) in &metadata.dependencies {
        validate_dependency_name(name)?;
        validate_dependency_spec(name, spec)?;
    }
    Ok(())
}

fn validate_package(package: &PackageMetadata) -> eyre::Result<()> {
    validate_package_name(&package.name)?;
    Version::parse(&package.version)
        .with_context(|| format!("package.version `{}` is not valid semver", package.version))?;

    if package.entrypoint.trim().is_empty() {
        bail!("package.entrypoint must not be empty");
    }
    Ok(())
}

fn validate_dependency_name(name: &str) -> eyre::Result<()> {
    validate_name_chars(name, "dependency key")
}

fn validate_package_name(name: &str) -> eyre::Result<()> {
    validate_name_chars(name, "package.name")
}

fn validate_name_chars(name: &str, field: &str) -> eyre::Result<()> {
    if name.is_empty() {
        bail!("{field} must not be empty");
    }

    let valid = name
        .chars()
        .all(|ch| ch.is_ascii_alphanumeric() || ch == '-' || ch == '_');
    if !valid {
        bail!("{field} `{name}` contains invalid characters (allowed: a-z, A-Z, 0-9, -, _)");
    }
    Ok(())
}

fn validate_dependency_spec(name: &str, spec: &DependencySpec) -> eyre::Result<()> {
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

    if let Some(version) = &spec.version {
        VersionReq::parse(version).with_context(|| {
            format!("dependency `{name}` has invalid version requirement `{version}`")
        })?;
    }
    if let Some(path) = &spec.path {
        if path.trim().is_empty() {
            bail!("dependency `{name}` has an empty `path`");
        }
    }
    if let Some(git) = &spec.git {
        if git.trim().is_empty() {
            bail!("dependency `{name}` has an empty `git` URL");
        }
    }
    if spec.rev.is_some() && spec.git.is_none() {
        bail!("dependency `{name}` sets `rev` but does not set `git`");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valid_metadata_passes_validation() {
        let metadata: DoraToml = toml::from_str(
            r#"
[package]
name = "camera_node"
version = "0.1.0"
entrypoint = "python -m camera_node"

[dependencies]
yolo = { version = "^1.2.0" }
math = { path = "../math-node" }
vision = { git = "https://github.com/example/vision-node.git", rev = "main" }
"#,
        )
        .unwrap();

        validate(&metadata).unwrap();
    }

    #[test]
    fn invalid_package_version_fails() {
        let metadata: DoraToml = toml::from_str(
            r#"
[package]
name = "camera_node"
version = "abc"
entrypoint = "python -m camera_node"
"#,
        )
        .unwrap();

        let err = validate(&metadata).unwrap_err().to_string();
        assert!(err.contains("not valid semver"));
    }

    #[test]
    fn dependency_with_rev_without_git_fails() {
        let metadata: DoraToml = toml::from_str(
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

        let err = validate(&metadata).unwrap_err().to_string();
        assert!(err.contains("sets `rev` but does not set `git`"));
    }

    #[test]
    fn dependency_with_only_rev_fails() {
        let metadata: DoraToml = toml::from_str(
            r#"
[package]
name = "camera_node"
version = "0.1.0"
entrypoint = "python -m camera_node"

[dependencies]
vision = { rev = "main" }
"#,
        )
        .unwrap();

        let err = validate(&metadata).unwrap_err().to_string();
        assert!(err.contains("must specify one source"));
    }

    #[test]
    fn package_with_invalid_name_fails() {
        let metadata: DoraToml = toml::from_str(
            r#"
[package]
name = "bad name"
version = "0.1.0"
entrypoint = "python -m camera_node"
"#,
        )
        .unwrap();

        let err = validate(&metadata).unwrap_err().to_string();
        assert!(err.contains("contains invalid characters"));
    }
}
