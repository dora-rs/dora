use eyre::{Context, bail};
use semver::{Version, VersionReq};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

const REGISTRY_INDEX_VERSION: u32 = 1;

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct RegistryIndex {
    pub version: u32,
    #[serde(default)]
    pub packages: Vec<RegistryPackage>,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct RegistryPackage {
    pub name: String,
    pub version: String,
    pub entrypoint: String,
    #[serde(default)]
    pub description: Option<String>,
    #[serde(default)]
    pub dependencies: BTreeMap<String, RegistryDependency>,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct RegistryDependency {
    pub version: String,
}

#[derive(Debug, Clone)]
pub struct RegistrySource {
    pub index_path: PathBuf,
}

impl RegistrySource {
    pub fn from_path(path: &Path) -> Self {
        // Resolve registry source without requiring the path to already exist.
        // `*.toml` is treated as an explicit index file; everything else is a
        // registry directory containing `index.toml`.
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
        let index: RegistryIndex = toml::from_str(&raw).with_context(|| {
            format!(
                "failed to parse registry index at `{}`",
                self.index_path.display()
            )
        })?;

        if index.version != REGISTRY_INDEX_VERSION {
            bail!(
                "unsupported registry index version `{}` (expected `{}`)",
                index.version,
                REGISTRY_INDEX_VERSION
            );
        }
        validate_index(&index)?;
        Ok(index)
    }
}

pub fn resolve_package<'a>(
    index: &'a RegistryIndex,
    name: &str,
    requirement: Option<&str>,
) -> eyre::Result<Option<&'a RegistryPackage>> {
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
        .filter(|pkg| pkg.name == name)
        .map(|pkg| {
            let parsed = Version::parse(&pkg.version).with_context(|| {
                format!(
                    "registry package `{}` has invalid version `{}`",
                    pkg.name, pkg.version
                )
            })?;
            Ok((pkg, parsed))
        })
        .collect::<eyre::Result<Vec<_>>>()?;

    if let Some(version_req) = version_req {
        matching.retain(|(_, version)| version_req.matches(version));
    }

    matching.sort_by(|(_, a), (_, b)| b.cmp(a));
    Ok(matching.first().map(|(pkg, _)| *pkg))
}

fn validate_index(index: &RegistryIndex) -> eyre::Result<()> {
    for pkg in &index.packages {
        validate_package(pkg)?;
    }
    Ok(())
}

fn validate_package(pkg: &RegistryPackage) -> eyre::Result<()> {
    validate_name(&pkg.name)?;
    Version::parse(&pkg.version).with_context(|| {
        format!(
            "package `{}` has invalid semver `{}`",
            pkg.name, pkg.version
        )
    })?;
    if pkg.entrypoint.trim().is_empty() {
        bail!("package `{}` has empty `entrypoint`", pkg.name);
    }
    for (dep_name, dep_spec) in &pkg.dependencies {
        validate_name(dep_name)?;
        VersionReq::parse(&dep_spec.version).with_context(|| {
            format!(
                "dependency `{dep_name}` in package `{}` has invalid version requirement `{}`",
                pkg.name, dep_spec.version
            )
        })?;
    }
    Ok(())
}

fn validate_name(name: &str) -> eyre::Result<()> {
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
    fn resolves_latest_matching_version() {
        let index: RegistryIndex = toml::from_str(
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

        let resolved = resolve_package(&index, "camera_node", Some(">=0.1.0,<1.0.0"))
            .unwrap()
            .unwrap();
        assert_eq!(resolved.version, "0.2.0");
    }

    #[test]
    fn resolves_none_when_no_match() {
        let index: RegistryIndex = toml::from_str(
            r#"
version = 1

[[packages]]
name = "camera_node"
version = "0.1.0"
entrypoint = "python -m camera_node"
"#,
        )
        .unwrap();

        let resolved = resolve_package(&index, "camera_node", Some(">=1.0.0")).unwrap();
        assert!(resolved.is_none());
    }

    #[test]
    fn invalid_entrypoint_fails_validation() {
        let index: RegistryIndex = toml::from_str(
            r#"
version = 1

[[packages]]
name = "camera_node"
version = "0.1.0"
entrypoint = "   "
"#,
        )
        .unwrap();

        let err = validate_index(&index).unwrap_err().to_string();
        assert!(err.contains("empty `entrypoint`"));
    }
}
