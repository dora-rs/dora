use eyre::{Context, bail};
use semver::{Version, VersionReq};
use std::collections::BTreeMap;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PackageIdentity {
    pub name: String,
    pub version: Version,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PackageDefinition {
    pub identity: PackageIdentity,
    pub entrypoint: String,
    pub description: Option<String>,
    pub dependencies: BTreeMap<String, DependencyRequirement>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DependencyRequirement {
    Version { requirement: VersionReq },
    Path { path: String },
    Git { repo: String, rev: Option<String> },
}

pub fn validate_name(name: &str, field: &str) -> eyre::Result<()> {
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

pub fn parse_version(raw: &str, field: &str) -> eyre::Result<Version> {
    Version::parse(raw).with_context(|| format!("{field} `{raw}` is not valid semver"))
}

pub fn parse_version_req(raw: &str, field: &str) -> eyre::Result<VersionReq> {
    VersionReq::parse(raw).with_context(|| format!("{field} `{raw}` is not valid semver"))
}

pub fn validate_entrypoint(entrypoint: &str, field: &str) -> eyre::Result<()> {
    if entrypoint.trim().is_empty() {
        bail!("{field} must not be empty");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_invalid_name() {
        let err = validate_name("bad name", "package.name")
            .unwrap_err()
            .to_string();
        assert!(err.contains("invalid characters"));
    }

    #[test]
    fn parses_valid_version_requirement() {
        let req = parse_version_req("^1.2.3", "dependency.version").unwrap();
        assert!(req.matches(&Version::parse("1.5.0").unwrap()));
    }
}
