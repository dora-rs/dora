//! Parsing of `hub:` package references (spec §7.2):
//! `[<namespace>/]<name>@<semver-req>`.

use semver::VersionReq;

use crate::OFFICIAL_NAMESPACE;

/// A parsed `hub:` reference. Bare names are official-namespace shorthand:
/// `dora-yolo@^0.5` means exactly `dora-rs/dora-yolo@^0.5`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PackageRef {
    /// Index namespace (`dora-rs` when the reference was written bare).
    pub namespace: String,
    /// Package name within the namespace.
    pub name: String,
    /// Requested version range.
    pub requirement: VersionReq,
}

impl PackageRef {
    /// Parse a reference like `dora-yolo@^0.5` or `acme/lidar-fusion@2.1`.
    pub fn parse(reference: &str) -> eyre::Result<Self> {
        let (path, requirement) = match reference.split_once('@') {
            Some((path, req)) => {
                let requirement = VersionReq::parse(req.trim()).map_err(|e| {
                    eyre::eyre!("invalid version requirement `{}`: {e}", req.trim())
                })?;
                (path.trim(), requirement)
            }
            // a bare `hub: dora-yolo` means any version
            None => (reference.trim(), VersionReq::STAR),
        };
        if path.is_empty() {
            eyre::bail!("empty package reference");
        }
        let (namespace, name) = match path.split_once('/') {
            Some((namespace, name)) => {
                if name.contains('/') {
                    eyre::bail!(
                        "invalid package reference `{reference}`: \
                         expected `[namespace/]name@version-req`"
                    );
                }
                (namespace, name)
            }
            None => (OFFICIAL_NAMESPACE, path),
        };
        if namespace.is_empty() || name.is_empty() {
            eyre::bail!("invalid package reference `{reference}`: empty namespace or name");
        }
        Ok(Self {
            namespace: namespace.to_string(),
            name: name.to_string(),
            requirement,
        })
    }

    /// The index key `namespace/name`.
    pub fn key(&self) -> String {
        format!("{}/{}", self.namespace, self.name)
    }
}

impl std::fmt::Display for PackageRef {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}/{}@{}", self.namespace, self.name, self.requirement)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bare_name_is_official_shorthand() {
        let r = PackageRef::parse("dora-yolo@^0.5").unwrap();
        assert_eq!(r.namespace, "dora-rs");
        assert_eq!(r.name, "dora-yolo");
        assert_eq!(r.requirement, VersionReq::parse("^0.5").unwrap());
    }

    #[test]
    fn namespaced_reference() {
        let r = PackageRef::parse("acme/lidar-fusion@2.1").unwrap();
        assert_eq!(r.namespace, "acme");
        assert_eq!(r.name, "lidar-fusion");
        assert_eq!(r.key(), "acme/lidar-fusion");
    }

    #[test]
    fn missing_requirement_means_any() {
        let r = PackageRef::parse("dora-yolo").unwrap();
        assert_eq!(r.requirement, VersionReq::STAR);
    }

    #[test]
    fn full_semver_req_syntax() {
        for req in [">=0.5, <0.7", "=1.2.3", "~0.4", "*"] {
            PackageRef::parse(&format!("x@{req}")).unwrap_or_else(|e| panic!("{req}: {e}"));
        }
    }

    #[test]
    fn rejects_malformed() {
        for bad in ["", "@1.0", "a/b/c@1", "a/@1", "/b@1", "x@not-a-req"] {
            assert!(PackageRef::parse(bad).is_err(), "{bad:?} should fail");
        }
    }
}
