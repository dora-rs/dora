//! The on-disk index catalog format and resolution (spec §7.1/§7.2/§8.1).
//!
//! A catalog is a directory tree `<root>/<namespace>/<name>/<version>.yml`
//! plus a per-package `package.yml`. Version files hold the node manifest
//! verbatim and a source pointer; resolution picks the highest non-yanked
//! version satisfying a requirement.

use std::path::{Path, PathBuf};

use dora_core::{manifest::NodeManifest, types::edit_distance};
use eyre::Context;
use semver::Version;
use serde::{Deserialize, Serialize};

use crate::reference::PackageRef;

/// Upper bound on an index entry file. Entries hold a manifest (already
/// capped) plus a small source stanza.
const MAX_ENTRY_SIZE: u64 = 2 * 1024 * 1024;

/// One published version: manifest snapshot + source pointer (spec §7.1).
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IndexEntry {
    /// The node's `dora-node.yml`, copied verbatim at publish time.
    pub manifest: NodeManifest,
    /// Where the bytes live (spec §8.1).
    pub source: SourceSpec,
    /// Publish timestamp (RFC 3339), informational.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub published: Option<String>,
    /// Yanked versions are skipped by resolution (UC10); existing lockfile
    /// pins keep working with a warning.
    #[serde(default)]
    pub yanked: bool,
    /// Reason recorded when yanking.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub yank_reason: Option<String>,
}

/// The source pointer of a published version (spec §8.1).
///
/// The git form is the default and the v1 source-of-record. The binary form
/// is reserved in the schema (spec §8.2) and parses, but consuming it is
/// deferred to P2.8.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceSpec {
    /// Git repository URL holding the node's source.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub git: Option<String>,
    /// Commit hash the version is pinned to (resolved from a tag at publish).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rev: Option<String>,
    /// Where in the repository the node lives (monorepo support).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub subdir: Option<String>,
    /// Reserved: per-platform prebuilt artifacts (spec §8.2, P2.8).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub binary: Vec<BinaryArtifact>,
    /// Reserved: source fallback for platforms without a prebuilt binary.
    #[serde(
        default,
        skip_serializing_if = "Option::is_none",
        rename = "fallback-git"
    )]
    pub fallback_git: Option<Box<SourceSpec>>,
}

/// Reserved binary-form artifact (spec §8.1); not consumed in v1.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BinaryArtifact {
    pub platform: String,
    pub url: String,
    pub sha256: String,
}

impl SourceSpec {
    /// The git repo + commit pin, or an error for binary-only sources
    /// (deferred to P2.8) and malformed entries. The URL is scheme-validated
    /// here, at the source — it ends up as a `git clone` argument.
    pub fn git_pin(&self) -> eyre::Result<(&str, &str)> {
        match (&self.git, &self.rev) {
            (Some(git), Some(rev)) => {
                crate::validate_git_url(git)?;
                let valid_rev = !rev.is_empty() && rev.chars().all(|c| c.is_ascii_alphanumeric());
                if !valid_rev {
                    eyre::bail!("index entry has an invalid commit hash");
                }
                Ok((git, rev))
            }
            (None, None) if !self.binary.is_empty() => eyre::bail!(
                "this version is published as prebuilt binaries only, which this \
                 dora version does not support yet"
            ),
            _ => eyre::bail!("index entry has an incomplete git source (needs `git` + `rev`)"),
        }
    }
}

/// Namespace-level metadata (`package.yml`, spec §7.1/§7.4).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PackageMeta {
    /// One-line description of the package.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// The package's primary repository.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub repo: Option<String>,
    /// GitHub accounts allowed to publish (enforced by index CI).
    #[serde(default)]
    pub owners: Vec<String>,
}

/// A locally available catalog directory.
#[derive(Debug, Clone)]
pub struct IndexCatalog {
    root: PathBuf,
}

/// A resolved package version.
#[derive(Debug)]
pub struct ResolvedVersion {
    pub version: Version,
    pub entry: IndexEntry,
}

impl IndexCatalog {
    /// Open a catalog rooted at `root` (the `node-index/` directory).
    pub fn open(root: impl Into<PathBuf>) -> eyre::Result<Self> {
        let root = root.into();
        if !root.is_dir() {
            eyre::bail!("index catalog `{}` is not a directory", root.display());
        }
        Ok(Self { root })
    }

    fn package_dir(&self, namespace: &str, name: &str) -> eyre::Result<PathBuf> {
        for part in [namespace, name] {
            if !is_valid_key_part(part) {
                eyre::bail!("invalid package key `{namespace}/{name}`");
            }
        }
        Ok(self.root.join(namespace).join(name))
    }

    /// All published versions of a package, including yanked ones.
    pub fn versions(&self, namespace: &str, name: &str) -> eyre::Result<Vec<Version>> {
        let dir = self.package_dir(namespace, name)?;
        let mut versions = Vec::new();
        let entries = match std::fs::read_dir(&dir) {
            Ok(entries) => entries,
            Err(_) => return Ok(versions), // unknown package = no versions
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().and_then(|e| e.to_str()) != Some("yml") {
                continue;
            }
            let Some(stem) = path.file_stem().and_then(|s| s.to_str()) else {
                continue;
            };
            if stem == "package" {
                continue;
            }
            if let Ok(version) = Version::parse(stem) {
                versions.push(version);
            }
        }
        versions.sort();
        Ok(versions)
    }

    /// Read one version's index entry.
    ///
    /// Note for lockfile consumers (P2.6): a *pinned* version is looked up
    /// directly through this method even when yanked — check `entry.yanked`
    /// and warn (UC10: existing pins keep working, with a warning).
    pub fn entry(
        &self,
        namespace: &str,
        name: &str,
        version: &Version,
    ) -> eyre::Result<IndexEntry> {
        let path = self
            .package_dir(namespace, name)?
            .join(format!("{version}.yml"));
        let raw = read_capped(&path)?;
        serde_yaml::from_str(&raw)
            .with_context(|| format!("invalid index entry `{}`", path.display()))
    }

    /// Read a package's namespace-level metadata, if present.
    pub fn package_meta(&self, namespace: &str, name: &str) -> eyre::Result<Option<PackageMeta>> {
        let path = self.package_dir(namespace, name)?.join("package.yml");
        if !path.is_file() {
            return Ok(None);
        }
        let raw = read_capped(&path)?;
        let meta = serde_yaml::from_str(&raw)
            .with_context(|| format!("invalid package metadata `{}`", path.display()))?;
        Ok(Some(meta))
    }

    /// Resolve a reference to the highest non-yanked version satisfying its
    /// requirement (spec §7.2).
    pub fn resolve(&self, reference: &PackageRef) -> eyre::Result<ResolvedVersion> {
        let versions = self.versions(&reference.namespace, &reference.name)?;
        if versions.is_empty() {
            eyre::bail!(
                "package `{}` not found in the index{}",
                reference.key(),
                self.suggest(reference)
                    .map(|s| format!(" — did you mean `{s}`?"))
                    .unwrap_or_default()
            );
        }
        // Note: matching follows the cargo convention — prerelease versions
        // only match requirements that themselves carry a prerelease tag.
        let matching: Vec<_> = versions
            .iter()
            .filter(|v| reference.requirement.matches(v))
            .collect();
        // versions() returns ascending order; walk highest-first, skipping
        // yanked versions
        for version in matching.iter().rev() {
            let entry = self.entry(&reference.namespace, &reference.name, version)?;
            if entry.yanked {
                continue;
            }
            return Ok(ResolvedVersion {
                version: (*version).clone(),
                entry,
            });
        }
        eyre::bail!(
            "no version of `{}` satisfies `{}` (available: {})",
            reference.key(),
            reference.requirement,
            versions
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(", ")
        )
    }

    /// All packages in the catalog as `(namespace, name)` pairs. Directory
    /// names outside the package-key charset are skipped — catalog content
    /// is untrusted and these strings end up in terminal output.
    pub fn all_packages(&self) -> Vec<(String, String)> {
        let mut packages = Vec::new();
        let Ok(namespaces) = std::fs::read_dir(&self.root) else {
            return packages;
        };
        for ns in namespaces.flatten() {
            if !ns.path().is_dir() {
                continue;
            }
            let Some(ns_name) = ns.file_name().to_str().map(String::from) else {
                continue;
            };
            if !is_valid_key_part(&ns_name) {
                continue;
            }
            let Ok(names) = std::fs::read_dir(ns.path()) else {
                continue;
            };
            for name in names.flatten() {
                if !name.path().is_dir() {
                    continue;
                }
                if let Some(name) = name.file_name().to_str()
                    && is_valid_key_part(name)
                {
                    packages.push((ns_name.clone(), name.to_string()));
                }
            }
        }
        packages.sort();
        packages
    }

    /// Suggest a close package name for a typo'd reference.
    fn suggest(&self, reference: &PackageRef) -> Option<String> {
        self.all_packages()
            .into_iter()
            .filter(|(ns, _)| *ns == reference.namespace)
            .map(|(ns, name)| {
                let dist = edit_distance(&reference.name, &name);
                (format!("{ns}/{name}"), dist)
            })
            .filter(|(_, dist)| *dist <= 2)
            .min_by_key(|(_, dist)| *dist)
            .map(|(key, _)| key)
    }
}

/// A valid namespace/name path segment of a package key. Keys feed into
/// filesystem paths and terminal output, so the charset is strict.
fn is_valid_key_part(part: &str) -> bool {
    !part.is_empty()
        && part.len() <= 64
        && part
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '-' | '_' | '.'))
        && !part.starts_with('.')
}

/// Read a small catalog file, bounding the read itself (a metadata-size
/// check would miss FIFOs/devices and growing files).
fn read_capped(path: &Path) -> eyre::Result<String> {
    use std::io::Read as _;
    let file = std::fs::File::open(path)
        .with_context(|| format!("no index entry at `{}`", path.display()))?;
    let mut raw = String::new();
    file.take(MAX_ENTRY_SIZE + 1)
        .read_to_string(&mut raw)
        .with_context(|| format!("failed to read `{}`", path.display()))?;
    if raw.len() as u64 > MAX_ENTRY_SIZE {
        eyre::bail!("index file `{}` too large", path.display());
    }
    Ok(raw)
}

#[cfg(test)]
mod tests {
    use super::*;

    const MANIFEST: &str = r#"
  apiVersion: 1
  name: dora-yolo
  namespace: dora-rs
  runtime: python
  entrypoint: dora-yolo
"#;

    fn entry_yaml(rev: &str, yanked: bool) -> String {
        format!(
            "manifest:{MANIFEST}source:\n  git: https://github.com/dora-rs/dora-hub\n  rev: {rev}\n  subdir: node-hub/dora-yolo\nyanked: {yanked}\n"
        )
    }

    fn fixture() -> (tempfile::TempDir, IndexCatalog) {
        let tmp = tempfile::tempdir().unwrap();
        let pkg = tmp.path().join("dora-rs/dora-yolo");
        std::fs::create_dir_all(&pkg).unwrap();
        std::fs::write(pkg.join("0.5.1.yml"), entry_yaml("aaa", false)).unwrap();
        std::fs::write(pkg.join("0.5.2.yml"), entry_yaml("bbb", false)).unwrap();
        std::fs::write(pkg.join("0.6.0.yml"), entry_yaml("ccc", true)).unwrap();
        std::fs::write(
            pkg.join("package.yml"),
            "description: object detection\nowners: [haixuanTao]\n",
        )
        .unwrap();
        let catalog = IndexCatalog::open(tmp.path()).unwrap();
        (tmp, catalog)
    }

    fn parse_ref(s: &str) -> PackageRef {
        PackageRef::parse(s).unwrap()
    }

    #[test]
    fn resolves_highest_matching_version() {
        let (_tmp, catalog) = fixture();
        let resolved = catalog.resolve(&parse_ref("dora-yolo@^0.5")).unwrap();
        assert_eq!(resolved.version, Version::parse("0.5.2").unwrap());
        let (git, rev) = resolved.entry.source.git_pin().unwrap();
        assert_eq!(git, "https://github.com/dora-rs/dora-hub");
        assert_eq!(rev, "bbb");
    }

    #[test]
    fn prerelease_versions_follow_cargo_convention() {
        let tmp = tempfile::tempdir().unwrap();
        let pkg = tmp.path().join("dora-rs/dora-yolo");
        std::fs::create_dir_all(&pkg).unwrap();
        std::fs::write(pkg.join("0.5.1.yml"), entry_yaml("aaa", false)).unwrap();
        std::fs::write(pkg.join("0.6.0-alpha.1.yml"), entry_yaml("pre", false)).unwrap();
        let catalog = IndexCatalog::open(tmp.path()).unwrap();
        // a plain requirement never matches a prerelease
        let resolved = catalog.resolve(&parse_ref("dora-yolo@>=0.5")).unwrap();
        assert_eq!(resolved.version.to_string(), "0.5.1");
        // a prerelease requirement opts in
        let resolved = catalog
            .resolve(&parse_ref("dora-yolo@>=0.6.0-alpha"))
            .unwrap();
        assert_eq!(resolved.version.to_string(), "0.6.0-alpha.1");
    }

    #[test]
    fn git_pin_rejects_hostile_urls() {
        for (git, rev) in [
            ("--upload-pack=evil", "abc123"),
            ("ext::sh -c evil", "abc123"),
            ("https://example.com/repo", "$(evil)"),
            ("https://example.com/repo", "--flag"),
        ] {
            let entry: IndexEntry = serde_yaml::from_str(&format!(
                "manifest:{MANIFEST}source:\n  git: \"{git}\"\n  rev: \"{rev}\"\n"
            ))
            .unwrap();
            assert!(entry.source.git_pin().is_err(), "{git} {rev} should fail");
        }
    }

    #[test]
    fn yanked_versions_are_skipped() {
        let (_tmp, catalog) = fixture();
        // 0.6.0 is yanked — `*` resolves to 0.5.2 instead
        let resolved = catalog.resolve(&parse_ref("dora-yolo")).unwrap();
        assert_eq!(resolved.version, Version::parse("0.5.2").unwrap());
    }

    #[test]
    fn unsatisfiable_requirement_lists_available() {
        let (_tmp, catalog) = fixture();
        let err = catalog.resolve(&parse_ref("dora-yolo@^2")).unwrap_err();
        let msg = format!("{err:#}");
        assert!(msg.contains("0.5.2"), "{msg}");
    }

    #[test]
    fn unknown_package_suggests_close_name() {
        let (_tmp, catalog) = fixture();
        let err = catalog.resolve(&parse_ref("dora-yolp@^0.5")).unwrap_err();
        let msg = format!("{err:#}");
        assert!(msg.contains("did you mean `dora-rs/dora-yolo`"), "{msg}");
    }

    #[test]
    fn package_meta_reads() {
        let (_tmp, catalog) = fixture();
        let meta = catalog
            .package_meta("dora-rs", "dora-yolo")
            .unwrap()
            .unwrap();
        assert_eq!(meta.owners, vec!["haixuanTao"]);
    }

    #[test]
    fn binary_only_source_errors_clearly() {
        let entry: IndexEntry = serde_yaml::from_str(&format!(
            "manifest:{MANIFEST}source:\n  binary:\n    - platform: linux-x86_64\n      url: https://example.com/x\n      sha256: \"ab\"\n"
        ))
        .unwrap();
        let err = entry.source.git_pin().unwrap_err();
        assert!(format!("{err}").contains("prebuilt binaries"), "{err}");
    }

    #[test]
    fn traversal_keys_are_rejected() {
        let (_tmp, catalog) = fixture();
        for (ns, name) in [("..", "x"), ("a", "../b"), (".hidden", "x"), ("a/b", "c")] {
            assert!(
                catalog.versions(ns, name).is_err(),
                "{ns}/{name} should be rejected"
            );
        }
    }

    #[test]
    fn all_packages_lists_sorted() {
        let (_tmp, catalog) = fixture();
        assert_eq!(
            catalog.all_packages(),
            vec![("dora-rs".to_string(), "dora-yolo".to_string())]
        );
    }
}
