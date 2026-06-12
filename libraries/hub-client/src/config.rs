//! Multi-index configuration and namespace binding (spec §7.3).
//!
//! `~/.config/dora/hub.toml` declares `[[index]]` entries. **A namespace
//! resolves against exactly one index**: explicit `namespaces` bindings win,
//! the official index carries every unbound namespace, and a namespace bound
//! by two entries is a configuration error — there is no search order to
//! race, which makes dependency-confusion attacks structurally impossible.

use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

use eyre::Context;
use serde::Deserialize;

use crate::{OFFICIAL_INDEX_ALIAS, OFFICIAL_INDEX_GIT, OFFICIAL_INDEX_PATH};

/// Parsed `hub.toml`.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HubConfig {
    #[serde(default, rename = "index")]
    pub indexes: Vec<IndexConfig>,
}

/// One `[[index]]` entry.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IndexConfig {
    /// Short name; also the cache directory name for git indexes.
    pub alias: String,
    /// Git repository holding the catalog (remote indexes).
    #[serde(default)]
    pub git: Option<String>,
    /// For git indexes: catalog directory within the repository
    /// (default `node-index`). For local indexes: the catalog directory
    /// itself — relative paths resolve against the config file location.
    #[serde(default)]
    pub path: Option<String>,
    /// Namespaces bound exclusively to this index. Empty = this index only
    /// serves as the official default (valid for the `official` alias).
    #[serde(default)]
    pub namespaces: Vec<String>,
}

impl IndexConfig {
    /// The implicit official index entry (spec §7.3: a missing config file
    /// means `official` only).
    pub fn official() -> Self {
        Self {
            alias: OFFICIAL_INDEX_ALIAS.into(),
            git: Some(OFFICIAL_INDEX_GIT.into()),
            path: Some(OFFICIAL_INDEX_PATH.into()),
            namespaces: Vec::new(),
        }
    }

    /// Whether this entry points at a local directory instead of a git repo.
    pub fn is_local(&self) -> bool {
        self.git.is_none()
    }

    /// For local indexes: the catalog directory, resolved against the
    /// directory the config file lives in.
    pub fn local_catalog_dir(&self, config_dir: &Path) -> eyre::Result<PathBuf> {
        let path = self
            .path
            .as_deref()
            .ok_or_else(|| eyre::eyre!("index `{}` has neither `git` nor `path`", self.alias))?;
        let path = Path::new(path);
        Ok(if path.is_absolute() {
            path.to_path_buf()
        } else {
            config_dir.join(path)
        })
    }
}

/// Validated configuration: every namespace maps to exactly one index.
#[derive(Debug, Clone)]
pub struct ResolvedConfig {
    /// All configured indexes, official included.
    pub indexes: Vec<IndexConfig>,
    /// Directory the config was loaded from (base for relative `path =`).
    pub config_dir: PathBuf,
    bindings: BTreeMap<String, usize>,
    default_index: usize,
}

impl ResolvedConfig {
    /// Build from parsed entries, enforcing the binding rules.
    pub fn new(mut indexes: Vec<IndexConfig>, config_dir: PathBuf) -> eyre::Result<Self> {
        // the official index is always present; an explicit `official` alias
        // entry may override its location (e.g. an enterprise mirror)
        if !indexes.iter().any(|i| i.alias == OFFICIAL_INDEX_ALIAS) {
            indexes.push(IndexConfig::official());
        }
        let mut bindings = BTreeMap::new();
        let mut seen_aliases = BTreeMap::new();
        for (idx, index) in indexes.iter().enumerate() {
            validate_index_entry(index)?;
            if let Some(previous) = seen_aliases.insert(index.alias.clone(), idx) {
                eyre::bail!(
                    "duplicate index alias `{}` (entries {} and {idx})",
                    index.alias,
                    previous
                );
            }
            for namespace in &index.namespaces {
                if let Some(previous) = bindings.insert(namespace.clone(), idx) {
                    eyre::bail!(
                        "namespace `{namespace}` is bound by two indexes \
                         (`{}` and `{}`) — a namespace resolves against \
                         exactly one index",
                        indexes[previous].alias,
                        index.alias
                    );
                }
            }
        }
        let default_index = indexes
            .iter()
            .position(|i| i.alias == OFFICIAL_INDEX_ALIAS)
            .expect("official index inserted above");
        Ok(Self {
            indexes,
            config_dir,
            bindings,
            default_index,
        })
    }

    /// Load from the conventional location (`<config_dir>/dora/hub.toml`),
    /// falling back to official-only when the file does not exist.
    pub fn load_default() -> eyre::Result<Self> {
        let Some(config_dir) = dirs::config_dir() else {
            return Self::new(Vec::new(), PathBuf::from("."));
        };
        Self::load_from(&config_dir.join("dora").join("hub.toml"))
    }

    /// Load from an explicit config file path (missing file = official only).
    pub fn load_from(path: &Path) -> eyre::Result<Self> {
        let config_dir = path.parent().unwrap_or(Path::new(".")).to_path_buf();
        if !path.is_file() {
            return Self::new(Vec::new(), config_dir);
        }
        let raw = std::fs::read_to_string(path)
            .with_context(|| format!("failed to read `{}`", path.display()))?;
        let config: HubConfig = toml::from_str(&raw)
            .with_context(|| format!("invalid hub config `{}`", path.display()))?;
        Self::new(config.indexes, config_dir)
            .with_context(|| format!("invalid hub config `{}`", path.display()))
    }

    /// The index a namespace resolves against (spec §7.3).
    pub fn index_for_namespace(&self, namespace: &str) -> &IndexConfig {
        match self.bindings.get(namespace) {
            Some(&idx) => &self.indexes[idx],
            None => &self.indexes[self.default_index],
        }
    }
}

/// Validate one `[[index]]` entry. The alias becomes a cache directory name
/// and the git URL/path become `git` subprocess arguments — all of it is
/// user- or repo-supplied, so each field is constrained before use.
fn validate_index_entry(index: &IndexConfig) -> eyre::Result<()> {
    let alias_valid = index.alias.len() <= 64
        && index
            .alias
            .chars()
            .next()
            .is_some_and(|c| c.is_ascii_alphanumeric())
        && index
            .alias
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '-' | '_'));
    if !alias_valid {
        eyre::bail!(
            "invalid index alias `{}`: expected `[A-Za-z0-9_-]+` (max 64 chars)",
            index
                .alias
                .chars()
                .filter(|c| !c.is_control())
                .collect::<String>()
        );
    }
    match (&index.git, &index.path) {
        (None, None) => {
            eyre::bail!("index `{}` has neither `git` nor `path`", index.alias)
        }
        (Some(git), path) => {
            crate::validate_git_url(git).with_context(|| format!("index `{}`", index.alias))?;
            // for git indexes the path is a directory *within* the clone:
            // it must stay relative and confined
            if let Some(path) = path {
                let confined = !path.is_empty()
                    && !path.starts_with('-')
                    && !path.starts_with('/')
                    && !path.starts_with('\\')
                    && !path.split(['/', '\\']).any(|c| c == "..")
                    && !path.contains(|c: char| c.is_control());
                if !confined {
                    eyre::bail!(
                        "index `{}`: `path` must be a relative directory inside \
                         the repository (no `..`, no leading `-`)",
                        index.alias
                    );
                }
            }
        }
        (None, Some(path)) => {
            if path.contains(|c: char| c.is_control()) {
                eyre::bail!("index `{}`: invalid `path`", index.alias);
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config(toml_str: &str) -> eyre::Result<ResolvedConfig> {
        let parsed: HubConfig = toml::from_str(toml_str).unwrap();
        ResolvedConfig::new(parsed.indexes, PathBuf::from("/cfg"))
    }

    const ACME: &str = r#"
[[index]]
alias = "acme"
git = "git@github.com:acme/dora-index.git"
namespaces = ["acme"]

[[index]]
alias = "fixtures"
path = "./tests/fixtures/index"
namespaces = ["test"]
"#;

    #[test]
    fn missing_config_is_official_only() {
        let resolved = ResolvedConfig::new(Vec::new(), PathBuf::from(".")).unwrap();
        assert_eq!(resolved.indexes.len(), 1);
        let index = resolved.index_for_namespace("anything");
        assert_eq!(index.alias, "official");
        assert_eq!(index.git.as_deref(), Some(crate::OFFICIAL_INDEX_GIT));
    }

    #[test]
    fn explicit_bindings_win_and_official_takes_the_rest() {
        let resolved = config(ACME).unwrap();
        assert_eq!(resolved.index_for_namespace("acme").alias, "acme");
        assert_eq!(resolved.index_for_namespace("test").alias, "fixtures");
        assert_eq!(resolved.index_for_namespace("dora-rs").alias, "official");
        assert_eq!(resolved.index_for_namespace("unbound").alias, "official");
    }

    #[test]
    fn double_binding_is_an_error() {
        let err = config(
            r#"
[[index]]
alias = "a"
git = "https://example.com/a"
namespaces = ["acme"]

[[index]]
alias = "b"
git = "https://example.com/b"
namespaces = ["acme"]
"#,
        )
        .unwrap_err();
        assert!(format!("{err}").contains("exactly one index"), "{err}");
    }

    #[test]
    fn duplicate_alias_is_an_error() {
        let err = config(
            r#"
[[index]]
alias = "a"
git = "https://example.com/a"

[[index]]
alias = "a"
git = "https://example.com/b"
"#,
        )
        .unwrap_err();
        assert!(format!("{err}").contains("duplicate index alias"), "{err}");
    }

    #[test]
    fn official_alias_can_be_overridden() {
        let resolved = config(
            r#"
[[index]]
alias = "official"
git = "https://git.corp.example/dora-index-mirror"
"#,
        )
        .unwrap();
        assert_eq!(resolved.indexes.len(), 1);
        assert_eq!(
            resolved.index_for_namespace("dora-rs").git.as_deref(),
            Some("https://git.corp.example/dora-index-mirror")
        );
    }

    #[test]
    fn entry_needs_git_or_path() {
        let err = config("[[index]]\nalias = \"x\"\n").unwrap_err();
        assert!(format!("{err}").contains("neither"), "{err}");
    }

    #[test]
    fn hostile_config_values_are_rejected() {
        // alias becomes a cache directory name
        for alias in ["", "../../evil", "a/b", "a\\b", "-x"] {
            let err = config(&format!(
                "[[index]]\nalias = '{alias}'\ngit = \"https://example.com/x\"\n"
            ))
            .unwrap_err();
            assert!(format!("{err}").contains("alias"), "{alias}: {err}");
        }
        // git URLs become subprocess arguments
        for url in ["--upload-pack=evil", "ext::sh -c evil", "host:path"] {
            let err = config(&format!("[[index]]\nalias = \"x\"\ngit = '{url}'\n")).unwrap_err();
            assert!(format!("{err:#}").contains("git URL"), "{url}: {err:#}");
        }
        // a git index's path must stay inside the clone
        for path in ["../outside", "/abs", "-flag", "a/../../b"] {
            let err = config(&format!(
                "[[index]]\nalias = \"x\"\ngit = \"https://example.com/x\"\npath = '{path}'\n"
            ))
            .unwrap_err();
            assert!(
                format!("{err}").contains("relative directory"),
                "{path}: {err}"
            );
        }
    }

    #[test]
    fn local_catalog_dir_resolves_relative_to_config() {
        let resolved = config(ACME).unwrap();
        let fixtures = resolved.index_for_namespace("test");
        let dir = fixtures.local_catalog_dir(&resolved.config_dir).unwrap();
        assert_eq!(dir, PathBuf::from("/cfg/tests/fixtures/index"));
    }
}
