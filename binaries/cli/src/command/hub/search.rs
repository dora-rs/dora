//! `dora hub search` — find packages by name/description/keyword/category.

use super::common::{HubContext, sanitize};
use crate::command::Executable;
use dora_core::manifest::Category;
use dora_hub_client::index::{IndexCatalog, IndexEntry};
use dora_hub_client::semver::Version;

/// Search the hub index for nodes
#[derive(Debug, clap::Args)]
pub struct Search {
    /// Substring to match against name, description, and keywords
    #[clap(value_name = "QUERY")]
    query: Option<String>,
    /// Restrict to a category (e.g. ml-inference, sensor)
    #[clap(long, value_name = "CATEGORY")]
    category: Option<String>,
    /// Restrict to packages supporting a platform (e.g. linux-x86_64)
    #[clap(long, value_name = "PLATFORM")]
    platform: Option<String>,
    /// Do not refresh indexes over the network (cache only)
    #[clap(long, action)]
    offline: bool,
}

impl Executable for Search {
    fn execute(self) -> eyre::Result<()> {
        let category = self.category.as_deref().map(parse_category).transpose()?;
        let query = self.query.as_deref().unwrap_or("").to_ascii_lowercase();

        let mut ctx = HubContext::load(self.offline)?;
        let mut hits = Vec::new();
        for (_alias, catalog) in ctx.all_catalogs() {
            for (namespace, name) in catalog.all_packages() {
                let Some((version, entry)) =
                    latest_listed_version(&catalog, &namespace, &name, |warning| {
                        eprintln!("  warning: {warning}");
                    })
                else {
                    continue;
                };
                let m = &entry.manifest;
                if let Some(category) = category
                    && !m.categories.contains(&category)
                {
                    continue;
                }
                if let Some(platform) = &self.platform
                    && !m.platforms.is_empty()
                    && !m.platforms.iter().any(|p| p == platform)
                {
                    continue;
                }
                if !query.is_empty() && !matches_query(m, &query) {
                    continue;
                }
                hits.push((format!("{namespace}/{name}"), version, entry));
            }
        }
        ctx.drain_warnings();

        if hits.is_empty() {
            println!("No matching nodes found.");
            return Ok(());
        }
        hits.sort_by(|a, b| a.0.cmp(&b.0));
        for (key, version, entry) in hits {
            let m = &entry.manifest;
            let categories = m
                .categories
                .iter()
                .map(|c| c.as_str())
                .collect::<Vec<_>>()
                .join(", ");
            print!("{key} {version}");
            if let Some(desc) = &m.description {
                print!(" — {}", sanitize(desc));
            }
            if !categories.is_empty() {
                print!(" ({categories})");
            }
            println!();
        }
        Ok(())
    }
}

fn matches_query(m: &dora_core::manifest::NodeManifest, query: &str) -> bool {
    let in_name = m
        .name
        .as_deref()
        .is_some_and(|n| n.to_ascii_lowercase().contains(query));
    let in_desc = m
        .description
        .as_deref()
        .is_some_and(|d| d.to_ascii_lowercase().contains(query));
    let in_keywords = m
        .keywords
        .iter()
        .any(|k| k.to_ascii_lowercase().contains(query));
    in_name || in_desc || in_keywords
}

/// The highest version of `namespace/name` that search should list: the
/// highest one that is readable and not yanked.
///
/// `versions()` returns every version, yanked ones included, so they are
/// filtered here to match `resolve()` and `dora hub info` / `dora hub
/// outdated`. Problems are reported through `warn` rather than returned, so
/// that one broken package does not end a search across every index, the same
/// way `HubContext::all_catalogs` skips a broken index (#4053):
///
/// - a package whose versions cannot be listed is skipped (`None`);
/// - an unreadable entry is skipped in favour of the next lower version, with
///   a warning, as `IndexCatalog::resolve` does, so that a corrupt highest
///   entry does not silently show an older version as the latest.
fn latest_listed_version(
    catalog: &IndexCatalog,
    namespace: &str,
    name: &str,
    mut warn: impl FnMut(String),
) -> Option<(Version, IndexEntry)> {
    let versions = match catalog.versions(namespace, name) {
        Ok(versions) => versions,
        Err(err) => {
            warn(format!("skipping package `{namespace}/{name}`: {err:#}"));
            return None;
        }
    };
    versions
        .into_iter()
        .rev()
        .find_map(|v| match catalog.entry(namespace, name, &v) {
            Ok(entry) => (!entry.yanked).then_some((v, entry)),
            Err(err) => {
                warn(format!(
                    "skipping unreadable index entry for `{namespace}/{name}` {v}: {err:#}"
                ));
                None
            }
        })
}

fn parse_category(s: &str) -> eyre::Result<Category> {
    serde_yaml::from_value(serde_yaml::Value::String(s.to_string()))
        .map_err(|_| eyre::eyre!("unknown category `{s}`"))
}

#[cfg(test)]
mod tests {
    use super::latest_listed_version;
    use dora_hub_client::index::IndexCatalog;

    fn entry_yaml(yanked: bool) -> String {
        format!(
            "manifest:\n  apiVersion: 1\n  name: foo\n  namespace: acme\n  runtime: python\n  entrypoint: foo\n\
             source:\n  git: https://github.com/acme/foo\n  rev: aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n\
             yanked: {yanked}\n"
        )
    }

    fn catalog(files: &[(&str, String)]) -> (tempfile::TempDir, IndexCatalog) {
        let tmp = tempfile::tempdir().unwrap();
        let pkg = tmp.path().join("acme/foo");
        std::fs::create_dir_all(&pkg).unwrap();
        for (file, content) in files {
            std::fs::write(pkg.join(file), content).unwrap();
        }
        let catalog = IndexCatalog::open(tmp.path()).unwrap();
        (tmp, catalog)
    }

    #[test]
    fn an_unreadable_highest_entry_is_skipped_with_a_warning() {
        let (_tmp, catalog) = catalog(&[
            ("1.0.0.yml", entry_yaml(false)),
            ("2.0.0.yml", "manifest: [not, a, manifest]\n".to_owned()),
        ]);
        let mut warnings = Vec::new();
        let (version, _) =
            latest_listed_version(&catalog, "acme", "foo", |w| warnings.push(w)).unwrap();
        assert_eq!(version.to_string(), "1.0.0");
        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(warnings[0].contains("acme/foo` 2.0.0"), "{warnings:?}");
    }

    #[test]
    fn yanked_versions_are_skipped_silently() {
        let (_tmp, catalog) = catalog(&[
            ("1.0.0.yml", entry_yaml(false)),
            ("2.0.0.yml", entry_yaml(true)),
        ]);
        let mut warnings = Vec::new();
        let (version, _) =
            latest_listed_version(&catalog, "acme", "foo", |w| warnings.push(w)).unwrap();
        assert_eq!(version.to_string(), "1.0.0");
        assert!(warnings.is_empty(), "{warnings:?}");
    }

    #[test]
    fn a_package_whose_versions_cannot_be_listed_is_skipped_with_a_warning() {
        let (_tmp, catalog) = catalog(&[("1.0.0.yml", entry_yaml(false))]);
        let mut warnings = Vec::new();
        // An invalid key part makes `versions()` fail, like an unreadable
        // package directory would.
        let found = latest_listed_version(&catalog, "acme", "bad name", |w| warnings.push(w));
        assert!(found.is_none());
        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(warnings[0].contains("skipping package"), "{warnings:?}");
    }
}
