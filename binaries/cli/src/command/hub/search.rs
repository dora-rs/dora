//! `dora hub search` — find packages by name/description/keyword/category.

use super::common::{HubContext, sanitize};
use crate::command::Executable;
use dora_core::manifest::Category;
use dora_hub_client::index::{IndexCatalog, IndexEntry};
use dora_hub_client::semver::Version;

/// Search the hub index for nodes
#[derive(Debug, clap::Args)]
pub struct Search {
    /// Substring to match against the package key (`namespace/name`), the
    /// manifest name, description, and keywords
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
        let query = self.query.as_deref().unwrap_or("").to_lowercase();

        let mut ctx = HubContext::load(self.offline)?;
        let mut hits = Vec::new();
        for (_alias, catalog) in ctx.all_catalogs() {
            for (namespace, name) in catalog.all_packages() {
                let (latest, warnings) = latest_listed_version(&catalog, &namespace, &name);
                let print_warnings = || {
                    for warning in &warnings {
                        eprintln!("  warning: {warning}");
                    }
                };
                let Some((version, entry)) = latest else {
                    // Nothing of this package can be listed: say why, whatever
                    // the query.
                    print_warnings();
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
                let key = format!("{namespace}/{name}");
                if !query.is_empty() && !matches_query(&key, m, &query) {
                    continue;
                }
                // Warnings about skipped versions matter only for a package
                // that is shown, where they explain why an older version is
                // listed. Printing them for every package would repeat them
                // on every search, e.g. for each entry an older CLI cannot
                // parse.
                print_warnings();
                hits.push((key, version, entry));
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

/// Whether `query` (already lowercased) occurs in the package `key` that
/// search prints (`namespace/name`, from the index layout), or in the
/// manifest's name, description, or keywords.
///
/// The key is matched as well as `manifest.name` because the two can differ:
/// `name` is optional in an index entry and nothing ties it to the entry's
/// directory, so a user must be able to find a package by the key they see
/// and pass to `dora hub info` (#4052).
fn matches_query(key: &str, m: &dora_core::manifest::NodeManifest, query: &str) -> bool {
    let contains = |s: &str| s.to_lowercase().contains(query);
    contains(key)
        || m.name.as_deref().is_some_and(contains)
        || m.description.as_deref().is_some_and(contains)
        || m.keywords.iter().any(|k| contains(k))
}

/// The highest version of `namespace/name` that search should list: the
/// highest one that is readable and not yanked.
///
/// `versions()` returns every version, yanked ones included, so they are
/// filtered here to match `resolve()` and `dora hub info` / `dora hub
/// outdated`. Problems are returned as warnings rather than as an error, so
/// that one broken package does not end a search across every index, the same
/// way `HubContext::all_catalogs` skips a broken index (#4053). The warnings
/// are sanitized, because they quote index content (a serde message names an
/// unknown field verbatim) and an index may be hostile:
///
/// - a package whose versions cannot be listed is skipped (`None`);
/// - an unreadable entry is skipped in favour of the next lower version, with
///   a warning, as `IndexCatalog::resolve` does, so that a corrupt highest
///   entry does not silently show an older version as the latest.
fn latest_listed_version(
    catalog: &IndexCatalog,
    namespace: &str,
    name: &str,
) -> (Option<(Version, IndexEntry)>, Vec<String>) {
    let mut warnings = Vec::new();
    let versions = match catalog.versions(namespace, name) {
        Ok(versions) => versions,
        Err(err) => {
            warnings.push(sanitize(&format!(
                "skipping package `{namespace}/{name}`: {err:#}"
            )));
            return (None, warnings);
        }
    };
    let latest =
        versions
            .into_iter()
            .rev()
            .find_map(|v| match catalog.entry(namespace, name, &v) {
                Ok(entry) => (!entry.yanked).then_some((v, entry)),
                Err(err) => {
                    warnings.push(sanitize(&format!(
                        "skipping unreadable index entry for `{namespace}/{name}` {v}: {err:#}"
                    )));
                    None
                }
            });
    (latest, warnings)
}

fn parse_category(s: &str) -> eyre::Result<Category> {
    serde_yaml::from_value(serde_yaml::Value::String(s.to_string()))
        .map_err(|_| eyre::eyre!("unknown category `{s}`"))
}

#[cfg(test)]
mod tests {
    use super::{latest_listed_version, matches_query};
    use dora_core::manifest::NodeManifest;
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
        let (latest, warnings) = latest_listed_version(&catalog, "acme", "foo");
        assert_eq!(latest.unwrap().0.to_string(), "1.0.0");
        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(warnings[0].contains("acme/foo` 2.0.0"), "{warnings:?}");
    }

    #[test]
    fn yanked_versions_are_skipped_silently() {
        let (_tmp, catalog) = catalog(&[
            ("1.0.0.yml", entry_yaml(false)),
            ("2.0.0.yml", entry_yaml(true)),
        ]);
        let (latest, warnings) = latest_listed_version(&catalog, "acme", "foo");
        assert_eq!(latest.unwrap().0.to_string(), "1.0.0");
        assert!(warnings.is_empty(), "{warnings:?}");
    }

    /// The serde message for an entry rejected by `deny_unknown_fields`
    /// quotes the unknown key from the index file, and a double-quoted YAML
    /// key can carry a terminal escape. The warning must not.
    #[test]
    fn warnings_strip_terminal_escapes_from_index_content() {
        let hostile = format!(
            "{}\"\\u001b[2J\\u001b]0;pwned\\u0007\": 1\n",
            entry_yaml(false)
        );
        let (_tmp, catalog) = catalog(&[("1.0.0.yml", entry_yaml(false)), ("2.0.0.yml", hostile)]);
        let (latest, warnings) = latest_listed_version(&catalog, "acme", "foo");
        assert_eq!(latest.unwrap().0.to_string(), "1.0.0");
        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(warnings[0].contains("pwned"), "{warnings:?}");
        assert!(!warnings[0].chars().any(char::is_control), "{warnings:?}");
    }

    #[test]
    fn a_package_whose_versions_cannot_be_listed_is_skipped_with_a_warning() {
        let tmp = tempfile::tempdir().unwrap();
        // A regular file where the package directory should be makes
        // `read_dir` fail with an IO error other than NotFound, like an
        // unreadable package directory would.
        std::fs::create_dir_all(tmp.path().join("acme")).unwrap();
        std::fs::write(tmp.path().join("acme/foo"), "").unwrap();
        let catalog = IndexCatalog::open(tmp.path()).unwrap();
        let (latest, warnings) = latest_listed_version(&catalog, "acme", "foo");
        assert!(latest.is_none());
        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(warnings[0].contains("skipping package"), "{warnings:?}");
    }

    /// A manifest with the required fields filled in, plus `extra`.
    fn manifest(extra: &str) -> NodeManifest {
        let yaml = format!("namespace: acme\nruntime: rust\nentrypoint: bin/node\n{extra}");
        serde_yaml::from_str(&yaml).unwrap()
    }

    #[test]
    fn matches_the_package_key_when_the_manifest_has_no_name() {
        let m = manifest("apiVersion: 1\ndescription: A sensor node\n");
        assert!(m.name.is_none());
        assert!(matches_query("acme/lidar-driver", &m, "lidar-driver"));
        assert!(matches_query("acme/lidar-driver", &m, "acme"));
        assert!(!matches_query("acme/lidar-driver", &m, "camera"));
    }

    #[test]
    fn matches_the_package_key_when_the_manifest_name_differs() {
        let m = manifest("apiVersion: 1\nname: lidar\n");
        assert!(matches_query("acme/lidar-driver", &m, "driver"));
    }

    #[test]
    fn matches_manifest_name_description_and_keywords() {
        let m = manifest(
            "apiVersion: 1\nname: yolo\ndescription: Object detection\nkeywords: [vision]\n",
        );
        assert!(matches_query("dora-rs/dora-yolo-v8", &m, "yolo"));
        assert!(matches_query("dora-rs/x", &m, "detection"));
        assert!(matches_query("dora-rs/x", &m, "vision"));
    }

    #[test]
    fn folds_case_beyond_ascii() {
        // A non-ASCII capital in the haystack: ASCII-only folding leaves
        // `É` as is, so a lowercase query would not match it.
        let m = manifest("apiVersion: 1\ndescription: DÉTECTION of objects\n");
        assert!(matches_query("acme/x", &m, "détection"));
    }
}
