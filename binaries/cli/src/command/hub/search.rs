//! `dora hub search` — find packages by name/description/keyword/category.

use super::common::{HubContext, sanitize};
use crate::command::Executable;
use dora_core::manifest::Category;

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
                // Skip yanked versions — find the highest non-yanked version.
                // versions() returns all versions including yanked ones, so we
                // must filter here to match the behavior of resolve() and
                // `dora hub info` / `dora hub outdated`.
                let Some((version, entry)) = catalog
                    .versions(&namespace, &name)?
                    .into_iter()
                    .rev()
                    .find_map(|v| {
                        catalog
                            .entry(&namespace, &name, &v)
                            .ok()
                            .filter(|e| !e.yanked)
                            .map(|e| (v, e))
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
                let key = format!("{namespace}/{name}");
                if !query.is_empty() && !matches_query(&key, m, &query) {
                    continue;
                }
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

fn parse_category(s: &str) -> eyre::Result<Category> {
    serde_yaml::from_value(serde_yaml::Value::String(s.to_string()))
        .map_err(|_| eyre::eyre!("unknown category `{s}`"))
}

#[cfg(test)]
mod tests {
    use super::matches_query;
    use dora_core::manifest::NodeManifest;

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
