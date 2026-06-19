//! `dora hub search` — find packages by name/description/keyword/category.

use super::common::{HubContext, sanitize};
use crate::command::Executable;
use dora_core::manifest::Category;

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

fn parse_category(s: &str) -> eyre::Result<Category> {
    serde_yaml::from_value(serde_yaml::Value::String(s.to_string()))
        .map_err(|_| eyre::eyre!("unknown category `{s}`"))
}
