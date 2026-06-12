//! Shared helpers for the discovery-side `dora hub` commands (P2.4/P2.11).

use dora_hub_client::{
    config::{IndexConfig, ResolvedConfig},
    index::IndexCatalog,
    transport::IndexFetcher,
};
use eyre::Context;

/// Strip control characters from index-supplied strings before printing them
/// (a hostile index entry could otherwise inject terminal escapes).
pub fn sanitize(s: &str) -> String {
    s.chars().filter(|c| !c.is_control()).collect()
}

/// Loaded hub configuration plus a fetcher, shared by the discovery commands.
pub struct HubContext {
    pub config: ResolvedConfig,
    pub fetcher: IndexFetcher,
}

impl HubContext {
    pub fn load(offline: bool) -> eyre::Result<Self> {
        Ok(Self {
            config: ResolvedConfig::load_default().context("failed to load hub configuration")?,
            fetcher: IndexFetcher::new(offline)?,
        })
    }

    /// Open the catalog the given namespace resolves against.
    pub fn catalog_for_namespace(&mut self, namespace: &str) -> eyre::Result<IndexCatalog> {
        let index = self.config.index_for_namespace(namespace).clone();
        self.open(&index)
    }

    /// Open every configured index's catalog (for cross-namespace search).
    pub fn all_catalogs(&mut self) -> Vec<(String, IndexCatalog)> {
        let indexes = self.config.indexes.clone();
        indexes
            .into_iter()
            .filter_map(|index| {
                let alias = index.alias.clone();
                match self.open(&index) {
                    Ok(catalog) => Some((alias, catalog)),
                    Err(err) => {
                        eprintln!("  warning: skipping index `{alias}`: {err:#}");
                        None
                    }
                }
            })
            .collect()
    }

    fn open(&mut self, index: &IndexConfig) -> eyre::Result<IndexCatalog> {
        let dir = self
            .fetcher
            .catalog_dir(index, &self.config.config_dir)
            .with_context(|| format!("failed to fetch index `{}`", index.alias))?;
        IndexCatalog::open(&dir)
    }

    /// Print any accumulated transport warnings (e.g. index rollback).
    pub fn drain_warnings(&mut self) {
        for warning in self.fetcher.warnings.drain(..) {
            eprintln!("  warning: {warning}");
        }
    }
}
