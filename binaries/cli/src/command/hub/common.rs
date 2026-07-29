//! Shared helpers for the discovery-side `dora hub` commands (P2.4/P2.11).

use dora_hub_client::{
    config::{IndexConfig, ResolvedConfig},
    index::IndexCatalog,
    transport::IndexFetcher,
};
use eyre::Context;

/// Strip control and bidirectional/format characters from index-supplied
/// strings before printing them (a hostile index entry could otherwise inject
/// terminal escapes or visually reorder the rendered line).
///
/// `char::is_control` only covers Unicode category **Cc** (C0/C1 controls such
/// as ESC). It does *not* cover the bidi and separator format characters used
/// in Trojan-Source-style spoofing (CVE-2021-42574) — right-to-left overrides,
/// isolates, and line/paragraph separators — which can reorder or split a line
/// on the terminal even after the controls are gone. Drop those too.
pub fn sanitize(s: &str) -> String {
    s.chars()
        .filter(|c| {
            !c.is_control()
                && !matches!(
                    *c,
                    '\u{061C}'                // arabic letter mark
                        | '\u{200E}'..='\u{200F}' // left/right-to-left mark
                        | '\u{2028}'..='\u{2029}' // line / paragraph separator
                        | '\u{202A}'..='\u{202E}' // bidi embeddings + overrides
                        | '\u{2066}'..='\u{2069}' // bidi isolates
                )
        })
        .collect()
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

#[cfg(test)]
mod tests {
    use super::sanitize;

    #[test]
    fn strips_c0_control_and_escape() {
        assert_eq!(sanitize("a\x1b[31mred\x07b"), "a[31mredb");
        assert_eq!(sanitize("line1\nline2\t!"), "line1line2!");
    }

    #[test]
    fn strips_bidi_and_separator_format_chars() {
        // Right-to-left override (the classic Trojan-Source vector) and an
        // isolate must be removed even though they are not Cc controls.
        assert_eq!(sanitize("safe\u{202E}txet.js"), "safetxet.js");
        assert_eq!(sanitize("a\u{2066}b\u{2069}c"), "abc");
        assert_eq!(sanitize("x\u{2028}y\u{2029}z"), "xyz");
        assert_eq!(sanitize("\u{200E}\u{200F}\u{061C}ok"), "ok");
    }

    #[test]
    fn keeps_ordinary_text_untouched() {
        assert_eq!(sanitize("dora-yolo v1.2.3"), "dora-yolo v1.2.3");
        // Ordinary non-ASCII (accents, CJK) is preserved.
        assert_eq!(sanitize("café 日本語"), "café 日本語");
    }
}
