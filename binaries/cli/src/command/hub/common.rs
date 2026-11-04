//! Shared helpers for the discovery-side `dora hub` commands (P2.4/P2.11).

use dora_hub_client::{
    config::{IndexConfig, ResolvedConfig},
    index::IndexCatalog,
    transport::IndexFetcher,
};
use eyre::Context;

/// A `git` command for the user's own node repository at `dir`, as `dora hub
/// init` and `dora hub publish` read it (the `origin` remote, the commit to
/// pin, the files committed at it).
///
/// Built by hub-client's `git_command`, which drops the repository-locating
/// variables (`GIT_DIR`, `GIT_WORK_TREE`, ...) that git exports to hooks and
/// CI steps sometimes set. They take precedence over the working directory,
/// so inherited they would make these commands read *another* repository's
/// HEAD and `origin`, and `publish` would pin the wrong commit (the same
/// class #3967 fixed for the index cache).
pub(super) fn node_repo_git(dir: &std::path::Path) -> std::process::Command {
    dora_hub_client::transport::git_command(Some(dir))
}

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
    use super::{node_repo_git, sanitize};

    /// `dora hub init`/`publish` must not let an inherited `GIT_DIR` and its
    /// siblings point their git commands at another repository.
    #[test]
    fn node_repo_git_drops_the_repository_locating_environment() {
        let dir = std::path::Path::new(".");
        let command = node_repo_git(dir);
        let removed: Vec<_> = command
            .get_envs()
            .filter(|(_, value)| value.is_none())
            .map(|(key, _)| key.to_string_lossy().into_owned())
            .collect();
        for var in [
            "GIT_DIR",
            "GIT_WORK_TREE",
            "GIT_INDEX_FILE",
            "GIT_COMMON_DIR",
        ] {
            assert!(removed.iter().any(|r| r == var), "`{var}` is inherited");
        }
        assert_eq!(command.get_current_dir(), Some(dir));
    }

    /// `dora hub publish` runs from the node's directory, usually a
    /// subdirectory of its repository (`node-hub/hello`), and relies on
    /// `rev-parse --show-prefix` finding the enclosing repository from there.
    /// A discovery ceiling on this helper, as managed clones get, would break
    /// that (dora-rs/dora#4017 review).
    #[test]
    fn node_repo_git_finds_the_repository_from_a_node_subdirectory() {
        let repo = tempfile::tempdir().unwrap();
        let node = repo.path().join("node-hub/hello");
        std::fs::create_dir_all(&node).unwrap();
        // Through `node_repo_git` too: a plain `git` would inherit `GIT_DIR` /
        // `GIT_INDEX_FILE` when the tests run from a git hook, and commit
        // into the developer's own repository (dora-rs/dora#4017 review).
        let git = |args: &[&str]| {
            let status = node_repo_git(repo.path())
                .args([
                    "-c",
                    "user.email=t@dora.rs",
                    "-c",
                    "user.name=t",
                    "-c",
                    "commit.gpgsign=false",
                ])
                .args(args)
                .status()
                .unwrap();
            assert!(status.success(), "git {args:?} failed");
        };
        git(&["init", "--quiet"]);
        std::fs::write(node.join("dora-node.yml"), "name: hello\n").unwrap();
        git(&["add", "."]);
        git(&["commit", "--quiet", "-m", "init"]);

        let output = node_repo_git(&node)
            .args(["rev-parse", "--show-prefix"])
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        assert_eq!(
            String::from_utf8_lossy(&output.stdout).trim(),
            "node-hub/hello/"
        );
    }

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
