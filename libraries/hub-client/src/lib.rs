//! Client for the Dora Hub package index (docs/plan-node-hub.md §7).
//!
//! The index is a git-hosted catalog mapping `namespace/name@version` to a
//! node manifest plus a source pointer. This crate provides:
//!
//! - [`reference`]: parsing `[<namespace>/]<name>@<semver-req>` references,
//! - [`index`]: the on-disk catalog format and semver/yank resolution,
//! - [`config`]: `hub.toml` multi-index configuration with namespace binding,
//! - [`transport`]: sparse-clone fetch and refresh of remote indexes.
//!
//! Everything network-facing lives in [`transport`]; the rest are pure
//! functions over parsed data, usable against fixture directories in tests.

pub mod config;
pub mod index;
pub mod reference;
pub mod transport;

/// The namespace bare package names resolve to (spec §7.2).
pub const OFFICIAL_NAMESPACE: &str = "dora-rs";

/// Validate a git URL before it is ever passed to a `git` subprocess.
///
/// URLs come from untrusted places (a public index entry's `source.git`, a
/// user-edited `hub.toml`). Restricting to known-remote schemes rejects both
/// argument injection (`--upload-pack=…` parsed as a flag) and command-running
/// transports (`ext::sh -c …`).
pub fn validate_git_url(url: &str) -> eyre::Result<()> {
    // cleartext transports (http://, git://) are deliberately absent: the
    // index is a supply-chain trust root and its fast-forward protection is
    // meaningless against a MITM-controlled remote
    const SCHEMES: &[&str] = &["https://", "ssh://", "file://", "git@"];
    // absolute paths are accepted for local repositories (tests, mirrors)
    let valid = SCHEMES.iter().any(|s| url.starts_with(s)) || url.starts_with('/');
    if valid && !url.contains(|c: char| c.is_control()) {
        return Ok(());
    }
    eyre::bail!(
        "invalid git URL `{}`: expected a https/ssh/file URL or absolute path",
        url.chars().filter(|c| !c.is_control()).collect::<String>()
    )
}

/// The official index location (spec §7.3).
pub const OFFICIAL_INDEX_GIT: &str = "https://github.com/dora-rs/dora-hub";
/// Catalog directory inside the official index repository.
pub const OFFICIAL_INDEX_PATH: &str = "node-index";
/// Alias of the implicit official index entry.
pub const OFFICIAL_INDEX_ALIAS: &str = "official";
