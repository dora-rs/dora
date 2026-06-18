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

// re-exported so consumers can name version types without a separate dep
pub use semver;

/// The namespace bare package names resolve to (spec §7.2).
pub const OFFICIAL_NAMESPACE: &str = "dora-rs";

/// True for a local-filesystem repository path (as opposed to a remote
/// transport URL): a Unix absolute path (`/repo`) or a Windows drive-letter
/// absolute path (`C:\repo` / `C:/repo`). UNC paths (`\\host\share`) are
/// deliberately excluded — they are remote SMB locations, not vetted local
/// repos. Used by both validators so the trusted/untrusted "is this local?"
/// decisions can never disagree (a Windows path the trusted validator accepts
/// must also trip the untrusted local-source gate).
fn is_local_path(url: &str) -> bool {
    if url.starts_with('/') {
        return true;
    }
    let b = url.as_bytes();
    b.len() >= 3 && b[0].is_ascii_alphabetic() && b[1] == b':' && (b[2] == b'/' || b[2] == b'\\')
}

/// Rewrite a local-filesystem repo path into its `file://` URL, or `None` for
/// anything that is not a local path (a remote/scp URL, a relative path).
///
/// `git` accepts a bare path directly, but consumers that parse the source with
/// `url::Url` (the CLI build's `GitManager`) read `C:\repo` as a `c:` URL and
/// reject `/repo` outright. Sharing this with [`is_local_path`] keeps the set of
/// paths the validators accept and the set the build can normalize identical.
pub fn local_path_to_file_url(url: &str) -> Option<String> {
    if !is_local_path(url) {
        return None;
    }
    let slashed = url.replace('\\', "/");
    // Unix `/srv/repo` -> `file:///srv/repo`; Windows `C:/repo` -> `file:///C:/repo`.
    Some(if slashed.starts_with('/') {
        format!("file://{slashed}")
    } else {
        format!("file:///{slashed}")
    })
}

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
    // absolute paths are accepted for local repositories (tests, mirrors) —
    // both Unix (`/repo`) and Windows (`C:\repo`) forms
    let valid = SCHEMES.iter().any(|s| url.starts_with(s)) || is_local_path(url);
    if !valid || url.contains(|c: char| c.is_control()) {
        eyre::bail!(
            "invalid git URL `{}`: expected a https/ssh/file URL or absolute path",
            url.chars().filter(|c| !c.is_control()).collect::<String>()
        );
    }
    // Defense-in-depth: reject a leading '-' in the host component.
    // ssh://-oProxyCommand=... or git@-... passes the scheme check above but
    // would look like an option flag to a bare `git` subprocess. All current
    // callers use libgit2 or a `--` separator, but the validator is documented
    // as the single pre-subprocess chokepoint (CVE-2017-1000117 shape).
    let host = if let Some(rest) = url
        .strip_prefix("https://")
        .or_else(|| url.strip_prefix("ssh://"))
        .or_else(|| url.strip_prefix("file://"))
    {
        // authority = everything before the first '/'; strip optional userinfo
        let authority = rest.split('/').next().unwrap_or(rest);
        let after_at = authority.rsplit('@').next().unwrap_or(authority);
        after_at.split(':').next().unwrap_or(after_at)
    } else if let Some(rest) = url.strip_prefix("git@") {
        rest.split(':').next().unwrap_or(rest)
    } else {
        ""
    };
    if host.starts_with('-') {
        eyre::bail!(
            "invalid git URL `{}`: host component starts with '-' (option injection risk)",
            url.chars().filter(|c| !c.is_control()).collect::<String>()
        );
    }
    Ok(())
}

/// Validate the `source.git` of an *untrusted* index entry — stricter than
/// [`validate_git_url`].
///
/// `file://` URLs and absolute paths are a supply-chain risk in a public index
/// (a hostile entry would clone victim-local content into the build), so they
/// are rejected for index entries unless `DORA_HUB_ALLOW_LOCAL_SOURCES` opts in
/// (hermetic tests, air-gapped mirrors). `hub.toml` index *locations* stay on
/// the permissive [`validate_git_url`] — that is the user's own trusted config.
pub fn validate_git_url_untrusted(url: &str) -> eyre::Result<()> {
    validate_git_url(url)?;
    let is_local = url.starts_with("file://") || is_local_path(url);
    if is_local && std::env::var_os("DORA_HUB_ALLOW_LOCAL_SOURCES").is_none() {
        eyre::bail!(
            "index entry source `{}` is a local path; a public index must use an \
             https/ssh remote (set DORA_HUB_ALLOW_LOCAL_SOURCES=1 to allow local \
             sources for testing or mirrors)",
            url.chars().filter(|c| !c.is_control()).collect::<String>()
        );
    }
    Ok(())
}

/// The official index location (spec §7.3).
pub const OFFICIAL_INDEX_GIT: &str = "https://github.com/dora-rs/dora-hub";
/// Catalog directory inside the official index repository.
pub const OFFICIAL_INDEX_PATH: &str = "node-index";
/// Alias of the implicit official index entry.
pub const OFFICIAL_INDEX_ALIAS: &str = "official";

#[cfg(test)]
mod tests {
    use super::{validate_git_url, validate_git_url_untrusted};

    #[test]
    fn untrusted_index_sources_reject_local_paths() {
        // remote transports are fine
        assert!(validate_git_url_untrusted("https://github.com/o/r.git").is_ok());
        assert!(validate_git_url_untrusted("ssh://git@host/o/r").is_ok());
        assert!(validate_git_url_untrusted("git@github.com:o/r.git").is_ok());
        // local sources would clone victim-local content into the build —
        // rejected for untrusted index entries (no opt-in env in this test)
        assert!(validate_git_url_untrusted("file:///tmp/repo").is_err());
        assert!(validate_git_url_untrusted("/srv/repo").is_err());
        // a Windows local path is just as much a local source: it must trip the
        // same gate, not slip through because the check only looked for `/`
        assert!(validate_git_url_untrusted("C:\\Users\\victim\\repo").is_err());
        assert!(validate_git_url_untrusted("c:/repo").is_err());
        // cleartext still rejected by the underlying validator
        assert!(validate_git_url_untrusted("http://github.com/o/r").is_err());
    }

    #[test]
    fn local_paths_rewrite_to_file_urls() {
        use super::local_path_to_file_url;
        assert_eq!(
            local_path_to_file_url("/srv/repo").as_deref(),
            Some("file:///srv/repo")
        );
        assert_eq!(
            local_path_to_file_url("C:\\repo").as_deref(),
            Some("file:///C:/repo")
        );
        assert_eq!(
            local_path_to_file_url("C:/repo").as_deref(),
            Some("file:///C:/repo")
        );
        // remote/scp/relative are not local paths — left for the caller to pass through
        assert_eq!(local_path_to_file_url("https://h/o/r"), None);
        assert_eq!(local_path_to_file_url("git@h:o/r"), None);
        assert_eq!(local_path_to_file_url("repo"), None);
    }

    #[test]
    fn accepts_windows_absolute_paths() {
        // local repos/mirrors are part of the contract on every platform
        assert!(validate_git_url("C:\\repo").is_ok());
        assert!(validate_git_url("C:/repo").is_ok());
        assert!(validate_git_url("/srv/repo").is_ok());
        // not absolute: a relative path or a drive-relative `C:repo` (no
        // separator) is rejected, as is a UNC path (remote SMB, not a local repo)
        assert!(validate_git_url("repo").is_err());
        assert!(validate_git_url("C:repo").is_err());
        assert!(validate_git_url("\\\\host\\share\\repo").is_err());
    }

    #[test]
    fn rejects_leading_dash_in_host() {
        // ssh://-oProxyCommand=... / git@-... are the classic option-injection
        // shapes (CVE-2017-1000117); they pass the scheme check but have a
        // leading '-' in the host component.
        assert!(validate_git_url("ssh://-oProxyCommand=touch /tmp/x/path").is_err());
        assert!(validate_git_url("https://-evil.example.com/r").is_err());
        assert!(validate_git_url("git@-evil.example.com:o/r.git").is_err());
        // normal URLs are unaffected
        assert!(validate_git_url("ssh://git@github.com/o/r").is_ok());
        assert!(validate_git_url("https://github.com/o/r.git").is_ok());
        assert!(validate_git_url("git@github.com:o/r.git").is_ok());
    }
}
