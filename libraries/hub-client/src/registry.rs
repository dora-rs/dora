//! HTTP registry transport (default: <https://dora.pub>).
//!
//! The official deployment is not live yet, so the URL is overridable with
//! `DORA_HUB_REGISTRY_URL` (local tests, enterprise mirrors, preview
//! environments). The registry speaks the same sparse-index protocol as the
//! static catalogs: `<base>/index/<namespace>/<prefix>/<name>` returns JSON
//! lines, one per version, each embedding the node manifest.
//!
//! To keep the existing resolver, lockfile and build pipeline unchanged, the
//! transport **materializes a dora index catalog** under the user cache:
//!
//! ```text
//! ~/.cache/dora/registry/<registry-key>/catalog/<ns>/<name>/<version>.yml
//! ```
//!
//! `IndexCatalog` then resolves/yanks/pins exactly as it does for git/path
//! indexes. The cache is refreshed once per package per CLI invocation; with
//! `--offline` it is used as-is and a cache miss is a hard error.

use std::{
    collections::BTreeSet,
    fs,
    io::{Read, Write},
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

use dora_core::manifest::NodeManifest;
use eyre::{Context, bail};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use url::Url;

use crate::{
    index::{BinaryArtifact, IndexEntry, SourceSpec},
    transport::git_command,
};

/// Official registry. The website is not launched yet; this is the default
/// that `dora build/run` and `dora hub publish` use when no override is set.
pub const DEFAULT_REGISTRY_URL: &str = "https://dora.pub";
/// Overrides the registry for local tests, mirrors and preview deployments.
pub const REGISTRY_URL_ENV: &str = "DORA_HUB_REGISTRY_URL";
/// Explicit opt-in for cleartext HTTP to a non-loopback registry (LAN mirror).
pub const ALLOW_INSECURE_REGISTRY_ENV: &str = "DORA_HUB_ALLOW_INSECURE_REGISTRY";
/// Token file override (shared with the `dora-hub` client in hub-registry).
pub const TOKEN_FILE_ENV: &str = "DORA_HUB_TOKEN_FILE";
/// One-off token environment variable. Only honored for the official registry;
/// a mirror must receive a `--token` or its own saved per-registry token.
pub const TOKEN_ENV: &str = "DORA_HUB_TOKEN";

const REQUEST_TIMEOUT: Duration = Duration::from_secs(30);
/// Upper bound for a sparse-index response.
const MAX_INDEX_BYTES: u64 = 4 * 1024 * 1024;
/// Upper bound for small JSON responses (search, yank/publish replies).
const MAX_JSON_BYTES: u64 = 1024 * 1024;
/// Upper bound for the registry's source archive download.
const MAX_ARCHIVE_BYTES: u64 = 64 * 1024 * 1024;
/// Upper bound for an error body kept for diagnostics.
const MAX_ERROR_BYTES: u64 = 64 * 1024;
/// Upper bound on archive entries, in addition to the expanded-size limit.
const MAX_ARCHIVE_ENTRIES: usize = 100_000;

/// The effective registry URL: env override or the official default.
pub fn registry_url() -> String {
    registry_url_from(std::env::var(REGISTRY_URL_ENV).ok().as_deref())
}

fn registry_url_from(value: Option<&str>) -> String {
    match value {
        Some(value) if !value.trim().is_empty() => value.trim().trim_end_matches('/').to_string(),
        _ => DEFAULT_REGISTRY_URL.to_string(),
    }
}

/// Validate a registry base URL before it is used for network requests.
/// Cleartext HTTP is accepted only for loopback hosts unless the operator
/// explicitly opts in; tokens must never travel over plaintext by accident.
pub fn validate_registry_url(raw: &str) -> eyre::Result<Url> {
    let url = Url::parse(raw).with_context(|| format!("invalid registry URL `{raw}`"))?;
    if url.username() != "" || url.password().is_some() {
        bail!("registry URL must not contain credentials");
    }
    if url.query().is_some() || url.fragment().is_some() {
        bail!("registry URL must not contain a query or fragment");
    }
    match url.scheme() {
        "https" => {}
        "http" => {
            let host = url.host_str().unwrap_or_default();
            let loopback =
                host == "localhost" || host == "127.0.0.1" || host == "::1" || host == "[::1]";
            if !loopback && std::env::var_os(ALLOW_INSECURE_REGISTRY_ENV).is_none() {
                bail!(
                    "refusing cleartext registry URL `{raw}`; use https, a loopback host, \
                     or set {ALLOW_INSECURE_REGISTRY_ENV}=1 for a trusted LAN mirror"
                );
            }
        }
        other => bail!("registry URL scheme `{other}` is not supported (use https)"),
    }
    if url.host_str().is_none() {
        bail!("registry URL must contain a host");
    }
    let path = url.path().trim_end_matches('/').to_string();
    let mut normalized = url;
    normalized.set_path(&format!("{path}/"));
    Ok(normalized)
}

/// A registry client with a per-invocation package cache.
pub struct RegistryClient {
    base: String,
    cache_root: PathBuf,
    offline: bool,
    fetched: BTreeSet<(String, String)>,
    client: reqwest::blocking::Client,
}

impl RegistryClient {
    pub fn new(offline: bool) -> eyre::Result<Self> {
        let base = registry_url();
        let cache_root = default_cache_root(&base)?;
        Self::with_base(&base, cache_root, offline)
    }

    /// Client for an explicit registry URL (CLI `--registry` override).
    pub fn with_url(base: &str, offline: bool) -> eyre::Result<Self> {
        let cache_root = default_cache_root(base)?;
        Self::with_base(base, cache_root, offline)
    }

    /// Explicit base URL (tests, or a CLI `--registry` override).
    pub fn with_base(base: &str, cache_root: PathBuf, offline: bool) -> eyre::Result<Self> {
        let normalized = validate_registry_url(base)?;
        let client = reqwest::blocking::Client::builder()
            .timeout(REQUEST_TIMEOUT)
            .user_agent(format!("dora-hub-client/{}", env!("CARGO_PKG_VERSION")))
            // Never let an https:// registry redirect a token-bearing request
            // to plaintext. Loopback test servers stay usable over http.
            .https_only(normalized.scheme() == "https")
            .build()
            .context("failed to build registry HTTP client")?;
        Ok(Self {
            base: normalized.as_str().trim_end_matches('/').to_string(),
            cache_root,
            offline,
            fetched: BTreeSet::new(),
            client,
        })
    }

    pub fn base_url(&self) -> &str {
        &self.base
    }

    /// Materialize `namespace/name` and return the catalog root.
    pub fn catalog_dir(&mut self, namespace: &str, name: &str) -> eyre::Result<PathBuf> {
        self.ensure_package(namespace, name)?;
        let root = self.cache_root.join("catalog");
        fs::create_dir_all(&root)
            .with_context(|| format!("failed to create `{}`", root.display()))?;
        Ok(root)
    }

    fn ensure_package(&mut self, namespace: &str, name: &str) -> eyre::Result<()> {
        validate_component("namespace", namespace)?;
        validate_component("package name", name)?;
        let key = (namespace.to_string(), name.to_string());
        if self.fetched.contains(&key) {
            return Ok(());
        }
        // Serialize refresh and offline reads per package. Without this, one
        // process can see a removed package dir, a half-written version file,
        // or a partial version set while another refreshes the cache.
        let _cache_lock = lock_package_cache(&self.cache_root, namespace, name)?;
        let package_dir = self.cache_root.join("catalog").join(namespace).join(name);
        if self.offline {
            if !package_dir.is_dir() {
                bail!(
                    "`{namespace}/{name}` is not in the registry cache and `--offline` is set\n  \
                     hint: run once with network access (registry: {})",
                    self.base
                );
            }
            self.fetched.insert(key);
            return Ok(());
        }

        self.fetch_package(namespace, name)
            .with_context(|| format!("failed to fetch `{namespace}/{name}` from {}", self.base))?;
        self.fetched.insert(key);
        Ok(())
    }

    fn fetch_package(&mut self, namespace: &str, name: &str) -> eyre::Result<()> {
        let url = self.sparse_index_url(namespace, name);
        let response = self
            .client
            .get(&url)
            .send()
            .with_context(|| format!("registry request failed: {url}"))?;
        let status = response.status();
        if status == reqwest::StatusCode::NOT_FOUND {
            bail!("package `{namespace}/{name}` was not found in the registry");
        }
        if !status.is_success() {
            let detail = read_limited(response, MAX_ERROR_BYTES, "registry error body")
                .map(|bytes| registry_error_detail(&String::from_utf8_lossy(&bytes)))
                .unwrap_or_default();
            bail!("registry returned {status} for `{namespace}/{name}`{detail}");
        }
        let body = read_limited(response, MAX_INDEX_BYTES, "sparse index response")
            .with_context(|| format!("failed to read registry response from {url}"))?;
        let body = String::from_utf8(body).with_context(|| {
            format!("registry returned non-UTF-8 data for `{namespace}/{name}`")
        })?;
        let mut entries = Vec::new();
        for (line_no, line) in body.lines().enumerate() {
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            let raw: SparseEntry = serde_json::from_str(line).with_context(|| {
                format!(
                    "invalid sparse-index line {} for `{namespace}/{name}`",
                    line_no + 1
                )
            })?;
            if raw.namespace != namespace || raw.name != name {
                bail!(
                    "registry index entry for `{namespace}/{name}` declares `{}/{}` — \
                     refusing a mismatched entry",
                    raw.namespace,
                    raw.name
                );
            }
            entries.push(raw.into_index_entry(namespace, name)?);
        }
        if entries.is_empty() {
            bail!("package `{namespace}/{name}` has no versions in the registry");
        }

        let catalog_root = self.cache_root.join("catalog");
        let package_dir = catalog_root.join(namespace).join(name);
        fs::create_dir_all(&catalog_root)?;
        // Build the new package view in a private staging directory, then swap
        // it into place. The package lock in `ensure_package` makes concurrent
        // readers/writers see a complete old or new view, never a partial one.
        // Keep staging/locks outside `catalog/`: the index scanner treats every
        // directory there as a namespace.
        let staging = self
            .cache_root
            .join(format!(".{namespace}-{name}.{}.tmp", std::process::id()));
        if staging.exists() {
            fs::remove_dir_all(&staging)
                .with_context(|| format!("failed to clear `{}`", staging.display()))?;
        }
        fs::create_dir_all(&staging)
            .with_context(|| format!("failed to create `{}`", staging.display()))?;
        for (version, entry) in entries {
            let path = staging.join(format!("{version}.yml"));
            let yaml = serde_yaml::to_string(&entry)
                .with_context(|| format!("failed to serialize registry entry `{version}`"))?;
            fs::write(&path, yaml)
                .with_context(|| format!("failed to cache `{}`", path.display()))?;
        }
        let backup = self
            .cache_root
            .join(format!(".{namespace}-{name}.{}.old", std::process::id()));
        if backup.exists() {
            fs::remove_dir_all(&backup)
                .with_context(|| format!("failed to clear `{}`", backup.display()))?;
        }
        if let Some(parent) = package_dir.parent() {
            fs::create_dir_all(parent)
                .with_context(|| format!("failed to create `{}`", parent.display()))?;
        }
        if package_dir.exists() {
            fs::rename(&package_dir, &backup).with_context(|| {
                format!("failed to stage refresh of `{}`", package_dir.display())
            })?;
        }
        if let Err(error) = fs::rename(&staging, &package_dir) {
            // Restore the previous complete view if the final swap fails.
            if backup.exists() {
                let _ = fs::rename(&backup, &package_dir);
            }
            return Err(error)
                .with_context(|| format!("failed to install `{}`", package_dir.display()));
        }
        if backup.exists() {
            fs::remove_dir_all(&backup)
                .with_context(|| format!("failed to remove `{}`", backup.display()))?;
        }
        Ok(())
    }

    fn sparse_index_url(&self, namespace: &str, name: &str) -> String {
        format!(
            "{}/index/{}/{}/{}",
            self.base,
            namespace,
            sparse_prefix(name),
            name
        )
    }

    /// Search packages through the registry's read API.
    pub fn search(&self, query: &str) -> eyre::Result<Vec<RegistryPackage>> {
        if self.offline {
            bail!(
                "registry search requires network access; remove `--offline` or use a \
                 configured index"
            );
        }
        let mut request = self.client.get(format!("{}/api/v1/packages", self.base));
        if !query.trim().is_empty() {
            request = request.query(&[("q", query.trim())]);
        }
        let response = request
            .send()
            .with_context(|| format!("failed to reach registry {}", self.base))?;
        let value = json_response(response)?;
        let packages = value
            .get("packages")
            .cloned()
            .unwrap_or_else(|| Value::Array(Vec::new()));
        serde_json::from_value(packages).context("invalid package search response from registry")
    }

    /// Yank or restore a version through the registry API.
    pub fn set_yanked(
        &self,
        token: &str,
        namespace: &str,
        name: &str,
        version: &str,
        yanked: bool,
    ) -> eyre::Result<Value> {
        let action = if yanked { "yank" } else { "unyank" };
        let response = self
            .client
            .post(format!(
                "{}/api/v1/packages/{namespace}/{name}/{version}/{action}",
                self.base
            ))
            .header(reqwest::header::AUTHORIZATION, format!("Bearer {token}"))
            .send()
            .with_context(|| format!("failed to reach registry {}", self.base))?;
        json_response(response)
    }

    /// Upload a package archive (`tar.gz`) with the Cargo-style length-prefixed
    /// body the registry expects.
    pub fn publish(
        &self,
        token: &str,
        metadata: &PublishMetadata,
        archive: &[u8],
    ) -> eyre::Result<Value> {
        let metadata =
            serde_json::to_vec(metadata).context("failed to serialize publish metadata")?;
        let mut body = Vec::with_capacity(8 + metadata.len() + archive.len());
        body.extend_from_slice(&(metadata.len() as u32).to_le_bytes());
        body.extend_from_slice(&metadata);
        body.extend_from_slice(&(archive.len() as u32).to_le_bytes());
        body.extend_from_slice(archive);
        let response = self
            .client
            .put(format!("{}/api/v1/packages/new", self.base))
            .header(reqwest::header::AUTHORIZATION, format!("Bearer {token}"))
            .header(reqwest::header::CONTENT_TYPE, "application/octet-stream")
            .body(body)
            .send()
            .with_context(|| format!("failed to reach registry {}", self.base))?;
        json_response(response)
    }
}

/// A package summary returned by the registry's search API.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegistryPackage {
    pub namespace: String,
    pub name: String,
    #[serde(default)]
    pub description: Option<String>,
    #[serde(default)]
    pub latest_version: Option<String>,
    #[serde(default)]
    pub downloads_total: Option<i64>,
}

#[derive(Debug, Clone, Serialize)]
pub struct PublishMetadata {
    pub namespace: String,
    pub name: String,
    pub version: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub license: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub repository: Option<String>,
    /// Hosting repository + commit for this version. The registry stores this
    /// so clients can fetch the source from the project's own platform; the
    /// uploaded archive remains the registry fallback.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<PublishSource>,
}

#[derive(Debug, Clone, Serialize)]
pub struct PublishSource {
    pub git: String,
    pub rev: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub subdir: Option<String>,
}

#[derive(Debug, Deserialize)]
struct SparseEntry {
    name: String,
    namespace: String,
    vers: String,
    manifest: Value,
    source: Value,
    /// SHA-256 of the uploaded source archive (the registry's `cksum`).
    /// Optional for compatibility with older registry builds, but a missing or
    /// malformed value disables the fallback rather than trusting the bytes.
    #[serde(default)]
    cksum: Option<String>,
    #[serde(default)]
    yanked: bool,
    #[serde(default)]
    yank_reason: Option<String>,
    #[serde(default)]
    published: Option<String>,
    #[serde(default)]
    pubtime: Option<String>,
}

impl SparseEntry {
    fn into_index_entry(
        self,
        namespace: &str,
        name: &str,
    ) -> eyre::Result<(semver::Version, IndexEntry)> {
        if self.namespace != namespace || self.name != name {
            bail!("sparse-index entry identity mismatch");
        }
        let version = semver::Version::parse(&self.vers)
            .with_context(|| format!("invalid version `{}` for `{namespace}/{name}`", self.vers))?;
        let manifest: NodeManifest = serde_json::from_value(self.manifest)
            .with_context(|| format!("invalid manifest for `{namespace}/{name}@{version}`"))?;
        let source = source_spec(&self.source)
            .with_context(|| format!("invalid source for `{namespace}/{name}@{version}`"))?;
        let archive_sha256 = match self.cksum {
            Some(cksum) => {
                if cksum.len() != 64 || !cksum.chars().all(|c| c.is_ascii_hexdigit()) {
                    bail!(
                        "registry returned a malformed archive checksum for \
                         `{namespace}/{name}@{version}`"
                    );
                }
                Some(cksum.to_ascii_lowercase())
            }
            None => None,
        };
        Ok((
            version,
            IndexEntry {
                manifest,
                source,
                published: self.published.or(self.pubtime),
                yanked: self.yanked,
                yank_reason: self.yank_reason,
                archive_sha256,
            },
        ))
    }
}

fn source_spec(source: &Value) -> eyre::Result<SourceSpec> {
    if let Some(git) = source.get("git") {
        let (repo, rev, subdir) = if let Some(object) = git.as_object() {
            (
                object.get("repo").and_then(Value::as_str),
                object.get("rev").and_then(Value::as_str),
                object.get("subdir").and_then(Value::as_str),
            )
        } else {
            (
                git.as_str(),
                source.get("rev").and_then(Value::as_str),
                source.get("subdir").and_then(Value::as_str),
            )
        };
        let (Some(repo), Some(rev)) = (repo, rev) else {
            bail!("git source needs `repo` and `rev`");
        };
        return Ok(SourceSpec {
            git: Some(repo.to_string()),
            rev: Some(rev.to_string()),
            subdir: subdir.map(str::to_string),
            ..SourceSpec::default()
        });
    }

    // Hosted artifacts: map to the existing per-platform binary source so the
    // resolver/lockfile use the same pinned-artifact path.
    let artifact = source.get("artifact").or_else(|| source.get("hosted"));
    let Some(artifact) = artifact else {
        bail!("source has neither `git` nor `artifact`");
    };
    let url = artifact
        .get("url")
        .and_then(Value::as_str)
        .ok_or_else(|| eyre::eyre!("artifact source has no `url`"))?;
    let sha256 = artifact
        .get("sha256")
        .or_else(|| artifact.get("cksum"))
        .and_then(Value::as_str)
        .ok_or_else(|| eyre::eyre!("artifact source has no `sha256`"))?;
    let platforms = artifact
        .get("platforms")
        .or_else(|| source.get("platforms"))
        .and_then(Value::as_array)
        .ok_or_else(|| {
            eyre::eyre!(
                "artifact source has no `platforms` list; refusing to use one binary on every host"
            )
        })?
        .iter()
        .filter_map(Value::as_str)
        .map(str::to_string)
        .collect::<Vec<_>>();
    if platforms.is_empty() {
        bail!("artifact source has an empty `platforms` list");
    }
    let binary = platforms
        .into_iter()
        .map(|platform| BinaryArtifact {
            platform,
            url: url.to_string(),
            sha256: sha256.to_string(),
        })
        .collect();
    Ok(SourceSpec {
        binary,
        ..SourceSpec::default()
    })
}

/// Cargo-style sparse prefix for a package name (directory part only).
fn sparse_prefix(name: &str) -> String {
    let name = name.to_ascii_lowercase();
    match name.len() {
        0 | 1 => "1".to_string(),
        2 => "2".to_string(),
        3 => format!("3/{}", &name[..1]),
        _ => format!("{}/{}", &name[..2], &name[2..4]),
    }
}

fn validate_component(kind: &str, value: &str) -> eyre::Result<()> {
    let valid = !value.is_empty()
        && value.len() <= 128
        && value != "."
        && value != ".."
        && value
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '-' | '_' | '.' | '+'));
    if !valid {
        bail!("invalid {kind} `{value}` in registry reference");
    }
    Ok(())
}

fn default_cache_root(base: &str) -> eyre::Result<PathBuf> {
    Ok(dirs::cache_dir()
        .ok_or_else(|| eyre::eyre!("could not determine the user cache directory"))?
        .join("dora")
        .join("registry")
        .join(cache_key(base)))
}

fn cache_key(base: &str) -> String {
    // Hash the full URL. Sanitizing characters can collide
    // (`https://dora.pub` vs `https://dorap.ub`); an offline cache hit from the
    // wrong host would then feed manifests and git pins across trust domains.
    let digest = sha256_hex(base.as_bytes());
    format!("registry-{}", &digest[..32])
}

fn sanitize_registry_text(text: &str) -> String {
    text.chars()
        .filter(|c| {
            !c.is_control()
                && !matches!(
                    *c,
                    '\u{061c}'
                        | '\u{200b}'..='\u{200f}'
                        | '\u{202a}'..='\u{202e}'
                        | '\u{2060}'..='\u{2069}'
                )
        })
        .take(1000)
        .collect()
}

fn registry_error_detail(body: &str) -> String {
    if let Ok(value) = serde_json::from_str::<Value>(body)
        && let Some(detail) = value["errors"][0]["detail"].as_str()
    {
        return format!(": {}", sanitize_registry_text(detail));
    }
    String::new()
}

fn json_response(response: reqwest::blocking::Response) -> eyre::Result<Value> {
    let status = response.status();
    let bytes = read_limited(response, MAX_JSON_BYTES, "registry JSON response")?;
    let body = String::from_utf8(bytes).context("registry returned non-UTF-8 JSON")?;
    if !status.is_success() {
        bail!("registry returned {status}{}", registry_error_detail(&body));
    }
    serde_json::from_str(&body).with_context(|| {
        format!(
            "invalid registry JSON response: {}",
            sanitize_registry_text(&body)
        )
    })
}

/// Path to the saved publish token for the *official* registry.
///
/// Kept for compatibility with the standalone `dora-hub` client. Use
/// [`token_file_for`] whenever a specific `--registry`/env target matters: a
/// token saved for dora.pub must not be sent to a mirror.
pub fn token_file() -> PathBuf {
    token_file_for(DEFAULT_REGISTRY_URL)
}

/// Path to the token file for `registry_url`.
///
/// The official registry keeps the historical `~/.config/dora/hub-token` path
/// (shared with the server's `dora-hub` client). Other registries get a
/// registry-keyed file so switching `DORA_HUB_REGISTRY_URL` cannot leak an
/// official token to a test mirror. `DORA_HUB_TOKEN_FILE` still overrides all
/// registries explicitly.
pub fn token_file_for(registry_url: &str) -> PathBuf {
    if let Ok(path) = std::env::var(TOKEN_FILE_ENV)
        && !path.trim().is_empty()
    {
        return PathBuf::from(path);
    }
    let base = registry_url.trim().trim_end_matches('/');
    let config_dir = dirs::config_dir().unwrap_or_else(|| PathBuf::from("."));
    if base.is_empty() || base.eq_ignore_ascii_case(DEFAULT_REGISTRY_URL) {
        return config_dir.join("dora").join("hub-token");
    }
    let key = &sha256_hex(base.as_bytes())[..16];
    config_dir.join("dora").join(format!("hub-token-{key}"))
}

pub fn read_saved_token() -> eyre::Result<Option<String>> {
    read_saved_token_for(&registry_url())
}

pub fn read_saved_token_for(registry_url: &str) -> eyre::Result<Option<String>> {
    let path = token_file_for(registry_url);
    match fs::read_to_string(&path) {
        Ok(token) => {
            let token = token.trim().to_string();
            Ok((!token.is_empty()).then_some(token))
        }
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(error) => Err(error).with_context(|| format!("failed to read `{}`", path.display())),
    }
}

/// Read `DORA_HUB_TOKEN`, but only for the official registry.
///
/// The environment is process-global: honoring it for an arbitrary mirror
/// would send the official token to whatever `DORA_HUB_REGISTRY_URL` points at.
pub fn read_env_token_for(registry_url: &str) -> Option<String> {
    let base = registry_url.trim().trim_end_matches('/');
    if !base.eq_ignore_ascii_case(DEFAULT_REGISTRY_URL) {
        return None;
    }
    std::env::var(TOKEN_ENV)
        .ok()
        .map(|token| token.trim().to_string())
        .filter(|token| !token.is_empty())
}

pub fn save_token(token: &str) -> eyre::Result<PathBuf> {
    save_token_for(DEFAULT_REGISTRY_URL, token)
}

pub fn save_token_for(registry_url: &str, token: &str) -> eyre::Result<PathBuf> {
    let path = token_file_for(registry_url);
    write_private_token_file(&path, token)?;
    Ok(path)
}

fn write_private_token_file(path: &Path, token: &str) -> eyre::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)
            .with_context(|| format!("failed to create `{}`", parent.display()))?;
    }
    let contents = format!("{}\n", token.trim());
    #[cfg(unix)]
    {
        use std::os::unix::fs::{OpenOptionsExt, PermissionsExt};

        // Refuse to clobber a symlink: a token write must not be redirectable
        // to an arbitrary file that an attacker can pre-create.
        if let Ok(metadata) = fs::symlink_metadata(path)
            && metadata.file_type().is_symlink()
        {
            eyre::bail!(
                "refusing to write the token through symlink `{}`",
                path.display()
            );
        }
        let mut options = fs::OpenOptions::new();
        options.write(true).create(true).truncate(true).mode(0o600);
        // O_NOFOLLOW closes the check-then-open race on Unix.
        options.custom_flags(libc::O_NOFOLLOW);
        let mut file = options
            .open(path)
            .with_context(|| format!("failed to open `{}`", path.display()))?;
        file.write_all(contents.as_bytes())
            .with_context(|| format!("failed to write `{}`", path.display()))?;
        // The mode above already creates it 0600; this also tightens a
        // pre-existing file written by an older dora.
        fs::set_permissions(path, fs::Permissions::from_mode(0o600))
            .with_context(|| format!("failed to restrict permissions on `{}`", path.display()))?;
    }
    #[cfg(not(unix))]
    {
        fs::write(path, contents)
            .with_context(|| format!("failed to write `{}`", path.display()))?;
    }
    Ok(())
}

pub fn clear_saved_token() -> eyre::Result<()> {
    clear_saved_token_for(&registry_url())
}

pub fn clear_saved_token_for(registry_url: &str) -> eyre::Result<()> {
    let path = token_file_for(registry_url);
    match fs::remove_file(&path) {
        Ok(()) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(error).with_context(|| format!("failed to remove `{}`", path.display())),
    }
}

/// A source materialized from the registry's fallback archive.
#[derive(Debug, Clone)]
pub struct FallbackSource {
    /// `file://` repository cloned by the build.
    pub repo: String,
    /// Commit created from the archive contents.
    pub commit_hash: String,
}

/// Fetch the registry's fallback archive for `hub_name@version` and turn it
/// into a local git repository that the existing GitManager can build from.
///
/// This is the *last resort* after the package's hosting repository failed:
/// the registry archive is the source-of-last-resort, not the primary source.
pub fn registry_fallback_source(
    hub_name: &str,
    version: &str,
    expected_archive_sha256: Option<&str>,
    offline: bool,
) -> eyre::Result<Option<FallbackSource>> {
    if offline {
        eyre::bail!("registry archive fallback is disabled by `--offline`");
    }
    let (namespace, name) = hub_name
        .split_once('/')
        .ok_or_else(|| eyre::eyre!("invalid hub package name `{hub_name}`"))?;
    let client = RegistryClient::new(offline)?;
    client.fallback_source(namespace, name, version, expected_archive_sha256)
}

impl RegistryClient {
    /// Materialize the registry archive as a local git repo (cached per
    /// registry/package/version). Returns `None` when the registry has no
    /// archive for that version.
    ///
    /// The archive is only accepted when its SHA-256 matches the checksum
    /// pinned at resolution time. A missing checksum is a hard error: the
    /// fallback executes build scripts, so "we could not verify it" must not
    /// silently degrade to "trust the network".
    pub fn fallback_source(
        &self,
        namespace: &str,
        name: &str,
        version: &str,
        expected_archive_sha256: Option<&str>,
    ) -> eyre::Result<Option<FallbackSource>> {
        validate_component("namespace", namespace)?;
        validate_component("package name", name)?;
        validate_component("version", version)?;
        let expected = expected_archive_sha256
            .filter(|value| !value.trim().is_empty())
            .ok_or_else(|| {
                eyre::eyre!(
                    "`{namespace}/{name}@{version}` has no pinned archive checksum; \
                     refusing the unverified registry fallback\n  hint: regenerate the \
                     lockfile with `dora build --write-lockfile`"
                )
            })?
            .to_ascii_lowercase();
        if expected.len() != 64 || !expected.chars().all(|c| c.is_ascii_hexdigit()) {
            eyre::bail!("`{namespace}/{name}@{version}` has a malformed pinned archive checksum");
        }

        let fallback_dir = self.cache_root.join("fallback").join(namespace).join(name);
        let repo_dir = fallback_dir.join(version);
        let marker = fallback_dir.join(format!("{version}.sha256"));

        if repo_dir.join(".git").is_dir() {
            let marker_matches = fs::read_to_string(&marker)
                .map(|cached| cached.trim().eq_ignore_ascii_case(&expected))
                .unwrap_or(false);
            if marker_matches {
                let rev = git(&repo_dir, &["rev-parse", "HEAD"])?;
                return Ok(Some(FallbackSource {
                    repo: file_url(&repo_dir)?,
                    commit_hash: rev,
                }));
            }
            // A fallback left by an older build (no marker) or a different
            // archive must never be reused: drop it and re-download.
            fs::remove_dir_all(&repo_dir)
                .with_context(|| format!("failed to clear `{}`", repo_dir.display()))?;
            let _ = fs::remove_file(&marker);
        }

        let url = format!(
            "{}/api/v1/packages/{namespace}/{name}/{version}/download",
            self.base
        );
        let response = self
            .client
            .get(&url)
            .send()
            .with_context(|| format!("failed to reach registry archive `{url}`"))?;
        let status = response.status();
        if status == reqwest::StatusCode::NOT_FOUND {
            return Ok(None);
        }
        if !status.is_success() {
            let detail = read_limited(response, MAX_ERROR_BYTES, "registry error body")
                .map(|bytes| sanitize_registry_text(String::from_utf8_lossy(&bytes).trim()))
                .unwrap_or_default();
            bail!(
                "registry archive returned {status} for `{namespace}/{name}@{version}`{}",
                if detail.is_empty() {
                    String::new()
                } else {
                    format!(": {detail}")
                }
            );
        }
        let bytes = read_limited(response, MAX_ARCHIVE_BYTES, "registry archive")?;
        let actual = sha256_hex(&bytes);
        if actual != expected {
            bail!(
                "registry archive checksum mismatch for `{namespace}/{name}@{version}`: \
                 pinned {expected}, downloaded {actual}"
            );
        }
        let temp = tempfile::TempDir::new().context("failed to create fallback tempdir")?;
        extract_archive(&bytes, temp.path())?;

        if repo_dir.exists() {
            fs::remove_dir_all(&repo_dir)
                .with_context(|| format!("failed to refresh `{}`", repo_dir.display()))?;
        }
        fs::create_dir_all(&repo_dir)
            .with_context(|| format!("failed to create `{}`", repo_dir.display()))?;
        copy_tree(temp.path(), &repo_dir)?;
        git(&repo_dir, &["init", "-q"])?;
        git(&repo_dir, &["config", "user.email", "fallback@dora.local"])?;
        git(
            &repo_dir,
            &["config", "user.name", "Dora registry fallback"],
        )?;
        git(&repo_dir, &["add", "-A"])?;
        git(
            &repo_dir,
            &[
                "commit",
                "-q",
                "-m",
                &format!("registry fallback {namespace}/{name}@{version}"),
            ],
        )?;
        let commit_hash = git(&repo_dir, &["rev-parse", "HEAD"])?;
        fs::write(&marker, format!("{expected}\n"))
            .with_context(|| format!("failed to record `{}`", marker.display()))?;
        Ok(Some(FallbackSource {
            repo: file_url(&repo_dir)?,
            commit_hash,
        }))
    }
}

fn lock_package_cache(
    cache_root: &Path,
    namespace: &str,
    name: &str,
) -> eyre::Result<std::fs::File> {
    let lock_dir = cache_root.join(".locks");
    fs::create_dir_all(&lock_dir)
        .with_context(|| format!("failed to create `{}`", lock_dir.display()))?;
    let path = lock_dir.join(format!("{namespace}--{name}.lock"));
    let file = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .open(&path)
        .with_context(|| format!("failed to open cache lock `{}`", path.display()))?;
    file.lock()
        .with_context(|| format!("failed to lock `{}`", path.display()))?;
    Ok(file)
}

fn sha256_hex(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    hasher
        .finalize()
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

fn read_limited(
    mut response: reqwest::blocking::Response,
    limit: u64,
    what: &str,
) -> eyre::Result<Vec<u8>> {
    if let Some(length) = response.content_length()
        && length > limit
    {
        bail!("{what} is {length} bytes, above the {limit}-byte limit");
    }
    let mut bytes = Vec::new();
    response
        .by_ref()
        .take(limit.saturating_add(1))
        .read_to_end(&mut bytes)
        .with_context(|| format!("failed to read {what}"))?;
    if bytes.len() as u64 > limit {
        bail!("{what} exceeds the {limit}-byte limit");
    }
    Ok(bytes)
}

fn file_url(path: &Path) -> eyre::Result<String> {
    let canonical = path
        .canonicalize()
        .with_context(|| format!("failed to canonicalize `{}`", path.display()))?;
    // `format!("file://{}", ...)` breaks on Windows, where canonicalize
    // returns a `\\?\C:\...` path. Let the URL crate build a valid file URL.
    Url::from_file_path(&canonical)
        .map(|url| url.to_string())
        .map_err(|_| eyre::eyre!("failed to build a file URL for `{}`", canonical.display()))
}

fn git(dir: &Path, args: &[&str]) -> eyre::Result<String> {
    let output = git_command(Some(dir))
        .args(args)
        .output()
        .context("failed to run git for registry fallback")?;
    if !output.status.success() {
        bail!(
            "git {} failed: {}",
            args.join(" "),
            String::from_utf8_lossy(&output.stderr).trim()
        );
    }
    Ok(String::from_utf8_lossy(&output.stdout).trim().to_string())
}

fn extract_archive(archive: &[u8], dest: &Path) -> eyre::Result<()> {
    let decoder = flate2::read::GzDecoder::new(std::io::Cursor::new(archive));
    let mut archive = tar::Archive::new(decoder);
    let mut total = 0u64;
    let mut entry_count = 0usize;
    for entry in archive.entries().context("invalid registry archive")? {
        entry_count += 1;
        if entry_count > MAX_ARCHIVE_ENTRIES {
            bail!("registry archive has more than {MAX_ARCHIVE_ENTRIES} entries");
        }
        let mut entry = entry.context("invalid registry archive entry")?;
        let entry_type = entry.header().entry_type();
        // tar metadata records (PAX/GNU long names) are not files to extract;
        // the tar crate already resolves their payloads for the real entries.
        if entry_type.is_pax_global_extensions()
            || entry_type.is_pax_local_extensions()
            || entry_type.is_gnu_longname()
            || entry_type.is_gnu_longlink()
        {
            continue;
        }
        if entry_type.is_symlink()
            || entry_type.is_hard_link()
            || entry_type.is_fifo()
            || entry_type.is_character_special()
            || entry_type.is_block_special()
            || entry_type.is_gnu_sparse()
        {
            bail!("registry archive contains an unsupported entry type");
        }
        if !(entry_type.is_file() || entry_type.is_dir()) {
            bail!("registry archive contains a non-file, non-directory entry");
        }
        let path = entry
            .path()
            .context("invalid registry archive path")?
            .into_owned();
        let relative = path.as_path();
        // Accept only relative normal/current-dir components. On Windows a
        // rooted path without a drive prefix (e.g. `/Users/...`) is not
        // `is_absolute()`, and joining it can escape the extraction root.
        if relative.components().any(|component| {
            !matches!(
                component,
                std::path::Component::Normal(_) | std::path::Component::CurDir
            )
        }) {
            bail!("registry archive path escapes its root");
        }
        total = total.saturating_add(entry.header().size().unwrap_or(0));
        if total > 200 * 1024 * 1024 {
            bail!("registry archive expands to more than 200 MiB");
        }
        let target = dest.join(relative);
        if entry.header().entry_type().is_dir() {
            fs::create_dir_all(&target)?;
            continue;
        }
        if let Some(parent) = target.parent() {
            fs::create_dir_all(parent)?;
        }
        let mut out = fs::File::create(&target)?;
        std::io::copy(&mut entry, &mut out)?;
    }
    Ok(())
}

fn copy_tree(src: &Path, dst: &Path) -> eyre::Result<()> {
    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let name = entry.file_name();
        // The checksum covers the whole archive; do not silently drop
        // directories (e.g. a tracked `src/target/`) after verification.
        // `.git` is the one exception: a fallback checkout is created as a
        // fresh repo and must not inherit archive metadata.
        if name == ".git" {
            continue;
        }
        let from = entry.path();
        let to = dst.join(&name);
        if from.is_dir() {
            fs::create_dir_all(&to)?;
            copy_tree(&from, &to)?;
        } else if from.is_file() {
            fs::copy(&from, &to)?;
        }
    }
    Ok(())
}

/// Cheap preflight used before building a hub node: is the hosting source
/// reachable? A broken source triggers the registry archive fallback before
/// the build pipeline tries (and fails) to clone it.
pub fn source_available(repo: &str, commit: &str) -> bool {
    if repo.contains(char::is_control) {
        return false;
    }
    if repo.starts_with("file://") || repo.starts_with('/') {
        let path = repo.strip_prefix("file://").unwrap_or(repo);
        let mut command = git_command(Some(Path::new(path)));
        command.args(["cat-file", "-e", &format!("{commit}^{{commit}}")]);
        return git_output_with_timeout(command, Duration::from_secs(5))
            .is_some_and(|output| output.status.success());
    }
    let mut command = git_command(None);
    // HEAD alone is not enough: a publisher can delete or force-push the
    // pinned commit while HEAD remains reachable. Enumerate refs and require
    // the exact object id; a non-ref pinned commit falls back conservatively.
    command.args(["ls-remote", "--quiet", "--", repo]);
    // A registry-supplied source URL must not be able to hang the build. The
    // child is killed on timeout; `spawn_blocking` cannot cancel a stuck
    // `Command::output()` otherwise.
    git_output_with_timeout(command, Duration::from_secs(15)).is_some_and(|output| {
        output.status.success()
            && String::from_utf8_lossy(&output.stdout)
                .lines()
                .any(|line| line.split_whitespace().next() == Some(commit))
    })
}

fn git_output_with_timeout(
    mut command: std::process::Command,
    timeout: Duration,
) -> Option<std::process::Output> {
    command.stdout(std::process::Stdio::piped());
    command.stderr(std::process::Stdio::null());
    let mut child = command.spawn().ok()?;
    let deadline = Instant::now() + timeout;
    loop {
        match child.try_wait() {
            Ok(Some(_)) => return child.wait_with_output().ok(),
            Ok(None) => {}
            Err(_) => return None,
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            let _ = child.wait();
            return None;
        }
        std::thread::sleep(Duration::from_millis(25));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn default_url_is_official() {
        assert_eq!(registry_url_from(None), DEFAULT_REGISTRY_URL);
        assert_eq!(registry_url_from(Some("  ")), DEFAULT_REGISTRY_URL);
        assert_eq!(
            registry_url_from(Some("http://127.0.0.1:8080/")),
            "http://127.0.0.1:8080"
        );
    }

    #[test]
    fn validates_https_and_loopback_http() {
        assert!(validate_registry_url("https://dora.pub").is_ok());
        assert!(validate_registry_url("http://127.0.0.1:8080").is_ok());
        assert!(validate_registry_url("http://localhost:8080").is_ok());
        assert!(validate_registry_url("http://example.com").is_err());
        assert!(validate_registry_url("ftp://dora.pub").is_err());
    }

    #[test]
    fn sparse_prefix_matches_cargo_layout() {
        assert_eq!(sparse_prefix("a"), "1");
        assert_eq!(sparse_prefix("ab"), "2");
        assert_eq!(sparse_prefix("abc"), "3/a");
        assert_eq!(sparse_prefix("hello-world"), "he/ll");
    }

    #[test]
    fn parses_git_source_objects() {
        let source = json!({"kind":"git","git":{"repo":"https://example.com/repo","rev":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"}});
        let spec = source_spec(&source).unwrap();
        assert_eq!(spec.git.as_deref(), Some("https://example.com/repo"));
        assert_eq!(
            spec.rev.as_deref(),
            Some("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        );
    }

    #[test]
    fn parses_hosted_artifacts() {
        let source = json!({
            "kind": "hosted",
            "artifact": {
                "url": "https://dora.pub/api/v1/packages/acme/hello/1.0.0/download",
                "sha256": "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
                "platforms": ["linux-x86_64"]
            }
        });
        let spec = source_spec(&source).unwrap();
        assert_eq!(spec.binary.len(), 1);
        assert_eq!(spec.binary[0].platform, "linux-x86_64");
    }

    #[test]
    fn fetches_and_materializes_a_sparse_index() {
        use std::{
            io::{Read, Write},
            net::TcpListener,
            thread,
        };

        let listener = TcpListener::bind("127.0.0.1:0").expect("bind test server");
        let address = listener.local_addr().unwrap();
        let server = thread::spawn(move || {
            let (mut stream, _) = listener.accept().expect("accept request");
            let mut request = [0u8; 4096];
            let _ = stream.read(&mut request);
            let entry = json!({
                "name": "hello-world",
                "namespace": "acme",
                "vers": "0.1.0",
                "manifest": {
                    "apiVersion": 1,
                    "name": "hello-world",
                    "namespace": "acme",
                    "runtime": "python",
                    "entrypoint": "hello.py"
                },
                "source": {
                    "kind": "git",
                    "git": {
                        "repo": "https://example.com/acme/hello-world",
                        "rev": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
                    }
                },
                "yanked": false
            });
            let body = format!("{entry}\n");
            let response = format!(
                "HTTP/1.1 200 OK\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
                body.len()
            );
            stream.write_all(response.as_bytes()).unwrap();
        });

        let cache = tempfile::tempdir().unwrap();
        let mut client = RegistryClient::with_base(
            &format!("http://{address}"),
            cache.path().to_path_buf(),
            false,
        )
        .unwrap();
        let dir = client.catalog_dir("acme", "hello-world").unwrap();
        let catalog = crate::index::IndexCatalog::open(dir).unwrap();
        let reference = crate::reference::PackageRef::parse("acme/hello-world@^0.1").unwrap();
        let resolved = catalog.resolve(&reference).unwrap();
        assert_eq!(resolved.version.to_string(), "0.1.0");
        assert_eq!(resolved.entry.manifest.entrypoint, "hello.py".to_string());
        server.join().unwrap();
    }

    #[test]
    fn checks_local_source_availability() {
        let dir = tempfile::tempdir().unwrap();
        let run = |args: &[&str]| {
            assert!(
                git_command(Some(dir.path()))
                    .args(args)
                    .status()
                    .unwrap()
                    .success()
            );
        };
        run(&["init", "-q"]);
        run(&["config", "user.email", "t@example.com"]);
        run(&["config", "user.name", "t"]);
        std::fs::write(dir.path().join("hello.py"), "print('hi')\n").unwrap();
        run(&["add", "-A"]);
        run(&["commit", "-q", "-m", "init"]);
        let rev = String::from_utf8(
            git_command(Some(dir.path()))
                .args(["rev-parse", "HEAD"])
                .output()
                .unwrap()
                .stdout,
        )
        .unwrap()
        .trim()
        .to_string();
        let url = format!("file://{}", dir.path().display());
        assert!(source_available(&url, &rev));
        assert!(!source_available(&format!("{url}-missing"), &rev));
        assert!(!source_available(&url, &"0".repeat(40)));
    }

    fn test_archive() -> Vec<u8> {
        use std::io::Write;

        let body = b"print('hi')\n";
        let mut builder = tar::Builder::new(Vec::new());
        let mut header = tar::Header::new_gnu();
        header.set_path("hello.py").unwrap();
        header.set_size(body.len() as u64);
        header.set_mode(0o644);
        header.set_cksum();
        builder.append(&header, &body[..]).unwrap();
        let tar = builder.into_inner().unwrap();
        let mut gz = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::default());
        gz.write_all(&tar).unwrap();
        gz.finish().unwrap()
    }

    fn serve_archive_once(body: Vec<u8>) -> (String, std::thread::JoinHandle<()>) {
        use std::{
            io::{Read, Write},
            net::TcpListener,
            thread,
        };

        let listener = TcpListener::bind("127.0.0.1:0").expect("bind test server");
        let address = listener.local_addr().unwrap();
        let server = thread::spawn(move || {
            let (mut stream, _) = listener.accept().expect("accept request");
            let mut request = [0u8; 4096];
            let _ = stream.read(&mut request);
            let response = format!(
                "HTTP/1.1 200 OK\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
                body.len()
            );
            stream.write_all(response.as_bytes()).unwrap();
            stream.write_all(&body).unwrap();
        });
        (format!("http://{address}"), server)
    }

    #[test]
    fn extraction_rejects_symlink_entries() {
        use std::io::Write;

        let mut builder = tar::Builder::new(Vec::new());
        let mut header = tar::Header::new_gnu();
        header.set_entry_type(tar::EntryType::Symlink);
        header.set_path("escape").unwrap();
        header.set_link_name("/etc/passwd").unwrap();
        header.set_size(0);
        header.set_mode(0o777);
        header.set_cksum();
        builder.append(&header, &[][..]).unwrap();
        let tar = builder.into_inner().unwrap();
        let mut gz = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::default());
        gz.write_all(&tar).unwrap();
        let archive = gz.finish().unwrap();

        let dest = tempfile::tempdir().unwrap();
        let err = extract_archive(&archive, dest.path()).unwrap_err();
        assert!(
            format!("{err:#}").contains("unsupported entry type"),
            "unexpected error: {err:#}"
        );
        assert!(!dest.path().join("escape").exists());
    }

    #[test]
    fn offline_registry_search_refuses_network() {
        let cache = tempfile::tempdir().unwrap();
        let client =
            RegistryClient::with_base("http://127.0.0.1:1", cache.path().to_path_buf(), true)
                .unwrap();
        let err = client.search("hello").unwrap_err();
        assert!(
            format!("{err:#}").contains("requires network access"),
            "unexpected error: {err:#}"
        );
    }

    #[test]
    fn fallback_verifies_archive_checksum_and_reuses_cache() {
        let archive = test_archive();
        let expected = sha256_hex(&archive);
        let (base, server) = serve_archive_once(archive);
        let cache = tempfile::tempdir().unwrap();
        let client = RegistryClient::with_base(&base, cache.path().to_path_buf(), false).unwrap();

        let first = client
            .fallback_source("acme", "hello", "0.1.0", Some(&expected))
            .unwrap()
            .expect("fallback archive");
        // the second call must use the checksum marker instead of the network;
        // the test server only accepts one request.
        let second = client
            .fallback_source("acme", "hello", "0.1.0", Some(&expected))
            .unwrap()
            .expect("cached fallback");
        assert_eq!(first.commit_hash, second.commit_hash);
        server.join().unwrap();
    }

    #[test]
    fn fallback_rejects_archive_checksum_mismatch() {
        let archive = test_archive();
        let (base, server) = serve_archive_once(archive);
        let cache = tempfile::tempdir().unwrap();
        let client = RegistryClient::with_base(&base, cache.path().to_path_buf(), false).unwrap();
        let err = client
            .fallback_source("acme", "hello", "0.1.0", Some(&"0".repeat(64)))
            .unwrap_err();
        assert!(
            format!("{err:#}").contains("checksum mismatch"),
            "unexpected error: {err:#}"
        );
        server.join().unwrap();
    }

    #[test]
    fn fallback_refuses_a_pin_without_checksum() {
        let cache = tempfile::tempdir().unwrap();
        let client =
            RegistryClient::with_base("http://127.0.0.1:1", cache.path().to_path_buf(), false)
                .unwrap();
        let err = client
            .fallback_source("acme", "hello", "0.1.0", None)
            .unwrap_err();
        assert!(
            format!("{err:#}").contains("no pinned archive checksum"),
            "unexpected error: {err:#}"
        );
    }

    #[test]
    fn rejects_hosted_artifacts_without_platforms() {
        let source = json!({
            "kind": "hosted",
            "artifact": {
                "url": "https://example.com/node",
                "sha256": "a".repeat(64)
            }
        });
        let err = source_spec(&source).unwrap_err();
        assert!(
            format!("{err:#}").contains("no `platforms` list"),
            "unexpected error: {err:#}"
        );
    }

    #[test]
    fn cache_keys_do_not_collide_on_punctuation() {
        assert_ne!(cache_key("https://dora.pub"), cache_key("https://dorap.ub"));
    }

    #[test]
    fn rejects_unsafe_components() {
        assert!(validate_component("name", "../etc").is_err());
        assert!(validate_component("name", "ok-name").is_ok());
    }
}
