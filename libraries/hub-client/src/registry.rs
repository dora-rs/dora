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

use std::{collections::BTreeSet, fs, path::PathBuf, time::Duration};

use dora_core::manifest::NodeManifest;
use eyre::{Context, bail};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use url::Url;

use crate::index::{BinaryArtifact, IndexEntry, SourceSpec, current_platform};

/// Official registry. The website is not launched yet; this is the default
/// that `dora build/run` and `dora hub publish` use when no override is set.
pub const DEFAULT_REGISTRY_URL: &str = "https://dora.pub";
/// Overrides the registry for local tests, mirrors and preview deployments.
pub const REGISTRY_URL_ENV: &str = "DORA_HUB_REGISTRY_URL";
/// Explicit opt-in for cleartext HTTP to a non-loopback registry (LAN mirror).
pub const ALLOW_INSECURE_REGISTRY_ENV: &str = "DORA_HUB_ALLOW_INSECURE_REGISTRY";
/// Token file override (shared with the `dora-hub` client in hub-registry).
pub const TOKEN_FILE_ENV: &str = "DORA_HUB_TOKEN_FILE";

const REQUEST_TIMEOUT: Duration = Duration::from_secs(30);

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
        let body = response
            .text()
            .with_context(|| format!("failed to read registry response from {url}"))?;
        if status == reqwest::StatusCode::NOT_FOUND {
            bail!("package `{namespace}/{name}` was not found in the registry");
        }
        if !status.is_success() {
            bail!(
                "registry returned {status} for `{namespace}/{name}`{}",
                registry_error_detail(&body)
            );
        }
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

        let package_dir = self.cache_root.join("catalog").join(namespace).join(name);
        if package_dir.exists() {
            fs::remove_dir_all(&package_dir)
                .with_context(|| format!("failed to refresh `{}`", package_dir.display()))?;
        }
        fs::create_dir_all(&package_dir)
            .with_context(|| format!("failed to create `{}`", package_dir.display()))?;
        for (version, entry) in entries {
            let path = package_dir.join(format!("{version}.yml"));
            let yaml = serde_yaml::to_string(&entry)
                .with_context(|| format!("failed to serialize registry entry `{version}`"))?;
            fs::write(&path, yaml)
                .with_context(|| format!("failed to cache `{}`", path.display()))?;
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

    /// Sign in and return the browser session cookie needed to mint a token.
    pub fn login(&self, login: &str, password: &str) -> eyre::Result<RegistrySession> {
        let response = self
            .client
            .post(format!("{}/api/v1/auth/login", self.base))
            .json(&json!({ "login": login, "password": password }))
            .send()
            .with_context(|| format!("failed to reach registry {}", self.base))?;
        let status = response.status();
        let cookie = session_cookie(&response);
        let body = response.text().unwrap_or_default();
        if !status.is_success() {
            bail!(
                "registry login failed ({status}){}",
                registry_error_detail(&body)
            );
        }
        let cookie = cookie
            .ok_or_else(|| eyre::eyre!("registry login succeeded without a session cookie"))?;
        Ok(RegistrySession { cookie })
    }

    /// Create a scoped API token using a browser session.
    pub fn create_token(
        &self,
        session: &RegistrySession,
        name: &str,
        scopes: &[String],
    ) -> eyre::Result<String> {
        let response = self
            .client
            .post(format!("{}/api/v1/me/tokens", self.base))
            .header(
                reqwest::header::COOKIE,
                format!("dora_hub_session={}", session.cookie),
            )
            .json(&json!({ "name": name, "scopes": scopes }))
            .send()
            .with_context(|| format!("failed to reach registry {}", self.base))?;
        let value = json_response(response)?;
        value["token"]
            .as_str()
            .map(str::to_string)
            .ok_or_else(|| eyre::eyre!("registry did not return a token"))
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

/// A browser session cookie returned by `/api/v1/auth/login`.
pub struct RegistrySession {
    cookie: String,
}

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
        Ok((
            version,
            IndexEntry {
                manifest,
                source,
                published: self.published.or(self.pubtime),
                yanked: self.yanked,
                yank_reason: self.yank_reason,
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
        .map(|values| {
            values
                .iter()
                .filter_map(Value::as_str)
                .map(str::to_string)
                .collect::<Vec<_>>()
        })
        .unwrap_or_else(|| vec![current_platform()]);
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
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '-' | '_' | '.'));
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
    let mut key = String::new();
    for ch in base.chars() {
        if ch.is_ascii_alphanumeric() || matches!(ch, '-' | '_') {
            key.push(ch);
        } else if ch == ':' || ch == '/' {
            key.push('_');
        }
    }
    if key.is_empty() {
        "registry".to_string()
    } else {
        key
    }
}

fn session_cookie(response: &reqwest::blocking::Response) -> Option<String> {
    response
        .headers()
        .get_all(reqwest::header::SET_COOKIE)
        .iter()
        .filter_map(|value| value.to_str().ok())
        .find_map(|value| value.split(';').next())
        .and_then(|pair| pair.strip_prefix("dora_hub_session="))
        .map(str::to_string)
}

fn registry_error_detail(body: &str) -> String {
    if let Ok(value) = serde_json::from_str::<Value>(body)
        && let Some(detail) = value["errors"][0]["detail"].as_str()
    {
        return format!(": {detail}");
    }
    String::new()
}

fn json_response(response: reqwest::blocking::Response) -> eyre::Result<Value> {
    let status = response.status();
    let body = response
        .text()
        .context("failed to read registry response")?;
    if !status.is_success() {
        bail!("registry returned {status}{}", registry_error_detail(&body));
    }
    serde_json::from_str(&body).with_context(|| format!("invalid registry JSON response: {body}"))
}

/// Path to the saved publish token (shared with the Hub website's CLI docs).
pub fn token_file() -> PathBuf {
    if let Ok(path) = std::env::var(TOKEN_FILE_ENV)
        && !path.trim().is_empty()
    {
        return PathBuf::from(path);
    }
    dirs::config_dir()
        .unwrap_or_else(|| PathBuf::from("."))
        .join("dora")
        .join("hub-token")
}

pub fn read_saved_token() -> eyre::Result<Option<String>> {
    let path = token_file();
    match fs::read_to_string(&path) {
        Ok(token) => {
            let token = token.trim().to_string();
            Ok((!token.is_empty()).then_some(token))
        }
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(error) => Err(error).with_context(|| format!("failed to read `{}`", path.display())),
    }
}

pub fn save_token(token: &str) -> eyre::Result<PathBuf> {
    let path = token_file();
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)
            .with_context(|| format!("failed to create `{}`", parent.display()))?;
    }
    fs::write(&path, format!("{}\n", token.trim()))
        .with_context(|| format!("failed to write `{}`", path.display()))?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        fs::set_permissions(&path, fs::Permissions::from_mode(0o600))
            .with_context(|| format!("failed to restrict permissions on `{}`", path.display()))?;
    }
    Ok(path)
}

pub fn clear_saved_token() -> eyre::Result<()> {
    let path = token_file();
    match fs::remove_file(&path) {
        Ok(()) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(error).with_context(|| format!("failed to remove `{}`", path.display())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn rejects_unsafe_components() {
        assert!(validate_component("name", "../etc").is_err());
        assert!(validate_component("name", "ok-name").is_ok());
    }
}
