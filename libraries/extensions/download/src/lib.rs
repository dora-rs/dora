use eyre::{Context, ContextCompat};
use sha2::{Digest, Sha256};
#[cfg(unix)]
use std::os::unix::prelude::PermissionsExt;
use std::path::{Path, PathBuf};
use tokio::io::AsyncWriteExt;

/// Derive a fallback filename from a URL's last path segment.
///
/// Uses the parsed path segments rather than the raw URL string: a query
/// string is not a path separator, so running `Path::file_name` over the full
/// URL would fold it into the name (e.g. a presigned/CDN URL like
/// `https://host/model.bin?X-Amz-Signature=...` would yield
/// `model.bin?X-Amz-Signature=...`). Returns `None` for URLs whose last
/// segment is empty (e.g. a trailing slash).
fn filename_from_url(url: &reqwest::Url) -> Option<String> {
    url.path_segments()
        .and_then(|mut segments| segments.next_back())
        .filter(|segment| !segment.is_empty())
        .map(|segment| segment.to_string())
}

fn get_filename(response: &reqwest::Response) -> Option<String> {
    let raw_name = if let Some(content_disposition) = response.headers().get("content-disposition")
    {
        if let Ok(filename) = content_disposition.to_str() {
            filename
                .split("filename=")
                .nth(1)
                .map(|n| n.trim_matches('"').to_string())
        } else {
            None
        }
    } else {
        None
    };

    // If Content-Disposition header is not available, extract from the URL path.
    let raw_name = raw_name.or_else(|| filename_from_url(response.url()));

    // Sanitize: strip path components to prevent traversal,
    // reject null bytes and overly long names
    raw_name.and_then(|name| {
        let sanitized = Path::new(&name)
            .file_name()
            .and_then(|n| n.to_str())
            .map(|s| s.to_string())?;
        if sanitized.contains('\0') || sanitized.len() > 255 {
            return None;
        }
        Some(sanitized)
    })
}

/// Download a file from a URL into `target_dir`.
///
/// If `expected_sha256` is provided, the downloaded content is verified
/// against the hex-encoded SHA-256 digest. Returns an error if the digest
/// does not match (prevents MITM or corrupted downloads).
pub async fn download_file<T>(
    url: T,
    target_dir: &Path,
    expected_sha256: Option<&str>,
) -> Result<PathBuf, eyre::ErrReport>
where
    T: reqwest::IntoUrl + std::fmt::Display + Copy,
{
    tokio::fs::create_dir_all(&target_dir)
        .await
        .wrap_err("failed to create parent folder")?;

    let response = reqwest::get(url)
        .await
        .wrap_err_with(|| format!("failed to request operator from `{url}`"))?;

    let filename = get_filename(&response).context("Could not find a filename")?;
    let bytes = response
        .bytes()
        .await
        .wrap_err_with(|| format!("failed to download from `{url}`"))?;

    // Verify integrity if a digest was provided.
    // Without a digest, the download is vulnerable to MITM or CDN compromise
    // since the downloaded binary may be executed or dlopen-ed.
    let mut hasher = Sha256::new();
    hasher.update(&bytes);
    let actual_hash = format!("{:x}", hasher.finalize());
    if let Some(expected) = expected_sha256 {
        // `actual_hash` is lowercase hex; a digest from an index/lockfile may be
        // uppercase, so compare case-insensitively rather than rejecting it.
        if !expected.eq_ignore_ascii_case(&actual_hash) {
            eyre::bail!("SHA-256 mismatch for `{url}`: expected {expected}, got {actual_hash}");
        }
    } else {
        tracing::warn!(
            url = %url,
            sha256 = %actual_hash,
            "downloading without integrity verification — \
             consider adding sha256 to the source definition"
        );
    }

    let path = target_dir.join(filename);
    let mut file = tokio::fs::File::create(&path)
        .await
        .wrap_err("failed to create target file")?;
    file.write_all(&bytes)
        .await
        .wrap_err("failed to write downloaded operator to file")?;
    file.sync_all().await.wrap_err("failed to `sync_all`")?;

    #[cfg(unix)]
    file.set_permissions(std::fs::Permissions::from_mode(0o700))
        .await
        .wrap_err("failed to make downloaded file executable")?;

    Ok(path.to_path_buf())
}

#[cfg(test)]
mod tests {
    use super::filename_from_url;
    use reqwest::Url;

    fn name(url: &str) -> Option<String> {
        filename_from_url(&Url::parse(url).unwrap())
    }

    #[test]
    fn plain_url() {
        assert_eq!(name("https://host/model.bin").as_deref(), Some("model.bin"));
    }

    #[test]
    fn query_string_is_not_part_of_filename() {
        // Regression: presigned/CDN URLs carry a query string that must not
        // leak into the saved filename.
        assert_eq!(
            name("https://host/path/model.bin?X-Amz-Signature=abc123&x=1").as_deref(),
            Some("model.bin"),
        );
    }

    #[test]
    fn trailing_slash_yields_none() {
        assert_eq!(name("https://host/path/"), None);
    }

    #[test]
    fn fragment_is_not_part_of_filename() {
        assert_eq!(
            name("https://host/model.bin#section").as_deref(),
            Some("model.bin"),
        );
    }
}
