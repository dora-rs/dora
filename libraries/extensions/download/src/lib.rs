use eyre::{Context, ContextCompat};
use sha2::{Digest, Sha256};
#[cfg(unix)]
use std::os::unix::prelude::PermissionsExt;
use std::path::{Path, PathBuf};
use tokio::io::AsyncWriteExt;

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

    // If Content-Disposition header is not available, extract from URL
    let raw_name = raw_name.or_else(|| {
        Path::new(response.url().as_str())
            .file_name()
            .and_then(|n| n.to_str())
            .map(|s| s.to_string())
    });

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

    // `reqwest::get` resolves to `Ok` for any completed HTTP exchange, including
    // 4xx/5xx responses. Without this check a 404/500 error page (HTML/JSON)
    // would be hashed, written to the build directory, and — on Unix — marked
    // executable, then handed to `libloading`/Python, masking the real "bad URL
    // / server error" behind a confusing downstream load failure.
    let response = response
        .error_for_status()
        .wrap_err_with(|| format!("server returned an error status for `{url}`"))?;

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
    use super::*;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    use tokio::net::TcpListener;

    /// A 404 (or any error status) must fail the download instead of writing
    /// the server's error page to disk as the artifact.
    #[tokio::test]
    async fn rejects_http_error_status() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();

        let server = tokio::spawn(async move {
            if let Ok((mut socket, _)) = listener.accept().await {
                // Consume the request line/headers, then reply with a 404.
                let mut buf = [0u8; 1024];
                let _ = socket.read(&mut buf).await;
                let body = "not found";
                let resp = format!(
                    "HTTP/1.1 404 Not Found\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
                    body.len(),
                );
                let _ = socket.write_all(resp.as_bytes()).await;
                let _ = socket.flush().await;
            }
        });

        let dir = std::env::temp_dir().join(format!("dora-download-test-{}", addr.port()));
        let url = format!("http://{addr}/model.bin");
        let result = download_file(url.as_str(), &dir, None).await;

        assert!(result.is_err(), "expected an error for a 404 response");
        assert!(
            !dir.join("model.bin").exists(),
            "the error page must not be written as the artifact"
        );

        let _ = server.await;
        let _ = tokio::fs::remove_dir_all(&dir).await;
    }
}
