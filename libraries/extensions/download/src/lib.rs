use eyre::{Context, ContextCompat};
use sha2::{Digest, Sha256};
#[cfg(unix)]
use std::os::unix::prelude::PermissionsExt;
use std::path::{Path, PathBuf};
use tokio::io::AsyncWriteExt;

/// Extract the `filename` parameter from a `Content-Disposition` header value.
///
/// Handles headers that carry additional parameters after the filename, e.g.
/// `attachment; filename="model.bin"; size=1000`, and both quoted and unquoted
/// forms. Per RFC 6266 the parameter name is matched case-insensitively, so
/// `Filename=`/`FILENAME=` are accepted too. Returns `None` when no non-empty
/// `filename=` value is present (the RFC 5987 extended `filename*=` form is not
/// decoded and falls through — the literal `filename=` substring never matches
/// the `filename*=` spelling, so it is skipped automatically).
fn parse_content_disposition_filename(header: &str) -> Option<String> {
    // RFC 6266: parameter names are case-insensitive. Locate `filename=`
    // regardless of case without copying the value bytes.
    let lower = header.to_ascii_lowercase();
    let value_start = lower.find("filename=")? + "filename=".len();
    let rest = header[value_start..].trim_start();

    let name = if let Some(after_quote) = rest.strip_prefix('"') {
        // Quoted form: read up to the closing quote, honoring RFC 6266
        // `\`-escapes (`filename="a\"b"` -> `a"b`), so a trailing
        // `; param=...` (or a `;`/`"` inside the quotes) is handled correctly.
        let mut name = String::new();
        let mut chars = after_quote.chars();
        while let Some(c) = chars.next() {
            match c {
                '"' => break,
                '\\' => name.extend(chars.next()),
                _ => name.push(c),
            }
        }
        name
    } else {
        // Token form: the value ends at the next parameter separator.
        rest.split(';').next().unwrap_or(rest).trim().to_string()
    };
    (!name.is_empty()).then_some(name)
}

fn get_filename(response: &reqwest::Response) -> Option<String> {
    let raw_name = response
        .headers()
        .get("content-disposition")
        .and_then(|value| value.to_str().ok())
        .and_then(parse_content_disposition_filename);

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
    let actual_hash: String = hasher
        .finalize()
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
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
    use super::parse_content_disposition_filename;

    #[test]
    fn quoted_filename_without_extra_params() {
        assert_eq!(
            parse_content_disposition_filename("attachment; filename=\"model.bin\""),
            Some("model.bin".to_string())
        );
    }

    #[test]
    fn quoted_filename_with_trailing_params() {
        // Regression: the trailing `; size=1000` must not leak into the name.
        assert_eq!(
            parse_content_disposition_filename("attachment; filename=\"model.bin\"; size=1000"),
            Some("model.bin".to_string())
        );
    }

    #[test]
    fn unquoted_filename_with_trailing_params() {
        assert_eq!(
            parse_content_disposition_filename("attachment; filename=model.bin; size=1000"),
            Some("model.bin".to_string())
        );
    }

    #[test]
    fn semicolon_inside_quotes_is_preserved() {
        assert_eq!(
            parse_content_disposition_filename("attachment; filename=\"a;b.bin\""),
            Some("a;b.bin".to_string())
        );
    }

    #[test]
    fn parameter_name_is_case_insensitive() {
        // RFC 6266 parameter names are case-insensitive; a non-lowercase
        // `filename` (common from Java/.NET/IIS stacks) must still be honored.
        assert_eq!(
            parse_content_disposition_filename("attachment; Filename=\"model.bin\""),
            Some("model.bin".to_string())
        );
        assert_eq!(
            parse_content_disposition_filename("attachment; FILENAME=model.bin"),
            Some("model.bin".to_string())
        );
        assert_eq!(
            parse_content_disposition_filename("inline; FileName=node.py"),
            Some("node.py".to_string())
        );
    }

    #[test]
    fn escaped_quote_inside_quoted_value() {
        // RFC 6266 quoted-string: a `\"` is a literal quote, not the terminator.
        assert_eq!(
            parse_content_disposition_filename("attachment; filename=\"a\\\"b.bin\""),
            Some("a\"b.bin".to_string())
        );
    }

    #[test]
    fn empty_or_missing_filename_returns_none() {
        assert_eq!(
            parse_content_disposition_filename("attachment; filename=\"\""),
            None
        );
        assert_eq!(parse_content_disposition_filename("inline"), None);
        // RFC 5987 extended form is not decoded here; it falls through to None.
        assert_eq!(
            parse_content_disposition_filename("attachment; filename*=UTF-8''model.bin"),
            None
        );
    }
}
