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
/// forms. Per RFC 6266 / RFC 2616 the parameter name is case-insensitive, so
/// `Filename`, `FILENAME`, etc. are matched too. Returns `None` when no
/// non-empty `filename` value is present.
///
/// The `filename` parameter is matched on a real parameter boundary — the
/// name before its `=`, compared case-insensitively to exactly `filename` —
/// not by a substring search. That distinguishes it from the RFC 5987 extended
/// `filename*=` form (which this does not decode) and from an unrelated longer
/// token such as `xfilename=` that merely ends in `filename`.
fn parse_content_disposition_filename(header: &str) -> Option<String> {
    split_disposition_params(header).find_map(|param| {
        let (name, value) = param.split_once('=')?;
        // RFC 6266: parameter names are case-insensitive. `filename*` (extended
        // form) and tokens like `xfilename` do not compare equal to `filename`.
        if !name.trim().eq_ignore_ascii_case("filename") {
            return None;
        }
        let value = value.trim();
        let name = if let Some(after_quote) = value.strip_prefix('"') {
            // Quoted form: the value runs up to the first quote that is not a
            // `\"` escape.
            unquote_disposition_value(after_quote)
        } else {
            value.to_string()
        };
        (!name.is_empty()).then_some(name)
    })
}

/// Decode a `Content-Disposition` quoted-string body (the text after the
/// opening `"`): return everything up to the first *unescaped* closing quote,
/// with a `\"` escape collapsed to a literal `"`. Without honoring the escape,
/// a header like `filename="my \"weird\" name.bin"` is truncated at the first
/// inner `\"` to `my \` instead of `my "weird" name.bin`.
///
/// Only `\"` is treated as an escape. A backslash that is *not* followed by a
/// quote is kept verbatim, because real-world (non-RFC-compliant) servers
/// routinely send unescaped Windows paths such as
/// `filename="C:\dir\file.bin"`, and decoding those backslashes as quoted-pairs
/// would silently corrupt the name. If no closing quote is present the whole
/// remainder is returned (best-effort).
fn unquote_disposition_value(body: &str) -> String {
    let mut out = String::with_capacity(body.len());
    let mut chars = body.chars().peekable();
    while let Some(c) = chars.next() {
        match c {
            // `\"` -> literal `"`. Any other backslash stays literal.
            '\\' if chars.peek() == Some(&'"') => {
                out.push('"');
                chars.next();
            }
            '"' => break,
            _ => out.push(c),
        }
    }
    out
}

/// Split a `Content-Disposition` header value into its `;`-separated
/// parameters, treating a `;` inside a double-quoted value as literal so that
/// `filename="a;b.bin"` stays a single parameter.
///
/// An escaped quote (`\"`) does not open or close a quoted value, so the split
/// agrees with [`unquote_disposition_value`] on where a value ends. Without
/// this, an escaped quote in one parameter would flip the quote state and merge
/// the following `;`-separated parameter (e.g. the real `filename=`) into it.
///
/// The escape policy is deliberately identical to [`unquote_disposition_value`]:
/// *only* `\"` is an escape — any other backslash is literal (so unescaped
/// Windows paths survive). Keeping the two in lockstep is what prevents the
/// splitter and the decoder from disagreeing on where a value ends.
fn split_disposition_params(header: &str) -> impl Iterator<Item = &str> {
    let mut params = Vec::new();
    let mut start = 0;
    let mut in_quotes = false;
    let mut chars = header.char_indices().peekable();
    while let Some((i, c)) = chars.next() {
        match c {
            // `\"` inside a quoted value is an escaped quote: consume the `"` so
            // it does not close the value. Any other backslash is literal.
            '\\' if in_quotes && chars.peek().is_some_and(|&(_, c)| c == '"') => {
                chars.next();
            }
            '"' => in_quotes = !in_quotes,
            ';' if !in_quotes => {
                params.push(&header[start..i]);
                start = i + 1;
            }
            _ => {}
        }
    }
    params.push(&header[start..]);
    params.into_iter()
}

/// Derive a candidate filename from the *last path segment* of a URL.
///
/// Uses the parsed URL structure rather than the serialized string, so the
/// query string and fragment (e.g. the long presigned-URL parameters that S3
/// and GitHub-release redirects append) never leak into the name. Returns
/// `None` when the URL has no non-empty path segment.
fn filename_from_url(url: &reqwest::Url) -> Option<String> {
    url.path_segments()?
        .rfind(|segment| !segment.is_empty())
        .map(|segment| segment.to_string())
}

/// Sanitize a candidate filename: strip path components to prevent traversal,
/// reject null bytes and overly long names. Returns `None` when nothing usable
/// survives — e.g. `".."` or `"."` (which `Path::file_name` maps to `None`),
/// or a name over 255 bytes / containing a NUL. (A trailing slash such as
/// `"dir/"` keeps its last component: `Path::file_name` returns `"dir"`.)
fn sanitize_filename(name: &str) -> Option<String> {
    let sanitized = Path::new(name)
        .file_name()
        .and_then(|n| n.to_str())
        .map(|s| s.to_string())?;
    if sanitized.contains('\0') || sanitized.len() > 255 {
        return None;
    }
    Some(sanitized)
}

/// Pick a sanitized filename from the `Content-Disposition` header (if any),
/// falling back to the URL's last path segment.
///
/// Each source is sanitized *independently* and then chained: a
/// `Content-Disposition` filename that sanitizes away (e.g. a
/// hostile/degenerate `filename=".."`, which `Path::file_name` maps to `None`)
/// must not suppress the URL fallback that would otherwise name the download
/// fine.
fn resolve_filename(content_disposition: Option<&str>, url: &reqwest::Url) -> Option<String> {
    content_disposition
        .and_then(parse_content_disposition_filename)
        .and_then(|name| sanitize_filename(&name))
        .or_else(|| filename_from_url(url).and_then(|name| sanitize_filename(&name)))
}

fn get_filename(response: &reqwest::Response) -> Option<String> {
    let content_disposition = response
        .headers()
        .get("content-disposition")
        .and_then(|value| value.to_str().ok());
    resolve_filename(content_disposition, response.url())
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

    // Reject 4xx/5xx before reading the body: otherwise the error page is
    // written out as if it were the requested artifact.
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
    use super::{filename_from_url, parse_content_disposition_filename};

    fn name_from(url: &str) -> Option<String> {
        filename_from_url(&reqwest::Url::parse(url).unwrap())
    }

    #[test]
    fn url_filename_ignores_query_string() {
        // Regression: presigned S3/GitHub-release redirects carry a long query
        // string but no Content-Disposition header. The query must not leak into
        // the filename (and must not push the name over the 255-char limit).
        assert_eq!(
            name_from("https://example.com/path/model.bin?X-Amz-Signature=deadbeef&expires=123"),
            Some("model.bin".to_string())
        );
    }

    #[test]
    fn url_filename_ignores_fragment() {
        assert_eq!(
            name_from("https://example.com/path/model.bin#section"),
            Some("model.bin".to_string())
        );
    }

    #[test]
    fn url_filename_normalises_traversal() {
        // The URL parser resolves `..` segments before we ever see them, so a
        // traversal attempt cannot escape via the fallback path.
        assert_eq!(
            name_from("https://example.com/a/../../etc/passwd"),
            Some("passwd".to_string())
        );
    }

    #[test]
    fn url_filename_plain() {
        assert_eq!(
            name_from("https://example.com/a/b/weights.safetensors"),
            Some("weights.safetensors".to_string())
        );
    }

    #[test]
    fn url_filename_skips_empty_trailing_segment() {
        // A trailing slash yields an empty last segment, which is skipped in
        // favor of the last non-empty one (matching the previous
        // `Path::file_name` behavior). A bare root has no non-empty segment.
        assert_eq!(
            name_from("https://example.com/dir/"),
            Some("dir".to_string())
        );
        assert_eq!(name_from("https://example.com/"), None);
    }

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
    fn escaped_quotes_inside_quoted_filename_are_unescaped() {
        // RFC 6266 / RFC 2616 quoted-pairs: an escaped `\"` inside the value is
        // literal, not the terminator. A substring split on the first `"` would
        // truncate this to `my \`.
        assert_eq!(
            parse_content_disposition_filename(
                "attachment; filename=\"my \\\"weird\\\" name.bin\""
            ),
            Some("my \"weird\" name.bin".to_string())
        );
    }

    #[test]
    fn escaped_quote_before_trailing_param_ends_the_value() {
        // The escaped `\"` is not the terminator; the value ends at the real
        // closing quote and the trailing `; size=...` parameter is ignored.
        assert_eq!(
            parse_content_disposition_filename(
                "attachment; filename=\"a \\\"b\\\".bin\"; size=1000"
            ),
            Some("a \"b\".bin".to_string())
        );
    }

    #[test]
    fn escaped_quote_in_earlier_param_does_not_swallow_filename() {
        // An escaped quote in a *preceding* parameter must not desync the
        // splitter from the value decoder and hide the real filename: the `;`
        // after the earlier value is still a parameter boundary.
        assert_eq!(
            parse_content_disposition_filename(
                "attachment; title=\"a \\\"b\"; filename=\"doc.bin\""
            ),
            Some("doc.bin".to_string())
        );
    }

    #[test]
    fn unescaped_backslashes_are_kept_verbatim() {
        // Real-world (non-RFC-compliant) servers send unescaped Windows paths;
        // a `\` that does not escape a quote must be preserved, not dropped.
        assert_eq!(
            parse_content_disposition_filename("attachment; filename=\"C:\\Users\\file.bin\""),
            Some("C:\\Users\\file.bin".to_string())
        );
    }

    #[test]
    fn filename_parameter_name_is_case_insensitive() {
        // RFC 6266: `Content-Disposition` parameter names are case-insensitive,
        // so a spec-compliant server sending a non-lowercase `filename` must not
        // have its filename silently dropped.
        for header in [
            "attachment; Filename=\"model.bin\"",
            "attachment; FILENAME=\"model.bin\"",
            "attachment; FileName=model.bin",
        ] {
            assert_eq!(
                parse_content_disposition_filename(header),
                Some("model.bin".to_string()),
                "header {header:?} should parse case-insensitively"
            );
        }
    }

    #[test]
    fn extended_form_before_plain_filename_is_skipped() {
        // A header may carry both the RFC 5987 extended form and a plain one;
        // the plain `filename=` value must still be found (and the `filename*=`
        // form skipped, not mistaken for it).
        assert_eq!(
            parse_content_disposition_filename(
                "attachment; filename*=UTF-8''extended.bin; filename=\"plain.bin\""
            ),
            Some("plain.bin".to_string())
        );
    }

    #[test]
    fn longer_token_ending_in_filename_is_not_matched() {
        // Regression: a substring search for `filename=` would false-match the
        // `xfilename=` parameter and return `evil`. The real `filename`
        // parameter must be the one that is used.
        assert_eq!(
            parse_content_disposition_filename(
                "attachment; xfilename=\"evil\"; filename=\"good.bin\""
            ),
            Some("good.bin".to_string())
        );
        // With no genuine `filename` parameter, a look-alike token yields None.
        assert_eq!(
            parse_content_disposition_filename("attachment; xfilename=\"evil\""),
            None
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

    // --- resolve_filename (Content-Disposition + URL fallback) ---

    fn resolve(cd: Option<&str>, url: &str) -> Option<String> {
        super::resolve_filename(cd, &reqwest::Url::parse(url).unwrap())
    }

    #[test]
    fn resolve_prefers_content_disposition() {
        assert_eq!(
            resolve(
                Some("attachment; filename=\"model.bin\""),
                "https://example.com/other.bin"
            ),
            Some("model.bin".to_string())
        );
    }

    #[test]
    fn resolve_falls_back_to_url_when_no_header() {
        assert_eq!(
            resolve(None, "https://example.com/dir/weights.safetensors"),
            Some("weights.safetensors".to_string())
        );
    }

    #[test]
    fn resolve_falls_back_to_url_when_header_sanitizes_away() {
        // Regression: a degenerate/hostile `Content-Disposition` filename that
        // `Path::file_name` maps to `None` (`..`, `.`, a trailing slash) must
        // not suppress the perfectly good URL fallback — previously
        // `get_filename` returned `None` and aborted the download.
        for cd in ["attachment; filename=\"..\"", "attachment; filename=\".\""] {
            assert_eq!(
                resolve(Some(cd), "https://example.com/model.bin"),
                Some("model.bin".to_string()),
                "header {cd:?} should fall back to the URL name"
            );
        }
    }

    #[test]
    fn resolve_returns_none_when_both_sources_are_unusable() {
        assert_eq!(
            resolve(Some("attachment; filename=\"..\""), "https://example.com/"),
            None
        );
    }
}
