use eyre::{Context, ContextCompat};
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

pub async fn download_file<T>(url: T, target_dir: &Path) -> Result<PathBuf, eyre::ErrReport>
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
        .wrap_err("failed to read operator from `{uri}`")?;
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
