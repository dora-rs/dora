use eyre::Context;
#[cfg(unix)]
use std::os::unix::prelude::PermissionsExt;
use std::path::Path;
use tokio::io::AsyncWriteExt;
use tracing::info;

pub async fn download_file<T>(url: T, target_path: &Path) -> Result<(), eyre::ErrReport>
where
    T: reqwest::IntoUrl + std::fmt::Display + Copy,
{
    if target_path.exists() {
        info!("Using cache: {:?}", target_path.to_str());
        return Ok(());
    }

    if let Some(parent) = target_path.parent() {
        tokio::fs::create_dir_all(parent)
            .await
            .wrap_err("failed to create parent folder")?;
    }

    let response = reqwest::get(url)
        .await
        .wrap_err_with(|| format!("failed to request operator from `{url}`"))?
        .bytes()
        .await
        .wrap_err("failed to read operator from `{uri}`")?;
    let mut file = tokio::fs::File::create(target_path)
        .await
        .wrap_err("failed to create target file")?;
    file.write_all(&response)
        .await
        .wrap_err("failed to write downloaded operator to file")?;
    file.sync_all().await.wrap_err("failed to `sync_all`")?;

    #[cfg(unix)]
    file.set_permissions(std::fs::Permissions::from_mode(0o764))
        .await
        .wrap_err("failed to make downloaded file executable")?;

    Ok(())
}
