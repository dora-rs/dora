use eyre::Context;
#[cfg(unix)]
use std::os::unix::prelude::PermissionsExt;
use std::{io::Write, path::Path};

pub fn download_file<T>(url: T, target_path: &Path) -> Result<(), eyre::ErrReport>
where
    T: reqwest::IntoUrl + std::fmt::Display + Copy,
{
    if let Some(parent) = target_path.parent() {
        std::fs::create_dir_all(parent).wrap_err("failed to create parent folder")?;
    }

    let response = futures::executor::block_on(async {
        reqwest::get(url)
            .await
            .wrap_err_with(|| format!("failed to request operator from `{url}`"))?
            .bytes()
            .await
            .wrap_err("failed to read operator from `{uri}`")
    })?;
    let mut file = std::fs::File::create(target_path).wrap_err("failed to create target file")?;
    file.write_all(&response)
        .wrap_err("failed to write downloaded operator to file")?;
    file.sync_all().wrap_err("failed to `sync_all`")?;

    #[cfg(unix)]
    file.set_permissions(std::fs::Permissions::from_mode(0o764))
        .wrap_err("failed to make downloaded file executable")?;

    Ok(())
}
