use eyre::Context;
use std::io::Write;

pub fn download_file<T>(url: T) -> Result<tempfile::NamedTempFile, eyre::ErrReport>
where
    T: reqwest::IntoUrl + std::fmt::Display + Copy,
{
    let response = futures::executor::block_on(async {
        reqwest::get(url)
            .await
            .wrap_err_with(|| format!("failed to request operator from `{url}`"))?
            .bytes()
            .await
            .wrap_err("failed to read operator from `{uri}`")
    })?;
    let mut tmp =
        tempfile::NamedTempFile::new().wrap_err("failed to create temp file for operator")?;
    tmp.as_file_mut()
        .write_all(&response)
        .wrap_err("failed to write downloaded operator to file")?;
    Ok(tmp)
}
