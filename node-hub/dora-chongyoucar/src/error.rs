use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("serial connect fail")]
    SerialConnect,
    #[error("serial settings set fail")]
    SerialSettingsSet,
    #[error("serial set timeout fail")]
    SerialSetTimeout,
}
