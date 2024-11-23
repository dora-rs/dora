use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("serial connect fail")]
    Connect,
    #[error("serial settings set fail")]
    SettingsSet,
    #[error("serial set timeout fail")]
    SetTimeout,
}
