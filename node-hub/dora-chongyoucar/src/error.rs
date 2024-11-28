use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("serial: {0} connect fail")]
    Connect(String),
    #[error("serial settings: {0} set fail")]
    SettingsSet(String),
    #[error("serial set timeout: {0} fail")]
    SetTimeout(String),
}
