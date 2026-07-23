use thiserror::Error;

#[derive(Debug, Error)]
pub enum BridgeError {
    #[error("Arrow error: {0}")]
    Arrow(#[from] arrow::error::ArrowError),

    #[error("config error: {0}")]
    Config(String),

    #[error("MAVLink error: {0}")]
    Mavlink(String),
}

pub type BridgeResult<T> = Result<T, BridgeError>;
