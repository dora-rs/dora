use thiserror::Error;

/// Typed errors for the Adora node API.
#[derive(Debug, Error)]
pub enum NodeError {
    /// Initialization failed (config parsing, env vars, daemon handshake).
    #[error("initialization failed: {0}")]
    Init(String),
    /// Could not connect to the daemon or lost connection.
    #[error("connection error: {0}")]
    Connection(String),
    /// Sending an output or closing outputs failed.
    #[error("output error: {0}")]
    Output(String),
    /// Data allocation or descriptor parsing failed.
    #[error("data error: {0}")]
    Data(String),
    /// Catch-all for internal/unexpected errors (use `{:?}` for full chain).
    #[error("internal error")]
    Internal(#[from] eyre::Report),
}

/// Convenience alias used by all public node API functions.
pub type NodeResult<T> = std::result::Result<T, NodeError>;
