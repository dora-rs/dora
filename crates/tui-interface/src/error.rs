//! Error types shared across TUI interface services.

/// Basic error enum used by the interface crate.
#[derive(Debug, Clone, thiserror::Error)]
pub enum InterfaceError {
    /// Placeholder variant until real errors are added.
    #[error("operation not implemented yet")]
    Unimplemented,
    /// Wrapper for string-based errors encountered by service implementations.
    #[error("{0}")]
    Message(String),
}

impl From<&str> for InterfaceError {
    fn from(value: &str) -> Self {
        InterfaceError::Message(value.to_string())
    }
}

impl From<String> for InterfaceError {
    fn from(value: String) -> Self {
        InterfaceError::Message(value)
    }
}
