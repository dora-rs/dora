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

/// Errors returned by the pattern-aware receive helpers on
/// [`EventStream`](crate::EventStream).
///
/// These cover the recoverable failure modes a service client or action
/// client needs to react to when waiting for a correlated reply. See
/// [`EventStream::recv_service_response`](crate::EventStream::recv_service_response)
/// and
/// [`EventStream::recv_action_result`](crate::EventStream::recv_action_result).
#[derive(Debug, Error)]
pub enum PatternError {
    /// The correlated reply did not arrive before the deadline elapsed.
    #[error("pattern reply timed out")]
    Timeout,
    /// The expected server node restarted, so the in-flight correlation
    /// is orphaned. Clients should retry the request against the new
    /// instance (dora-rs/adora#148).
    #[error("server `{0}` restarted while waiting for reply")]
    ServerRestarted(String),
    /// The event stream ended (e.g. dataflow stopping) before the
    /// correlated reply arrived.
    #[error("event stream ended before pattern reply arrived")]
    StreamEnded,
    /// The event stream surfaced an error while waiting.
    #[error("pattern wait failed: {0}")]
    StreamError(String),
}
