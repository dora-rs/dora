use mio_extras::channel::TrySendError;

/// This is a specialized Result, similar to [`std::io::Result`]
pub type Result<T> = std::result::Result<T, Error>;

/// This corresponds to "Return codes" in DDS spec 2.2.1.1 Format and
/// Conventions
///
/// Deviations from the DDS spec:
/// * `OK` is not included. It is not an error. Ok/Error should be distinguished
///   with the `Result` type.
/// * `Error` is too unspecific.
/// * `AlreadyDeleted` We should use Rust type system to avoid these, so no need
///   for run-time error.
/// * `Timeout`  This is normal operation and should be encoded as `Option` or
///   `Result`
/// * `NoData`  This should be encoded as `Option<SomeData>`, not an error code.
#[derive(Debug, thiserror::Error)]
pub enum Error {
  /// This is out of the DDS spec.
  /// If DataWriter with Reliability == RELIABLE cannot send within the
  /// QoS-specified timeout, this error will occur. This can be caused by
  /// writing data faster than can be written to network, or some of the
  /// matched reliable DataReaders cannot keep up, i.e. acknowledge samples
  /// fast enough.
  #[error("Write must wait for network or reliable DataReaders")]
  MustBlock,
  /// Illegal parameter value.
  #[error("Bad parameter: {reason}")]
  BadParameter { reason: String },

  /// Unsupported operation. Can only be returned by operations that are
  /// optional.
  #[error("Unsupported operation")]
  Unsupported,

  /// Service ran out of the resources needed to complete the operation.
  #[error("Out of resources")]
  OutOfResources,

  /// Operation invoked on an Entity that is not yet enabled.
  #[error("Entity not yet enabled")]
  NotEnabled,

  /// Application attempted to modify an immutable QosPolicy.
  #[error("Attempted to modify immutable entity")]
  ImmutablePolicy, // can we check this statically?

  /// Application specified a set of policies that are not consistent with each
  /// other.
  #[error("Inconsistent policies: {reason}")]
  InconsistentPolicy { reason: String },

  /// A pre-condition for the operation was not met.
  #[error("Precondition not met: {precondition}")]
  PreconditionNotMet { precondition: String },

  /// An operation was invoked on an inappropriate object or at
  /// an inappropriate time (as determined by policies set by the
  /// specification or the Service implementation). There is no
  /// precondition that could be changed to make the operation
  /// succeed.
  #[error("Illegal operation: {reason}")]
  IllegalOperation { reason: String },

  // Our own additions to the DDS spec below:
  /// Synchronization with another thread failed because the [other thread
  /// has exited while holding a lock.](https://doc.rust-lang.org/std/sync/struct.PoisonError.html)
  /// Does not exist in the DDS spec.
  #[error("Lock poisoned")]
  LockPoisoned,

  /// Something that should not go wrong went wrong anyway.
  /// This is usually a bug in RustDDS
  #[error("Internal error: {reason}")]
  Internal { reason: String },

  #[error(transparent)]
  Io(#[from] std::io::Error),

  #[error("Serialization error: {reason}")]
  Serialization { reason: String },

  #[error("Discovery error: {reason}")]
  Discovery { reason: String },
}

impl Error {
  pub fn bad_parameter<T>(reason: impl Into<String>) -> Result<T> {
    Err(Self::BadParameter {
      reason: reason.into(),
    })
  }

  pub fn precondition_not_met<T>(precondition: impl Into<String>) -> Result<T> {
    Err(Self::PreconditionNotMet {
      precondition: precondition.into(),
    })
  }

  pub fn serialization_error<T>(reason: impl Into<String>) -> Result<T> {
    Err(Self::Serialization {
      reason: reason.into(),
    })
  }
}

#[doc(hidden)]
#[macro_export]
macro_rules! log_and_err_precondition_not_met {
  ($err_msg:literal) => {{
    log::error!($err_msg);
    Error::precondition_not_met($err_msg)
  }};
}

#[doc(hidden)]
#[macro_export]
macro_rules! log_and_err_internal {
  ($($arg:tt)*) => (
      { log::error!($($arg)*);
        Err( Error::Internal{ reason: format!($($arg)*) } )
      }
    )
}

#[doc(hidden)]
#[macro_export]
macro_rules! log_and_err_discovery {
  ($($arg:tt)*) => (
      { error!($($arg)*);
        Error::Message(format!($($arg)*) )
      }
    )
}

impl<T> From<std::sync::PoisonError<T>> for Error {
  fn from(_e: std::sync::PoisonError<T>) -> Self {
    Self::LockPoisoned
  }
}

impl<T> From<TrySendError<T>> for Error
where
  TrySendError<T>: std::error::Error,
{
  fn from(e: TrySendError<T>) -> Self {
    Self::Internal {
      reason: format!("Cannot send to internal mio channel: {e:?}"),
    }
  }
}
