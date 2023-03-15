//! Enable tracing using Opentelemetry and Jaeger.
//!
//! This module init a tracing propagator for Rust code that requires tracing, and is
//! able to serialize and deserialize context that has been sent via the middleware.

use eyre::Context as EyreContext;
use tracing_subscriber::{EnvFilter, Layer};

#[cfg(feature = "telemetry")]
pub mod telemetry;

pub fn set_up_tracing() -> eyre::Result<()> {
    use tracing_subscriber::prelude::__tracing_subscriber_SubscriberExt;
    let filter = EnvFilter::from_default_env();

    let stdout_log = tracing_subscriber::fmt::layer()
        .pretty()
        .with_filter(filter);
    let subscriber = tracing_subscriber::Registry::default().with(stdout_log);
    tracing::subscriber::set_global_default(subscriber)
        .context("failed to set tracing global subscriber")
}
