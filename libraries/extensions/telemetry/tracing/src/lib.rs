//! Enable tracing using Opentelemetry and Jaeger.
//!
//! This module init a tracing propagator for Rust code that requires tracing, and is
//! able to serialize and deserialize context that has been sent via the middleware.

use eyre::Context as EyreContext;
use tracing::metadata::LevelFilter;
use tracing_subscriber::{
    filter::FilterExt, prelude::__tracing_subscriber_SubscriberExt, EnvFilter, Layer,
};

use eyre::ContextCompat;
use tracing_subscriber::Registry;
pub mod telemetry;

pub fn set_up_tracing(name: &str) -> eyre::Result<()> {
    // Filter log using `RUST_LOG`. More useful for CLI.
    let filter = EnvFilter::from_default_env().or(LevelFilter::WARN);
    let stdout_log = tracing_subscriber::fmt::layer()
        .pretty()
        .with_filter(filter);

    let registry = Registry::default().with(stdout_log);
    if let Some(endpoint) = std::env::var_os("DORA_JAEGER_TRACING") {
        let endpoint = endpoint
            .to_str()
            .wrap_err("Could not parse env variable: DORA_JAEGER_TRACING")?;
        let tracer = crate::telemetry::init_jaeger_tracing(name, endpoint)
            .wrap_err("Could not instantiate tracing")?;
        let telemetry = tracing_opentelemetry::layer().with_tracer(tracer);
        let subscriber = registry.with(telemetry);
        tracing::subscriber::set_global_default(subscriber).context(format!(
            "failed to set tracing global subscriber for {name}"
        ))
    } else {
        tracing::subscriber::set_global_default(registry).context(format!(
            "failed to set tracing global subscriber for {name}"
        ))
    }
}
