//! Enable tracing using Opentelemetry and Jaeger.
//!
//! This module init a tracing propagator for Rust code that requires tracing, and is
//! able to serialize and deserialize context that has been sent via the middleware.

use eyre::Context as EyreContext;
use tracing::metadata::LevelFilter;
use tracing_subscriber::{prelude::__tracing_subscriber_SubscriberExt, EnvFilter, Layer};

#[cfg(feature = "telemetry")]
pub mod telemetry;

pub fn set_up_tracing(name: &str) -> eyre::Result<()> {
    // Filter log using `RUST_LOG`. More useful for CLI.
    let filter = EnvFilter::from_default_env();
    let stdout_log = tracing_subscriber::fmt::layer()
        .pretty()
        .with_filter(filter);

    #[cfg(feature = "telemetry")]
    let subscriber = {
        use eyre::ContextCompat;
        use opentelemetry::sdk::export::trace::stdout;
        use tracing_subscriber::Registry;

        let filter = EnvFilter::from_default_env();
        let telemetry = match std::env::var_os("DORA_JAEGER_TRACING") {
            Some(endpoint) => {
                let endpoint = endpoint
                    .to_str()
                    .wrap_err("Could not parse env variable: DORA_JAEGER_TRACING")?;
                let tracer = crate::telemetry::init_jaeger_tracing(name, endpoint)
                    .wrap_err("Could not instantiate tracing")?;
                tracing_opentelemetry::layer()
                    .with_tracer(tracer)
                    .with_filter(filter.add_directive(LevelFilter::TRACE.into()))
            }
            None => {
                let tracer = stdout::new_pipeline().install_simple();
                tracing_opentelemetry::layer()
                    .with_tracer(tracer)
                    .with_filter(filter)
            }
        };
        // Create a tracing layer with the configured tracer
        Registry::default().with(stdout_log).with(telemetry)
    };
    #[cfg(not(feature = "telemetry"))]
    let subscriber = tracing_subscriber::Registry::default().with(stdout_log);

    // Use the tracing subscriber `Registry`, or any other subscriber
    // that impls `LookupSpan`
    tracing::subscriber::set_global_default(subscriber).context(format!(
        "failed to set tracing global subscriber for {name}"
    ))
}
