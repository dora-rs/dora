//! Enable tracing using Opentelemetry and Jaeger.
//!
//! This module init a tracing propagator for Rust code that requires tracing, and is
//! able to serialize and deserialize context that has been sent via the middleware.

use eyre::Context as EyreContext;
use opentelemetry::sdk::export::trace::stdout;
use tracing_subscriber::{EnvFilter, Layer, Registry};

#[cfg(feature = "telemetry")]
pub mod telemetry;

pub fn set_up_tracing(name: &str) -> eyre::Result<()> {
    use tracing_subscriber::prelude::__tracing_subscriber_SubscriberExt;
    let filter = EnvFilter::from_default_env();

    //let stdout_log = tracing_subscriber::fmt::layer()
    //.pretty()
    //.with_filter(filter);

    #[cfg(feature = "telemetry")]
    {
        let tracer = match std::env::var_os("DORA_TELEMETRY") {
            Some(_) => {
                crate::telemetry::init_tracing(name).wrap_err("Could not instantiate tracing")?
            }
            None => stdout::new_pipeline().install_simple(),
        };

        // Create a tracing layer with the configured tracer
        let telemetry = tracing_opentelemetry::layer().with_tracer(tracer);

        // Use the tracing subscriber `Registry`, or any other subscriber
        // that impls `LookupSpan`
        let subscriber = Registry::default().with(telemetry);
        tracing::subscriber::set_global_default(subscriber)
            .context("failed to set tracing global subscriber")
    }
    #[cfg(not(feature = "telemetry"))]
    {
        let subscriber = tracing_subscriber::Registry::default().with(stdout_log);
        tracing::subscriber::set_global_default(subscriber)
            .context("failed to set tracing global subscriber")
    }
}
