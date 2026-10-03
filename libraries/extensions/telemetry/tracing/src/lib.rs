//! **Internal to dora — not a public API.**
//!
//! This crate is published to crates.io only because cargo requires every
//! dependency of a published crate to be published; `dora-node-api` and
//! `dora-cli` depend on it. It is not covered by dora's 1.0 stability
//! guarantee and may change in any release, including a patch.
//!
//! Depend on it directly at your own risk. See the "Stability scope at 1.0"
//! section of `docs/api-rust.md`.
//!
//! Enable tracing using OpenTelemetry with OTLP.
//!
//! This module initializes a tracing propagator for Rust code that requires tracing, and is
//! able to serialize and deserialize context that has been sent via the middleware.
//! Supports any OTLP-compatible backend (Jaeger, Zipkin, Tempo, etc.).

use std::path::{Path, PathBuf};

use eyre::Context as EyreContext;
use opentelemetry::trace::TracerProvider;
use opentelemetry_sdk::metrics::SdkMeterProvider;
use opentelemetry_sdk::trace::SdkTracerProvider;
use tracing::metadata::LevelFilter;

use tracing_opentelemetry::{MetricsLayer, OpenTelemetryLayer};
use tracing_subscriber::{
    EnvFilter, Layer, filter::FilterExt, prelude::__tracing_subscriber_SubscriberExt,
};

use tracing_subscriber::Registry;
use tracing_subscriber::filter::Directive;
pub mod metrics;
pub mod span_store;
pub mod telemetry;

/// Parse a tracing directive known to be syntactically valid by
/// construction. Panics on failure — call sites must only pass string
/// literals. Replaces ~20 individual `.parse().unwrap()` sites with a
/// single documented helper, and keeps the panic path centralized.
fn directive(literal: &'static str) -> Directive {
    literal
        .parse()
        .unwrap_or_else(|e| panic!("invalid hardcoded tracing directive `{literal}`: {e}"))
}

/// Standard set of noisy-crate suppressions applied to every filter in
/// this module. Centralized so the list is the same everywhere.
const NOISY_CRATES_OFF: &[&str] = &[
    "hyper=off",
    "tonic=off",
    "tokio=off",
    "process_wrap=off",
    "h2=off",
    "reqwest=off",
];

/// Add the `NOISY_CRATES_OFF` directives to an `EnvFilter`.
fn with_noisy_crates_off(mut filter: EnvFilter) -> EnvFilter {
    for d in NOISY_CRATES_OFF {
        filter = filter.add_directive(directive(d));
    }
    filter
}

/// Whether `RUST_LOG` already carries a directive targeting `target` (the exact
/// crate/module, or a submodule of it), so the user's directive for it must
/// override the hard-coded default (see [`with_target_default`]).
///
/// `RUST_LOG` is a comma-separated list of `[target[=level]]` directives. Match
/// the directive's target on a name boundary — exactly `target`, or a
/// `target::…` submodule — rather than with a bare substring, so an unrelated
/// directive that merely contains the letters (e.g. `zenoh_transport=trace` or
/// `my_dora_daemon=info`) is not applied to the wrong target.
fn env_configures_target(env_log: &str, target: &str) -> bool {
    env_log.split(',').any(|directive| {
        // The bare target name is everything up to the first `=` (level) or `[`
        // (span filter); trim surrounding whitespace so we compare the crate/
        // module path itself.
        let name = directive.split(['=', '[']).next().unwrap_or("").trim();
        name == target
            || name
                .strip_prefix(target)
                .is_some_and(|rest| rest.starts_with("::"))
    })
}

/// Add the hard-coded `default` directive for `target` to `filter`, followed
/// by any `RUST_LOG` directives for that target (or a submodule of it), which
/// take precedence over the default.
///
/// Just leaving the default out when `RUST_LOG` names the target is not
/// enough: the stdout filters are `EnvFilter::from_default_env().or(filter)`,
/// which enables an event when *either* side does, so a target missing from
/// `filter` falls back to its global level. A `RUST_LOG=zenoh=error` meant to
/// quiet zenoh would then let zenoh through at `info`, louder than the
/// `zenoh=warn` default.
fn with_target_default(
    filter: EnvFilter,
    env_log: &str,
    target: &str,
    default: &'static str,
) -> EnvFilter {
    let mut filter = filter.add_directive(directive(default));
    for user_directive in env_log
        .split(',')
        .filter(|d| env_configures_target(d, target))
    {
        if let Ok(user_directive) = user_directive.trim().parse::<Directive>() {
            filter = filter.add_directive(user_directive);
        }
    }
    filter
}

/// The non-`RUST_LOG` side of [`TracingBuilder::with_stdout`]'s filter.
fn stdout_filter(filter: &str, env_log: &str) -> EnvFilter {
    let parsed = with_noisy_crates_off(EnvFilter::builder().parse_lossy(filter));
    let parsed = with_target_default(parsed, env_log, "dora_daemon", "dora_daemon=info");
    let parsed = with_target_default(parsed, env_log, "dora_core", "dora_core=warn");
    with_target_default(parsed, env_log, "zenoh", "zenoh=warn")
}

/// Setup tracing with a default configuration.
///
/// This will set up a global subscriber that logs to stdout with a filter level of "warn".
/// Outputs structured JSONL that the dora daemon parses as `LogMessage`,
/// so `tracing::info!()` calls in user nodes are automatically routed through
/// the dora log pipeline (file, coordinator, `dora/logs` subscribers).
///
/// Should **ONLY** be used in `DoraNode` implementations.
pub fn set_up_tracing(name: &str) -> eyre::Result<()> {
    TracingBuilder::new(name)
        .with_node_stdout("warn")
        .build()
        .wrap_err(format!(
            "failed to set tracing global subscriber for {name}"
        ))?;
    Ok(())
}

pub struct OtelGuard {
    tracer_provider: SdkTracerProvider,
    meter_provider: SdkMeterProvider,
}

#[must_use = "call `build` to finalize the tracing setup"]
pub struct TracingBuilder {
    name: String,
    layers: Vec<Box<dyn Layer<Registry> + Send + Sync>>,
    pub guard: Option<OtelGuard>,
}

impl TracingBuilder {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            layers: Vec::new(),
            guard: None,
        }
    }

    /// Add a layer that writes structured JSONL to stdout, compatible with the
    /// dora daemon's `LogMessageHelper` parser.
    ///
    /// This is the recommended layer for user nodes: `tracing::info!()` calls
    /// are automatically parsed by the daemon and routed through the log pipeline.
    pub fn with_node_stdout(mut self, filter: impl AsRef<str>) -> Self {
        let parsed = with_noisy_crates_off(EnvFilter::builder().parse_lossy(filter));
        let env_log = std::env::var("RUST_LOG").unwrap_or_default();
        let parsed = with_target_default(parsed, &env_log, "zenoh", "zenoh=warn");
        let env_filter = EnvFilter::from_default_env().or(parsed);
        let layer = tracing_subscriber::fmt::layer()
            .json()
            .with_writer(std::io::stdout)
            .with_filter(env_filter);
        self.layers.push(layer.boxed());
        self
    }

    /// Add a layer that write logs to the [std::io::stdout] with the given filter.
    ///
    /// **DO NOT** use this in `DoraNode` implementations,
    /// it uses [std::io::stdout] which is synchronous
    /// and might block the logging thread.
    pub fn with_stdout(mut self, filter: impl AsRef<str>, json: bool) -> Self {
        let env_log = std::env::var("RUST_LOG").unwrap_or_default();
        let parsed = stdout_filter(filter.as_ref(), &env_log);
        let env_filter = EnvFilter::from_default_env().or(parsed);
        let layer = tracing_subscriber::fmt::layer()
            .compact()
            .with_writer(std::io::stdout);

        if json {
            let layer = layer.json().with_filter(env_filter);
            self.layers.push(layer.boxed());
        } else {
            let layer = layer.with_filter(env_filter);
            self.layers.push(layer.boxed());
        };
        self
    }

    /// Add a layer that write logs to a file with the given name and filter.
    pub fn with_file(
        mut self,
        file_name: impl Into<String>,
        filter: LevelFilter,
    ) -> eyre::Result<Self> {
        let file_name = file_name.into();
        let out_dir = Path::new("out");
        std::fs::create_dir_all(out_dir).context("failed to create `out` directory")?;
        let path = log_file_path(out_dir, &file_name);
        let file = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .context("failed to create log file")?;
        let layer = tracing_subscriber::fmt::layer()
            .with_ansi(false)
            .json()
            .with_writer(file)
            .with_filter(filter);
        self.layers.push(layer.boxed());
        Ok(self)
    }

    /// Add OpenTelemetry tracing layer with OTLP exporter.
    ///
    /// Reads the OTLP endpoint from `DORA_OTLP_ENDPOINT` environment variable.
    /// If not set, falls back to `DORA_JAEGER_TRACING` for backward compatibility.
    ///
    /// The endpoint should be in the format: "http://localhost:4317"
    pub fn with_otlp_tracing(mut self) -> eyre::Result<Self> {
        let endpoint = std::env::var("DORA_OTLP_ENDPOINT")
            .or_else(|_| std::env::var("DORA_JAEGER_TRACING"))
            .wrap_err("DORA_OTLP_ENDPOINT or DORA_JAEGER_TRACING environment variable not set")?;

        // Initialize OTLP tracing - this returns a tracer and sets the global provider
        let sdk_tracer_provider = crate::telemetry::init_tracing(&self.name, &endpoint)
            .wrap_err("failed to initialize OTLP tracing exporter")?;
        let meter_provider = metrics::init_meter_provider(&self.name, &endpoint)
            .wrap_err("failed to initialize OTLP metrics exporter")?;

        // TODO: Maybe this needs to be removed in favor of application level global.
        // global::set_meter_provider(meter_provider.clone());
        // Use the specific tracer instance returned from init_tracing
        let tracer = sdk_tracer_provider.tracer("tracing-otel-subscriber");

        let guard = OtelGuard {
            tracer_provider: sdk_tracer_provider,
            meter_provider: meter_provider.clone(),
        };

        self.guard = Some(guard);
        self.layers.push(MetricsLayer::new(meter_provider).boxed());
        let env_log = std::env::var("RUST_LOG").unwrap_or_default();
        let filter_otel = with_target_default(
            with_noisy_crates_off(EnvFilter::new("trace")),
            &env_log,
            "dora_daemon",
            "dora_daemon=debug",
        );
        self.layers.push(
            OpenTelemetryLayer::new(tracer)
                .with_filter(filter_otel)
                .boxed(),
        );
        Ok(self)
    }

    /// Add a layer that captures completed spans into the given store.
    ///
    /// Only captures spans from the `dora_coordinator` and `dora_core` crates
    /// at info level (all other targets are filtered out) to avoid noise from
    /// third-party dependencies.
    pub fn with_span_capture(mut self, store: span_store::SharedSpanStore) -> Self {
        let filter = EnvFilter::new("off")
            .add_directive(directive("dora_coordinator=info"))
            .add_directive(directive("dora_core=info"));
        let layer = span_store::SpanCaptureLayer::new(store).with_filter(filter);
        self.layers.push(layer.boxed());
        self
    }

    pub fn add_layer<L>(mut self, layer: L) -> Self
    where
        L: Layer<Registry> + Send + Sync + 'static,
    {
        self.layers.push(layer.boxed());
        self
    }

    pub fn with_layers<I, L>(mut self, layers: I) -> Self
    where
        I: IntoIterator<Item = L>,
        L: Layer<Registry> + Send + Sync + 'static,
    {
        for layer in layers {
            self.layers.push(layer.boxed());
        }
        self
    }

    pub fn build(self) -> eyre::Result<()> {
        let registry = Registry::default().with(self.layers);

        // TODO: Maybe this needs to be removed in favor of application level global.
        tracing::subscriber::set_global_default(registry).context(format!(
            "failed to set tracing global subscriber for {}",
            self.name
        ))
    }
}

impl Drop for OtelGuard {
    fn drop(&mut self) {
        self.meter_provider.force_flush().ok();
        self.meter_provider.shutdown().ok();
        self.tracer_provider.force_flush().ok();
        self.tracer_provider.shutdown().ok();
    }
}

/// Initialize tracing with OTLP (if configured) or stdout/file logging.
///
/// This function should be called after creating a tokio runtime and calling `runtime.enter()`.
///
/// # Parameters
/// - `name`: Service name for tracing
/// - `stdout_filter`: Optional RUST_LOG-style filter for stdout logging (e.g., "info", "debug")
/// - `file_name`: Optional filename for file logging (will be placed in `out/` directory)
/// - `file_filter`: Level filter for file logging (only used if `file_name` is Some)
///
/// # Returns
/// Returns `Option<OtelGuard>` which must be kept alive for the duration of the program.
/// When dropped, the guard will flush and shutdown telemetry providers.
///
/// # Example
/// ```no_run
/// use dora_tracing::init_tracing_subscriber;
/// use tracing::level_filters::LevelFilter;
///
/// // Note: This function requires a tokio runtime context to be active
/// // when using OTLP tracing. Use runtime.enter() before calling.
/// let _guard = init_tracing_subscriber(
///     "my-service",
///     Some("info"),
///     Some("my-service"),
///     LevelFilter::INFO,
/// ).unwrap();
/// ```
pub fn init_tracing_subscriber(
    name: &str,
    stdout_filter: Option<&str>,
    file_name: Option<&str>,
    file_filter: LevelFilter,
) -> eyre::Result<Option<OtelGuard>> {
    let mut builder = TracingBuilder::new(name);

    let guard: Option<OtelGuard> = if std::env::var("DORA_OTLP_ENDPOINT").is_ok()
        || std::env::var("DORA_JAEGER_TRACING").is_ok()
    {
        builder = builder
            .with_otlp_tracing()
            .wrap_err("failed to set up OTLP tracing")?;
        builder.guard.take()
    } else {
        if let Some(filter) = stdout_filter {
            builder = builder.with_stdout(filter, false);
        }
        None
    };

    if let Some(filename) = file_name {
        builder = builder.with_file(filename, file_filter)?;
    }

    builder
        .build()
        .wrap_err("failed to set up tracing subscriber")?;

    Ok(guard)
}

/// `<out_dir>/<file_name>.txt`.
///
/// Appends the extension rather than using `Path::with_extension`, which
/// *replaces* everything after the last `.`: the daemon names its file
/// `dora-daemon-<machine_id>`, so machine ids `10.0.0.1` and `10.0.0.2` would
/// both log to `dora-daemon-10.0.0.txt`, and `robot.local` to
/// `dora-daemon-robot.txt`.
fn log_file_path(out_dir: &Path, file_name: &str) -> PathBuf {
    out_dir.join(format!("{file_name}.txt"))
}

#[cfg(test)]
mod tests {
    use super::{EnvFilter, Layer, env_configures_target, log_file_path, stdout_filter};
    use std::path::Path;
    use tracing::Level;
    use tracing_subscriber::{filter::FilterExt, layer::SubscriberExt};

    /// Whether `target` at `level` passes `with_stdout("info", _)`'s filter
    /// with `RUST_LOG` set to `env_log`.
    fn stdout_enabled(env_log: &str, target: &'static str, level: Level) -> bool {
        let filter = EnvFilter::builder()
            .parse_lossy(env_log)
            .or(stdout_filter("info", env_log));
        let layer = tracing_subscriber::fmt::layer()
            .with_writer(std::io::sink)
            .with_filter(filter);
        let subscriber = tracing_subscriber::Registry::default().with(layer);
        tracing::subscriber::with_default(subscriber, || match (target, level) {
            ("zenoh", Level::INFO) => tracing::enabled!(target: "zenoh", Level::INFO),
            ("zenoh", Level::WARN) => tracing::enabled!(target: "zenoh", Level::WARN),
            ("zenoh", Level::DEBUG) => tracing::enabled!(target: "zenoh", Level::DEBUG),
            ("zenoh::net", Level::INFO) => tracing::enabled!(target: "zenoh::net", Level::INFO),
            ("zenoh::net", Level::WARN) => tracing::enabled!(target: "zenoh::net", Level::WARN),
            ("dora_core", Level::INFO) => tracing::enabled!(target: "dora_core", Level::INFO),
            other => unreachable!("add a case for {other:?}"),
        })
    }

    #[test]
    fn quieter_rust_log_for_a_target_is_not_louder_than_the_default() {
        // Defaults with no RUST_LOG.
        assert!(!stdout_enabled("", "zenoh", Level::INFO));
        assert!(stdout_enabled("", "zenoh", Level::WARN));
        // Asking for less zenoh output must not yield more.
        assert!(!stdout_enabled("zenoh=error", "zenoh", Level::INFO));
        assert!(!stdout_enabled("zenoh=error", "zenoh", Level::WARN));
        assert!(!stdout_enabled("zenoh=off", "zenoh", Level::WARN));
        assert!(!stdout_enabled("dora_core=error", "dora_core", Level::INFO));
        // A submodule directive keeps the default for the rest of the crate.
        assert!(!stdout_enabled("zenoh::net=error", "zenoh", Level::INFO));
        assert!(!stdout_enabled(
            "zenoh::net=error",
            "zenoh::net",
            Level::WARN
        ));
        // Raising verbosity still works.
        assert!(stdout_enabled("zenoh=debug", "zenoh", Level::DEBUG));
        assert!(stdout_enabled("zenoh::net=info", "zenoh::net", Level::INFO));
    }

    #[test]
    fn log_file_path_keeps_dots_in_the_name() {
        let out = Path::new("out");
        assert_eq!(
            log_file_path(out, "dora-daemon-10.0.0.1"),
            out.join("dora-daemon-10.0.0.1.txt")
        );
        assert_eq!(
            log_file_path(out, "dora-daemon-robot.local"),
            out.join("dora-daemon-robot.local.txt")
        );
        assert_eq!(
            log_file_path(out, "dora-coordinator"),
            out.join("dora-coordinator.txt")
        );
    }

    #[test]
    fn exact_target_is_configured() {
        assert!(env_configures_target("zenoh=debug", "zenoh"));
        assert!(env_configures_target("zenoh", "zenoh"));
        // among several comma-separated directives
        assert!(env_configures_target(
            "info,dora_daemon=trace,zenoh=warn",
            "dora_daemon"
        ));
        // tolerant of surrounding whitespace
        assert!(env_configures_target("info, zenoh = trace", "zenoh"));
    }

    #[test]
    fn submodule_target_is_configured() {
        assert!(env_configures_target("zenoh::transport=trace", "zenoh"));
        assert!(env_configures_target(
            "dora_daemon::spawn=debug",
            "dora_daemon"
        ));
    }

    #[test]
    fn unrelated_substring_is_not_configured() {
        // the bug this replaced: a bare `contains` matched these and silently
        // dropped the intended default for the real target.
        assert!(!env_configures_target("zenoh_transport=trace", "zenoh"));
        assert!(!env_configures_target(
            "my_dora_daemon_helper=info",
            "dora_daemon"
        ));
        assert!(!env_configures_target("azenoh=trace", "zenoh"));
        assert!(!env_configures_target("info", "zenoh"));
        assert!(!env_configures_target("", "zenoh"));
    }
}
