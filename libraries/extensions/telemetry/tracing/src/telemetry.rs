use opentelemetry::propagation::Extractor;
use opentelemetry::{Context, global};
use opentelemetry_otlp::WithExportConfig;

use std::collections::HashMap;

use opentelemetry_sdk::trace::{self as sdktrace, SdkTracerProvider};

struct MetadataMap<'a>(HashMap<&'a str, &'a str>);

impl Extractor for MetadataMap<'_> {
    /// Get a value for a key from the MetadataMap.  If the value can't be converted to &str, returns None
    fn get(&self, key: &str) -> Option<&str> {
        self.0.get(key).cloned()
    }

    /// Collect all the keys from the MetadataMap.
    fn keys(&self) -> Vec<&str> {
        self.0.keys().cloned().collect()
    }
}

/// Init opentelemetry tracing with OTLP exporter
///
/// Uses the OpenTelemetry Protocol (OTLP) to export traces to any compatible backend:
/// - Jaeger (via OTLP receiver on port 4317/4318)
/// - Zipkin
/// - Tempo
/// - OpenTelemetry Collector
/// - Any other OTLP-compatible backend
///
/// # Arguments
/// * `name` - Service name for the traces
/// * `endpoint` - OTLP endpoint (e.g., "http://localhost:4317")
///
/// # Example
/// To use with Jaeger via OTLP:
/// ```bash
/// docker run -d -p 4317:4317 -p 4318:4318 -p 16686:16686 jaegertracing/all-in-one:latest
/// ```
///
pub fn init_tracing(name: &str, endpoint: &str) -> sdktrace::SdkTracerProvider {
    let exporter = opentelemetry_otlp::SpanExporter::builder()
        .with_tonic()
        .with_endpoint(endpoint)
        .build()
        .unwrap();

    SdkTracerProvider::builder()
        // Customize sampling strategy
        .with_batch_exporter(exporter)
        .build()
}

/// Legacy function name for backward compatibility
#[deprecated(since = "0.3.14", note = "Use `init_tracing` instead")]
pub fn init_jaeger_tracing(name: &str, endpoint: &str) -> sdktrace::SdkTracerProvider {
    init_tracing(name, endpoint)
}

pub fn serialize_context(context: &Context) -> String {
    let mut map = HashMap::new();
    global::get_text_map_propagator(|propagator| propagator.inject_context(context, &mut map));
    let mut string_context = String::new();
    for (k, v) in map.iter() {
        string_context.push_str(k);
        string_context.push(':');
        string_context.push_str(v);
        string_context.push(';');
    }
    string_context
}

pub fn deserialize_context(string_context: &str) -> Context {
    let map = MetadataMap(deserialize_to_hashmap(string_context));
    global::get_text_map_propagator(|prop| prop.extract(&map))
}

pub fn deserialize_to_hashmap(string_context: &str) -> HashMap<&str, &str> {
    let mut map = HashMap::new();
    for s in string_context.split(';') {
        let mut values = s.split(':');
        let key = values.next().unwrap();
        let value = values.next().unwrap_or("");
        map.insert(key, value);
    }
    map
}
