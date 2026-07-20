use eyre::Context as _;
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
pub fn init_tracing(_name: &str, endpoint: &str) -> eyre::Result<sdktrace::SdkTracerProvider> {
    let exporter = opentelemetry_otlp::SpanExporter::builder()
        .with_tonic()
        .with_endpoint(endpoint)
        .build()
        .wrap_err("failed to build OTLP span exporter")?;

    Ok(SdkTracerProvider::builder()
        .with_batch_exporter(exporter)
        .build())
}

/// Serialize the trace context (trace ID, span ID) into a compact string.
/// Only W3C TraceContext keys (`traceparent`, `tracestate`) are included.
/// OTel Baggage keys are stripped to prevent sensitive data from leaking
/// across node boundaries in the dataflow.
/// Separator between entries in the serialized trace context.
///
/// The previous format joined entries with `;` and split key/value on `:`,
/// but both bytes are legal inside a W3C `tracestate` *value*: the spec allows
/// the byte ranges `0x20-0x2B`, `0x2D-0x3C`, `0x3E-0x7E`, which include `:`
/// (`0x3A`) and `;` (`0x3B`). A vendor `tracestate` such as `vendor=a;b` was
/// therefore split into two bogus entries. Newline (`0x0A`) is a control byte
/// that can never appear in a `traceparent`/`tracestate` value, so it is a
/// safe entry separator; the key/value split uses only the first `:`.
const CONTEXT_ENTRY_SEP: char = '\n';

pub fn serialize_context(context: &Context) -> String {
    let mut map = HashMap::new();
    global::get_text_map_propagator(|propagator| propagator.inject_context(context, &mut map));
    // Strip baggage to avoid propagating sensitive data across nodes
    map.remove("baggage");
    let mut string_context = String::new();
    for (k, v) in map.iter() {
        string_context.push_str(k);
        string_context.push(':');
        string_context.push_str(v);
        string_context.push(CONTEXT_ENTRY_SEP);
    }
    string_context
}

pub fn deserialize_context(string_context: &str) -> Context {
    let map = MetadataMap(deserialize_to_hashmap(string_context));
    global::get_text_map_propagator(|prop| prop.extract(&map))
}

pub fn deserialize_to_hashmap(string_context: &str) -> HashMap<&str, &str> {
    let mut map = HashMap::new();
    for entry in string_context.split(CONTEXT_ENTRY_SEP) {
        // Skip the trailing empty segment left after the final separator (and
        // an empty input), which would otherwise inject a bogus `""` key.
        if entry.is_empty() {
            continue;
        }
        // Split on the first `:` only: the key (`traceparent`/`tracestate`)
        // never contains one, but a `tracestate` value legitimately can.
        let (key, value) = entry.split_once(':').unwrap_or((entry, ""));
        map.insert(key, value);
    }
    map
}

#[cfg(test)]
mod tests {
    use super::{deserialize_context, deserialize_to_hashmap, serialize_context};

    #[test]
    fn round_trip_context_preserves_multi_member_tracestate_delimiters() {
        use opentelemetry::Context;
        use opentelemetry::trace::{
            SpanContext, SpanId, TraceContextExt, TraceFlags, TraceId, TraceState,
        };
        use opentelemetry_sdk::propagation::TraceContextPropagator;

        // `serialize_context`/`deserialize_context` route through the process-global
        // text-map propagator, exactly as production does; install the same W3C
        // TraceContext propagator the daemon/runtime rely on.
        opentelemetry::global::set_text_map_propagator(TraceContextPropagator::new());

        // A realistic multi-member W3C `tracestate`. Members are joined with `,`
        // and each key/value pair with `=`; the `vendorb` value additionally
        // embeds `:` and `;`, both legal inside a value (W3C spec) and exactly
        // the bytes the old `;`-joined / `:`-split codec corrupted.
        let trace_state = TraceState::from_key_value(vec![
            ("vendora", "t61rcwkgmze"),
            ("vendorb", "00f067aa0ba902b7:child;sampled"),
        ])
        .expect("valid W3C tracestate members");
        let span_context = SpanContext::new(
            TraceId::from_hex("0af7651916cd43dd8448eb211c80319c").unwrap(),
            SpanId::from_hex("b7ad6b7169203331").unwrap(),
            TraceFlags::SAMPLED,
            true,
            trace_state.clone(),
        );
        let context = Context::new().with_remote_span_context(span_context.clone());

        // Encode (the path no prior test exercised) then decode.
        let serialized = serialize_context(&context);
        let map = deserialize_to_hashmap(&serialized);

        // `traceparent` carries the ids; `tracestate` must survive byte-for-byte.
        assert_eq!(
            map.get("traceparent").copied(),
            Some("00-0af7651916cd43dd8448eb211c80319c-b7ad6b7169203331-01"),
        );
        // Derive the expected `tracestate` from the source of truth so the
        // assertion is independent of member ordering.
        let expected_tracestate = trace_state.header();
        assert!(
            [',', '=', ':', ';']
                .iter()
                .all(|d| expected_tracestate.contains(*d)),
            "fixture must exercise all four delimiters: {expected_tracestate}",
        );
        assert_eq!(
            map.get("tracestate").copied(),
            Some(expected_tracestate.as_str())
        );
        assert_eq!(map.len(), 2, "no spurious entries expected: {map:?}");

        // Full round-trip: decoding the serialized form reconstructs the
        // original span context, delimiters intact and no truncation.
        let recovered = deserialize_context(&serialized);
        let recovered = recovered.span().span_context().clone();
        assert_eq!(recovered.trace_id(), span_context.trace_id());
        assert_eq!(recovered.span_id(), span_context.span_id());
        assert_eq!(recovered.trace_state().header(), trace_state.header());
    }

    #[test]
    fn deserialize_preserves_tracestate_with_colon_and_semicolon() {
        // A `tracestate` value may legally contain `:` and `;` (W3C spec).
        // The decoder must keep the whole value intact and must not split it
        // into extra bogus entries.
        let serialized = "traceparent:00-0af7651916cd43dd8448eb211c80319c-b7ad6b7169203331-01\n\
                          tracestate:vendor=a;b:c\n";
        let map = deserialize_to_hashmap(serialized);
        assert_eq!(
            map.get("traceparent"),
            Some(&"00-0af7651916cd43dd8448eb211c80319c-b7ad6b7169203331-01")
        );
        assert_eq!(map.get("tracestate"), Some(&"vendor=a;b:c"));
        assert_eq!(map.len(), 2, "no spurious entries expected: {map:?}");
    }

    #[test]
    fn deserialize_empty_input_yields_empty_map() {
        // The previous `split(';')` produced a single `{"": ""}` entry for an
        // empty string; the decoder should yield an empty map instead.
        assert!(deserialize_to_hashmap("").is_empty());
    }

    #[test]
    fn deserialize_entry_without_value_defaults_to_empty() {
        let map = deserialize_to_hashmap("traceparent\n");
        assert_eq!(map.get("traceparent"), Some(&""));
        assert_eq!(map.len(), 1);
    }
}
