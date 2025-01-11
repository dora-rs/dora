use opentelemetry::propagation::Extractor;
use opentelemetry::{global, Context};
use opentelemetry::{Key, KeyValue, Value};
use opentelemetry_otlp::WithExportConfig;
use opentelemetry_otlp::{LogExporter, Protocol};
use opentelemetry_sdk::{logs::LoggerProvider, Resource};
use std::collections::HashMap;

struct MetadataMap<'a>(HashMap<&'a str, &'a str>);

impl<'a> Extractor for MetadataMap<'a> {
    /// Get a value for a key from the MetadataMap.  If the value can't be converted to &str, returns None
    fn get(&self, key: &str) -> Option<&str> {
        self.0.get(key).cloned()
    }

    /// Collect all the keys from the MetadataMap.
    fn keys(&self) -> Vec<&str> {
        self.0.keys().cloned().collect()
    }
}

/// Init opentelemetry tracing
///
/// Use the default exporter Jaeger as exporter with
/// - host: `172.17.0.1` which correspond to the docker address
/// - port: 6831 which is the default Jaeger port.
///
/// To launch the associated Jaeger docker container, launch the
/// following command:
/// ```bash
/// docker run -d -p 6831:6831/udp -p 6832:6832/udp -p 16686:16686 -p 14268:14268 jaegertracing/all-in-one:latest
/// ```
///
/// TODO: Make Jaeger configurable
///
pub fn init_jaeger_tracing(
    name: String,
    endpoint: &str,
) -> Result<LoggerProvider, opentelemetry_sdk::logs::LogError> {
    let exporter = LogExporter::builder()
        .with_http()
        .with_endpoint(endpoint)
        .with_protocol(Protocol::HttpBinary)
        .build()?;

    Ok(LoggerProvider::builder()
        .with_simple_exporter(exporter)
        .with_resource(Resource::new([KeyValue::new(
            Key::new("name"),
            Value::String(name.into()),
        )]))
        .build())
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
