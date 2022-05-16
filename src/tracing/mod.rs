use std::collections::HashMap;

use opentelemetry::{global, Context};

use opentelemetry::propagation::Extractor;

use opentelemetry::sdk::{propagation::TraceContextPropagator, trace as sdktrace};
use opentelemetry::trace::TraceError;

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

pub fn tracing_init() -> Result<sdktrace::Tracer, TraceError> {
    global::set_text_map_propagator(TraceContextPropagator::new());
    opentelemetry_jaeger::new_agent_pipeline()
        .with_endpoint("172.17.0.1:6831")
        .with_service_name("test-client")
        .install_batch(opentelemetry::runtime::Tokio)
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
    let mut map = MetadataMap(HashMap::new());
    for s in string_context.split(';') {
        let mut values = s.split(':');
        let key = values.next().unwrap();
        let value = values.next().unwrap_or("");
        map.0.insert(key, value);
    }
    global::get_text_map_propagator(|prop| prop.extract(&map))
}
