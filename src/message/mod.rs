use std::collections::HashMap;

use opentelemetry::{global, Context};

use opentelemetry::propagation::Extractor;

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
pub mod message_capnp;

pub fn serialize_message(data: &[u8], otel_context: &str, depth: u32) -> Vec<u8> {
    let mut meta_builder = capnp::message::Builder::new_default();
    // And now we can set up the actual message we're trying to create
    let mut metadata = meta_builder.init_root::<message_capnp::metadata::Builder>();
    metadata.set_otel_context(otel_context);
    metadata.set_depth(depth);

    let mut builder = capnp::message::Builder::new_default();
    let mut message = builder.init_root::<message_capnp::message::Builder>();
    message.set_data(data);
    message.set_metadata(metadata.into_reader()).unwrap();

    let mut buffer = Vec::new();
    capnp::serialize::write_message(&mut buffer, &builder).unwrap();
    buffer
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
