//! Enable serialisation and deserialisation of capnproto messages
//!
pub mod message_capnp {
    include!(concat!(env!("OUT_DIR"), "/message_capnp.rs"));
}

/// Helper function to serialize message
pub fn serialize_message(data: &[u8], otel_context: &str) -> Vec<u8> {
    // Build the metadata
    let mut meta_builder = capnp::message::Builder::new_default();
    let mut metadata = meta_builder.init_root::<message_capnp::metadata::Builder>();
    metadata.set_otel_context(otel_context);

    // Build the data of the message
    let mut builder = capnp::message::Builder::new_default();
    let mut message = builder.init_root::<message_capnp::message::Builder>();
    message.set_data(data);
    message.set_metadata(metadata.into_reader()).unwrap();

    let mut buffer = Vec::new();
    capnp::serialize::write_message(&mut buffer, &builder).unwrap();
    buffer
}
