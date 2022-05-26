//! Enable serialisation and deserialisation of capnproto messages
//!
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
