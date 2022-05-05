pub mod message_capnp;

pub fn serialize_message(data: &[u8]) -> Vec<u8> {
    let mut meta_builder = capnp::message::Builder::new_default();
    // And now we can set up the actual message we're trying to create
    let mut metadata = meta_builder.init_root::<message_capnp::metadata::Builder>();
    metadata.set_watermark(1u64);

    let mut builder = capnp::message::Builder::new_default();
    let mut message = builder.init_root::<message_capnp::message::Builder>();
    message.set_data(data);
    message.set_metadata(metadata.into_reader()).unwrap();

    let mut buffer = Vec::new();
    capnp::serialize::write_message(&mut buffer, &builder).unwrap();
    buffer
}

pub fn deserialize_message(binary: Vec<u8>) -> Vec<u8> {
    let deserialized = capnp::serialize::read_message(
        &mut binary.as_slice(),
        capnp::message::ReaderOptions::new(),
    )
    .unwrap();
    let message = deserialized
        .get_root::<message_capnp::message::Reader>()
        .unwrap();
    let _metadata = message.get_metadata().unwrap();
    message.get_data().unwrap().to_owned()
}
