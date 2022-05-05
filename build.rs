fn main() {
    capnpc::CompilerCommand::new()
        .src_prefix("schema")
        .file("schema/message.capnp")
        .output_path("src/message/")
        .default_parent_module(vec!["message".to_string()])
        .run()
        .expect("schema compiler command");
}
