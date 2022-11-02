fn main() {
    build_runtime();

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=schema/message.capnp");
}

fn build_runtime() {
    capnpc::CompilerCommand::new()
        .src_prefix("schema")
        .file("schema/message.capnp")
        .output_path("src")
        .run()
        .expect("schema compiler command");
}
