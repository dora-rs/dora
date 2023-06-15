use std::path::Path;

fn main() {
    dora_operator_api_types::generate_headers(Path::new("operator_types.h"))
        .expect("failed to create operator_api.h");

    // don't rebuild on changes (otherwise we rebuild on every run as we're
    // writing the `operator_types.h` file; cargo will still rerun this script
    // when the `dora_operator_api_types` crate changes)
    println!("cargo:rerun-if-changed=");
}
