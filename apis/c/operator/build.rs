use std::path::Path;

fn main() {
    dora_operator_api_types::generate_headers(Path::new("operator_api.h"))
        .expect("failed to create operator_api.h");
}
