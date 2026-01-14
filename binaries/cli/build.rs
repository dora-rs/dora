fn main() {
    pyo3_build_config::add_extension_module_link_args();
    println!(
        "cargo:rustc-env=TARGET={}",
        std::env::var("TARGET").unwrap()
    );

    // Export dora-message version for use in --version output
    // This reads the version from the Cargo.toml metadata
    if let Ok(metadata) = std::env::var("DEP_DORA_MESSAGE_VERSION") {
        println!("cargo:rustc-env=DORA_MESSAGE_VERSION={}", metadata);
    } else {
        // Fallback: read from the dora-message Cargo.toml directly
        let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
        let message_toml_path =
            std::path::Path::new(&manifest_dir).join("../../libraries/message/Cargo.toml");

        if let Ok(contents) = std::fs::read_to_string(message_toml_path) {
            for line in contents.lines() {
                if line.trim().starts_with("version") && line.contains('=') {
                    if let Some(version) = line.split('=').nth(1) {
                        let version = version.trim().trim_matches('"').trim_matches('\'');
                        println!("cargo:rustc-env=DORA_MESSAGE_VERSION={}", version);
                        break;
                    }
                }
            }
        }
    }
}
