fn main() {
    #[cfg(feature = "python")]
    pyo3_build_config::add_extension_module_link_args();
    println!(
        "cargo:rustc-env=TARGET={}",
        std::env::var("TARGET").unwrap()
    );
}
