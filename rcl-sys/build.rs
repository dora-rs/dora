use std::path::PathBuf;

use bindgen::{Builder, EnumVariation};

fn main() {
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rerun-if-changed=./src/wrapper.h");

    let mut builder = Builder::default()
        .header("src/wrapper.h")
        .size_t_is_usize(true)
        .derive_copy(false)
        .default_enum_style(EnumVariation::Rust {
            non_exhaustive: false,
        });

    let ament_prefix_paths =
        std::env::var("AMENT_PREFIX_PATH").expect("$AMENT_PREFIX_PATH is supposed to be set.");
    for ament_prefix_path in ament_prefix_paths.split(':') {
        builder = builder.clang_arg(format!("-I{}/include", ament_prefix_path));
        println!("cargo:rustc-link-search=native={}/lib", ament_prefix_path);
    }

    println!("cargo:rustc-link-lib=dylib=rcl");
    println!("cargo:rustc-link-lib=dylib=rcutils");
    println!("cargo:rustc-link-lib=dylib=rmw");

    let out_path = PathBuf::from(std::env::var("OUT_DIR").unwrap()).join("bindings.rs");
    builder
        .allowlist_type("rcl_.*")
        .allowlist_type("rmw_.*")
        .allowlist_type("rcutils_.*")
        .allowlist_type("RCUTILS_.*")
        .allowlist_function("rcl_.*")
        .allowlist_function("rmw_.*")
        .allowlist_function("rcutils_.*")
        .allowlist_var("RCL_.*")
        .allowlist_var("RMW_.*")
        .allowlist_var("RCUTILS_.*")
        .allowlist_var("g_rcutils_.*")
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file(out_path)
        .expect("Couldn't write bindings!");
}
