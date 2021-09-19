fn main() {
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");

    let ament_prefix_paths =
        std::env::var("AMENT_PREFIX_PATH").expect("$AMENT_PREFIX_PATH is supposed to be set.");
    for ament_prefix_path in ament_prefix_paths.split(':') {
        println!("cargo:rustc-link-search=native={}/lib", ament_prefix_path);
    }

    println!("cargo:rustc-link-lib=dylib=rcl");
    println!("cargo:rustc-link-lib=dylib=rcl_yaml_param_parser");
    println!("cargo:rustc-link-lib=dylib=rcutils");
    println!("cargo:rustc-link-lib=dylib=rmw");
    println!("cargo:rustc-link-lib=dylib=rosidl_runtime_c");
}
