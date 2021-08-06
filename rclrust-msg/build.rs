fn main() {
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");

    let ament_prefix_paths =
        std::env::var("AMENT_PREFIX_PATH").expect("$AMENT_PREFIX_PATH is supposed to be set.");
    for ament_prefix_path in ament_prefix_paths.split(':') {
        println!("cargo:rustc-link-search=native={}/lib", ament_prefix_path);
    }
}
