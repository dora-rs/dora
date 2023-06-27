use std::path::Path;

fn main() {
    let ament_prefix_path =
        std::env::var("AMENT_PREFIX_PATH").expect("$AMENT_PREFIX_PATH must be set");
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rustc-env=DETECTED_AMENT_PREFIX_PATH={ament_prefix_path}");

    let paths = ament_prefix_path
        .split(':')
        .map(Path::new)
        .collect::<Vec<_>>();
    for path in &paths {
        println!("cargo:rerun-if-changed={}", path.display());
    }
}
