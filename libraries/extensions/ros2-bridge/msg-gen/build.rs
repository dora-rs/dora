use std::path::Path;

fn main() {
    let ament_prefix_path = match std::env::var("AMENT_PREFIX_PATH") {
        Ok(path) => path,
        Err(std::env::VarError::NotPresent) => {
            println!("cargo:warning='AMENT_PREFIX_PATH not set'");
            String::new()
        }
        Err(std::env::VarError::NotUnicode(s)) => {
            panic!(
                "AMENT_PREFIX_PATH is not valid unicode: `{}`",
                s.to_string_lossy()
            );
        }
    };
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");

    let paths = ament_prefix_path.split(':').map(Path::new);
    for path in paths {
        println!("cargo:rerun-if-changed={}", path.display());
    }

    println!("cargo:rustc-env=DETECTED_AMENT_PREFIX_PATH={ament_prefix_path}");
}
