#[cfg(feature = "generate-messages")]
use std::path::PathBuf;

#[cfg(not(feature = "generate-messages"))]
fn main() {}

#[cfg(feature = "generate-messages")]
fn main() {
    use rust_format::Formatter;
    let paths = ament_prefix_paths();
    let generated = dora_ros2_bridge_msg_gen::gen(paths.as_slice(), false);
    let generated_string = rust_format::PrettyPlease::default()
        .format_tokens(generated)
        .unwrap();
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let target_file = out_dir.join("messages.rs");
    std::fs::write(&target_file, generated_string).unwrap();
    println!("cargo:rustc-env=MESSAGES_PATH={}", target_file.display());
}

#[cfg(feature = "generate-messages")]
fn ament_prefix_paths() -> Vec<PathBuf> {
    let ament_prefix_path: String = match std::env::var("AMENT_PREFIX_PATH") {
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

    let paths: Vec<_> = ament_prefix_path.split(':').map(PathBuf::from).collect();
    for path in &paths {
        println!("cargo:rerun-if-changed={}", path.display());
    }

    paths
}
