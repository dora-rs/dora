use std::path::{Component, Path, PathBuf};

fn main() {
    let _build = cxx_build::bridge("src/lib.rs");
    println!("cargo:rerun-if-changed=src/lib.rs");

    if cfg!(feature = "ros2-bridge") {
        generate_ros2_message_header();
    }
}

fn generate_ros2_message_header() {
    let prefix = std::env::var("DEP_DORA_ROS2_BRIDGE_CXXBRIDGE_PREFIX").unwrap();
    let include_dir = PathBuf::from(std::env::var("DEP_DORA_ROS2_BRIDGE_CXXBRIDGE_DIR0").unwrap());
    let _crate_dir = std::env::var("DEP_DORA_ROS2_BRIDGE_CXXBRIDGE_DIR1").unwrap();

    let header_path = include_dir
        .join(&prefix)
        .join(local_relative_path(&include_dir))
        .ancestors()
        .nth(2)
        .unwrap()
        .join("messages.rs.h");
    let code_path = include_dir
        .parent()
        .unwrap()
        .join("sources")
        .join(&prefix)
        .join(local_relative_path(&include_dir))
        .ancestors()
        .nth(2)
        .unwrap()
        .join("messages.rs.cc");

    // copy message files to target directory
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(3)
        .unwrap();
    let target_path = root
        .join("target")
        .join("cxxbridge")
        .join("dora-node-api-cxx")
        .join("src")
        .join("messages.rs.h");

    std::fs::copy(&header_path, &target_path).unwrap();
    println!("cargo:rerun-if-changed={}", header_path.display());
    std::fs::copy(&code_path, target_path.with_file_name("messages.rs.cc")).unwrap();
    println!("cargo:rerun-if-changed={}", code_path.display());
}

// copy from cxx-build source
fn local_relative_path(path: &Path) -> PathBuf {
    let mut rel_path = PathBuf::new();
    for component in path.components() {
        match component {
            Component::Prefix(_) | Component::RootDir | Component::CurDir => {}
            Component::ParentDir => drop(rel_path.pop()), // noop if empty
            Component::Normal(name) => rel_path.push(name),
        }
    }
    rel_path
}
