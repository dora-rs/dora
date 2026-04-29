use std::fs;
use std::path::{Path, PathBuf};

const CONFIG_CMAKE: &str = include_str!("cmake/dora-node-api-cConfig.cmake.in");
const CONFIG_VERSION_CMAKE: &str = include_str!("cmake/dora-node-api-cConfigVersion.cmake.in");

fn main() {
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").expect("OUT_DIR not set"));
    let cmake_dir = out_dir.join("lib").join("cmake").join("dora-node-api-c");
    let include_dir = out_dir.join("include");

    fs::create_dir_all(&cmake_dir).expect("failed to create cmake directory");
    fs::create_dir_all(&include_dir).expect("failed to create include directory");

    let version = env!("CARGO_PKG_VERSION");

    generate_config_cmake(&cmake_dir);
    generate_config_version_cmake(&cmake_dir, version);
    copy_header(&include_dir);

    // Also stage artifacts next to the .a library in the target directory
    let target_dir = out_dir
        .ancestors()
        .nth(3)
        .expect("failed to derive target directory from OUT_DIR")
        .to_path_buf();
    let staging = target_dir.join("dora-node-api-c");
    copy_dir_contents(
        &out_dir.join("lib/cmake/dora-node-api-c"),
        &staging.join("lib/cmake/dora-node-api-c"),
    );
    copy_dir_contents(&out_dir.join("include"), &staging.join("include"));

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=cmake/dora-node-api-cConfig.cmake.in");
    println!("cargo:rerun-if-changed=cmake/dora-node-api-cConfigVersion.cmake.in");
    println!("cargo:rerun-if-changed=node_api.h");
}

fn generate_config_cmake(cmake_dir: &Path) {
    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_else(|_| "unknown".into());
    let target_arch = std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_else(|_| "unknown".into());
    let target = format!("{}-{}", target_os, target_arch);

    let content = CONFIG_CMAKE.replace("@TARGET@", &target);
    fs::write(cmake_dir.join("dora-node-api-cConfig.cmake"), content)
        .expect("failed to write Config.cmake");
}

fn generate_config_version_cmake(cmake_dir: &Path, version: &str) {
    let content = CONFIG_VERSION_CMAKE.replace("@VERSION@", version);
    fs::write(
        cmake_dir.join("dora-node-api-cConfigVersion.cmake"),
        content,
    )
    .expect("failed to write ConfigVersion.cmake");
}

fn copy_header(include_dir: &Path) {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let header_src = manifest_dir.join("node_api.h");
    let header_dst = include_dir.join("node_api.h");
    fs::copy(&header_src, &header_dst).expect("failed to copy header");
}

fn copy_dir_contents(src: &Path, dst: &Path) {
    fs::create_dir_all(dst).expect("failed to create staging directory");
    for entry in fs::read_dir(src).expect("failed to read source directory") {
        let entry = entry.expect("failed to read entry");
        let file_type = entry.file_type().expect("failed to get file type");
        let dst_path = dst.join(entry.file_name());
        if file_type.is_dir() {
            copy_dir_contents(&entry.path(), &dst_path);
        } else {
            fs::copy(entry.path(), &dst_path).expect("failed to copy file");
        }
    }
}
