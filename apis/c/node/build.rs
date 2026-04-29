use std::fs;
use std::path::{Path, PathBuf};

const CONFIG_TEMPLATE: &str = include_str!("cmake/dora-api-config.cmake.in");
const CONFIG_VERSION: &str = include_str!("cmake/dora-api-version.cmake.in");

const PACKAGE: &str = "dora-node-api-c";
const LIB_UNIX: &str = "libdora_node_api_c.a";
const LIB_WIN: &str = "dora_node_api_c.lib";

fn main() {
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").expect("OUT_DIR not set"));
    let cmake_dir = out_dir.join("lib").join("cmake").join(PACKAGE);
    let include_dir = out_dir.join("include");

    fs::create_dir_all(&cmake_dir).expect("failed to create cmake directory");
    fs::create_dir_all(&include_dir).expect("failed to create include directory");

    let version = env!("CARGO_PKG_VERSION");
    let target = compute_target();

    generate_config_cmake(&cmake_dir, &target);
    generate_config_version_cmake(&cmake_dir, version);
    copy_header(&include_dir);

    // Also stage artifacts next to the .a library in the target directory
    let target_dir = out_dir
        .ancestors()
        .nth(3)
        .expect("failed to derive target directory from OUT_DIR")
        .to_path_buf();
    let staging = target_dir.join(PACKAGE);
    copy_dir_contents(
        &out_dir.join("lib/cmake").join(PACKAGE),
        &staging.join("lib/cmake").join(PACKAGE),
    );
    copy_dir_contents(&out_dir.join("include"), &staging.join("include"));

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=cmake/dora-api-config.cmake.in");
    println!("cargo:rerun-if-changed=cmake/dora-api-version.cmake.in");
    println!("cargo:rerun-if-changed=node_api.h");
}

fn compute_target() -> String {
    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_else(|_| "unknown".into());
    let target_arch = std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_else(|_| "unknown".into());
    format!("{}-{}", target_os, target_arch)
}

fn generate_config_cmake(cmake_dir: &Path, target: &str) {
    let content = CONFIG_TEMPLATE
        .replace("@PACKAGE@", PACKAGE)
        .replace("@TARGET@", target)
        .replace("@LIB_UNIX@", LIB_UNIX)
        .replace("@LIB_WIN@", LIB_WIN);
    fs::write(cmake_dir.join(format!("{}Config.cmake", PACKAGE)), content)
        .expect("failed to write Config.cmake");
}

fn generate_config_version_cmake(cmake_dir: &Path, version: &str) {
    let content = CONFIG_VERSION.replace("@VERSION@", version);
    fs::write(
        cmake_dir.join(format!("{}ConfigVersion.cmake", PACKAGE)),
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
