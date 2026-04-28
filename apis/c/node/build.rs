use std::fs;
use std::path::{Path, PathBuf};

const CONFIG_CMAKE: &str = include_str!("cmake/dora-node-api-cConfig.cmake.in");
const CONFIG_VERSION_CMAKE: &str = include_str!("cmake/dora-node-api-cConfigVersion.cmake.in");

fn main() {
    let target_dir = get_target_dir();
    let lib_dir = target_dir.join("lib");
    let cmake_dir = lib_dir.join("cmake").join("dora-node-api-c");
    let include_dir = target_dir.join("include");

    fs::create_dir_all(&lib_dir).expect("failed to create lib directory");
    fs::create_dir_all(&cmake_dir).expect("failed to create cmake directory");
    fs::create_dir_all(&include_dir).expect("failed to create include directory");

    let version = env!("CARGO_PKG_VERSION");

    generate_config_cmake(&cmake_dir);
    generate_config_version_cmake(&cmake_dir, version);
    copy_header(&include_dir);
    link_library(&lib_dir);

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=cmake/dora-node-api-cConfig.cmake.in");
    println!("cargo:rerun-if-changed=cmake/dora-node-api-cConfigVersion.cmake.in");
    println!("cargo:rerun-if-changed=node_api.h");
}

fn get_target_dir() -> PathBuf {
    let base_target_dir = std::env::var("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
            manifest_dir
                .parent()
                .and_then(|p| p.parent())
                .and_then(|p| p.parent())
                .expect("failed to get workspace root")
                .join("target")
        });

    let profile = std::env::var("PROFILE").expect("PROFILE not set");

    if let Ok(target) = std::env::var("TARGET") {
        base_target_dir.join(target).join(profile)
    } else {
        base_target_dir.join(profile)
    }
}

fn generate_config_cmake(cmake_dir: &Path) {
    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_else(|_| "unknown".into());
    let target_arch = std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_else(|_| "unknown".into());
    let target = format!("{}-{}", target_os, target_arch);

    println!("cargo:warning=dora-node-api-c: built for {}", target);

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

#[cfg(unix)]
fn link_library(lib_dir: &Path) {
    let lib_symlink = lib_dir.join("libdora_node_api_c.a");
    let _ = fs::remove_file(&lib_symlink);
    std::os::unix::fs::symlink("../libdora_node_api_c.a", &lib_symlink)
        .expect("failed to create symlink");
}

#[cfg(windows)]
fn link_library(lib_dir: &Path) {
    let lib_symlink = lib_dir.join("dora_node_api_c.lib");
    let _ = fs::remove_file(&lib_symlink);
    let target_dir = lib_dir.parent().expect("failed to get parent");
    let lib_target = target_dir.join("dora_node_api_c.lib");
    if lib_target.exists() {
        fs::copy(&lib_target, &lib_symlink).expect("failed to copy library");
    }
}
