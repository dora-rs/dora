use std::fs;
use std::path::{Path, PathBuf};

const CONFIG_CMAKE: &str = include_str!("cmake/dora-node-api-cConfig.cmake.in");
const CONFIG_VERSION_CMAKE: &str = include_str!("cmake/dora-node-api-cConfigVersion.cmake.in");

fn main() {
    let target_dir = get_target_dir();
    let cmake_dir = target_dir.join("lib").join("cmake").join("dora-node-api-c");
    let include_dir = target_dir.join("include");

    fs::create_dir_all(&cmake_dir).expect("failed to create cmake directory");
    fs::create_dir_all(&include_dir).expect("failed to create include directory");

    let version = env!("CARGO_PKG_VERSION");

    generate_config_cmake(&cmake_dir);
    generate_config_version_cmake(&cmake_dir, version);
    copy_header(&include_dir);

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=cmake/dora-node-api-cConfig.cmake.in");
    println!("cargo:rerun-if-changed=cmake/dora-node-api-cConfigVersion.cmake.in");
    println!("cargo:rerun-if-changed=node_api.h");
}

fn get_target_dir() -> PathBuf {
    let profile = std::env::var("PROFILE").expect("PROFILE not set");

    // Get base target directory
    let base_target_dir = std::env::var("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            // Fallback: derive from manifest_dir
            let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
            manifest_dir
                .parent()
                .and_then(|p| p.parent())
                .and_then(|p| p.parent())
                .expect("failed to get workspace root")
                .join("target")
        });

    // Check if --target was specified
    if let Ok(build_target) = std::env::var("CARGO_BUILD_TARGET") {
        // With --target: target/<target>/<profile>
        base_target_dir.join(build_target).join(profile)
    } else {
        // Without --target: target/<profile>
        base_target_dir.join(profile)
    }
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
