use std::path::{Path, PathBuf};

const CONFIG_TEMPLATE: &str = include_str!("../../c/node/cmake/dora-api-config.cmake.in");
const CONFIG_VERSION: &str = include_str!("../../c/node/cmake/dora-api-version.cmake.in");

const PACKAGE: &str = "dora-operator-api-cxx";
const LIB_UNIX: &str = "libdora_operator_api_cxx.a";
const LIB_WIN: &str = "dora_operator_api_cxx.lib";

fn main() {
    let _ = cxx_build::bridge("src/lib.rs");
    println!("cargo:rerun-if-changed=src/lib.rs");

    let src_dir = origin_dir();
    let cxxbridge_crate_dir = src_dir
        .parent()
        .expect("failed to get cxxbridge crate directory");

    let cmake_dir = cxxbridge_crate_dir.join("lib/cmake").join(PACKAGE);
    let include_dir = cxxbridge_crate_dir.join("include");

    std::fs::create_dir_all(&cmake_dir).expect("failed to create cmake directory");
    std::fs::create_dir_all(&include_dir).expect("failed to create include directory");

    let version = env!("CARGO_PKG_VERSION");
    let target = compute_target();

    generate_config_cmake(&cmake_dir, &target);
    generate_config_version_cmake(&cmake_dir, version);
    copy_cxx_header(&src_dir, &include_dir);
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
    std::fs::write(cmake_dir.join(format!("{}Config.cmake", PACKAGE)), content)
        .expect("failed to write Config.cmake");
}

fn generate_config_version_cmake(cmake_dir: &Path, version: &str) {
    let content = CONFIG_VERSION.replace("@VERSION@", version);
    std::fs::write(
        cmake_dir.join(format!("{}ConfigVersion.cmake", PACKAGE)),
        content,
    )
    .expect("failed to write ConfigVersion.cmake");
}

fn copy_cxx_header(src_dir: &Path, include_dir: &Path) {
    let header_src = src_dir.join("lib.rs.h");
    let header_dst = include_dir.join("dora-operator-api.h");
    std::fs::copy(&header_src, &header_dst).expect("failed to copy cxx header");
}

fn origin_dir() -> PathBuf {
    let default_target = std::env::var("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            let root = Path::new(env!("CARGO_MANIFEST_DIR"))
                .ancestors()
                .nth(3)
                .expect("failed to get root directory from manifest path");
            root.join("target")
        });
    let cross_target = default_target
        .join(std::env::var("TARGET").unwrap())
        .join("cxxbridge")
        .join(PACKAGE)
        .join("src");

    if cross_target.exists() {
        cross_target
    } else {
        default_target.join("cxxbridge").join(PACKAGE).join("src")
    }
}
