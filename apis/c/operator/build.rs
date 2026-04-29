use std::fs;
use std::path::{Path, PathBuf};

const CONFIG_TEMPLATE: &str = include_str!("../node/cmake/dora-api-config.cmake.in");
const CONFIG_VERSION: &str = include_str!("../node/cmake/dora-api-version.cmake.in");

const PACKAGE: &str = "dora-operator-api-c";
const LIB_UNIX: &str = "libdora_operator_api_c.a";
const LIB_WIN: &str = "dora_operator_api_c.lib";

fn main() {
    dora_operator_api_types::generate_headers(Path::new("operator_types.h"))
        .expect("failed to create operator_types.h");

    let out_dir = PathBuf::from(std::env::var("OUT_DIR").expect("OUT_DIR not set"));
    let cmake_dir = out_dir.join("lib").join("cmake").join(PACKAGE);
    let include_dir = out_dir.join("include");

    fs::create_dir_all(&cmake_dir).expect("failed to create cmake directory");
    fs::create_dir_all(&include_dir).expect("failed to create include directory");

    let version = env!("CARGO_PKG_VERSION");
    let target = compute_target();

    generate_config_cmake(&cmake_dir, &target);
    generate_config_version_cmake(&cmake_dir, version);
    copy_headers(&include_dir);

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

    // don't rebuild on changes (otherwise we rebuild on every run as we're
    // writing the `operator_types.h` file; cargo will still rerun this script
    // when the `dora_operator_api_types` crate changes)
    println!("cargo:rerun-if-changed=build.rs");
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

fn copy_headers(include_dir: &Path) {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));

    let types_src = manifest_dir.join("operator_types.h");
    let types_dst = include_dir.join("operator_types.h");
    fs::copy(&types_src, &types_dst).expect("failed to copy operator_types.h");

    let api_src = manifest_dir.join("operator_api.h");
    let api_dst = include_dir.join("operator_api.h");
    fs::copy(&api_src, &api_dst).expect("failed to copy operator_api.h");
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
