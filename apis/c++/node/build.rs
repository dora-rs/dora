use std::{
    path::{Path, PathBuf},
    str::FromStr,
};

const CONFIG_TEMPLATE: &str = include_str!("../../c/node/cmake/dora-api-config.cmake.in");
const CONFIG_VERSION: &str = include_str!("../../c/node/cmake/dora-api-version.cmake.in");

const PACKAGE: &str = "dora-node-api-cxx";
const LIB_UNIX: &str = "libdora_node_api_cxx.a";
const LIB_WIN: &str = "dora_node_api_cxx.lib";

fn main() {
    let mut bridge_files = vec![PathBuf::from("src/lib.rs")];
    #[cfg(feature = "ros2-bridge")]
    bridge_files.extend(ros2::generate());

    let mut build = cxx_build::bridges(&bridge_files);
    build.flag("-std=c++20");
    build.flag_if_supported("-Wno-deprecated-declarations");
    println!("cargo:rerun-if-changed=src/lib.rs");

    // rename header files
    let src_dir = origin_dir();
    let target_dir = src_dir
        .parent()
        .expect("failed to get parent directory of source directory");

    let install_dir = if let Ok(target_path) = std::env::var("DORA_NODE_API_CXX_INSTALL") {
        PathBuf::from_str(&target_path).expect("failed to parse DORA_NODE_API_CXX_INSTALL path")
    } else {
        target_dir.join("install")
    };
    println!("cargo:rerun-if-env-changed=DORA_NODE_API_CXX_INSTALL");

    // recreate target dir
    if install_dir.exists() {
        std::fs::remove_dir_all(&install_dir).unwrap();
    }
    std::fs::create_dir(&install_dir).unwrap();

    std::fs::copy(
        src_dir.join("lib.rs.h"),
        install_dir.join("dora-node-api.h"),
    )
    .unwrap();
    std::fs::copy(
        src_dir.join("lib.rs.cc"),
        install_dir.join("dora-node-api.cc"),
    )
    .unwrap();

    #[cfg(feature = "ros2-bridge")]
    ros2::generate_ros2_message_header(&install_dir);

    // Generate cmake config files in cxxbridge directory
    let cxxbridge_crate_dir = src_dir
        .parent()
        .expect("failed to get cxxbridge crate directory");

    let cmake_dir = cxxbridge_crate_dir.join("lib/cmake").join(PACKAGE);
    let include_dir = cxxbridge_crate_dir.join("include");
    let src_cmake_dir = cxxbridge_crate_dir.join("src");

    std::fs::create_dir_all(&cmake_dir).expect("failed to create cmake directory");
    std::fs::create_dir_all(&include_dir).expect("failed to create include directory");
    std::fs::create_dir_all(&src_cmake_dir).expect("failed to create src directory");

    let version = env!("CARGO_PKG_VERSION");
    let target = compute_target();

    generate_config_cmake(&cmake_dir, &target);
    generate_config_version_cmake(&cmake_dir, version);
    copy_cxx_header(&src_dir, &include_dir);
    copy_cxx_source(&src_dir, &src_cmake_dir);

    // to avoid unnecessary `mut` warning
    bridge_files.clear();
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
        .replace("@LIB_WIN@", LIB_WIN)
        .replace(
            "@CXX_BRIDGE_FILES@",
            "${PACKAGE_PREFIX_DIR}/src/dora-node-api.cc",
        );
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
    let header_dst = include_dir.join("dora-node-api.h");
    std::fs::copy(&header_src, &header_dst).expect("failed to copy cxx header");
}

fn copy_cxx_source(src_dir: &Path, src_cmake_dir: &Path) {
    let source_src = src_dir.join("lib.rs.cc");
    let source_dst = src_cmake_dir.join("dora-node-api.cc");
    std::fs::copy(&source_src, &source_dst).expect("failed to copy cxx source");
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

#[cfg(feature = "ros2-bridge")]
mod ros2 {

    use std::{
        io::BufRead,
        path::{Component, Path, PathBuf},
    };

    pub fn generate() -> Vec<PathBuf> {
        use rust_format::Formatter;
        let paths = ament_prefix_paths();
        let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
        let generated = dora_ros2_bridge_msg_gen::generate(paths.as_slice(), &out_dir, true);
        let generated_string = rust_format::PrettyPlease::default()
            .format_tokens(generated)
            .unwrap();
        let target_file = out_dir.join("messages.rs");
        std::fs::write(&target_file, generated_string).unwrap();
        println!(
            "cargo:rustc-env=ROS2_BINDINGS_PATH={}",
            target_file.display()
        );
        let mut target_files: Vec<_> = out_dir
            .join("msg")
            .read_dir()
            .unwrap()
            .map(|entry| entry.unwrap().path())
            .collect();
        target_files.push(out_dir.join("impl.rs"));
        target_files
    }

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

    fn copy_dir_all(src: impl AsRef<Path>, dst: impl AsRef<Path>) -> std::io::Result<()> {
        use std::borrow::Borrow;
        use std::io::Write;
        std::fs::create_dir_all(&dst)?;
        for entry in std::fs::read_dir(src)? {
            let entry = entry?;
            let ty = entry.file_type()?;
            if ty.is_dir() {
                copy_dir_all(entry.path(), dst.as_ref().join(entry.file_name()))?;
            } else {
                let entry_path = entry.path();
                let entry_name = entry_path.file_name().unwrap().to_string_lossy();
                let entry_name: &str = entry_name.borrow();

                if entry_name.ends_with(".rs.cc") {
                    let entry_stem = entry_name.strip_suffix(".rs.cc").unwrap();
                    std::fs::copy(
                        entry.path(),
                        dst.as_ref().join(entry_stem).with_extension("cc"),
                    )?;
                } else if entry_name.ends_with(".rs.h") {
                    let entry_stem = entry_name.strip_suffix(".rs.h").unwrap();
                    let wrap_header = dst.as_ref().join(entry_stem).with_extension("h");
                    let mut wrap_file = std::fs::File::create(wrap_header)?;
                    wrap_file.write(
                        format!(
                            "
                            // Generated by Dora\n
                            #include \"./{}\"\n
                            ",
                            entry_name
                        )
                        .as_bytes(),
                    )?;

                    let origin_header = dst.as_ref().join(entry_name);
                    std::fs::copy(entry.path(), dst.as_ref().join(origin_header))?;
                }
            }
        }
        Ok(())
    }

    pub fn generate_ros2_message_header(target_path: &Path) {
        let default_target = std::env::var("CARGO_TARGET_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(|_| {
                let root = Path::new(env!("CARGO_MANIFEST_DIR"))
                    .ancestors()
                    .nth(3)
                    .expect("failed to get root directory from manifest path");
                root.join("target")
            });
        let out_dir_str = std::env::var("OUT_DIR").unwrap();
        let _out_dir = PathBuf::from(&out_dir_str);
        let relative_dir = PathBuf::from(out_dir_str.strip_prefix("/").unwrap());
        let header_path = default_target
            .join("cxxbridge")
            .join(PACKAGE)
            .join(&relative_dir);

        let target_path = target_path.join("ros2-bridge");

        copy_dir_all(&header_path, &target_path).unwrap();
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
}
