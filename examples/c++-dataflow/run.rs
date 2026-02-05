use eyre::{Context, bail};
use std::{
    env::consts::{DLL_PREFIX, DLL_SUFFIX, EXE_SUFFIX},
    path::Path,
};

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let target = root.join("target");
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    std::fs::create_dir_all("build")?;
    let build_dir = Path::new("build");

    build_package("dora-node-api-cxx")?;
    let node_cxxbridge = target
        .join("cxxbridge")
        .join("dora-node-api-cxx")
        .join("src");
    std::fs::copy(
        node_cxxbridge.join("lib.rs.cc"),
        build_dir.join("node-bridge.cc"),
    )?;
    std::fs::copy(
        node_cxxbridge.join("lib.rs.h"),
        build_dir.join("dora-node-api.h"),
    )?;
    std::fs::write(
        build_dir.join("operator.h"),
        r###"#include "../operator-rust-api/operator.h""###,
    )?;

    build_package("dora-operator-api-cxx")?;
    let operator_cxxbridge = target
        .join("cxxbridge")
        .join("dora-operator-api-cxx")
        .join("src");
    std::fs::copy(
        operator_cxxbridge.join("lib.rs.cc"),
        build_dir.join("operator-bridge.cc"),
    )?;
    std::fs::copy(
        operator_cxxbridge.join("lib.rs.h"),
        build_dir.join("dora-operator-api.h"),
    )?;

    build_package("dora-node-api-c")?;
    build_package("dora-operator-api-c")?;
    build_cxx_node(
        root,
        &[
            &dunce::canonicalize(Path::new("node-rust-api").join("main.cc"))?,
            &dunce::canonicalize(build_dir.join("node-bridge.cc"))?,
        ],
        "node_rust_api",
        &["-l", "dora_node_api_cxx"],
    )?;
    build_cxx_node(
        root,
        &[&dunce::canonicalize(
            Path::new("node-c-api").join("main.cc"),
        )?],
        "node_c_api",
        &["-l", "dora_node_api_c"],
    )?;
    build_cxx_operator(
        &[
            &dunce::canonicalize(Path::new("operator-rust-api").join("operator.cc"))?,
            &dunce::canonicalize(build_dir.join("operator-bridge.cc"))?,
        ],
        "operator_rust_api",
        &[
            "-l",
            "dora_operator_api_cxx",
            "-L",
            root.join("target").join("debug").to_str().unwrap(),
        ],
    )?;
    build_cxx_operator(
        &[&dunce::canonicalize(
            Path::new("operator-c-api").join("operator.cc"),
        )?],
        "operator_c_api",
        &[
            "-l",
            "dora_operator_api_c",
            "-L",
            root.join("target").join("debug").to_str().unwrap(),
        ],
    )?;

    build_package("dora-runtime")?;

    dora_cli::run("dataflow.yml".to_string(), false)?;

    Ok(())
}

fn build_package(package: &str) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = std::process::Command::new(&cargo);
    cmd.arg("build");
    cmd.arg("--package").arg(package);
    if !cmd.status()?.success() {
        bail!("failed to build {package}");
    };
    Ok(())
}

fn build_cxx_node(root: &Path, paths: &[&Path], out_name: &str, args: &[&str]) -> eyre::Result<()> {
    let mut clang = std::process::Command::new("clang++");
    clang.args(paths);
    clang.arg("-std=c++20");
    #[cfg(target_os = "linux")]
    {
        clang.arg("-l").arg("m");
        clang.arg("-l").arg("rt");
        clang.arg("-l").arg("dl");
        clang.arg("-l").arg("z");
        clang.arg("-pthread");
    }
    #[cfg(target_os = "windows")]
    {
        clang.arg("-ladvapi32");
        clang.arg("-luserenv");
        clang.arg("-lkernel32");
        clang.arg("-lws2_32");
        clang.arg("-lbcrypt");
        clang.arg("-lncrypt");
        clang.arg("-lschannel");
        clang.arg("-lntdll");
        clang.arg("-liphlpapi");

        clang.arg("-lcfgmgr32");
        clang.arg("-lcredui");
        clang.arg("-lcrypt32");
        clang.arg("-lcryptnet");
        clang.arg("-lfwpuclnt");
        clang.arg("-lgdi32");
        clang.arg("-lmsimg32");
        clang.arg("-lmswsock");
        clang.arg("-lole32");
        clang.arg("-loleaut32");
        clang.arg("-lopengl32");
        clang.arg("-lsecur32");
        clang.arg("-lshell32");
        clang.arg("-lsynchronization");
        clang.arg("-luser32");
        clang.arg("-lwinspool");

        clang.arg("-Wl,-nodefaultlib:libcmt");
        clang.arg("-D_DLL");
        clang.arg("-lmsvcrt");
    }
    #[cfg(target_os = "macos")]
    {
        clang.arg("-framework").arg("CoreServices");
        clang.arg("-framework").arg("Security");
        clang.arg("-l").arg("System");
        clang.arg("-l").arg("resolv");
        clang.arg("-l").arg("pthread");
        clang.arg("-l").arg("c");
        clang.arg("-l").arg("m");
    }
    clang.args(args);
    clang.arg("-L").arg(root.join("target").join("debug"));
    clang
        .arg("--output")
        .arg(Path::new("../build").join(format!("{out_name}{EXE_SUFFIX}")));
    if let Some(parent) = paths[0].parent() {
        clang.current_dir(parent);
    }

    if !clang.status()?.success() {
        bail!("failed to compile c++ node");
    };
    Ok(())
}

fn build_cxx_operator(paths: &[&Path], out_name: &str, link_args: &[&str]) -> eyre::Result<()> {
    let mut object_file_paths = Vec::new();

    for path in paths {
        let mut compile = std::process::Command::new("clang++");
        compile.arg("-c").arg(path);
        compile.arg("-std=c++20");
        let object_file_path = path.with_extension("o");
        compile.arg("-o").arg(&object_file_path);
        #[cfg(unix)]
        compile.arg("-fPIC");
        #[cfg(target_os = "windows")]
        {
            // Use dynamic CRT linkage for consistency with the Rust libraries
            compile.arg("-D_DLL");
        }
        if let Some(parent) = path.parent() {
            compile.current_dir(parent);
        }
        println!("Compiling operator: {:?}", path);
        if !compile.status()?.success() {
            bail!("failed to compile cxx operator");
        };
        if !object_file_path.exists() {
            bail!(
                "object file was not created: {}",
                object_file_path.display()
            );
        }
        object_file_paths.push(object_file_path);
    }

    let output_path = Path::new("../build").join(format!("{DLL_PREFIX}{out_name}{DLL_SUFFIX}"));
    let mut link = std::process::Command::new("clang++");
    link.arg("-shared").args(&object_file_paths);
    link.args(link_args);
    #[cfg(target_os = "windows")]
    {
        link.arg("-ladvapi32");
        link.arg("-luserenv");
        link.arg("-lkernel32");
        link.arg("-lws2_32");
        link.arg("-lbcrypt");
        link.arg("-lncrypt");
        link.arg("-lschannel");
        link.arg("-lntdll");
        link.arg("-liphlpapi");

        link.arg("-lcfgmgr32");
        link.arg("-lcredui");
        link.arg("-lcrypt32");
        link.arg("-lcryptnet");
        link.arg("-lfwpuclnt");
        link.arg("-lgdi32");
        link.arg("-lmsimg32");
        link.arg("-lmswsock");
        link.arg("-lole32");
        link.arg("-loleaut32");
        link.arg("-lopengl32");
        link.arg("-lsecur32");
        link.arg("-lshell32");
        link.arg("-lsynchronization");
        link.arg("-luser32");
        link.arg("-lwinspool");

        // Use dynamic CRT and avoid conflicts with static CRT
        link.arg("-Wl,-nodefaultlib:libcmt");
        link.arg("-lmsvcrt");
    }
    #[cfg(target_os = "macos")]
    {
        link.arg("-framework").arg("CoreServices");
        link.arg("-framework").arg("Security");
        link.arg("-l").arg("System");
        link.arg("-l").arg("resolv");
        link.arg("-l").arg("pthread");
        link.arg("-l").arg("c");
        link.arg("-l").arg("m");
    }
    link.arg("-o").arg(&output_path);
    if let Some(parent) = paths[0].parent() {
        link.current_dir(parent);
    }
    println!("Linking operator: {} -> {:?}", out_name, output_path);
    if !link.status()?.success() {
        bail!("failed to create shared library from cxx operator (c api)");
    };

    // Verify the shared library was created
    let final_path = if let Some(parent) = paths[0].parent() {
        parent.join(&output_path)
    } else {
        output_path.clone()
    };
    if !final_path.exists() {
        bail!(
            "shared library was not created: {} (expected at {})",
            out_name,
            final_path.display()
        );
    }
    println!(
        "Successfully built operator shared library: {}",
        final_path.display()
    );

    Ok(())
}
