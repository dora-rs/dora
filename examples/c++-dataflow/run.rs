use dora_tracing::set_up_tracing;
use eyre::{bail, Context};
use std::{
    env::consts::{DLL_PREFIX, DLL_SUFFIX, EXE_SUFFIX},
    path::Path,
    // process::Command,
};

#[tokio::main]
async fn main() -> eyre::Result<()> {
    set_up_tracing("c++-dataflow-runner").wrap_err("failed to set up tracing")?;

    if cfg!(windows) {
        tracing::error!(
            "The c++ example does not work on Windows currently because of a linker error"
        );
        return Ok(());
    }

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let target = root.join("target");
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    tokio::fs::create_dir_all("build").await?;
    let build_dir = Path::new("build");

    build_package("dora-node-api-cxx").await?;
    let node_cxxbridge = target
        .join("cxxbridge")
        .join("dora-node-api-cxx")
        .join("src");
    tokio::fs::copy(
        node_cxxbridge.join("lib.rs.cc"),
        build_dir.join("node-bridge.cc"),
    )
    .await?;
    tokio::fs::copy(
        node_cxxbridge.join("lib.rs.h"),
        build_dir.join("dora-node-api.h"),
    )
    .await?;
    tokio::fs::write(
        build_dir.join("operator.h"),
        r###"#include "../operator-rust-api/operator.h""###,
    )
    .await?;

    archive_node_bridge(root, build_dir).await?;
    
    build_package("dora-operator-api-cxx").await?;
    let operator_cxxbridge = target
        .join("cxxbridge")
        .join("dora-operator-api-cxx")
        .join("src");
    tokio::fs::copy(
        operator_cxxbridge.join("lib.rs.cc"),
        build_dir.join("operator-bridge.cc"),
    )
    .await?;
    tokio::fs::copy(
        operator_cxxbridge.join("lib.rs.h"),
        build_dir.join("dora-operator-api.h"),
    )
    .await?;

    build_package("dora-node-api-c").await?;
    build_package("dora-operator-api-c").await?;
    build_cxx_node(
        root,
        &[
            &dunce::canonicalize(Path::new("node-rust-api").join("main.cc"))?,
            &dunce::canonicalize(build_dir.join("node-bridge.cc"))?,
        ],
        "node_rust_api",
        &["-l", "dora_node_api_cxx"],
    )
    .await?;
    build_cxx_node(
        root,
        &[&dunce::canonicalize(
            Path::new("node-c-api").join("main.cc"),
        )?],
        "node_c_api",
        &["-l", "dora_node_api_c"],
    )
    .await?;
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
    )
    .await?;
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
    )
    .await?;

    let dataflow = Path::new("dataflow.yml").to_owned();
    build_package("dora-runtime").await?;
    run_dataflow(&dataflow).await?;

    Ok(())
}

/// Compiles "node-bridge.cc" into "node-bridge.o" using clang++ (falling back to g++ if clang++ is unavailable) with C++17 and PIC flags
/// and then archives it into the static library
async fn archive_node_bridge(root: &Path, build_dir: &Path) -> eyre::Result<()> {
    let node_bridge_cc = build_dir.join("node-bridge.cc");
    let node_bridge_o = build_dir.join("node-bridge.o");

    let compiler = if tokio::process::Command::new("which")
        .arg("clang++")
        .output()
        .await?
        .status
        .success()
    {
        "clang++"
    } else {
        "g++"
    };
    tracing::info!("Using C++ compiler: {}", compiler);

    let mut compile = tokio::process::Command::new(compiler);
    compile
        .arg("-c")
        .arg(&node_bridge_cc)
        .arg("-std=c++17")
        .arg("-fPIC")
        .arg("-o")
        .arg(&node_bridge_o);
        .success() {
            "clang++"
        } else {
            "g++"
        };
    tracing::info!("Using C++ compiler: {}", compiler);

    let mut compile = tokio::process::Command::new(compiler);
    compile.arg("-c")
           .arg(&node_bridge_cc)
           .arg("-std=c++17")
           .arg("-fPIC")
           .arg("-o")
           .arg(&node_bridge_o);
    if !compile.status().await?.success() {
        bail!("failed to compile node-bridge.cc with {}", compiler);
    }

    let static_lib = root
        .join("target")
        .join("debug")
        .join("libdora_node_api_cxx.a");

    let mut ar = tokio::process::Command::new("ar");
    ar.arg("rcs").arg(&static_lib).arg(&node_bridge_o);
    let static_lib = root.join("target").join("debug").join("libdora_node_api_cxx.a");

    let mut ar = tokio::process::Command::new("ar");
    ar.arg("rcs")
      .arg(&static_lib)
      .arg(&node_bridge_o);
    if !ar.status().await?.success() {
        bail!("failed to archive node-bridge.o into libdora_node_api_cxx.a");
    }
    Ok(())
}

async fn build_package(package: &str) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = tokio::process::Command::new(&cargo);
    cmd.arg("build");
    cmd.arg("--package").arg(package);
    if !cmd.status().await?.success() {
        bail!("failed to build {package}");
    };
    Ok(())
}

async fn run_dataflow(dataflow: &Path) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = tokio::process::Command::new(&cargo);
    cmd.arg("run");
    cmd.arg("--package").arg("dora-cli");
    cmd.arg("--")
        .arg("daemon")
        .arg("--run-dataflow")
        .arg(dataflow);
    if !cmd.status().await?.success() {
        bail!("failed to run dataflow");
    };
    Ok(())
}

async fn build_cxx_node(
    root: &Path,
    paths: &[&Path],
    out_name: &str,
    args: &[&str],
) -> eyre::Result<()> {
    let mut clang = tokio::process::Command::new("clang++");
    clang.args(paths);
    clang.arg("-std=c++17");
    #[cfg(target_os = "linux")]
    {
        clang.arg("-l").arg("m");
        clang.arg("-l").arg("rt");
        clang.arg("-l").arg("dl");
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

    if !clang.status().await?.success() {
        bail!("failed to compile c++ node");
    };
    Ok(())
}

async fn build_cxx_operator(
    paths: &[&Path],
    out_name: &str,
    link_args: &[&str],
) -> eyre::Result<()> {
    let mut object_file_paths = Vec::new();

    for path in paths {
        let mut compile = tokio::process::Command::new("clang++");
        compile.arg("-c").arg(path);
        compile.arg("-std=c++17");
        let object_file_path = path.with_extension("o");
        compile.arg("-o").arg(&object_file_path);
        #[cfg(unix)]
        compile.arg("-fPIC");
        if let Some(parent) = path.parent() {
            compile.current_dir(parent);
        }
        if !compile.status().await?.success() {
            bail!("failed to compile cxx operator");
        };
        object_file_paths.push(object_file_path);
    }

    let mut link = tokio::process::Command::new("clang++");
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
        link.arg("-lopengl32");
        link.arg("-lsecur32");
        link.arg("-lshell32");
        link.arg("-lsynchronization");
        link.arg("-luser32");
        link.arg("-lwinspool");

        link.arg("-Wl,-nodefaultlib:libcmt");
        link.arg("-D_DLL");
        link.arg("-lmsvcrt");
        link.arg("-fms-runtime-lib=static");
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
    link.arg("-o")
        .arg(Path::new("../build").join(format!("{DLL_PREFIX}{out_name}{DLL_SUFFIX}")));
    if let Some(parent) = paths[0].parent() {
        link.current_dir(parent);
    }
    if !link.status().await?.success() {
        bail!("failed to create shared library from cxx operator (c api)");
    };

    Ok(())
}
