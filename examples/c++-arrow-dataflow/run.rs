use dora_tracing::set_up_tracing;
use eyre::{bail, Context};
use std::{env::consts::EXE_SUFFIX, path::Path, process::Command};

struct ArrowConfig {
    cflags: String,
    libs: String,
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    set_up_tracing("c++-dataflow-runner").wrap_err("failed to set up tracing")?;

    if cfg!(windows) {
        tracing::error!(
            "The c++ example does not work on Windows currently because of a linker error"
        );
        return Ok(());
    }

    let arrow_config = find_arrow_config().wrap_err("Failed to find Arrow configuration")?;
    tracing::info!(
        "Found Arrow configuration: cflags={}, libs={}",
        arrow_config.cflags,
        arrow_config.libs
    );

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

    build_cxx_node(
        root,
        &[
            &dunce::canonicalize(Path::new("node-rust-api").join("main.cc"))?,
            &dunce::canonicalize(build_dir.join("node-bridge.cc"))?,
        ],
        "node_rust_api",
        &[
            "-l",
            "dora_node_api_cxx",
            &arrow_config.cflags,
            &arrow_config.libs,
        ],
    )
    .await?;
    let dataflow = Path::new("dataflow.yml").to_owned();
    run_dataflow(&dataflow).await?;

    Ok(())
}

fn find_arrow_config() -> eyre::Result<ArrowConfig> {
    let output = Command::new("pkg-config")
        .args(&["--cflags", "arrow"])
        .output()
        .wrap_err("Failed to run pkg-config. Make sure Arrow C++ is installed")?;

    if !output.status.success() {
        bail!("Arrow C++ not found via pkg-config. Make sure it's installed and in your PKG_CONFIG_PATH");
    }

    let cflags = String::from_utf8(output.stdout)?.trim().to_string();

    let output = Command::new("pkg-config")
        .args(&["--libs", "arrow"])
        .output()
        .wrap_err("Failed to get Arrow library flags")?;

    if !output.status.success() {
        bail!("Failed to get Arrow library flags");
    }

    let libs = String::from_utf8(output.stdout)?.trim().to_string();

    Ok(ArrowConfig { cflags, libs })
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
    for arg in args {
        if arg.contains(" ") {
            for part in arg.split_whitespace() {
                clang.arg(part);
            }
        } else {
            clang.arg(arg);
        }
    }
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
