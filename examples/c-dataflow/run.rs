use eyre::{bail, Context};
use std::{
    env::consts::{DLL_PREFIX, DLL_SUFFIX, EXE_SUFFIX},
    path::Path,
};
use tracing::metadata::LevelFilter;
use tracing_subscriber::Layer;

#[derive(Debug, Clone, clap::Parser)]
pub struct Args {
    #[clap(long)]
    pub run_dora_runtime: bool,
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let Args { run_dora_runtime } = clap::Parser::parse();

    if run_dora_runtime {
        return tokio::task::block_in_place(dora_daemon::run_dora_runtime);
    }

    set_up_tracing().wrap_err("failed to set up tracing")?;

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    tokio::fs::create_dir_all("build").await?;

    build_package("dora-node-api-c").await?;
    build_c_node(root, "node.c", "c_node").await?;
    build_c_node(root, "sink.c", "c_sink").await?;

    build_package("dora-operator-api-c").await?;
    build_c_operator().await?;

    let dataflow = Path::new("dataflow.yml").to_owned();
    dora_daemon::Daemon::run_dataflow(&dataflow).await?;

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

async fn build_c_node(root: &Path, name: &str, out_name: &str) -> eyre::Result<()> {
    let mut clang = tokio::process::Command::new("clang");
    clang.arg(name);
    clang.arg("-l").arg("dora_node_api_c");
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
    clang.arg("-L").arg(root.join("target").join("debug"));
    clang
        .arg("--output")
        .arg(Path::new("build").join(format!("{out_name}{EXE_SUFFIX}")));
    if !clang.status().await?.success() {
        bail!("failed to compile c node");
    };
    Ok(())
}

async fn build_c_operator() -> eyre::Result<()> {
    let mut compile = tokio::process::Command::new("clang");
    compile.arg("-c").arg("operator.c");
    compile.arg("-o").arg("build/operator.o");
    compile.arg("-fdeclspec");
    #[cfg(unix)]
    compile.arg("-fPIC");
    if !compile.status().await?.success() {
        bail!("failed to compile c operator");
    };

    let mut link = tokio::process::Command::new("clang");
    link.arg("-shared").arg("build/operator.o");
    link.arg("-o")
        .arg(Path::new("build").join(format!("{DLL_PREFIX}operator{DLL_SUFFIX}")));
    if !link.status().await?.success() {
        bail!("failed to link c operator");
    };

    Ok(())
}

fn set_up_tracing() -> eyre::Result<()> {
    use tracing_subscriber::prelude::__tracing_subscriber_SubscriberExt;

    let stdout_log = tracing_subscriber::fmt::layer()
        .pretty()
        .with_filter(LevelFilter::DEBUG);
    let subscriber = tracing_subscriber::Registry::default().with(stdout_log);
    tracing::subscriber::set_global_default(subscriber)
        .context("failed to set tracing global subscriber")
}
