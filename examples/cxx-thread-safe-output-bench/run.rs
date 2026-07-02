use eyre::{Context, bail};
use std::{env::consts::EXE_SUFFIX, path::Path};

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let target = root.join("target");
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    std::fs::create_dir_all("build")?;
    let build_dir = Path::new("build");

    // Build the Dora C++ node library and copy its generated glue.
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

    // Compile the two C++ programs: the timestamping source and the
    // mode-switchable contestant.
    let bridge = dunce::canonicalize(build_dir.join("node-bridge.cc"))?;
    build_cxx_node(
        root,
        &[
            &dunce::canonicalize(Path::new("source").join("main.cc"))?,
            &bridge,
        ],
        "source",
        &["-l", "dora_node_api_cxx"],
    )?;
    build_cxx_node(
        root,
        &[
            &dunce::canonicalize(Path::new("bench-node").join("main.cc"))?,
            &bridge,
        ],
        "bench_node",
        &["-l", "dora_node_api_cxx"],
    )?;

    // Run the three configurations back to back, each as its own `dora run`
    // child process. (Running them in-process fails: Dora's global tracing
    // subscriber can only be installed once per process.) Each prints a
    // single `BENCH_RESULT ...` line per measuring node; compare them below.
    build_package("dora-cli")?;
    let dora_bin = target.join("debug").join(format!("dora{EXE_SUFFIX}"));
    for yaml in [
        "dataflow-blocking.yml",
        "dataflow-worker.yml",
        "dataflow-twonode.yml",
    ] {
        println!("\n===== running {yaml} =====");
        let status = std::process::Command::new(&dora_bin)
            .args(["run", yaml, "--stop-after", "12s"])
            .status()
            .wrap_err("failed to spawn `dora run`")?;
        if !status.success() {
            bail!("dataflow {yaml} exited with failure");
        }
    }

    println!("\n===== benchmark done -- compare the BENCH_RESULT lines above =====");
    Ok(())
}

/// Builds a workspace crate by name.
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

/// Compiles and links one C++ node: its `main.cc` plus the cxx bridge.
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
