use dora_cli::{Executable, RunCommand};
use eyre::{Context, bail};
use std::{env::consts::EXE_SUFFIX, path::Path, time::Duration};

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let target = root.join("target");
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    std::fs::create_dir_all("build")?;
    let build_dir = Path::new("build");

    // 1. Build the Dora C++ node library. This compiles `apis/c++/node/src/lib.rs`
    //    (which now contains `create_safe_output_sender` / `safe_send_output`) and
    //    generates the C++ glue files `lib.rs.cc` / `lib.rs.h`.
    build_package("dora-node-api-cxx")?;
    let node_cxxbridge = target
        .join("cxxbridge")
        .join("dora-node-api-cxx")
        .join("src");

    // Copy the generated glue next to our node so the compiler can find it.
    std::fs::copy(
        node_cxxbridge.join("lib.rs.cc"),
        build_dir.join("node-bridge.cc"),
    )?;
    std::fs::copy(
        node_cxxbridge.join("lib.rs.h"),
        build_dir.join("dora-node-api.h"),
    )?;

    // 2. Compile our example: main.cc + the generated bridge, linked against
    //    the Dora C++ static library.
    build_cxx_node(
        root,
        &[
            &dunce::canonicalize(Path::new("node-rust-api").join("main.cc"))?,
            &dunce::canonicalize(build_dir.join("node-bridge.cc"))?,
        ],
        "node_rust_api",
        &["-l", "dora_node_api_cxx"],
    )?;

    // 3. Run the dataflow, bounded so a wedged node fails fast instead of hanging.
    let mut run = RunCommand::new("dataflow.yml".to_string());
    run.stop_after = Some(Duration::from_secs(30));
    run.execute()?;

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
