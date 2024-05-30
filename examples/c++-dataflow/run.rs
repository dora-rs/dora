use dora_tracing::set_up_tracing;
use eyre::Context;
use std::env::consts::{DLL_PREFIX, DLL_SUFFIX, EXE_SUFFIX};
use std::path::{Path, PathBuf};
use xshell::{cmd, Shell};
fn main() -> eyre::Result<()> {
    set_up_tracing("c++-dataflow-runner").wrap_err("failed to set up tracing")?;

    if cfg!(windows) {
        tracing::error!(
            "The c++ example does not work on Windows currently because of a linker error"
        );
        return Ok(());
    }

    // create a new shell in this folder
    let sh = prepare_shell()?;
    // build the `dora` binary (you can skip this if you use `cargo install dora-cli`)
    let dora = prepare_dora(&sh)?;

    cmd!(sh, "cargo build --package dora-node-api-cxx").run()?;
    cmd!(sh, "cargo build --package dora-operator-api-cxx").run()?;
    cmd!(sh, "cargo build --package dora-node-api-c").run()?;
    cmd!(sh, "cargo build --package dora-operator-api-c").run()?;

    sh.create_dir("build")?;
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let target = root.join("target");
    let target_debug = target.join("debug");

    let node_cxxbridge = target
        .join("cxxbridge")
        .join("dora-node-api-cxx")
        .join("src");
    sh.copy_file(node_cxxbridge.join("lib.rs.cc"), "build/node-bridge.cc")?;
    sh.copy_file(node_cxxbridge.join("lib.rs.h"), "build/dora-node-api.h")?;
    sh.write_file(
        "build/operator.h",
        r###"#include "../operator-rust-api/operator.h""###,
    )?;

    let operator_cxxbridge = target
        .join("cxxbridge")
        .join("dora-operator-api-cxx")
        .join("src");
    sh.copy_file(
        operator_cxxbridge.join("lib.rs.cc"),
        "build/operator-bridge.cc",
    )?;
    sh.copy_file(
        operator_cxxbridge.join("lib.rs.h"),
        "build/dora-operator-api.h",
    )?;

    // compile nodes
    let args: &[&str] = if cfg!(target_os = "linux") {
        &["-l", "m", "-l", "rt", "-l", "dl", "-pthread"]
    } else if cfg!(target_os = "windows") {
        &[
            "-ladvapi32",
            "-luserenv",
            "-lkernel32",
            "-lws2_32",
            "-lbcrypt",
            "-lncrypt",
            "-lschannel",
            "-lntdll",
            "-liphlpapi",
            "-lcfgmgr32",
            "-lcredui",
            "-lcrypt32",
            "-lcryptnet",
            "-lfwpuclnt",
            "-lgdi32",
            "-lmsimg32",
            "-lmswsock",
            "-lole32",
            "-loleaut32",
            "-lopengl32",
            "-lsecur32",
            "-lshell32",
            "-lsynchronization",
            "-luser32",
            "-lwinspool",
            "-Wl,-nodefaultlib:libcmt",
            "-D_DLL",
            "-lmsvcrt",
        ]
    } else if cfg!(target_os = "macos") {
        &[
            "-framework",
            "CoreServices",
            "-framework",
            "Security",
            "-l",
            "System",
            "-l",
            "resolv",
            "-l",
            "pthread",
            "-l",
            "c",
            "-l",
            "m",
        ]
    } else {
        panic!("unsupported target platform")
    };
    cmd!(
        sh,
        "clang++ node-rust-api/main.cc build/node-bridge.cc -std=c++17 -l dora_node_api_cxx {args...} -L {target_debug} --output build/node_rust_api{EXE_SUFFIX}"
    )
    .run()?;
    cmd!(
        sh,
        "clang++ node-c-api/main.cc -std=c++17 -l dora_node_api_c {args...} -L {target_debug} --output build/node_c_api{EXE_SUFFIX}"
    )
    .run()?;

    // compile operators
    let operator_args: &[&str] = if cfg!(unix) { &["-fPIC"] } else { &[] };
    cmd!(
        sh,
        "clang++ -c build/operator-bridge.cc -std=c++17 -o build/operator-bridge.o {operator_args...}"
    )
    .run()?;
    cmd!(
        sh,
        "clang++ -c operator-rust-api/operator.cc -o operator-rust-api/operator.o -std=c++17 {operator_args...}"
    )
    .run()?;
    cmd!(
        sh,
        "clang++ -c operator-c-api/operator.cc -o operator-c-api/operator.o -std=c++17 {operator_args...}"
    )
    .run()?;

    // link operators
    cmd!(
        sh,
        "clang++ -shared operator-rust-api/operator.o build/operator-bridge.o -l dora_operator_api_cxx {args...} -L {target_debug} --output build/{DLL_PREFIX}operator_rust_api{DLL_SUFFIX}"
    )
    .run()?;
    cmd!(
        sh,
        "clang++ -shared operator-c-api/operator.o -l dora_operator_api_c {args...} -L {target_debug} --output build/{DLL_PREFIX}operator_c_api{DLL_SUFFIX}"
    )
    .run()?;

    // start up the dora daemon and coordinator
    cmd!(sh, "{dora} up").run()?;

    // start running the dataflow.yml
    cmd!(sh, "{dora} start dataflow.yml --attach").run()?;

    // stop the dora daemon and coordinator again
    cmd!(sh, "{dora} destroy").run()?;

    Ok(())
}

/// Prepares a shell and set the working directory to the parent folder of this file.
///
/// You can use your system shell instead (e.g. `bash`);
fn prepare_shell() -> Result<Shell, eyre::Error> {
    let sh = Shell::new()?;
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    sh.change_dir(root.join(file!()).parent().unwrap());
    Ok(sh)
}

/// Build the `dora` command-line executable from this repo.
///
/// You can skip this step and run `cargo install dora-cli --locked` instead.
fn prepare_dora(sh: &Shell) -> eyre::Result<PathBuf> {
    cmd!(sh, "cargo build --package dora-cli").run()?;
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let dora = root.join("target").join("debug").join("dora");
    Ok(dora)
}
