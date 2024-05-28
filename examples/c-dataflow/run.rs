use std::env::consts::{DLL_PREFIX, DLL_SUFFIX, EXE_SUFFIX};
use std::path::{Path, PathBuf};
use xshell::{cmd, Shell};

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));

    // create a new shell in this folder
    let sh = prepare_shell()?;
    // build the `dora` binary (you can skip this if you use `cargo install dora-cli`)
    let dora = prepare_dora(&sh)?;

    cmd!(sh, "cargo build --package dora-node-api-c").run()?;
    cmd!(sh, "cargo build --package dora-operator-api-c").run()?;

    sh.create_dir("build")?;
    let target_debug = root.join("target").join("debug");

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
        "clang node.c -l dora_node_api_c {args...} -L {target_debug} --output build/c_node{EXE_SUFFIX}"
    )
    .run()?;
    cmd!(
        sh,
        "clang sink.c -l dora_node_api_c {args...} -L {target_debug} --output build/c_sink{EXE_SUFFIX}"
    )
    .run()?;

    // compile operator
    let operator_args: &[&str] = if cfg!(unix) { &["-fPIC"] } else { &[] };
    cmd!(
        sh,
        "clang -c operator.c -o build/operator.o -fdeclspec {operator_args...}"
    )
    .run()?;
    // link operator
    let operator_link_args: &[&str] = if cfg!(target_os = "windows") {
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
        &[]
    };
    cmd!(
        sh,
        "clang -shared build/operator.o -L {target_debug} -l dora_operator_api_c {operator_link_args...} -o build/{DLL_PREFIX}operator{DLL_SUFFIX}"
    ).run()?;

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
