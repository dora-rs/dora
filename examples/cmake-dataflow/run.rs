use dora_tracing::set_up_tracing;
use eyre::{Context, ContextCompat};
use std::path::{Path, PathBuf};
use xshell::{cmd, Shell};

fn main() -> eyre::Result<()> {
    set_up_tracing("cmake-dataflow-runner").wrap_err("failed to set up tracing")?;

    if cfg!(windows) {
        tracing::error!(
            "The c++ example does not work on Windows currently because of a linker error"
        );
        return Ok(());
    }

    // create a new shell in this folder
    let sh = prepare_shell()?;

    // build C++ source code using cmake
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    cmd!(sh, "cmake -DDORA_ROOT_DIR={root} -B build .").run()?;
    cmd!(sh, "cmake --build build").run()?;
    cmd!(sh, "cmake --install build").run()?;

    // build the `dora` binary (you can skip this if you use `cargo install dora-cli`)
    let dora = prepare_dora(&sh)?;

    // start up the dora daemon and coordinator
    cmd!(sh, "{dora} up").run()?;

    // start running the dataflow.yml -> outputs the UUID assigned to the dataflow
    let output = cmd!(sh, "{dora} start dataflow.yml --attach").read_stderr()?;
    let uuid = output.lines().next().context("no output")?;

    // stop the dora daemon and coordinator again
    cmd!(sh, "{dora} destroy").run()?;

    // verify that the node output was written to `out`
    sh.change_dir("out");
    sh.change_dir(uuid);
    let sink_output = sh.read_file("log_runtime-node-2.txt")?;
    if sink_output.lines().count() < 20 {
        eyre::bail!("sink did not receive the expected number of messages")
    }

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
