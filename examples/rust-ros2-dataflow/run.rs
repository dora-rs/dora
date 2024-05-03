use eyre::ContextCompat;
use std::path::{Path, PathBuf};
use xshell::{cmd, Shell};

#[tokio::main]
async fn main() -> eyre::Result<()> {
    // create a new shell in this folder
    let sh = prepare_shell()?;
    // build the `dora` binary (you can skip this if you use `cargo install dora-cli`)
    let dora = prepare_dora(&sh)?;

    // build the dataflow using `dora build`
    cmd!(sh, "{dora} build dataflow.yml").run()?;

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
    let sink_output = sh.read_file("log_rust-node.txt")?;
    if !sink_output
        .lines()
        .any(|l| l.starts_with("received pose event: Ok("))
    {
        eyre::bail!("node did not receive any pose events")
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
