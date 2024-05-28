use std::path::Path;
use std::path::PathBuf;
use xshell::{cmd, Shell};

fn main() -> eyre::Result<()> {
    // create a new shell in this folder
    let sh = prepare_shell()?;
    // build the `dora` binary (you can skip this if you use `cargo install dora-cli`)
    let dora = prepare_dora(&sh)?;

    // build the dataflow using `dora build`
    cmd!(sh, "{dora} build dataflow.yml").run()?;

    // start up the dora daemon and coordinator
    cmd!(sh, "{dora} up").run()?;

    // start running the dataflow.yml
    cmd!(sh, "{dora} start dataflow.yml --attach").run()?;

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
