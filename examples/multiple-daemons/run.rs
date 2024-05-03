use eyre::ContextCompat;
use std::{
    path::{Path, PathBuf},
    process::Command,
    time::Duration,
};
use xshell::{cmd, Shell};

fn main() -> eyre::Result<()> {
    // create a new shell in this folder
    let sh = prepare_shell()?;
    // build the `dora` binary (you can skip this if you use `cargo install dora-cli`)
    let dora = prepare_dora(&sh)?;

    // build the dataflow using `dora build`
    cmd!(sh, "{dora} build dataflow.yml").run()?;

    // start the dora coordinator (in background)
    Command::from(cmd!(sh, "{dora} coordinator")).spawn()?;
    // wait until coordinator is ready
    loop {
        match cmd!(sh, "{dora} list").quiet().ignore_stderr().run() {
            Ok(_) => {
                println!("coordinator connected");
                break;
            }
            Err(_) => {
                eprintln!("waiting for coordinator");
                std::thread::sleep(Duration::from_millis(100))
            }
        }
    }

    // start two daemons (in background)
    Command::from(cmd!(sh, "{dora} daemon --machine-id A")).spawn()?;
    Command::from(cmd!(sh, "{dora} daemon --machine-id B")).spawn()?;
    // wait until both daemons are connected
    loop {
        let output = cmd!(sh, "{dora} connected-machines")
            .quiet()
            .ignore_stderr()
            .read();
        match output {
            Ok(output) => {
                let connected: Vec<&str> = output.lines().collect();
                if connected == ["A", "B"] {
                    println!("both daemons connected");
                    break;
                } else {
                    eprintln!("not all daemons connected yet (connected: {connected:?})");
                }
            }
            Err(err) => eprintln!("failed to query connected-machines: {err:?}"),
        }
        std::thread::sleep(Duration::from_millis(100));
    }

    // start running the dataflow.yml -> outputs the UUID assigned to the dataflow
    println!("starting dataflow");
    let output = cmd!(sh, "{dora} start dataflow.yml --attach").read_stderr()?;
    println!("dataflow finished successfully");
    let uuid = output.lines().next().context("no output")?;

    // stop the coordinator and both daemons again
    cmd!(sh, "{dora} destroy").run()?;

    // verify that the node output was written to `out`
    sh.change_dir("out");
    sh.change_dir(uuid);
    let sink_output = sh.read_file("log_rust-sink.txt")?;
    if sink_output.lines().count() < 50 {
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
