use dora_core::{get_pip_path, get_python_path};
use dora_download::download_file;
use eyre::{ContextCompat, WrapErr};
use std::path::{Path, PathBuf};
use xshell::{cmd, Shell};

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let python = get_python_path().context("Could not get python binary")?;

    // create a new shell in this folder
    let sh = prepare_shell()?;

    // prepare Python virtual environment
    prepare_venv(&sh, &python)?;

    // build the `dora` binary (you can skip this if you use `cargo install dora-cli`)
    let dora = prepare_dora(&sh)?;

    // install/upgrade pip, then install requirements
    cmd!(sh, "{python} -m pip install --upgrade pip").run()?;
    let pip = get_pip_path().context("Could not get pip binary")?;
    cmd!(sh, "{pip} install -r requirements.txt").run()?;

    // build the dora Python package (you can skip this if you installed the Python dora package)
    {
        let python_node_api_dir = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("apis")
            .join("python")
            .join("node");
        let _dir = sh.push_dir(python_node_api_dir);
        cmd!(sh, "maturin develop").run()?;
    }

    download_file(
        "https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt",
        Path::new("yolov8n.pt"),
    )
    .await
    .context("Could not download weights.")?;

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
    let sink_output = sh.read_file("log_object_detection.txt")?;
    if sink_output.lines().count() < 50 {
        eyre::bail!("object dectection node did not receive the expected number of messages")
    }

    Ok(())
}

/// Prepares a Python virtual environment.
///
/// You can use the normal `python3 -m venv .venv` + `source .venv/bin/activate`
/// if you're running bash.
fn prepare_venv(sh: &Shell, python: &Path) -> eyre::Result<()> {
    cmd!(sh, "{python} -m venv ../.env").run()?;
    let venv = sh.current_dir().join("..").join(".env");
    sh.set_var(
        "VIRTUAL_ENV",
        venv.to_str().context("venv path not valid unicode")?,
    );

    // bin folder is named Scripts on windows.
    // 🤦‍♂️ See: https://github.com/pypa/virtualenv/commit/993ba1316a83b760370f5a3872b3f5ef4dd904c1
    let venv_bin = if cfg!(windows) {
        venv.join("Scripts")
    } else {
        venv.join("bin")
    };
    let path_separator = if cfg!(windows) { ';' } else { ':' };

    sh.set_var(
        "PATH",
        format!(
            "{}{path_separator}{}",
            venv_bin.to_str().context("venv path not valid unicode")?,
            std::env::var("PATH")?
        ),
    );

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
