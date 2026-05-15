use std::{
    collections::BTreeMap,
    ffi::OsString,
    path::{Path, PathBuf},
    process::Stdio,
};

use crate::build::{managed_python_bin_dir, managed_python_interpreter};
use dora_message::descriptor::EnvValue;
use eyre::{Context, eyre};
use tokio::{
    io::{AsyncBufReadExt, BufReader},
    process::Command,
};

pub async fn run_build_command(
    build: &str,
    working_dir: &Path,
    uv: bool,
    python_env_dir: Option<PathBuf>,
    envs: &Option<BTreeMap<String, EnvValue>>,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) -> eyre::Result<()> {
    std::fs::create_dir_all(working_dir).context("failed to create working directory")?;

    let lines = build.lines().collect::<Vec<_>>();
    for build_line in lines {
        let mut split = splitty::split_unquoted_whitespace(build_line).unwrap_quotes(true);

        let program = split
            .next()
            .ok_or_else(|| eyre!("build command is empty"))?;
        let mut cmd = if uv && (program == "pip" || program == "pip3") {
            let mut cmd = Command::new("uv");
            cmd.arg("pip");
            cmd
        } else {
            Command::new(program)
        };
        cmd.args(split);

        // Inject Environment Variables
        if let Some(envs) = envs {
            for (key, value) in envs {
                let value = value.to_string();
                cmd.env(key, value);
            }
        }
        if uv && let Some(python_env_dir) = &python_env_dir {
            apply_managed_python_env(&mut cmd, python_env_dir, envs)
                .context("failed to apply managed Python env")?;
        }

        cmd.current_dir(dunce::simplified(working_dir));

        cmd.stdin(Stdio::null());
        cmd.stdout(Stdio::piped());
        cmd.stderr(Stdio::piped());

        cmd.env("CLICOLOR", "1");
        cmd.env("CLICOLOR_FORCE", "1");

        let mut child = cmd
            .spawn()
            .wrap_err_with(|| format!("failed to spawn `{build}`"))?;

        let child_stdout = BufReader::new(
            child
                .stdout
                .take()
                .ok_or_else(|| eyre!("failed to capture stdout pipe from build command"))?,
        );
        let child_stderr = BufReader::new(
            child
                .stderr
                .take()
                .ok_or_else(|| eyre!("failed to capture stderr pipe from build command"))?,
        );
        let stdout_tx = stdout_tx.clone();

        tokio::spawn(async move {
            let mut stdout_lines = child_stdout.lines();
            let mut stderr_lines = child_stderr.lines();
            loop {
                let line = tokio::select! {
                    line = stdout_lines.next_line() => line,
                    line = stderr_lines.next_line() => line,
                };
                let Some(line) = line.transpose() else {
                    break;
                };
                if stdout_tx.send(line).await.is_err() {
                    break;
                }
            }
        });

        let exit_status = child
            .wait()
            .await
            .wrap_err_with(|| format!("failed to run `{build}`"))?;
        if !exit_status.success() {
            return Err(eyre!("build command `{build_line}` returned {exit_status}"));
        }
    }
    Ok(())
}

/// Sets `VIRTUAL_ENV` and prepends the managed env's bin dir to `PATH` on `cmd`.
///
/// Fails closed if the managed interpreter is missing: falling through to the ambient
/// `PATH` could silently pick a different Python than the one Dora prepared for this node.
fn apply_managed_python_env(
    cmd: &mut Command,
    python_env_dir: &Path,
    envs: &Option<BTreeMap<String, EnvValue>>,
) -> eyre::Result<()> {
    let interpreter = managed_python_interpreter(python_env_dir);
    if !interpreter.is_file() {
        return Err(eyre!(
            "managed Python interpreter `{}` is missing; run Dora's Python env preparation first",
            interpreter.display()
        ));
    }
    cmd.env("VIRTUAL_ENV", python_env_dir);
    if let Some(path) = managed_python_path(python_env_dir, envs)? {
        cmd.env("PATH", path);
    }
    Ok(())
}

/// Composes the PATH for a spawned command: managed bin dir first, then user envs, then ambient.
fn managed_python_path(
    python_env_dir: &Path,
    envs: &Option<BTreeMap<String, EnvValue>>,
) -> eyre::Result<Option<OsString>> {
    // Prepend the managed env so `python`/`pip` resolve there before any machine-level entries.
    let mut paths = vec![managed_python_bin_dir(python_env_dir)];
    let base_path = envs
        .as_ref()
        .and_then(|envs| envs.get("PATH"))
        .map(|value| OsString::from(value.to_string()))
        .or_else(|| std::env::var_os("PATH"));

    if let Some(base_path) = base_path {
        paths.extend(std::env::split_paths(&base_path));
    }

    std::env::join_paths(paths)
        .map(Some)
        .wrap_err("failed to compose managed Python PATH")
}

/// Creates the managed Python venv if missing and ensures `dora` is importable inside it.
///
/// Idempotent: returns immediately if both the interpreter exists and `import dora` succeeds.
/// Uses `uv venv --clear` for creation and `uv pip install` for the runtime install,
/// preferring an in-tree workspace package when available.
pub async fn prepare_managed_python_env(
    working_dir: &Path,
    python_env_dir: &Path,
    envs: &Option<BTreeMap<String, EnvValue>>,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) -> eyre::Result<()> {
    std::fs::create_dir_all(working_dir).context("failed to create working directory")?;
    if let Some(parent) = python_env_dir.parent() {
        std::fs::create_dir_all(parent).context("failed to create Python env parent dir")?;
    }
    if !managed_python_interpreter(python_env_dir).is_file() {
        let mut cmd = Command::new("uv");
        cmd.arg("venv");
        cmd.arg("--clear");
        cmd.arg(python_env_dir);

        if let Some(envs) = envs {
            for (key, value) in envs {
                cmd.env(key, value.to_string());
            }
        }

        cmd.current_dir(dunce::simplified(working_dir));
        cmd.stdin(Stdio::null());
        cmd.stdout(Stdio::piped());
        cmd.stderr(Stdio::piped());
        cmd.env("CLICOLOR", "1");
        cmd.env("CLICOLOR_FORCE", "1");

        let mut child = cmd
            .spawn()
            .wrap_err_with(|| format!("failed to spawn `uv venv {}`", python_env_dir.display()))?;

        let child_stdout = BufReader::new(child.stdout.take().expect("failed to take stdout"));
        let child_stderr = BufReader::new(child.stderr.take().expect("failed to take stderr"));
        let stdout_tx_clone = stdout_tx.clone();

        tokio::spawn(async move {
            forward_build_output(child_stdout, child_stderr, stdout_tx_clone).await;
        });

        let exit_status = child
            .wait()
            .await
            .wrap_err_with(|| format!("failed to run `uv venv {}`", python_env_dir.display()))?;
        if !exit_status.success() {
            return Err(eyre!(
                "managed Python env preparation `{}` returned {exit_status}",
                python_env_dir.display()
            ));
        }
    }

    ensure_managed_python_runtime(working_dir, python_env_dir, envs, stdout_tx).await
}

/// Installs `dora` into the managed env if `import dora` doesn't succeed there yet.
async fn ensure_managed_python_runtime(
    working_dir: &Path,
    python_env_dir: &Path,
    envs: &Option<BTreeMap<String, EnvValue>>,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) -> eyre::Result<()> {
    if managed_python_can_import_dora(working_dir, python_env_dir, envs).await? {
        return Ok(());
    }

    // Prefer the local workspace package so the managed env runs against the
    // Dora Python API from this checkout when it is available.
    let mut cmd = Command::new("uv");
    cmd.arg("pip");
    cmd.arg("install");
    match local_dora_python_package_dir(working_dir) {
        Some(package_dir) => {
            cmd.arg("-e");
            cmd.arg(package_dir);
        }
        None => {
            cmd.arg(format!("dora-rs=={}", env!("CARGO_PKG_VERSION")));
        }
    }

    if let Some(envs) = envs {
        for (key, value) in envs {
            cmd.env(key, value.to_string());
        }
    }
    apply_managed_python_env(&mut cmd, python_env_dir, envs)
        .context("failed to target managed env for Dora runtime install")?;
    cmd.current_dir(dunce::simplified(working_dir));
    cmd.stdin(Stdio::null());
    cmd.stdout(Stdio::piped());
    cmd.stderr(Stdio::piped());
    cmd.env("CLICOLOR", "1");
    cmd.env("CLICOLOR_FORCE", "1");

    let mut child = cmd.spawn().wrap_err_with(|| {
        format!(
            "failed to spawn Dora runtime install into `{}`",
            python_env_dir.display()
        )
    })?;

    let child_stdout = BufReader::new(child.stdout.take().expect("failed to take stdout"));
    let child_stderr = BufReader::new(child.stderr.take().expect("failed to take stderr"));

    tokio::spawn(async move {
        forward_build_output(child_stdout, child_stderr, stdout_tx).await;
    });

    let exit_status = child.wait().await.wrap_err_with(|| {
        format!(
            "failed to install Dora runtime into `{}`",
            python_env_dir.display()
        )
    })?;
    if !exit_status.success() {
        return Err(eyre!(
            "managed Python runtime installation `{}` returned {exit_status}",
            python_env_dir.display()
        ));
    }

    Ok(())
}

/// Returns true if `python -c 'import dora'` succeeds inside the managed env.
async fn managed_python_can_import_dora(
    working_dir: &Path,
    python_env_dir: &Path,
    envs: &Option<BTreeMap<String, EnvValue>>,
) -> eyre::Result<bool> {
    let interpreter = managed_python_interpreter(python_env_dir);
    let mut cmd = Command::new(&interpreter);
    apply_managed_python_env(&mut cmd, python_env_dir, envs)
        .context("failed to target managed env for Dora runtime probe")?;
    cmd.arg("-c");
    cmd.arg("import dora");
    cmd.current_dir(dunce::simplified(working_dir));
    cmd.stdin(Stdio::null());
    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());

    let exit_status = cmd.status().await.wrap_err_with(|| {
        format!(
            "failed to probe Dora runtime in `{}`",
            python_env_dir.display()
        )
    })?;
    Ok(exit_status.success())
}

/// Walk up from `working_dir` looking for `apis/python/node/pyproject.toml`; returns the
/// directory containing it if found (so in-tree dev workflows install the local Python
/// package instead of pulling from PyPI).
fn local_dora_python_package_dir(working_dir: &Path) -> Option<PathBuf> {
    working_dir.ancestors().find_map(|dir| {
        let package_dir = dir.join("apis").join("python").join("node");
        package_dir
            .join("pyproject.toml")
            .is_file()
            .then_some(package_dir)
    })
}

/// Forwards stdout and stderr to the build logger using interleaved select.
async fn forward_build_output(
    child_stdout: BufReader<tokio::process::ChildStdout>,
    child_stderr: BufReader<tokio::process::ChildStderr>,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) {
    let mut stdout_lines = child_stdout.lines();
    let mut stderr_lines = child_stderr.lines();
    loop {
        let line = tokio::select! {
            line = stdout_lines.next_line() => line,
            line = stderr_lines.next_line() => line,
        };
        let Some(line) = line.transpose() else {
            break;
        };
        if stdout_tx.send(line).await.is_err() {
            break;
        }
    }
}
