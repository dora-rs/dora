use std::{
    collections::BTreeMap,
    ffi::OsString,
    path::{Path, PathBuf},
    process::Stdio,
    sync::LazyLock,
};

use crate::build::{managed_python_bin_dir, managed_python_interpreter};
use dora_message::descriptor::EnvValue;
use eyre::{Context, eyre};
use tokio::{
    io::{AsyncBufRead, AsyncBufReadExt, BufReader},
    process::Command,
};
use tokio_stream::{StreamExt, wrappers::LinesStream};

static LOCAL_DORA_WHEEL_CACHE: LazyLock<tokio::sync::Mutex<BTreeMap<PathBuf, PathBuf>>> =
    LazyLock::new(|| tokio::sync::Mutex::new(BTreeMap::new()));

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
            forward_build_output(child_stdout, child_stderr, stdout_tx).await;
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

    let mut cmd = Command::new("uv");
    cmd.arg("pip");
    cmd.arg("install");
    let install_args = dora_runtime_install_args(working_dir, envs, stdout_tx.clone()).await?;
    cmd.args(install_args);

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

/// Returns the package argument for installing `dora-rs` into a managed Python env.
async fn dora_runtime_install_args(
    working_dir: &Path,
    envs: &Option<BTreeMap<String, EnvValue>>,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) -> eyre::Result<Vec<OsString>> {
    match local_dora_python_package_dir(working_dir) {
        Some(package_dir) => Ok(vec![
            ensure_local_dora_python_wheel(&package_dir, envs, stdout_tx)
                .await?
                .into_os_string(),
        ]),
        None => Ok(vec![OsString::from(format!(
            "dora-rs=={}",
            env!("CARGO_PKG_VERSION")
        ))]),
    }
}

/// Builds the in-tree Python API once per Dora process and returns its wheel path.
///
/// Installing the local source tree directly asks maturin to build once per managed
/// node env. A wheel install keeps local checkout behavior while sharing that build
/// across all node envs created by the current CLI invocation.
async fn ensure_local_dora_python_wheel(
    package_dir: &Path,
    envs: &Option<BTreeMap<String, EnvValue>>,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) -> eyre::Result<PathBuf> {
    let package_dir = dunce::canonicalize(package_dir).wrap_err_with(|| {
        format!(
            "failed to canonicalize local Dora Python package `{}`",
            package_dir.display()
        )
    })?;
    let mut cache = LOCAL_DORA_WHEEL_CACHE.lock().await;
    if let Some(wheel) = cache.get(&package_dir)
        && wheel.is_file()
    {
        return Ok(wheel.clone());
    }

    let wheel_dir = local_dora_python_wheel_dir(&package_dir)?;
    std::fs::create_dir_all(&wheel_dir).wrap_err_with(|| {
        format!(
            "failed to create local Dora Python wheel cache `{}`",
            wheel_dir.display()
        )
    })?;

    let mut cmd = Command::new("uv");
    cmd.arg("build");
    cmd.arg("--wheel");
    cmd.arg("--clear");
    cmd.arg("--no-create-gitignore");
    cmd.arg("--out-dir");
    cmd.arg(&wheel_dir);
    cmd.arg(&package_dir);

    if let Some(envs) = envs {
        for (key, value) in envs {
            cmd.env(key, value.to_string());
        }
    }
    cmd.current_dir(dunce::simplified(&package_dir));
    cmd.stdin(Stdio::null());
    cmd.stdout(Stdio::piped());
    cmd.stderr(Stdio::piped());
    cmd.env("CLICOLOR", "1");
    cmd.env("CLICOLOR_FORCE", "1");

    let mut child = cmd.spawn().wrap_err_with(|| {
        format!(
            "failed to spawn local Dora Python wheel build for `{}`",
            package_dir.display()
        )
    })?;

    let child_stdout = BufReader::new(child.stdout.take().expect("failed to take stdout"));
    let child_stderr = BufReader::new(child.stderr.take().expect("failed to take stderr"));

    tokio::spawn(async move {
        forward_build_output(child_stdout, child_stderr, stdout_tx).await;
    });

    let exit_status = child.wait().await.wrap_err_with(|| {
        format!(
            "failed to build local Dora Python wheel from `{}`",
            package_dir.display()
        )
    })?;
    if !exit_status.success() {
        return Err(eyre!(
            "local Dora Python wheel build `{}` returned {exit_status}",
            package_dir.display()
        ));
    }

    let wheel = find_local_dora_python_wheel(&wheel_dir)?;
    cache.insert(package_dir, wheel.clone());
    Ok(wheel)
}

fn local_dora_python_wheel_dir(package_dir: &Path) -> eyre::Result<PathBuf> {
    let workspace_root = package_dir
        .parent()
        .and_then(|path| path.parent())
        .and_then(|path| path.parent())
        .ok_or_else(|| {
            eyre!(
                "local Dora Python package `{}` is not under apis/python/node",
                package_dir.display()
            )
        })?;
    Ok(workspace_root
        .join("target")
        .join("dora-python-wheels")
        .join(format!("process-{}", std::process::id())))
}

fn find_local_dora_python_wheel(wheel_dir: &Path) -> eyre::Result<PathBuf> {
    let mut wheels = std::fs::read_dir(wheel_dir)
        .wrap_err_with(|| format!("failed to read wheel cache `{}`", wheel_dir.display()))?
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|path| {
            path.extension().is_some_and(|extension| extension == "whl")
                && path
                    .file_name()
                    .is_some_and(|name| name.to_string_lossy().starts_with("dora_rs-"))
        })
        .collect::<Vec<_>>();
    wheels.sort();
    wheels.into_iter().next().ok_or_else(|| {
        eyre!(
            "local Dora Python wheel build did not create a dora-rs wheel in `{}`",
            wheel_dir.display()
        )
    })
}

/// Forwards stdout and stderr to the build logger until both streams close.
async fn forward_build_output<R1, R2>(
    child_stdout: R1,
    child_stderr: R2,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) where
    R1: AsyncBufRead + Unpin,
    R2: AsyncBufRead + Unpin,
{
    let mut merged =
        LinesStream::new(child_stdout.lines()).merge(LinesStream::new(child_stderr.lines()));

    while let Some(line) = merged.next().await {
        if stdout_tx.send(line).await.is_err() {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        dora_runtime_install_args, find_local_dora_python_wheel, forward_build_output,
        local_dora_python_package_dir, local_dora_python_wheel_dir,
    };
    use std::{ffi::OsString, fs};
    use tokio::io::{AsyncWriteExt, BufReader};

    #[test]
    fn local_runtime_install_discovers_workspace_package() {
        let temp = tempfile::tempdir().unwrap();
        let package_dir = temp.path().join("apis").join("python").join("node");
        fs::create_dir_all(&package_dir).unwrap();
        fs::write(
            package_dir.join("pyproject.toml"),
            "[project]\nname = \"dora-rs\"\n",
        )
        .unwrap();

        let working_dir = temp.path().join("examples").join("python-dataflow");
        fs::create_dir_all(&working_dir).unwrap();

        assert_eq!(
            local_dora_python_package_dir(&working_dir),
            Some(package_dir)
        );
    }

    #[tokio::test]
    async fn runtime_install_falls_back_to_versioned_pypi_requirement() {
        let temp = tempfile::tempdir().unwrap();
        let (stdout_tx, _stdout_rx) = tokio::sync::mpsc::channel(1);

        assert_eq!(
            dora_runtime_install_args(temp.path(), &None, stdout_tx)
                .await
                .unwrap(),
            vec![OsString::from(format!(
                "dora-rs=={}",
                env!("CARGO_PKG_VERSION")
            ))]
        );
    }

    #[test]
    fn local_runtime_wheel_cache_is_under_workspace_target() {
        let temp = tempfile::tempdir().unwrap();
        let package_dir = temp.path().join("apis").join("python").join("node");
        fs::create_dir_all(&package_dir).unwrap();

        assert_eq!(
            local_dora_python_wheel_dir(&package_dir).unwrap(),
            temp.path()
                .join("target")
                .join("dora-python-wheels")
                .join(format!("process-{}", std::process::id()))
        );
    }

    #[test]
    fn finds_built_local_runtime_wheel() {
        let temp = tempfile::tempdir().unwrap();
        let wheel = temp
            .path()
            .join("dora_rs-0.2.1-cp37-abi3-manylinux_2_34_x86_64.whl");
        fs::write(&wheel, "").unwrap();
        fs::write(temp.path().join("other-0.1.0-py3-none-any.whl"), "").unwrap();

        assert_eq!(find_local_dora_python_wheel(temp.path()).unwrap(), wheel);
    }

    #[tokio::test]
    async fn keeps_draining_stdout_after_stderr_closes() {
        run_forward_output_test(true).await;
    }

    #[tokio::test]
    async fn keeps_draining_stderr_after_stdout_closes() {
        run_forward_output_test(false).await;
    }

    async fn run_forward_output_test(stdout_stays_open: bool) {
        let (stdout_reader, mut stdout_writer) = tokio::io::duplex(64);
        let (stderr_reader, mut stderr_writer) = tokio::io::duplex(64);
        let (tx, mut rx) = tokio::sync::mpsc::channel(16);

        let forward_task = tokio::spawn(forward_build_output(
            BufReader::new(stdout_reader),
            BufReader::new(stderr_reader),
            tx,
        ));

        let writer_task = tokio::spawn(async move {
            if stdout_stays_open {
                stderr_writer
                    .shutdown()
                    .await
                    .expect("failed to close stderr writer");

                for index in 0..256 {
                    stdout_writer
                        .write_all(format!("line-{index}\n").as_bytes())
                        .await
                        .expect("failed to write test line");
                }
                stdout_writer
                    .shutdown()
                    .await
                    .expect("failed to close stdout writer");
            } else {
                stdout_writer
                    .shutdown()
                    .await
                    .expect("failed to close stdout writer");

                for index in 0..256 {
                    stderr_writer
                        .write_all(format!("line-{index}\n").as_bytes())
                        .await
                        .expect("failed to write test line");
                }
                stderr_writer
                    .shutdown()
                    .await
                    .expect("failed to close stderr writer");
            }
        });

        let collect_task = tokio::spawn(async move {
            let mut lines = Vec::new();
            while let Some(line) = rx.recv().await {
                lines.push(line.expect("unexpected line forwarding error"));
            }
            lines
        });

        writer_task.await.expect("writer task failed");
        forward_task.await.expect("forward task failed");
        let lines = collect_task.await.expect("collector task failed");

        assert_eq!(lines.len(), 256);
        assert_eq!(lines.first().map(String::as_str), Some("line-0"));
        assert_eq!(lines.last().map(String::as_str), Some("line-255"));
    }
}
