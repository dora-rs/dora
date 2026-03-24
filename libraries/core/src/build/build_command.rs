use crate::build::{managed_python_bin_dir, managed_python_interpreter};
use std::{
    collections::BTreeMap,
    ffi::OsString,
    path::{Path, PathBuf},
    process::Stdio,
};

use dora_message::descriptor::EnvValue;
use eyre::{Context, eyre};
use tokio::{
    io::{AsyncBufRead, AsyncBufReadExt, BufReader},
    process::Command,
};
use tokio_stream::{StreamExt, wrappers::LinesStream};

pub async fn run_build_command(
    build: &str,
    working_dir: &Path,
    uv: bool,
    python_env_dir: Option<PathBuf>,
    envs: &Option<BTreeMap<String, EnvValue>>,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) -> eyre::Result<()> {
    std::fs::create_dir_all(working_dir).context("failed to create working directory")?;

    let lines = build
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty())
        .collect::<Vec<_>>();
    if lines.is_empty() {
        return Err(eyre!("build command is empty"));
    }
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
        if uv {
            if let Some(python_env_dir) = &python_env_dir {
                apply_managed_python_env(&mut cmd, python_env_dir, envs)
                    .context("failed to apply managed Python env")?;
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
            .wrap_err_with(|| format!("failed to spawn `{build_line}`"))?;

        let child_stdout = BufReader::new(child.stdout.take().expect("failed to take stdout"));
        let child_stderr = BufReader::new(child.stderr.take().expect("failed to take stderr"));
        let stdout_tx = stdout_tx.clone();

        tokio::spawn(async move {
            forward_build_output(child_stdout, child_stderr, stdout_tx).await;
        });

        let exit_status = child
            .wait()
            .await
            .wrap_err_with(|| format!("failed to run `{build_line}`"))?;
        if !exit_status.success() {
            return Err(eyre!("build command `{build_line}` returned {exit_status}"));
        }
    }
    Ok(())
}

fn apply_managed_python_env(
    cmd: &mut Command,
    python_env_dir: &Path,
    envs: &Option<BTreeMap<String, EnvValue>>,
) -> eyre::Result<()> {
    let interpreter = managed_python_interpreter(python_env_dir);
    // Fail closed here: if the managed interpreter is missing, falling through to the ambient
    // PATH could silently pick a different Python than the one Dora prepared for this node.
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
        let stdout_tx = stdout_tx.clone();

        tokio::spawn(async move {
            forward_build_output(child_stdout, child_stderr, stdout_tx).await;
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

fn local_dora_python_package_dir(working_dir: &Path) -> Option<PathBuf> {
    working_dir.ancestors().find_map(|dir| {
        let package_dir = dir.join("apis").join("python").join("node");
        package_dir
            .join("pyproject.toml")
            .is_file()
            .then_some(package_dir)
    })
}

async fn forward_build_output<R1, R2>(
    stdout: R1,
    stderr: R2,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) where
    R1: AsyncBufRead + Unpin,
    R2: AsyncBufRead + Unpin,
{
    let mut merged = LinesStream::new(stdout.lines()).merge(LinesStream::new(stderr.lines()));

    while let Some(line) = merged.next().await {
        if stdout_tx.send(line).await.is_err() {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        forward_build_output, local_dora_python_package_dir, managed_python_path,
        prepare_managed_python_env, run_build_command,
    };
    use crate::build::{managed_python_bin_dir, managed_python_interpreter};
    use dora_message::descriptor::EnvValue;
    use std::{
        collections::BTreeMap,
        fs,
        path::{Path, PathBuf},
        time::{SystemTime, UNIX_EPOCH},
    };
    use tempfile::tempdir;
    use tokio::io::{AsyncWriteExt, BufReader};

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

    #[tokio::test]
    async fn reports_the_failing_line_for_multi_line_build_errors() {
        let working_dir = test_working_dir();
        let first_line = format!(
            "\"{}\" --help",
            std::env::current_exe()
                .expect("failed to locate test binary")
                .display()
        );
        let failing_line = "definitely-not-a-real-command";
        let build = format!("{first_line}\n{failing_line}");
        let envs = Some(BTreeMap::new());
        let (stdout_tx, _stdout_rx) = tokio::sync::mpsc::channel(4);

        let err = run_build_command(&build, &working_dir, false, None, &envs, stdout_tx)
            .await
            .expect_err("missing executable should fail");
        let msg = format!("{err:#}");

        assert!(msg.contains(failing_line));
        assert!(
            !msg.contains(&first_line),
            "error should reference the failing line instead of the full build block: {msg}"
        );

        let _ = fs::remove_dir_all(&working_dir);
    }

    #[tokio::test]
    async fn ignores_blank_lines_in_multi_line_build_commands() {
        let working_dir = tempdir().unwrap();
        let (tx, mut rx) = tokio::sync::mpsc::channel(16);

        run_build_command(
            "cargo --version\n\ncargo --version\n",
            working_dir.path(),
            false,
            None,
            &None::<BTreeMap<String, EnvValue>>,
            tx,
        )
        .await
        .unwrap();

        let mut lines = Vec::new();
        while let Some(line) = rx.recv().await {
            lines.push(line.unwrap());
        }

        assert!(
            lines.iter().any(|line| line.contains("cargo")),
            "expected cargo output to be forwarded"
        );
    }

    #[tokio::test]
    async fn rejects_truly_empty_build_commands() {
        let working_dir = tempdir().unwrap();
        let (tx, _rx) = tokio::sync::mpsc::channel(1);

        let err = run_build_command(
            "\n   \n",
            working_dir.path(),
            false,
            None,
            &None::<BTreeMap<String, EnvValue>>,
            tx,
        )
        .await
        .unwrap_err();

        assert!(err.to_string().contains("build command is empty"));
    }

    #[tokio::test]
    async fn prepare_managed_python_env_reuses_existing_interpreter() {
        let unique = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("system time should be after unix epoch")
            .as_nanos();
        let temp_root =
            std::env::temp_dir().join(format!("dora-managed-python-env-test-{unique}"));
        let working_dir = temp_root.join("workdir");
        let env_dir = temp_root.join("env");
        let interpreter = managed_python_interpreter(&env_dir);

        std::fs::create_dir_all(interpreter.parent().expect("interpreter should have parent"))
            .expect("failed to create fake env dir");
        std::fs::write(&interpreter, b"").expect("failed to create fake interpreter");

        let (tx, mut rx) = tokio::sync::mpsc::channel(1);
        prepare_managed_python_env(&working_dir, &env_dir, &None, tx)
            .await
            .expect("existing interpreter should be reused without error");

        assert!(interpreter.is_file());
        assert!(rx.try_recv().is_err(), "reused env should not emit build output");

        let _ = std::fs::remove_dir_all(&temp_root);
    }

    #[test]
    fn local_dora_python_package_dir_finds_workspace_package() {
        let working_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("src")
            .join("build");
        let package_dir =
            local_dora_python_package_dir(&working_dir).expect("workspace package should exist");

        assert!(package_dir.ends_with(Path::new("apis").join("python").join("node")));
        assert!(package_dir.join("pyproject.toml").is_file());
    }

    #[test]
    fn local_dora_python_package_dir_returns_none_outside_workspace() {
        let unique = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("system time should be after unix epoch")
            .as_nanos();
        let outside_dir = std::env::temp_dir().join(format!("dora-non-workspace-test-{unique}"));
        std::fs::create_dir_all(&outside_dir).expect("failed to create temp dir");

        assert!(
            local_dora_python_package_dir(&outside_dir).is_none(),
            "outside dirs should not resolve a workspace package"
        );

        let _ = std::fs::remove_dir_all(&outside_dir);
    }

    #[test]
    fn managed_python_path_prepends_bin_dir_to_path() {
        let env_dir = PathBuf::from("managed-env");
        let path_sep = if cfg!(windows) { ";" } else { ":" };
        let mut envs = BTreeMap::new();
        envs.insert(
            "PATH".to_owned(),
            EnvValue::String(format!("base-one{path_sep}base-two")),
        );

        let path = managed_python_path(&env_dir, &Some(envs))
            .expect("path composition should succeed")
            .expect("path should be set");
        let paths: Vec<_> = std::env::split_paths(&path).collect();

        assert_eq!(paths[0], managed_python_bin_dir(&env_dir));
        assert_eq!(paths[1], PathBuf::from("base-one"));
        assert_eq!(paths[2], PathBuf::from("base-two"));
    }

    #[tokio::test]
    async fn run_build_command_errors_if_managed_env_is_missing() {
        let unique = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("system time should be after unix epoch")
            .as_nanos();
        let temp_root =
            std::env::temp_dir().join(format!("dora-managed-python-missing-env-test-{unique}"));
        let working_dir = temp_root.join("workdir");
        let env_dir = temp_root.join("env");
        let (tx, _rx) = tokio::sync::mpsc::channel(1);

        let err = run_build_command(
            "python --version",
            &working_dir,
            true,
            Some(env_dir),
            &None,
            tx,
        )
        .await
        .expect_err("missing managed env should fail before spawning build command");

        assert!(
            format!("{err:#}").contains("managed Python interpreter"),
            "unexpected error: {err:#}"
        );

        let _ = std::fs::remove_dir_all(&temp_root);
    }

    fn test_working_dir() -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "dora-build-command-test-{}-{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        fs::create_dir_all(&dir).expect("failed to create test working dir");
        dir
    }
}
