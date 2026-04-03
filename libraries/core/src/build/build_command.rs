use std::{collections::BTreeMap, path::Path, process::Stdio};

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
    use super::{forward_build_output, run_build_command};
    use dora_message::descriptor::EnvValue;
    use std::{
        collections::BTreeMap,
        fs,
        path::PathBuf,
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

        let err = run_build_command(&build, &working_dir, false, &envs, stdout_tx)
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
            &None::<BTreeMap<String, EnvValue>>,
            tx,
        )
        .await
        .unwrap_err();

        assert!(err.to_string().contains("build command is empty"));
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
