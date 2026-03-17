use std::{collections::BTreeMap, path::Path, process::Stdio};

use dora_message::descriptor::EnvValue;
use eyre::{Context, eyre};
use tokio::{
    io::{AsyncBufRead, AsyncBufReadExt, BufReader},
    process::Command,
};

pub async fn run_build_command(
    build: &str,
    working_dir: &Path,
    uv: bool,
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

        cmd.current_dir(dunce::simplified(working_dir));

        cmd.stdin(Stdio::null());
        cmd.stdout(Stdio::piped());
        cmd.stderr(Stdio::piped());

        cmd.env("CLICOLOR", "1");
        cmd.env("CLICOLOR_FORCE", "1");

        let mut child = cmd
            .spawn()
            .wrap_err_with(|| format!("failed to spawn `{build}`"))?;

        let child_stdout = BufReader::new(child.stdout.take().expect("failed to take stdout"));
        let child_stderr = BufReader::new(child.stderr.take().expect("failed to take stderr"));
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

async fn forward_build_output<R1, R2>(
    stdout: R1,
    stderr: R2,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) where
    R1: AsyncBufRead + Unpin,
    R2: AsyncBufRead + Unpin,
{
    let mut stdout_lines = stdout.lines();
    let mut stderr_lines = stderr.lines();
    let mut stdout_open = true;
    let mut stderr_open = true;

    while stdout_open || stderr_open {
        tokio::select! {
            line = stdout_lines.next_line(), if stdout_open => {
                stdout_open = forward_line(line, &stdout_tx).await;
            }
            line = stderr_lines.next_line(), if stderr_open => {
                stderr_open = forward_line(line, &stdout_tx).await;
            }
            else => break,
        }
    }
}

async fn forward_line(
    line: std::io::Result<Option<String>>,
    stdout_tx: &tokio::sync::mpsc::Sender<std::io::Result<String>>,
) -> bool {
    match line {
        Ok(Some(line)) => stdout_tx.send(Ok(line)).await.is_ok(),
        Ok(None) => false,
        Err(err) => {
            let _ = stdout_tx.send(Err(err)).await;
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::forward_build_output;
    use tokio::io::{AsyncWriteExt, BufReader};

    #[test]
    fn keeps_draining_stdout_after_stderr_closes() {
        run_forward_output_test(true);
    }

    #[test]
    fn keeps_draining_stderr_after_stdout_closes() {
        run_forward_output_test(false);
    }

    fn run_forward_output_test(stdout_stays_open: bool) {
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("failed to build tokio runtime");

        runtime.block_on(async move {
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
        });
    }
}
