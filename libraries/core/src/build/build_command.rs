use std::{collections::BTreeMap, path::Path, process::Stdio};

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
