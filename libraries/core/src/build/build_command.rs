use std::{
    collections::BTreeMap,
    io::{BufRead, BufReader},
    path::Path,
    process::{Command, Stdio},
};

use dora_message::descriptor::EnvValue;
use eyre::{Context, eyre};

pub fn run_build_command(
    build: &str,
    working_dir: &Path,
    uv: bool,
    envs: &Option<BTreeMap<String, EnvValue>>,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) -> eyre::Result<()> {
    std::fs::create_dir_all(working_dir).context("failed to create working directory")?;

    let lines = build.lines().collect::<Vec<_>>();
    for build_line in lines {
        let quote: Vec<&str> = build_line.split('"').collect();
        if quote.len() % 2 == 0 {
            return Err(eyre!(
                "build command `{build_line}`. quote(s) are not in pair"
            ));
        }
        let mut split_vec: Vec<&str> = vec![];
        quote.iter().enumerate().for_each(|(i, part)| {
            if i % 2 == 0 {
                let mut split_with_white: Vec<&str> = part.split_whitespace().collect();
                split_vec.append(&mut split_with_white);
            } else {
                split_vec.push(part);
            }
        });
        let mut split = split_vec.iter();

        let program = split
            .next()
            .ok_or_else(|| eyre!("build command is empty"))?;
        let mut cmd = if uv && (*program == "pip" || *program == "pip3") {
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
        let stderr_tx = stdout_tx.clone();
        let stdout_tx = stdout_tx.clone();

        std::thread::spawn(move || {
            for line in child_stdout.lines() {
                if stdout_tx.blocking_send(line).is_err() {
                    break;
                }
            }
        });
        std::thread::spawn(move || {
            for line in child_stderr.lines() {
                if stderr_tx.blocking_send(line).is_err() {
                    break;
                }
            }
        });

        let exit_status = cmd
            .status()
            .wrap_err_with(|| format!("failed to run `{build}`"))?;
        if !exit_status.success() {
            return Err(eyre!("build command `{build_line}` returned {exit_status}"));
        }
    }
    Ok(())
}
