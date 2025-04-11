use std::{collections::BTreeMap, path::Path, process::Command};

use dora_message::descriptor::EnvValue;
use eyre::{eyre, Context};

pub fn run_build_command(
    build: &str,
    working_dir: &Path,
    uv: bool,
    envs: &Option<BTreeMap<String, EnvValue>>,
) -> eyre::Result<()> {
    let lines = build.lines().collect::<Vec<_>>();
    for build_line in lines {
        let mut split = build_line.split_whitespace();

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
        let exit_status = cmd
            .status()
            .wrap_err_with(|| format!("failed to run `{}`", build))?;
        if !exit_status.success() {
            return Err(eyre!("build command `{build_line}` returned {exit_status}"));
        }
    }
    Ok(())
}
