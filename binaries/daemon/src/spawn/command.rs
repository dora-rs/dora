use crate::log::NodeLogger;
use clonable_command::Command;
use dora_core::{
    descriptor::{DYNAMIC_SOURCE, SHELL_SOURCE, resolve_path, source_is_url},
    get_python_path,
};
use dora_download::download_file;
use dora_message::common::LogLevel;
use eyre::WrapErr;
use std::path::Path;

pub(super) async fn path_spawn_command(
    working_dir: &Path,
    uv: bool,
    logger: &mut NodeLogger<'_>,
    node: &dora_core::descriptor::CustomNode,
    permit_url: bool,
) -> eyre::Result<Option<Command>> {
    let cmd = match node.path.as_str() {
        DYNAMIC_SOURCE => return Ok(None),
        SHELL_SOURCE => {
            if cfg!(target_os = "windows") {
                let cmd = Command::new("cmd");
                cmd.args(["/C", &node.args.clone().unwrap_or_default()])
            } else {
                let cmd = Command::new("sh");
                cmd.args(["-c", &node.args.clone().unwrap_or_default()])
            }
        }
        source => {
            let resolved_path = if source_is_url(source) {
                if !permit_url {
                    eyre::bail!("URL paths are not supported in this case");
                }
                // try to download the shared library
                let target_dir = Path::new("build");
                download_file(source, target_dir)
                    .await
                    .wrap_err("failed to download custom node")?
            } else {
                let source = shellexpand::env(source)?;
                resolve_path(source.as_ref(), working_dir)
                    .wrap_err_with(|| format!("failed to resolve node source `{source}`"))?
            };

            // If extension is .py, use python to run the script
            let mut cmd = match resolved_path.extension().map(|ext| ext.to_str()) {
                Some(Some("py")) => {
                    let mut cmd = if uv {
                        let mut cmd = Command::new("uv");
                        cmd = cmd.arg("run");
                        cmd = cmd.arg("python");
                        logger
                            .log(
                                LogLevel::Info,
                                Some("spawner".into()),
                                format!("spawning: uv run python -u {}", resolved_path.display()),
                            )
                            .await;
                        cmd
                    } else {
                        let python = get_python_path()
                            .wrap_err("Could not find python path when spawning custom node")?;
                        logger
                            .log(
                                LogLevel::Info,
                                Some("spawner".into()),
                                format!("spawning: {:?} -u {}", &python, resolved_path.display()),
                            )
                            .await;

                        Command::new(python)
                    };
                    // Force python to always flush stdout/stderr buffer
                    cmd = cmd.arg("-u");
                    cmd = cmd.arg(&resolved_path);
                    cmd
                }
                _ => {
                    if uv {
                        let mut cmd = Command::new("uv");
                        cmd = cmd.arg("run");
                        cmd = cmd.arg(&resolved_path);
                        cmd
                    } else {
                        Command::new(&resolved_path)
                    }
                }
            };

            if let Some(args) = &node.args {
                cmd = cmd.args(args.split_ascii_whitespace());
            }
            cmd
        }
    };

    Ok(Some(cmd))
}
