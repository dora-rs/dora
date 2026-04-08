use crate::log::NodeLogger;
use clonable_command::Command;
use dora_core::{
    descriptor::{DYNAMIC_SOURCE, SHELL_SOURCE, resolve_path, source_is_url},
    get_python_path,
};
use dora_download::download_file;
use dora_message::common::LogLevel;
use eyre::WrapErr;
use std::path::{Path, PathBuf};

pub(super) fn uv_python_interpreter_from_env() -> Option<PathBuf> {
    uv_python_interpreter_from_virtual_env(std::env::var_os("VIRTUAL_ENV").map(PathBuf::from))
}

fn uv_python_interpreter_from_virtual_env(virtual_env: Option<PathBuf>) -> Option<PathBuf> {
    let mut python = virtual_env?;
    #[cfg(target_os = "windows")]
    {
        python.push("Scripts");
        python.push("python.exe");
    }
    #[cfg(not(target_os = "windows"))]
    {
        python.push("bin");
        python.push("python");
    }
    python.exists().then_some(python)
}

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
                resolve_path(source.as_ref(), working_dir).wrap_err_with(|| {
                    if let Some(build_cmd) = &node.build {
                        format!(
                            "failed to resolve node source `{source}`\n  \
                                Hint: this node defines a `build` command:\n    \
                                {build_cmd}\n  \
                                Did you forget to run `dora build <dataflow.yml>` first?"
                        )
                    } else {
                        format!("failed to resolve node source `{source}`")
                    }
                })?
            };

            // If extension is .py, use python to run the script
            let mut cmd = match resolved_path.extension().map(|ext| ext.to_str()) {
                Some(Some("py")) => {
                    let mut cmd = if uv {
                        let uv_python = uv_python_interpreter_from_env();
                        let mut cmd = Command::new("uv");
                        cmd = cmd.arg("run");
                        cmd = cmd.arg("--no-project");
                        if let Some(ref uv_python) = uv_python {
                            cmd = cmd.arg("--python");
                            cmd = cmd.arg(uv_python);
                        } else {
                            cmd = cmd.arg("--active");
                        }
                        cmd = cmd.arg("--no-managed-python");
                        cmd = cmd.arg("python");
                        logger
                            .log(
                                LogLevel::Info,
                                Some("spawner".into()),
                                match uv_python {
                                    Some(path) => format!(
                                        "spawning: uv run --no-project --python {} --no-managed-python python -u {}",
                                        path.display(),
                                        resolved_path.display()
                                    ),
                                    None => format!(
                                        "spawning: uv run --active --no-project --no-managed-python python -u {}",
                                        resolved_path.display()
                                    ),
                                },
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

#[cfg(test)]
mod tests {
    use super::uv_python_interpreter_from_virtual_env;

    #[test]
    fn resolves_python_from_virtual_env_layout() {
        let temp_root =
            std::env::temp_dir().join(format!("dora-daemon-test-{}", uuid::Uuid::new_v4()));
        #[cfg(target_os = "windows")]
        let python_path = temp_root.join("Scripts").join("python.exe");
        #[cfg(not(target_os = "windows"))]
        let python_path = temp_root.join("bin").join("python");

        std::fs::create_dir_all(python_path.parent().expect("python path has parent"))
            .expect("failed to create python parent dir");
        std::fs::write(&python_path, b"").expect("failed to create python executable stub");

        let resolved = uv_python_interpreter_from_virtual_env(Some(temp_root.clone()));
        assert_eq!(resolved.as_deref(), Some(python_path.as_path()));

        std::fs::remove_dir_all(temp_root).expect("failed to clean temp test directory");
    }

    #[test]
    fn returns_none_if_virtual_env_python_missing() {
        let temp_root =
            std::env::temp_dir().join(format!("dora-daemon-test-missing-{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&temp_root).expect("failed to create temp test directory");

        let resolved = uv_python_interpreter_from_virtual_env(Some(temp_root.clone()));
        assert!(resolved.is_none());

        std::fs::remove_dir_all(temp_root).expect("failed to clean temp test directory");
    }
}
