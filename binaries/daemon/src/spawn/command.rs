use crate::log::NodeLogger;
use clonable_command::Command;
use dora_core::{
    build::managed_python_interpreter,
    descriptor::{
        DYNAMIC_SOURCE, SHELL_SOURCE, resolve_path, resolve_path_confined, source_is_url,
    },
    get_python_path,
};
use dora_download::download_file;
use dora_message::common::LogLevel;
use eyre::WrapErr;
use std::path::Path;

#[allow(clippy::too_many_arguments)]
pub(super) async fn path_spawn_command(
    working_dir: &Path,
    uv: bool,
    python_env_dir: Option<&Path>,
    confined: bool,
    logger: &mut NodeLogger<'_>,
    node: &dora_core::descriptor::CustomNode,
    permit_url: bool,
) -> eyre::Result<Option<Command>> {
    let cmd = match node.path.as_str() {
        DYNAMIC_SOURCE => return Ok(None),
        SHELL_SOURCE => {
            if std::env::var("DORA_ALLOW_SHELL_NODES").as_deref() != Ok("true") {
                eyre::bail!(
                    "Shell nodes are disabled by default (security risk). \
                     Set DORA_ALLOW_SHELL_NODES=true to enable."
                );
            }
            logger
                .log(
                    LogLevel::Warn,
                    Some("spawner".into()),
                    "DORA_ALLOW_SHELL_NODES is set: shell node will execute arbitrary commands"
                        .to_string(),
                )
                .await;
            if cfg!(target_os = "windows") {
                let cmd = Command::new("cmd");
                cmd.args(["/C", &node.args.clone().unwrap_or_default()])
            } else {
                let cmd = Command::new("sh");
                cmd.args(["-c", &node.args.clone().unwrap_or_default()])
            }
        }
        source => {
            // a checksum only means anything for a URL download — reject a
            // `path_sha256` on a local/PATH source rather than silently ignoring it.
            if node.path_sha256.is_some() && !source_is_url(source) {
                eyre::bail!(
                    "node has `path_sha256` set but its path `{source}` is not a URL — \
                     a download checksum only applies to URL paths"
                );
            }
            let resolved_path = if let Some(expected_sha256) = node
                .path_sha256
                .as_deref()
                .filter(|_| source_is_url(source))
            {
                // `path_sha256` is also a descriptor field, so a hand-written
                // descriptor could set it to a traversal string. It is used as a
                // path segment below, and `download_file` creates the target dir
                // before verifying, so enforce the 64-hex-char shape at this
                // trust boundary first (a valid digest has no `/`, `.`, …).
                if expected_sha256.len() != 64
                    || !expected_sha256.bytes().all(|b| b.is_ascii_hexdigit())
                {
                    eyre::bail!("invalid `path_sha256` (must be 64 hex chars)");
                }
                // hub binary artifact (spec §8.2): a sha256-pinned URL download.
                // The checksum is the integrity guarantee, so this is allowed
                // regardless of confinement / `permit_url`. `download_file`
                // re-fetches and re-verifies on every spawn (§8.4).
                //
                // Download under the node's own working dir and return an
                // absolute path: the spawned child's cwd is set to `working_dir`,
                // so a relative download path (resolved against the daemon's cwd)
                // would exec the wrong file or fail to find it. Scope by the
                // sha256 (content-addressed) so two nodes whose artifacts share a
                // filename — and a `working_dir` that defaults to the shared
                // dataflow dir — don't overwrite each other.
                let target_dir = working_dir.join("build").join(expected_sha256);
                let downloaded = download_file(source, &target_dir, Some(expected_sha256))
                    .await
                    .wrap_err("failed to download hub binary artifact")?;
                downloaded.canonicalize().unwrap_or(downloaded)
            } else if confined {
                // hub-sourced node (spec §11): the entrypoint resolves only
                // within the node's own working dir or managed env — no env
                // expansion, no URL download, no ambient-$PATH fallback.
                // (The *interpreter* for a script-only `.py` entrypoint still
                // comes from `uv run` like any non-hub script node — hub
                // Python nodes are expected to ship a `build:` so they get a
                // managed env, spec §15 Q4.)
                resolve_path_confined(source, working_dir, python_env_dir)
                    .wrap_err_with(|| format!("failed to resolve hub entrypoint `{source}`"))?
            } else if source_is_url(source) {
                if !permit_url {
                    eyre::bail!("URL paths are not supported in this case");
                }
                // Download under the node's own working dir and return an
                // absolute path — mirroring the `path_sha256` branch above.
                // The spawned child's cwd is set to `working_dir`, so a
                // relative download path (resolved against the daemon's cwd)
                // would exec the wrong file or fail to find it. No content
                // address is available here, so nodes that share a
                // `working_dir` may still collide on filename in this shared
                // `build/` dir (a content-addressed layout would be a
                // follow-up).
                let target_dir = working_dir.join("build");
                let downloaded = download_file(source, &target_dir, None)
                    .await
                    .wrap_err("failed to download custom node")?;
                downloaded.canonicalize().unwrap_or(downloaded)
            } else {
                let source = shellexpand::env_with_context_no_errors(source, |var| {
                    // Only expand a controlled allowlist of safe variables
                    const ALLOWED_VARS: &[&str] = &[
                        "HOME",
                        "USER",
                        "DORA_WORKSPACE",
                        "CARGO_MANIFEST_DIR",
                        "PWD",
                    ];
                    if ALLOWED_VARS.contains(&var) {
                        std::env::var(var).ok()
                    } else {
                        tracing::warn!(
                            "skipping env expansion for '${var}' in node path \
                             (only HOME, USER, DORA_WORKSPACE, CARGO_MANIFEST_DIR, PWD are allowed)"
                        );
                        None
                    }
                });
                resolve_path(source.as_ref(), working_dir)
                    .wrap_err_with(|| format!("failed to resolve node source `{source}`"))?
            };

            // If extension is .py, use python to run the script
            let mut cmd = match resolved_path.extension().map(|ext| ext.to_str()) {
                Some(Some("py")) => {
                    let mut cmd = if uv {
                        if let Some(python_env_dir) =
                            python_env_dir.filter(|_| node.build.is_some())
                        {
                            // Reuse the interpreter from Dora's prepared env so
                            // custom Python nodes see the dependencies built there.
                            let python = managed_python_interpreter(python_env_dir);
                            if !python.is_file() {
                                eyre::bail!(
                                    "managed Python interpreter `{}` is missing",
                                    python.display()
                                );
                            }
                            logger
                                .log(
                                    LogLevel::Info,
                                    Some("spawner".into()),
                                    format!(
                                        "spawning managed Python {} -u {}",
                                        python.display(),
                                        resolved_path.display()
                                    ),
                                )
                                .await;
                            Command::new(python)
                        } else {
                            // Nodes without build-time Python setup still rely on
                            // the caller's active uv environment for imports.
                            let mut cmd = Command::new("uv");
                            cmd = cmd.arg("run");
                            cmd = cmd.arg("python");
                            logger
                                .log(
                                    LogLevel::Info,
                                    Some("spawner".into()),
                                    format!(
                                        "spawning: uv run python -u {}",
                                        resolved_path.display()
                                    ),
                                )
                                .await;
                            cmd
                        }
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
                let parsed = shlex::split(args)
                    .ok_or_else(|| eyre::eyre!("invalid quoting in node args: {args}"))?;
                cmd = cmd.args(parsed);
            }
            cmd
        }
    };

    Ok(Some(cmd))
}

#[cfg(test)]
mod tests {
    #[test]
    fn shlex_splits_quoted_args() {
        let input = "--foo 'hello world' --bar";
        let result = shlex::split(input).unwrap();
        assert_eq!(result, vec!["--foo", "hello world", "--bar"]);
    }

    #[test]
    fn shlex_rejects_unmatched_quote() {
        let input = "--foo 'unclosed";
        assert!(shlex::split(input).is_none());
    }
}
