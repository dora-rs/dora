//! Runtime registry: resolves the base spawn command for a runtime node's
//! operators.
//!
//! Today this maps the two built-in runtimes — `python` and `shared-library` —
//! to their launch commands, reproducing the historical daemon behavior. It is
//! the single seam where a future third-party-runtime resolver (driven by a
//! descriptor `runtimes:` map) will slot in: the caller passes the operators,
//! and this module decides which launcher hosts them.

use clonable_command::Command;
use dora_core::{
    build::managed_python_interpreter,
    config::NodeId,
    descriptor::{OperatorDefinition, OperatorSource, PythonSource, RUNTIME_PYTHON},
    get_python_path,
};
use eyre::{ContextCompat, WrapErr, bail};
use std::path::Path;

/// Build the base command that hosts a runtime node's `operators`.
///
/// All operators in a runtime node must belong to the same runtime family
/// (Python vs. native shared-library/WASM); mixing is rejected. The returned
/// command has only the launcher + args set — the caller injects
/// `DORA_RUNTIME_CONFIG`, zenoh connect vars, per-node env, and stdio.
pub(super) fn runtime_command(
    node_id: &NodeId,
    operators: &[OperatorDefinition],
    uv: bool,
    python_env_dir: Option<&Path>,
) -> eyre::Result<Command> {
    let python_operators: Vec<&OperatorDefinition> = operators
        .iter()
        .filter(|x| x.config.source.runtime_name() == RUNTIME_PYTHON)
        .collect();

    let other_operators = operators
        .iter()
        .any(|x| x.config.source.runtime_name() != RUNTIME_PYTHON);

    if !python_operators.is_empty() && !other_operators {
        python_runtime_command(node_id, &python_operators, uv, python_env_dir)
    } else if python_operators.is_empty() && other_operators {
        native_runtime_command(node_id)
    } else {
        bail!(
            "Cannot spawn runtime with both Python and non-Python operators. \
            Please use a single operator or ensure that all operators are Python-based."
        )
    }
}

/// The `python` built-in runtime: launch the runtime inside a Python
/// interpreter via `import dora; dora.start_runtime()`.
fn python_runtime_command(
    node_id: &NodeId,
    python_operators: &[&OperatorDefinition],
    uv: bool,
    python_env_dir: Option<&Path>,
) -> eyre::Result<Command> {
    // Use python to spawn runtime if there is a python operator

    // TODO: Handle multi-operator runtime once sub-interpreter is supported
    if python_operators.len() > 1 {
        bail!(
            "Runtime currently only supports one Python Operator.
     This is because PyO3 sub-interpreter is not yet available.
     See: https://github.com/PyO3/pyo3/issues/576"
        );
    }

    let python_operator = python_operators
        .first()
        .context("Runtime had no operators definition.")?;

    if let OperatorSource::Python(PythonSource {
        source: _,
        conda_env: Some(conda_env),
    }) = &python_operator.config.source
    {
        let conda = which::which("conda").context(
            "failed to find `conda`, yet a `conda_env` was defined. Make sure that `conda` is available.",
        )?;
        let mut command = Command::new(conda);
        command = command.args([
            "run",
            "-n",
            conda_env,
            "python",
            "-uc",
            format!("import dora; dora.start_runtime() # {}", node_id).as_str(),
        ]);
        Ok(command)
    } else {
        let mut cmd = if uv {
            if let Some(python_env_dir) = python_env_dir {
                // Reuse the managed interpreter so Python operators run
                // against the same environment Dora prepared during build.
                let python = managed_python_interpreter(python_env_dir);
                if !python.is_file() {
                    bail!(
                        "managed Python interpreter `{}` is missing",
                        python.display()
                    );
                }
                tracing::info!(
                    "spawning managed Python {} -uc import dora; dora.start_runtime() # {}",
                    python.display(),
                    node_id
                );
                Command::new(python)
            } else {
                let mut cmd = Command::new("uv");
                cmd = cmd.arg("run");
                cmd = cmd.arg("python");
                tracing::info!(
                    "spawning: uv run python -uc import dora; dora.start_runtime() # {}",
                    node_id
                );
                cmd
            }
        } else {
            let python = get_python_path()
                .wrap_err("Could not find python path when spawning custom node")?;
            tracing::info!(
                "spawning: python -uc import dora; dora.start_runtime() # {}",
                node_id
            );

            Command::new(python)
        };
        // Force python to always flush stdout/stderr buffer
        cmd = cmd.args([
            "-uc",
            format!("import dora; dora.start_runtime() # {}", node_id).as_str(),
        ]);
        Ok(cmd)
    }
}

/// The `shared-library` built-in runtime: launch the native `dora runtime`
/// subcommand (which hosts shared-library operators via `libloading`).
fn native_runtime_command(node_id: &NodeId) -> eyre::Result<Command> {
    let current_exe = std::env::current_exe().wrap_err("failed to get current executable path")?;
    let mut file_name = current_exe.clone();
    file_name.set_extension("");
    let file_name = file_name
        .file_name()
        .and_then(|s| s.to_str())
        .context("failed to get file name from current executable")?;

    // Check if the current executable is a python binary meaning that dora is installed within the python environment
    if file_name.ends_with("python") || file_name.ends_with("python3") {
        // Use the current executable to spawn runtime
        let python =
            get_python_path().wrap_err("Could not find python path when spawning custom node")?;
        let mut cmd = Command::new(python);

        tracing::info!(
            "spawning: python -uc import dora; dora.start_runtime() # {}",
            node_id
        );

        cmd = cmd.args([
            "-uc",
            format!("import dora; dora.start_runtime() # {}", node_id).as_str(),
        ]);
        Ok(cmd)
    } else if file_name == "dora" {
        // current_exe is the dora binary — use it so the
        // spawned runtime always matches the daemon version.
        // See #1797.
        let mut cmd = Command::new(&current_exe);
        cmd = cmd.arg("runtime");
        Ok(cmd)
    } else {
        // current_exe is something else, e.g. an embedded
        // example runner that calls `dora_cli::run()` —
        // see examples/c-dataflow/run.rs:21. Spawning
        // current_exe with `runtime` would recurse into
        // the example runner. Fall back to PATH lookup
        // for the dora binary. See #1805.
        let mut cmd =
            Command::new(which::which("dora").wrap_err("failed to find dora binary on PATH")?);
        cmd = cmd.arg("runtime");
        Ok(cmd)
    }
}
