use crate::{
    adjust_shared_library_path,
    config::{DataId, Input, InputMapping, OperatorId, UserInputMapping},
    descriptor::{self, source_is_url, CoreNodeKind, OperatorSource},
};

use eyre::{bail, eyre, Context};
use std::{
    env::consts::EXE_EXTENSION,
    path::{Path, PathBuf},
    process::Command,
};

use super::Descriptor;
const VERSION: &str = env!("CARGO_PKG_VERSION");

pub fn check_dataflow(
    dataflow: &Descriptor,
    path: &Path,
    runtime_path: Option<PathBuf>,
) -> eyre::Result<()> {
    let nodes = dataflow.resolve_aliases();
    let base = path.canonicalize().unwrap().parent().unwrap().to_owned();
    let mut has_python_operator = false;
    let mut has_shared_lib_operator = false;

    // check that nodes and operators exist
    for node in &nodes {
        match &node.kind {
            descriptor::CoreNodeKind::Custom(node) => {
                let path = if source_is_url(&node.source) {
                    todo!("check URL");
                } else {
                    let raw = Path::new(&node.source);
                    if raw.extension().is_none() {
                        raw.with_extension(EXE_EXTENSION)
                    } else {
                        raw.to_owned()
                    }
                };
                base.join(&path)
                    .canonicalize()
                    .wrap_err_with(|| format!("no node exists at `{}`", path.display()))?;
            }
            descriptor::CoreNodeKind::Runtime(node) => {
                for operator_definition in &node.operators {
                    match &operator_definition.config.source {
                        OperatorSource::SharedLibrary(path) => {
                            has_shared_lib_operator = true;
                            if source_is_url(path) {
                                todo!("check URL");
                            } else {
                                let path = adjust_shared_library_path(Path::new(&path))?;
                                if !base.join(&path).exists() {
                                    bail!("no shared library at `{}`", path.display());
                                }
                            }
                        }
                        OperatorSource::Python(path) => {
                            has_python_operator = true;
                            if source_is_url(path) {
                                todo!("check URL");
                            } else if !base.join(path).exists() {
                                bail!("no Python library at `{path}`");
                            }
                        }
                        OperatorSource::Wasm(path) => {
                            if source_is_url(path) {
                                todo!("check URL");
                            } else if !base.join(path).exists() {
                                bail!("no WASM library at `{path}`");
                            }
                        }
                    }
                }
            }
        }
    }

    // check that all inputs mappings point to an existing output
    for node in &nodes {
        match &node.kind {
            descriptor::CoreNodeKind::Custom(custom_node) => {
                for (input_id, input) in &custom_node.run_config.inputs {
                    check_input(input, &nodes, &format!("{}/{input_id}", node.id))?;
                }
            }
            descriptor::CoreNodeKind::Runtime(runtime_node) => {
                for operator_definition in &runtime_node.operators {
                    for (input_id, input) in &operator_definition.config.inputs {
                        check_input(
                            input,
                            &nodes,
                            &format!("{}/{}/{input_id}", operator_definition.id, node.id),
                        )?;
                    }
                }
            }
        };
    }

    if has_python_operator {
        check_python_runtime()?;
    }

    if has_shared_lib_operator {
        check_shared_lib_runtime(runtime_path)?;
    }

    Ok(())
}

fn check_input(
    input: &Input,
    nodes: &[super::ResolvedNode],
    input_id_str: &str,
) -> Result<(), eyre::ErrReport> {
    match &input.mapping {
        InputMapping::Timer { interval: _ } => {}
        InputMapping::User(UserInputMapping { source, output }) => {
            let source_node = nodes.iter().find(|n| &n.id == source).ok_or_else(|| {
                eyre!("source node `{source}` mapped to input `{input_id_str}` does not exist",)
            })?;
            match &source_node.kind {
                CoreNodeKind::Custom(custom_node) => {
                    if !custom_node.run_config.outputs.contains(output) {
                        bail!(
                            "output `{source}/{output}` mapped to \
                            input `{input_id_str}` does not exist",
                        );
                    }
                }
                CoreNodeKind::Runtime(runtime) => {
                    let (operator_id, output) = output.split_once('/').unwrap_or_default();
                    let operator_id = OperatorId::from(operator_id.to_owned());
                    let output = DataId::from(output.to_owned());

                    let operator = runtime
                        .operators
                        .iter()
                        .find(|o| o.id == operator_id)
                        .ok_or_else(|| {
                            eyre!(
                                "source operator `{source}/{operator_id}` used \
                                for input `{input_id_str}` does not exist",
                            )
                        })?;

                    if !operator.config.outputs.contains(&output) {
                        bail!(
                            "output `{source}/{operator_id}/{output}` mapped to \
                            input `{input_id_str}` does not exist",
                        );
                    }
                }
            }
        }
    };
    Ok(())
}

fn check_shared_lib_runtime(runtime_path: Option<PathBuf>) -> eyre::Result<()> {
    // Check if runtime path exists
    let runtime_bin = if let Some(runtime_bin) = runtime_path {
        if runtime_bin.with_extension(EXE_EXTENSION).is_file() == false {
            bail!(
                "Provided Dora Runtime could not be found: {}",
                runtime_bin.display()
            )
        } else {
            runtime_bin
        }
    } else {
        match which::which("dora-runtime") {
            Err(err) => {
                bail!("Dora Runtime binary could not be found: {}", err);
            }
            Ok(runtime_bin) => runtime_bin,
        }
    };

    // Get runtime version
    let mut command = Command::new(runtime_bin.clone());
    command.arg("--version");
    let result = command.spawn().wrap_err(format!(
        "Could not find version of the dora-runtime. Please reinstall dora-runtime"
    ))?;
    let stdout = result
        .wait_with_output()
        .wrap_err(format!(
            "Could not retrieve dora version from dora-runtime. Please reinstall dora-runtime`"
        ))?
        .stdout;
    let rust_version = String::from_utf8(stdout).wrap_err("Could not read python version")?;

    // Check runtime version
    if rust_version == format!("dora-runtime {VERSION}") {
        bail!(
            "Dora Runtime version {:#?} should be {}",
            rust_version,
            VERSION
        )
    };

    Ok(())
}

fn check_python_runtime() -> eyre::Result<()> {
    // Check if python dora-rs is installed and match cli version
    let reinstall_command =
        format!("Please reinstall it with: `pip install dora-rs=={VERSION} --force`");
    let mut command = Command::new("python3");
    command.args([
        "-c",
        &format!(
            "
import dora;
assert dora.__version__=='{VERSION}',  'Python dora-rs should be {VERSION}. {reinstall_command}'
        "
        ),
    ]);
    let mut result = command
        .spawn()
        .wrap_err(format!("Could not spawn python dora-rs command."))?;
    let status = result.wait().wrap_err(format!(
        "Could not get exit status when checking python dora-rs"
    ))?;

    if !status.success() {
        bail!("Something went wrong with Python dora-rs. {reinstall_command}")
    }

    Ok(())
}
