use crate::{
    adjust_shared_library_path,
    config::{DataId, Input, InputMapping, OperatorId, UserInputMapping},
    descriptor::{self, source_is_url, CoreNodeKind, OperatorSource},
};

use eyre::{bail, eyre, Context};
use std::{env::consts::EXE_EXTENSION, path::Path, process::Command};
use tracing::info;

use super::{Descriptor, SHELL_SOURCE};
const VERSION: &str = env!("CARGO_PKG_VERSION");

pub fn check_dataflow(dataflow: &Descriptor, working_dir: &Path) -> eyre::Result<()> {
    if dataflow.daemon_config.is_some() {
        tracing::warn!("ignoring deprecated `daemon_config` key in dataflow config");
    }
    if dataflow.communication.zenoh.is_some() {
        tracing::warn!("ignoring deprecated `communication.zenoh` key in dataflow config");
    }

    let nodes = dataflow.resolve_aliases_and_set_defaults();
    let mut has_python_operator = false;

    // check that nodes and operators exist
    for node in &nodes {
        match &node.kind {
            descriptor::CoreNodeKind::Custom(node) => match node.source.as_str() {
                SHELL_SOURCE => (),
                source => {
                    if source_is_url(source) {
                        info!("{source} is a URL."); // TODO: Implement url check.
                    } else {
                        let raw = Path::new(source);
                        let path = if raw.extension().is_none() {
                            raw.with_extension(EXE_EXTENSION)
                        } else {
                            raw.to_owned()
                        };
                        working_dir
                            .join(&path)
                            .canonicalize()
                            .wrap_err_with(|| format!("no node exists at `{}`", path.display()))?;
                    };
                }
            },
            descriptor::CoreNodeKind::Runtime(node) => {
                for operator_definition in &node.operators {
                    match &operator_definition.config.source {
                        OperatorSource::SharedLibrary(path) => {
                            if source_is_url(path) {
                                info!("{path} is a URL."); // TODO: Implement url check.
                            } else {
                                let path = adjust_shared_library_path(Path::new(&path))?;
                                if !working_dir.join(&path).exists() {
                                    bail!("no shared library at `{}`", path.display());
                                }
                            }
                        }
                        OperatorSource::Python(path) => {
                            has_python_operator = true;
                            if source_is_url(path) {
                                info!("{path} is a URL."); // TODO: Implement url check.
                            } else if !working_dir.join(path).exists() {
                                bail!("no Python library at `{path}`");
                            }
                        }
                        OperatorSource::Wasm(path) => {
                            if source_is_url(path) {
                                info!("{path} is a URL."); // TODO: Implement url check.
                            } else if !working_dir.join(path).exists() {
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
assert dora.__version__=='{VERSION}',  'Python dora-rs should be {VERSION}, but current version is %s. {reinstall_command}' % (dora.__version__)
        "
        ),
    ]);
    let mut result = command
        .spawn()
        .wrap_err("Could not spawn python dora-rs command.")?;
    let status = result
        .wait()
        .wrap_err("Could not get exit status when checking python dora-rs")?;

    if !status.success() {
        bail!("Something went wrong with Python dora-rs. {reinstall_command}")
    }

    Ok(())
}
