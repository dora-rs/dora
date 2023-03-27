use crate::{
    adjust_shared_library_path,
    config::{DataId, Input, InputMapping, OperatorId, UserInputMapping},
    descriptor::{self, source_is_url, CoreNodeKind, OperatorSource},
};

use eyre::{bail, eyre, Context};
use std::{env::consts::EXE_EXTENSION, path::Path};

use super::Descriptor;
pub fn check_dataflow(dataflow: &Descriptor) -> eyre::Result<()> {
    let nodes = dataflow.resolve_aliases();
    let base = &dataflow.base_path;

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

    //   check_environment()?;

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
