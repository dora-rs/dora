use crate::graph::read_descriptor;
use dora_core::{
    adjust_shared_library_path,
    descriptor::{self, CoreNodeKind, InputMapping, OperatorSource, UserInputMapping},
};
use eyre::{bail, eyre, Context};
use std::{env::consts::EXE_EXTENSION, path::Path};

pub fn check(dataflow_path: &Path, runtime: &Path) -> eyre::Result<()> {
    let runtime = runtime.with_extension(EXE_EXTENSION);
    let descriptor = read_descriptor(&dataflow_path).wrap_err_with(|| {
        format!(
            "failed to read dataflow descriptor at {}",
            dataflow_path.display()
        )
    })?;
    let base = dataflow_path
        .canonicalize()
        .unwrap()
        .parent()
        .unwrap()
        .to_owned();

    let nodes = descriptor.resolve_aliases();

    if nodes
        .iter()
        .any(|n| matches!(n.kind, CoreNodeKind::Runtime(_)))
        && !runtime.is_file()
    {
        bail!(
            "There is no runtime at {}, or it is not a file",
            runtime.display()
        );
    }

    // check that nodes and operators exist
    for node in &nodes {
        match &node.kind {
            descriptor::CoreNodeKind::Custom(node) => {
                let mut args = node.run.split_ascii_whitespace();
                let raw = Path::new(
                    args.next()
                        .ok_or_else(|| eyre!("`run` field must not be empty"))?,
                );
                let path = if raw.extension().is_none() {
                    raw.with_extension(EXE_EXTENSION)
                } else {
                    raw.to_owned()
                };
                base.join(&path)
                    .canonicalize()
                    .wrap_err_with(|| format!("no node exists at `{}`", path.display()))?;
            }
            descriptor::CoreNodeKind::Runtime(node) => {
                for operator_definition in &node.operators {
                    match &operator_definition.config.source {
                        OperatorSource::SharedLibrary(path) => {
                            let path = adjust_shared_library_path(path)?;

                            if !base.join(&path).exists() {
                                bail!("no shared library at `{}`", path.display());
                            }
                        }
                        OperatorSource::Python(path) => {
                            if !base.join(&path).exists() {
                                bail!("no Python library at `{}`", path.display());
                            }
                        }
                        OperatorSource::Wasm(path) => {
                            if !base.join(&path).exists() {
                                bail!("no WASM library at `{}`", path.display());
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
                for (input_id, mapping) in &custom_node.run_config.inputs {
                    check_input(mapping, &nodes, &format!("{}/{input_id}", node.id))?;
                }
            }
            descriptor::CoreNodeKind::Runtime(runtime_node) => {
                for operator_definition in &runtime_node.operators {
                    for (input_id, mapping) in &operator_definition.config.inputs {
                        check_input(
                            mapping,
                            &nodes,
                            &format!("{}/{}/{input_id}", operator_definition.id, node.id),
                        )?;
                    }
                }
            }
        };
    }

    Ok(())
}

fn check_input(
    mapping: &InputMapping,
    nodes: &[dora_core::descriptor::ResolvedNode],
    input_id_str: &str,
) -> Result<(), eyre::ErrReport> {
    Ok(match mapping {
        InputMapping::Timer { interval: _ } => {}
        InputMapping::User(UserInputMapping {
            source,
            operator,
            output,
        }) => {
            let source_node = nodes.iter().find(|n| &n.id == source).ok_or_else(|| {
                eyre!("source node `{source}` mapped to input `{input_id_str}` does not exist",)
            })?;
            if let Some(operator_id) = operator {
                let operator = match &source_node.kind {
                    CoreNodeKind::Runtime(runtime) => {
                        let operator = runtime.operators.iter().find(|o| &o.id == operator_id);
                        operator.ok_or_else(|| {
                            eyre!(
                                "source operator `{source}/{operator_id}` used \
                                for input `{input_id_str}` does not exist",
                            )
                        })?
                    }
                    CoreNodeKind::Custom(_) => {
                        bail!(
                            "input `{input_id_str}` references operator \
                            `{source}/{operator_id}`, but `{source}` is a \
                            custom node",
                        );
                    }
                };

                if !operator.config.outputs.contains(output) {
                    bail!(
                        "output `{source}/{operator_id}/{output}` mapped to \
                        input `{input_id_str}` does not exist",
                    );
                }
            } else {
                match &source_node.kind {
                    CoreNodeKind::Runtime(_) => bail!(
                        "input `{input_id_str}` references output \
                        `{source}/{output}`, but `{source}` is a \
                        runtime node",
                    ),
                    CoreNodeKind::Custom(custom_node) => {
                        if !custom_node.run_config.outputs.contains(output) {
                            bail!(
                                "output `{source}/{output}` mapped to \
                                input `{input_id_str}` does not exist",
                            );
                        }
                    }
                }
            }
        }
    })
}
