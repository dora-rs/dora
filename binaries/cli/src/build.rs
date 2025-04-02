use dora_core::{
    build::run_build_command,
    config::OperatorId,
    descriptor::{Descriptor, DescriptorExt, NodeExt, SINGLE_OPERATOR_DEFAULT_ID},
};
use eyre::Context;

use crate::resolve_dataflow;

pub fn build(dataflow: String, uv: bool) -> eyre::Result<()> {
    let dataflow = resolve_dataflow(dataflow).context("could not resolve dataflow")?;
    let descriptor = Descriptor::blocking_read(&dataflow)?;
    let dataflow_absolute = if dataflow.is_relative() {
        std::env::current_dir().unwrap().join(dataflow)
    } else {
        dataflow.to_owned()
    };
    let working_dir = dataflow_absolute.parent().unwrap();

    let default_op_id = OperatorId::from(SINGLE_OPERATOR_DEFAULT_ID.to_string());

    for node in descriptor.nodes {
        match node.kind()? {
            dora_core::descriptor::NodeKind::Standard(_) => {
                if let Some(build) = &node.build {
                    run_build_command(build, working_dir, uv, &node.env).with_context(|| {
                        format!("build command failed for standard node `{}`", node.id)
                    })?
                }
            }
            dora_core::descriptor::NodeKind::Runtime(runtime_node) => {
                for operator in &runtime_node.operators {
                    if let Some(build) = &operator.config.build {
                        run_build_command(build, working_dir, uv, &node.env).with_context(
                            || {
                                format!(
                                    "build command failed for operator `{}/{}`",
                                    node.id, operator.id
                                )
                            },
                        )?;
                    }
                }
            }
            dora_core::descriptor::NodeKind::Custom(custom_node) => {
                if let Some(build) = &custom_node.build {
                    run_build_command(build, working_dir, uv, &node.env).with_context(|| {
                        format!("build command failed for custom node `{}`", node.id)
                    })?
                }
            }
            dora_core::descriptor::NodeKind::Operator(operator) => {
                if let Some(build) = &operator.config.build {
                    run_build_command(build, working_dir, uv, &node.env).with_context(|| {
                        format!(
                            "build command failed for operator `{}/{}`",
                            node.id,
                            operator.id.as_ref().unwrap_or(&default_op_id)
                        )
                    })?
                }
            }
        }
    }

    Ok(())
}
