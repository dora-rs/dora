use std::{collections::BTreeMap, path::PathBuf};

use dora_core::{
    build::run_build_command,
    descriptor::{Descriptor, NodeExt, SINGLE_OPERATOR_DEFAULT_ID},
};
use dora_message::{
    common::GitSource,
    id::{NodeId, OperatorId},
};
use eyre::Context;

use crate::session::DataflowSession;

pub fn build_dataflow_locally(
    dataflow: Descriptor,
    git_sources: &BTreeMap<NodeId, GitSource>,
    dataflow_session: &DataflowSession,
    working_dir: PathBuf,
    uv: bool,
) -> eyre::Result<()> {
    let default_op_id = OperatorId::from(SINGLE_OPERATOR_DEFAULT_ID.to_string());
    let (stdout_tx, mut stdout) = tokio::sync::mpsc::channel::<std::io::Result<String>>(10);

    tokio::spawn(async move {
        while let Some(line) = stdout.recv().await {
            println!(
                "{}",
                line.unwrap_or_else(|err| format!("io err: {}", err.kind()))
            );
        }
    });

    for node in dataflow.nodes {
        match node.kind()? {
            dora_core::descriptor::NodeKind::Standard(_) => {
                let Some(build) = node.build.as_deref() else {
                    continue;
                };
                run_build_command(build, &working_dir, uv, &node.env, stdout_tx.clone())
                    .with_context(|| {
                        format!("build command failed for standard node `{}`", node.id)
                    })?
            }
            dora_core::descriptor::NodeKind::Runtime(runtime_node) => {
                for operator in &runtime_node.operators {
                    let Some(build) = operator.config.build.as_deref() else {
                        continue;
                    };
                    run_build_command(build, &working_dir, uv, &node.env, stdout_tx.clone())
                        .with_context(|| {
                            format!(
                                "build command failed for operator `{}/{}`",
                                node.id, operator.id
                            )
                        })?;
                }
            }
            dora_core::descriptor::NodeKind::Custom(custom_node) => {
                let Some(build) = custom_node.build.as_deref() else {
                    continue;
                };
                run_build_command(build, &working_dir, uv, &node.env, stdout_tx.clone())
                    .with_context(|| {
                        format!("build command failed for custom node `{}`", node.id)
                    })?
            }
            dora_core::descriptor::NodeKind::Operator(operator) => {
                let Some(build) = operator.config.build.as_deref() else {
                    continue;
                };
                run_build_command(build, &working_dir, uv, &node.env, stdout_tx.clone())
                    .with_context(|| {
                        format!(
                            "build command failed for operator `{}/{}`",
                            node.id,
                            operator.id.as_ref().unwrap_or(&default_op_id)
                        )
                    })?;
            }
        }
    }
    std::mem::drop(stdout_tx);

    Ok(())
}
