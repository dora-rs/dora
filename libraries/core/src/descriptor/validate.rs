use crate::{
    adjust_shared_library_path,
    descriptor::{self, source_is_url},
    get_python_path,
};

use adora_message::{
    config::{Input, InputMapping, UserInputMapping},
    descriptor::{CoreNodeKind, DYNAMIC_SOURCE, OperatorSource, ResolvedNode, SHELL_SOURCE},
    id::{DataId, NodeId, OperatorId},
};
use eyre::{Context, bail, eyre};
use std::{collections::BTreeMap, path::Path, process::Command};
use tracing::info;

use super::{Descriptor, DescriptorExt, resolve_path};
const VERSION: &str = env!("CARGO_PKG_VERSION");

pub fn check_dataflow(
    dataflow: &Descriptor,
    working_dir: &Path,
    remote_daemon_id: Option<&[&str]>,
    coordinator_is_remote: bool,
) -> eyre::Result<()> {
    let nodes = dataflow.resolve_aliases_and_set_defaults()?;
    let mut has_python_operator = false;

    // check that nodes and operators exist
    for node in nodes.values() {
        match &node.kind {
            descriptor::CoreNodeKind::Custom(custom) => match &custom.source {
                adora_message::descriptor::NodeSource::Local => match custom.path.as_str() {
                    SHELL_SOURCE => (),
                    DYNAMIC_SOURCE => (),
                    source => {
                        if source_is_url(source) {
                            info!("{source} is a URL."); // TODO: Implement url check.
                        } else if let Some(remote_daemon_id) = remote_daemon_id {
                            if let Some(deploy) = &node.deploy {
                                if let Some(machine) = &deploy.machine {
                                    if remote_daemon_id.contains(&machine.as_str())
                                        || coordinator_is_remote
                                    {
                                        info!("skipping path check for remote node `{}`", node.id);
                                    }
                                }
                            }
                        } else if custom.build.is_some() {
                            info!("skipping path check for node with build command");
                        } else {
                            resolve_path(source, working_dir).wrap_err_with(|| {
                                format!("Could not find source path `{source}`")
                            })?;
                        };
                    }
                },
                adora_message::descriptor::NodeSource::GitBranch { .. } => {
                    info!("skipping check for node with git source");
                }
            },
            descriptor::CoreNodeKind::Runtime(node) => {
                for operator_definition in &node.operators {
                    match &operator_definition.config.source {
                        OperatorSource::SharedLibrary(path) => {
                            if source_is_url(path) {
                                info!("{path} is a URL."); // TODO: Implement url check.
                            } else if operator_definition.config.build.is_some() {
                                info!("skipping path check for operator with build command");
                            } else {
                                let path = adjust_shared_library_path(Path::new(&path))?;
                                if !working_dir.join(&path).exists() {
                                    bail!("no shared library at `{}`", path.display());
                                }
                            }
                        }
                        OperatorSource::Python(python_source) => {
                            has_python_operator = true;
                            let path = &python_source.source;
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
    for node in nodes.values() {
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

    // Check that nodes can resolve `send_stdout_as`, `send_logs_as`, `min_log_level`
    for node in nodes.values() {
        node.send_stdout_as()
            .context("Could not resolve `send_stdout_as` configuration")?;
        node.send_logs_as()
            .context("Could not resolve `send_logs_as` configuration")?;
        node.min_log_level()
            .context("Could not resolve `min_log_level` configuration")?;
    }

    if has_python_operator {
        check_python_runtime()?;
    }

    Ok(())
}

pub trait ResolvedNodeExt {
    fn send_stdout_as(&self) -> eyre::Result<Option<String>>;
    fn send_logs_as(&self) -> eyre::Result<Option<String>>;
    fn min_log_level(&self) -> eyre::Result<Option<adora_message::common::LogLevelOrStdout>>;
}

impl ResolvedNodeExt for ResolvedNode {
    fn send_stdout_as(&self) -> eyre::Result<Option<String>> {
        match &self.kind {
            // TODO: Split stdout between operators
            CoreNodeKind::Runtime(n) => {
                let count = n
                    .operators
                    .iter()
                    .filter(|op| op.config.send_stdout_as.is_some())
                    .count();
                if count == 1 && n.operators.len() > 1 {
                    tracing::warn!(
                        "All stdout from all operators of a runtime are going to be sent in the selected `send_stdout_as` operator."
                    )
                } else if count > 1 {
                    return Err(eyre!(
                        "More than one `send_stdout_as` entries for a runtime node. Please only use one `send_stdout_as` per runtime."
                    ));
                }
                Ok(n.operators.iter().find_map(|op| {
                    op.config
                        .send_stdout_as
                        .clone()
                        .map(|stdout| format!("{}/{}", op.id, stdout))
                }))
            }
            CoreNodeKind::Custom(n) => Ok(n.send_stdout_as.clone()),
        }
    }

    fn send_logs_as(&self) -> eyre::Result<Option<String>> {
        match &self.kind {
            CoreNodeKind::Runtime(n) => {
                let count = n
                    .operators
                    .iter()
                    .filter(|op| op.config.send_logs_as.is_some())
                    .count();
                if count > 1 {
                    return Err(eyre!(
                        "More than one `send_logs_as` entries for a runtime node. Please only use one `send_logs_as` per runtime."
                    ));
                }
                Ok(n.operators.iter().find_map(|op| {
                    op.config
                        .send_logs_as
                        .clone()
                        .map(|logs| format!("{}/{}", op.id, logs))
                }))
            }
            CoreNodeKind::Custom(n) => Ok(n.send_logs_as.clone()),
        }
    }

    fn min_log_level(&self) -> eyre::Result<Option<adora_message::common::LogLevelOrStdout>> {
        let level_str = match &self.kind {
            CoreNodeKind::Runtime(n) => {
                // Use the first operator's min_log_level (only one allowed)
                let levels: Vec<_> = n
                    .operators
                    .iter()
                    .filter_map(|op| op.config.min_log_level.as_deref())
                    .collect();
                if levels.len() > 1 {
                    return Err(eyre!(
                        "More than one `min_log_level` entries for a runtime node. Please only use one `min_log_level` per runtime."
                    ));
                }
                levels.first().map(|s| s.to_string())
            }
            CoreNodeKind::Custom(n) => n.min_log_level.clone(),
        };
        match level_str {
            None => Ok(None),
            Some(s) => {
                let level = parse_log_level(&s)?;
                Ok(Some(level))
            }
        }
    }
}

fn parse_log_level(s: &str) -> eyre::Result<adora_message::common::LogLevelOrStdout> {
    match s.to_lowercase().as_str() {
        "error" => Ok(adora_message::common::LogLevelOrStdout::LogLevel(
            log::Level::Error,
        )),
        "warn" => Ok(adora_message::common::LogLevelOrStdout::LogLevel(
            log::Level::Warn,
        )),
        "info" => Ok(adora_message::common::LogLevelOrStdout::LogLevel(
            log::Level::Info,
        )),
        "debug" => Ok(adora_message::common::LogLevelOrStdout::LogLevel(
            log::Level::Debug,
        )),
        "trace" => Ok(adora_message::common::LogLevelOrStdout::LogLevel(
            log::Level::Trace,
        )),
        "stdout" => Ok(adora_message::common::LogLevelOrStdout::Stdout),
        _ => bail!(
            "invalid min_log_level: '{s}', expected one of: error, warn, info, debug, trace, stdout"
        ),
    }
}

fn check_input(
    input: &Input,
    nodes: &BTreeMap<NodeId, super::ResolvedNode>,
    input_id_str: &str,
) -> Result<(), eyre::ErrReport> {
    match &input.mapping {
        InputMapping::Timer { interval: _ } => {}
        InputMapping::User(UserInputMapping { source, output }) => {
            let source_node = nodes.values().find(|n| &n.id == source).ok_or_else(|| {
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
    // Check if python adora-rs is installed and match cli version
    let reinstall_command =
        format!("Please reinstall it with: `pip install adora-rs=={VERSION} --force`");
    let mut command = Command::new(get_python_path().context("Could not get python binary")?);
    command.args([
        "-c",
        &format!(
            "
import adora;
assert adora.__version__=='{VERSION}',  'Python adora-rs should be {VERSION}, but current version is %s. {reinstall_command}' % (adora.__version__)
        "
        ),
    ]);
    let mut result = command
        .spawn()
        .wrap_err("Could not spawn python adora-rs command.")?;
    let status = result
        .wait()
        .wrap_err("Could not get exit status when checking python adora-rs")?;

    if !status.success() {
        bail!("Something went wrong with Python adora-rs. {reinstall_command}")
    }

    Ok(())
}
