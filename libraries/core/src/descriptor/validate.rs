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
use std::{
    collections::{BTreeMap, BTreeSet},
    path::Path,
    process::Command,
};
use tracing::info;

use super::{Descriptor, DescriptorExt, resolve_path};
const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Validate input/output wiring without checking path existence.
///
/// This is safe to run before building nodes. It verifies every input
/// reference points to a declared output on the source node.
pub fn check_wiring(dataflow: &Descriptor) -> eyre::Result<()> {
    let nodes = dataflow.resolve_aliases_and_set_defaults()?;

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

    Ok(())
}

pub fn check_dataflow(
    dataflow: &Descriptor,
    working_dir: &Path,
    remote_daemon_id: Option<&[&str]>,
    coordinator_is_remote: bool,
) -> eyre::Result<()> {
    // validate ROS2 bridge configs before resolution
    for node in &dataflow.nodes {
        if let Some(ros2) = &node.ros2 {
            validate_ros2_config(&node.id, ros2, &node.inputs, &node.outputs)?;
        }
    }

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
    check_wiring(dataflow)?;

    // Check that nodes can resolve `send_stdout_as`, `send_logs_as`, `min_log_level`
    for node in nodes.values() {
        node.send_stdout_as()
            .context("Could not resolve `send_stdout_as` configuration")?;
        node.send_logs_as()
            .context("Could not resolve `send_logs_as` configuration")?;
        node.min_log_level()
            .context("Could not resolve `min_log_level` configuration")?;
        node.max_log_size()
            .context("Could not resolve `max_log_size` configuration")?;
        node.max_rotated_files()
            .context("Could not resolve `max_rotated_files` configuration")?;
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
    fn max_log_size(&self) -> eyre::Result<Option<u64>>;
    fn max_rotated_files(&self) -> eyre::Result<Option<u32>>;
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

    fn max_log_size(&self) -> eyre::Result<Option<u64>> {
        let size_str = match &self.kind {
            CoreNodeKind::Runtime(n) => {
                let sizes: Vec<_> = n
                    .operators
                    .iter()
                    .filter_map(|op| op.config.max_log_size.as_deref())
                    .collect();
                if sizes.len() > 1 {
                    return Err(eyre!(
                        "More than one `max_log_size` entries for a runtime node. Please only use one `max_log_size` per runtime."
                    ));
                }
                sizes.first().map(|s| s.to_string())
            }
            CoreNodeKind::Custom(n) => n.max_log_size.clone(),
        };
        match size_str {
            None => Ok(None),
            Some(s) => {
                let bytes = parse_byte_size(&s)?;
                Ok(Some(bytes))
            }
        }
    }

    fn max_rotated_files(&self) -> eyre::Result<Option<u32>> {
        let value = match &self.kind {
            CoreNodeKind::Runtime(n) => {
                let values: Vec<_> = n
                    .operators
                    .iter()
                    .filter_map(|op| op.config.max_rotated_files)
                    .collect();
                if values.len() > 1 {
                    return Err(eyre!(
                        "More than one `max_rotated_files` entries for a runtime node. Please only use one `max_rotated_files` per runtime."
                    ));
                }
                values.first().copied()
            }
            CoreNodeKind::Custom(n) => n.max_rotated_files,
        };
        if let Some(n) = value {
            if n == 0 {
                bail!("`max_rotated_files` must be at least 1");
            }
            if n > 100 {
                bail!("`max_rotated_files` must not exceed 100");
            }
        }
        Ok(value)
    }
}

fn parse_byte_size(s: &str) -> eyre::Result<u64> {
    let s = s.trim();
    let (num_str, unit) = match s.find(|c: char| c.is_ascii_alphabetic()) {
        Some(pos) => (&s[..pos], s[pos..].trim().to_uppercase()),
        None => {
            return s
                .parse::<u64>()
                .map_err(|_| eyre!("invalid byte size: '{s}'"));
        }
    };
    let num_str = num_str.trim();
    let multiplier: u64 = match unit.as_str() {
        "B" => 1,
        "KB" | "K" => 1024,
        "MB" | "M" => 1024 * 1024,
        "GB" | "G" => 1024 * 1024 * 1024,
        _ => bail!("unknown byte size unit: '{unit}', expected B, KB, MB, or GB"),
    };
    // Use integer parse when possible to avoid float rounding
    if let Ok(num) = num_str.parse::<u64>() {
        return Ok(num * multiplier);
    }
    let num: f64 = num_str
        .parse()
        .map_err(|_| eyre!("invalid byte size number: '{num_str}'"))?;
    Ok((num * multiplier as f64) as u64)
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
        InputMapping::Timer { interval: _ } | InputMapping::Logs(_) => {}
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

fn validate_ros2_config(
    node_id: &NodeId,
    config: &adora_message::descriptor::Ros2BridgeConfig,
    node_inputs: &BTreeMap<DataId, Input>,
    node_outputs: &BTreeSet<DataId>,
) -> eyre::Result<()> {
    use adora_message::descriptor::{Ros2Direction, Ros2Role};

    // Exactly one of topic, topics, service, action must be set
    let mode_count = [
        config.topic.is_some(),
        config.topics.is_some(),
        config.service.is_some(),
        config.action.is_some(),
    ]
    .iter()
    .filter(|&&v| v)
    .count();
    if mode_count == 0 {
        bail!(
            "node `{node_id}`: ros2 config requires one of \
             `topic`, `topics`, `service`, or `action`"
        );
    }
    if mode_count > 1 {
        bail!(
            "node `{node_id}`: ros2 config has multiple of \
             `topic`, `topics`, `service`, `action` - only one is allowed"
        );
    }

    if let Some(topic) = &config.topic {
        validate_ros2_name(node_id, "topic", topic)?;
        let message_type = config.message_type.as_ref().ok_or_else(|| {
            eyre!("node `{node_id}`: ros2 config with `topic` requires `message_type`")
        })?;
        validate_ros2_type_format(node_id, topic, message_type)?;

        match &config.direction {
            Ros2Direction::Subscribe => {
                if node_outputs.is_empty() {
                    bail!("node `{node_id}`: ros2 subscribe bridge requires at least one output");
                }
            }
            Ros2Direction::Publish => {
                if node_inputs.is_empty() {
                    bail!("node `{node_id}`: ros2 publish bridge requires at least one input");
                }
            }
        }
    } else if let Some(topics) = &config.topics {
        if topics.is_empty() {
            bail!("node `{node_id}`: ros2 `topics` list must not be empty");
        }
        if topics.len() > 64 {
            bail!(
                "node `{node_id}`: ros2 `topics` list has {} entries, maximum is 64",
                topics.len()
            );
        }
        let mut has_subscribe = false;
        let mut has_publish = false;
        for t in topics {
            validate_ros2_name(node_id, "topic", &t.topic)?;
            validate_ros2_type_format(node_id, &t.topic, &t.message_type)?;
            match &t.direction {
                Ros2Direction::Subscribe => has_subscribe = true,
                Ros2Direction::Publish => has_publish = true,
            }
        }
        if has_subscribe && node_outputs.is_empty() {
            bail!(
                "node `{node_id}`: ros2 multi-topic bridge with subscribe topics \
                 requires at least one output"
            );
        }
        if has_publish && node_inputs.is_empty() {
            bail!(
                "node `{node_id}`: ros2 multi-topic bridge with publish topics \
                 requires at least one input"
            );
        }
    } else if let Some(service) = &config.service {
        validate_ros2_name(node_id, "service", service)?;
        let service_type = config.service_type.as_ref().ok_or_else(|| {
            eyre!("node `{node_id}`: ros2 config with `service` requires `service_type`")
        })?;
        validate_ros2_type_format(node_id, service, service_type)?;
        let role = config.role.as_ref().ok_or_else(|| {
            eyre!("node `{node_id}`: ros2 service bridge requires `role` (client or server)")
        })?;
        match role {
            Ros2Role::Client => {
                if node_inputs.is_empty() {
                    bail!(
                        "node `{node_id}`: ros2 service client requires at least one input (request)"
                    );
                }
                if node_outputs.is_empty() {
                    bail!(
                        "node `{node_id}`: ros2 service client requires at least one output (response)"
                    );
                }
            }
            Ros2Role::Server => {
                if node_inputs.is_empty() {
                    bail!(
                        "node `{node_id}`: ros2 service server requires at least one input (response)"
                    );
                }
                if node_outputs.is_empty() {
                    bail!(
                        "node `{node_id}`: ros2 service server requires at least one output (request)"
                    );
                }
            }
        }
    } else if let Some(action) = &config.action {
        validate_ros2_name(node_id, "action", action)?;
        let action_type = config.action_type.as_ref().ok_or_else(|| {
            eyre!("node `{node_id}`: ros2 config with `action` requires `action_type`")
        })?;
        validate_ros2_type_format(node_id, action, action_type)?;
        let role = config
            .role
            .as_ref()
            .ok_or_else(|| eyre!("node `{node_id}`: ros2 action bridge requires `role`"))?;
        match role {
            Ros2Role::Client => {
                if node_inputs.is_empty() {
                    bail!(
                        "node `{node_id}`: ros2 action client requires at least one input (goal)"
                    );
                }
                if node_outputs.is_empty() {
                    bail!(
                        "node `{node_id}`: ros2 action client requires at least one output \
                         (feedback/result)"
                    );
                }
            }
            Ros2Role::Server => {
                if node_inputs.is_empty() {
                    bail!(
                        "node `{node_id}`: ros2 action server requires at least one input \
                         (feedback/result)"
                    );
                }
                if node_outputs.is_empty() {
                    bail!(
                        "node `{node_id}`: ros2 action server requires at least one output (goal)"
                    );
                }
            }
        }
    }

    // Validate QoS config
    validate_ros2_qos(node_id, &config.qos)?;
    if let Some(topics) = &config.topics {
        for t in topics {
            if let Some(qos) = &t.qos {
                validate_ros2_qos(node_id, qos)?;
            }
        }
    }

    Ok(())
}

fn validate_ros2_qos(
    node_id: &NodeId,
    qos: &adora_message::descriptor::Ros2QosConfig,
) -> eyre::Result<()> {
    if let Some(d) = &qos.durability {
        match d.as_str() {
            "volatile" | "transient_local" => {}
            _ => bail!(
                "node `{node_id}`: invalid QoS durability `{d}`, \
                 expected \"volatile\" or \"transient_local\""
            ),
        }
    }
    if let Some(l) = &qos.liveliness {
        match l.as_str() {
            "automatic" | "manual_by_participant" | "manual_by_topic" => {}
            _ => bail!(
                "node `{node_id}`: invalid QoS liveliness `{l}`, \
                 expected \"automatic\", \"manual_by_participant\", or \"manual_by_topic\""
            ),
        }
    }
    if let Some(depth) = qos.keep_last {
        if !(1..=10_000).contains(&depth) {
            bail!(
                "node `{node_id}`: QoS keep_last depth {depth} out of range, \
                 must be between 1 and 10000"
            );
        }
    }
    if let Some(t) = qos.max_blocking_time {
        if !t.is_finite() || t < 0.0 {
            bail!("node `{node_id}`: QoS max_blocking_time must be a finite non-negative number");
        }
    }
    if let Some(t) = qos.lease_duration {
        if !t.is_finite() || t < 0.0 {
            bail!("node `{node_id}`: QoS lease_duration must be a finite non-negative number");
        }
    }
    Ok(())
}

/// Validate a ROS2 name (topic, service, or action) contains only valid characters.
/// ROS2 names allow: ASCII alphanumeric, underscore, and forward slash.
fn validate_ros2_name(node_id: &NodeId, field: &str, name: &str) -> eyre::Result<()> {
    if name.is_empty() {
        bail!("node `{node_id}`: `{field}` must not be empty");
    }
    if !name
        .chars()
        .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '/')
    {
        bail!(
            "node `{node_id}`: invalid `{field}` name `{name}`, \
             only ASCII alphanumeric, underscore, and '/' characters allowed"
        );
    }
    Ok(())
}

/// A non-fatal warning about type annotations in the dataflow.
#[derive(Debug)]
pub struct TypeWarning {
    /// Node that produced the warning.
    pub node_id: String,
    /// Human-readable warning message.
    pub message: String,
}

impl std::fmt::Display for TypeWarning {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "node \"{}\": {}", self.node_id, self.message)
    }
}

/// A type inferred from an upstream annotated output (Phase 3).
#[derive(Debug)]
pub struct TypeInference {
    /// Node that received the inferred type.
    pub node_id: String,
    /// Port that received the inferred type.
    pub port_id: String,
    /// The inferred type URN.
    pub inferred_urn: String,
    /// Source of the inference (e.g. "sensor/reading").
    pub source: String,
}

impl std::fmt::Display for TypeInference {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "inferred {} on {}/{} (from {})",
            self.inferred_urn, self.node_id, self.port_id, self.source
        )
    }
}

/// Result of type checking a dataflow.
pub struct TypeCheckResult {
    /// Type warnings found.
    pub warnings: Vec<TypeWarning>,
    /// Types inferred from upstream annotations (Phase 3).
    pub inferences: Vec<TypeInference>,
}

/// Timer nodes auto-inject this type.
const TIMER_TYPE: &str = "std/core/v1/UInt64";

/// Check type annotations in a dataflow.
///
/// Checks:
/// 1. `output_types` keys exist in `outputs`
/// 2. `input_types` keys exist in `inputs`
/// 3. All type URNs resolve in the registry
/// 4. Connected edges have compatible types (using compatibility graph)
/// 5. Type inference across edges (Phase 3)
/// 6. Metadata annotation validation (Phase 5)
/// 7. Arrow schema validation (Phase 6)
/// 8. Strict mode: warn on unannotated ports connected to annotated ports
pub fn check_type_annotations(
    dataflow: &super::Descriptor,
    registry: &crate::types::TypeRegistry,
) -> Vec<TypeWarning> {
    check_type_annotations_full(dataflow, registry, false).warnings
}

/// Full type checking with strict mode support. Returns warnings + inferences.
pub fn check_type_annotations_full(
    dataflow: &super::Descriptor,
    registry: &crate::types::TypeRegistry,
    strict: bool,
) -> TypeCheckResult {
    use crate::types::{CompatibilityGraph, TypeRule};

    let mut warnings = Vec::new();
    let mut inferences = Vec::new();

    // Build compatibility graph from user rules
    let user_rules: Vec<TypeRule> = dataflow
        .type_rules
        .iter()
        .map(|r| TypeRule {
            from: r.from.clone(),
            to: r.to.clone(),
        })
        .collect();
    let compat = CompatibilityGraph::new(&user_rules);

    // Map of (source_node_id, output_ref) -> type_urn for cross-edge checking.
    let mut output_type_map: BTreeMap<(String, String), String> = BTreeMap::new();

    for node in &dataflow.nodes {
        let nid = node.id.to_string();

        // Check node-level output_types
        check_port_types(
            &nid,
            &node.output_types,
            |id| node.outputs.contains(id),
            "output",
            registry,
            &mut warnings,
        );
        // Register node outputs for cross-edge checking
        for (output_id, urn) in &node.output_types {
            if registry.resolve(urn).is_some() {
                output_type_map.insert((nid.clone(), output_id.to_string()), urn.clone());
            }
        }

        // Check node-level input_types
        check_port_types(
            &nid,
            &node.input_types,
            |id| node.inputs.contains_key(id),
            "input",
            registry,
            &mut warnings,
        );

        // Check metadata annotations (Phase 5)
        check_metadata_annotations(
            &nid,
            &node.output_metadata,
            &node.pattern,
            &node.outputs,
            &mut warnings,
        );

        // Check operator output/input types
        if let Some(op) = &node.operator {
            let op_id = op
                .id
                .as_ref()
                .map(|id| id.to_string())
                .unwrap_or_else(|| super::SINGLE_OPERATOR_DEFAULT_ID.to_string());
            check_port_types(
                &nid,
                &op.config.output_types,
                |id| op.config.outputs.contains(id),
                "output",
                registry,
                &mut warnings,
            );
            for (output_id, urn) in &op.config.output_types {
                if registry.resolve(urn).is_some() {
                    output_type_map
                        .insert((nid.clone(), format!("{op_id}/{output_id}")), urn.clone());
                }
            }
            check_port_types(
                &nid,
                &op.config.input_types,
                |id| op.config.inputs.contains_key(id),
                "input",
                registry,
                &mut warnings,
            );
            check_metadata_annotations(
                &nid,
                &op.config.output_metadata,
                &op.config.pattern,
                &op.config.outputs,
                &mut warnings,
            );
        }
        if let Some(runtime) = &node.operators {
            for op in &runtime.operators {
                let label = format!("{nid}/{}", op.id);
                check_port_types(
                    &label,
                    &op.config.output_types,
                    |id| op.config.outputs.contains(id),
                    "output",
                    registry,
                    &mut warnings,
                );
                for (output_id, urn) in &op.config.output_types {
                    if registry.resolve(urn).is_some() {
                        output_type_map
                            .insert((nid.clone(), format!("{}/{output_id}", op.id)), urn.clone());
                    }
                }
                check_port_types(
                    &label,
                    &op.config.input_types,
                    |id| op.config.inputs.contains_key(id),
                    "input",
                    registry,
                    &mut warnings,
                );
                check_metadata_annotations(
                    &label,
                    &op.config.output_metadata,
                    &op.config.pattern,
                    &op.config.outputs,
                    &mut warnings,
                );
            }
        }
    }

    // Cross-edge type checking + inference (Phase 3 + 4)
    // Also check timer inputs against input_types annotations.
    for node in &dataflow.nodes {
        let nid = node.id.to_string();
        let timer_types = timer_input_types(&node.inputs);
        check_edge_mismatches_with_compat(
            &nid,
            &node.input_types,
            &node.inputs,
            &output_type_map,
            &timer_types,
            &compat,
            registry,
            strict,
            &mut warnings,
            &mut inferences,
        );

        if let Some(op) = &node.operator {
            let op_timer = timer_input_types(&op.config.inputs);
            check_edge_mismatches_with_compat(
                &nid,
                &op.config.input_types,
                &op.config.inputs,
                &output_type_map,
                &op_timer,
                &compat,
                registry,
                strict,
                &mut warnings,
                &mut inferences,
            );
        }
        if let Some(runtime) = &node.operators {
            for op in &runtime.operators {
                let label = format!("{nid}/{}", op.id);
                let op_timer = timer_input_types(&op.config.inputs);
                check_edge_mismatches_with_compat(
                    &label,
                    &op.config.input_types,
                    &op.config.inputs,
                    &output_type_map,
                    &op_timer,
                    &compat,
                    registry,
                    strict,
                    &mut warnings,
                    &mut inferences,
                );
            }
        }
    }

    TypeCheckResult {
        warnings,
        inferences,
    }
}

/// Build a map of (input_id -> timer_type) for Timer-mapped inputs.
fn timer_input_types(inputs: &BTreeMap<DataId, Input>) -> BTreeMap<DataId, String> {
    // Quick check: skip allocation if no timer inputs
    if !inputs
        .values()
        .any(|i| matches!(i.mapping, InputMapping::Timer { .. }))
    {
        return BTreeMap::new();
    }
    let mut result = BTreeMap::new();
    for (input_id, input) in inputs {
        if matches!(input.mapping, InputMapping::Timer { .. }) {
            result.insert(input_id.clone(), TIMER_TYPE.to_string());
        }
    }
    result
}

/// Validate that port type keys exist in the port list and URNs resolve.
fn check_port_types(
    node_id: &str,
    type_map: &BTreeMap<DataId, String>,
    contains: impl Fn(&DataId) -> bool,
    port_kind: &str,
    registry: &crate::types::TypeRegistry,
    warnings: &mut Vec<TypeWarning>,
) {
    for (port_id, urn) in type_map {
        if !contains(port_id) {
            warnings.push(TypeWarning {
                node_id: node_id.to_string(),
                message: format!(
                    "{port_kind}_types key \"{port_id}\" not found in {port_kind}s list"
                ),
            });
        }
        if registry.resolve(urn).is_none() {
            let hint = registry
                .suggest(urn)
                .map(|s| format!(" (did you mean \"{s}\"?)"))
                .unwrap_or_default();
            warnings.push(TypeWarning {
                node_id: node_id.to_string(),
                message: format!("unknown type \"{urn}\" on {port_kind} \"{port_id}\"{hint}"),
            });
        }
    }
}

/// Check metadata annotations on outputs (Phase 5).
fn check_metadata_annotations(
    node_id: &str,
    output_metadata: &BTreeMap<DataId, Vec<String>>,
    pattern: &Option<String>,
    outputs: &BTreeSet<DataId>,
    warnings: &mut Vec<TypeWarning>,
) {
    // Check explicit output_metadata keys exist in outputs
    for output_id in output_metadata.keys() {
        if !outputs.contains(output_id) {
            warnings.push(TypeWarning {
                node_id: node_id.to_string(),
                message: format!("output_metadata key \"{output_id}\" not found in outputs list"),
            });
        }
    }

    // Validate pattern shorthand
    if let Some(pat) = pattern {
        if crate::types::pattern_metadata_keys(pat).is_none() {
            warnings.push(TypeWarning {
                node_id: node_id.to_string(),
                message: format!(
                    "unknown pattern \"{pat}\", expected one of: \
                     service-server, service-client, action-server, action-client"
                ),
            });
        }
    }
}

/// Check cross-edge type mismatches using compatibility graph.
///
/// Also performs type inference (Phase 3) and strict-mode unannotated checks.
/// `timer_types` maps input IDs to auto-injected timer type URNs.
#[allow(clippy::too_many_arguments)]
fn check_edge_mismatches_with_compat(
    node_id: &str,
    input_types: &BTreeMap<DataId, String>,
    inputs: &BTreeMap<DataId, Input>,
    output_type_map: &BTreeMap<(String, String), String>,
    timer_types: &BTreeMap<DataId, String>,
    compat: &crate::types::CompatibilityGraph,
    registry: &crate::types::TypeRegistry,
    strict: bool,
    warnings: &mut Vec<TypeWarning>,
    inferences: &mut Vec<TypeInference>,
) {
    for (input_id, input) in inputs {
        match &input.mapping {
            InputMapping::User(mapping) => {
                let key = (mapping.source.to_string(), mapping.output.to_string());
                let upstream_urn = output_type_map.get(&key);
                let downstream_urn = input_types.get(input_id);

                match (upstream_urn, downstream_urn) {
                    (Some(out_urn), Some(in_urn)) => {
                        if !compat.is_compatible(out_urn, in_urn) {
                            let schema_detail = check_schema_compat(out_urn, in_urn, registry);
                            let detail =
                                schema_detail.map(|d| format!(" ({d})")).unwrap_or_default();
                            warnings.push(TypeWarning {
                                node_id: node_id.to_string(),
                                message: format!(
                                    "type mismatch on input \"{input_id}\": \
                                     upstream {}/{} declares \"{out_urn}\", \
                                     but expected \"{in_urn}\"{detail}",
                                    mapping.source, mapping.output,
                                ),
                            });
                        }
                    }
                    (Some(out_urn), None) => {
                        inferences.push(TypeInference {
                            node_id: node_id.to_string(),
                            port_id: input_id.to_string(),
                            inferred_urn: out_urn.clone(),
                            source: format!("{}/{}", mapping.source, mapping.output),
                        });
                    }
                    (None, Some(in_urn)) if strict => {
                        warnings.push(TypeWarning {
                            node_id: node_id.to_string(),
                            message: format!(
                                "input \"{input_id}\" expects type \"{in_urn}\" but upstream \
                                 {}/{} has no type annotation",
                                mapping.source, mapping.output,
                            ),
                        });
                    }
                    _ => {}
                }
            }
            InputMapping::Timer { .. } => {
                // Timer inputs auto-inject std/core/v1/UInt64
                if let Some(expected_urn) = input_types.get(input_id) {
                    let timer_urn = timer_types
                        .get(input_id)
                        .map(|s| s.as_str())
                        .unwrap_or(TIMER_TYPE);
                    if !compat.is_compatible(timer_urn, expected_urn) {
                        warnings.push(TypeWarning {
                            node_id: node_id.to_string(),
                            message: format!(
                                "type mismatch on input \"{input_id}\": \
                                 timer provides \"{timer_urn}\", \
                                 but expected \"{expected_urn}\"",
                            ),
                        });
                    }
                }
            }
            // Log subscriptions deliver JSON strings — skip type checking.
            InputMapping::Logs(_) => {}
        }
    }
}

/// Check schema-level compatibility for struct types (Phase 6).
/// Returns an optional detail string for the error message.
fn check_schema_compat(
    out_urn: &str,
    in_urn: &str,
    registry: &crate::types::TypeRegistry,
) -> Option<String> {
    let out_def = registry.resolve(out_urn)?;
    let in_def = registry.resolve(in_urn)?;
    let out_schema = out_def.to_arrow_schema()?;
    let in_schema = in_def.to_arrow_schema()?;
    // in_schema = expected (consumer), out_schema = actual (producer)
    match crate::types::schema_compatible(&in_schema, &out_schema) {
        Ok(()) => None,
        Err(e) => Some(e.to_string()),
    }
}

fn validate_ros2_type_format(node_id: &NodeId, name: &str, type_str: &str) -> eyre::Result<()> {
    // type must be "package/TypeName" format with alphanumeric+underscore parts
    let parts: Vec<&str> = type_str.split('/').collect();
    if parts.len() != 2 || parts[0].is_empty() || parts[1].is_empty() {
        bail!(
            "node `{node_id}`: invalid type `{type_str}` for `{name}`, \
             expected format `package/TypeName` (e.g. `sensor_msgs/Image`)"
        );
    }
    for (label, part) in [("package", parts[0]), ("type name", parts[1])] {
        if !part.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
            bail!(
                "node `{node_id}`: invalid {label} `{part}` in type `{type_str}` for `{name}`, \
                 only ASCII alphanumeric and underscore characters allowed"
            );
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::TypeRegistry;
    use adora_message::config::{Input, InputMapping};
    use adora_message::descriptor::{Descriptor, Ros2BridgeConfig, Ros2Role};
    use std::time::Duration;

    fn dummy_input() -> Input {
        Input {
            mapping: InputMapping::Timer {
                interval: Duration::from_secs(1),
            },
            queue_size: None,
            input_timeout: None,
            queue_policy: None,
        }
    }

    fn service_config(role: Ros2Role) -> Ros2BridgeConfig {
        Ros2BridgeConfig {
            service: Some("/add_two_ints".into()),
            service_type: Some("example_interfaces/AddTwoInts".into()),
            role: Some(role),
            ..Default::default()
        }
    }

    fn action_config(role: Ros2Role) -> Ros2BridgeConfig {
        Ros2BridgeConfig {
            action: Some("/navigate".into()),
            action_type: Some("nav2_msgs/NavigateToPose".into()),
            role: Some(role),
            ..Default::default()
        }
    }

    #[test]
    fn validate_no_mode_set() {
        let config = Ros2BridgeConfig::default();
        let err = validate_ros2_config(
            &NodeId::from("n".to_owned()),
            &config,
            &BTreeMap::new(),
            &BTreeSet::new(),
        )
        .unwrap_err();
        assert!(err.to_string().contains("requires one of"));
    }

    #[test]
    fn validate_multiple_modes() {
        let config = Ros2BridgeConfig {
            topic: Some("/t".into()),
            service: Some("/s".into()),
            message_type: Some("a/B".into()),
            service_type: Some("a/B".into()),
            role: Some(Ros2Role::Client),
            ..Default::default()
        };
        let err = validate_ros2_config(
            &NodeId::from("n".to_owned()),
            &config,
            &BTreeMap::new(),
            &BTreeSet::new(),
        )
        .unwrap_err();
        assert!(err.to_string().contains("multiple of"));
    }

    #[test]
    fn validate_service_client_ok() {
        let config = service_config(Ros2Role::Client);
        let mut inputs = BTreeMap::new();
        inputs.insert(DataId::from("request".to_owned()), dummy_input());
        let mut outputs = BTreeSet::new();
        outputs.insert(DataId::from("response".to_owned()));
        validate_ros2_config(&NodeId::from("n".to_owned()), &config, &inputs, &outputs).unwrap();
    }

    #[test]
    fn validate_service_client_missing_service_type() {
        let config = Ros2BridgeConfig {
            service: Some("/svc".into()),
            role: Some(Ros2Role::Client),
            ..Default::default()
        };
        let mut inputs = BTreeMap::new();
        inputs.insert(DataId::from("request".to_owned()), dummy_input());
        let mut outputs = BTreeSet::new();
        outputs.insert(DataId::from("response".to_owned()));
        let err = validate_ros2_config(&NodeId::from("n".to_owned()), &config, &inputs, &outputs)
            .unwrap_err();
        assert!(err.to_string().contains("service_type"));
    }

    #[test]
    fn validate_service_client_missing_role() {
        let config = Ros2BridgeConfig {
            service: Some("/svc".into()),
            service_type: Some("a/B".into()),
            ..Default::default()
        };
        let mut inputs = BTreeMap::new();
        inputs.insert(DataId::from("request".to_owned()), dummy_input());
        let mut outputs = BTreeSet::new();
        outputs.insert(DataId::from("response".to_owned()));
        let err = validate_ros2_config(&NodeId::from("n".to_owned()), &config, &inputs, &outputs)
            .unwrap_err();
        assert!(err.to_string().contains("role"));
    }

    #[test]
    fn validate_service_client_no_inputs() {
        let config = service_config(Ros2Role::Client);
        let mut outputs = BTreeSet::new();
        outputs.insert(DataId::from("response".to_owned()));
        let err = validate_ros2_config(
            &NodeId::from("n".to_owned()),
            &config,
            &BTreeMap::new(),
            &outputs,
        )
        .unwrap_err();
        assert!(err.to_string().contains("input"));
    }

    #[test]
    fn validate_service_client_no_outputs() {
        let config = service_config(Ros2Role::Client);
        let mut inputs = BTreeMap::new();
        inputs.insert(DataId::from("request".to_owned()), dummy_input());
        let err = validate_ros2_config(
            &NodeId::from("n".to_owned()),
            &config,
            &inputs,
            &BTreeSet::new(),
        )
        .unwrap_err();
        assert!(err.to_string().contains("output"));
    }

    #[test]
    fn validate_service_server_ok() {
        let config = service_config(Ros2Role::Server);
        let mut inputs = BTreeMap::new();
        inputs.insert(DataId::from("response".to_owned()), dummy_input());
        let mut outputs = BTreeSet::new();
        outputs.insert(DataId::from("request".to_owned()));
        validate_ros2_config(&NodeId::from("n".to_owned()), &config, &inputs, &outputs).unwrap();
    }

    #[test]
    fn validate_service_server_no_inputs() {
        let config = service_config(Ros2Role::Server);
        let mut outputs = BTreeSet::new();
        outputs.insert(DataId::from("request".to_owned()));
        let err = validate_ros2_config(
            &NodeId::from("n".to_owned()),
            &config,
            &BTreeMap::new(),
            &outputs,
        )
        .unwrap_err();
        assert!(err.to_string().contains("input"));
    }

    #[test]
    fn validate_service_server_no_outputs() {
        let config = service_config(Ros2Role::Server);
        let mut inputs = BTreeMap::new();
        inputs.insert(DataId::from("response".to_owned()), dummy_input());
        let err = validate_ros2_config(
            &NodeId::from("n".to_owned()),
            &config,
            &inputs,
            &BTreeSet::new(),
        )
        .unwrap_err();
        assert!(err.to_string().contains("output"));
    }

    #[test]
    fn validate_action_client_ok() {
        let config = action_config(Ros2Role::Client);
        let mut inputs = BTreeMap::new();
        inputs.insert(DataId::from("goal".to_owned()), dummy_input());
        let mut outputs = BTreeSet::new();
        outputs.insert(DataId::from("feedback".to_owned()));
        outputs.insert(DataId::from("result".to_owned()));
        validate_ros2_config(&NodeId::from("n".to_owned()), &config, &inputs, &outputs).unwrap();
    }

    #[test]
    fn validate_action_client_missing_action_type() {
        let config = Ros2BridgeConfig {
            action: Some("/nav".into()),
            role: Some(Ros2Role::Client),
            ..Default::default()
        };
        let mut inputs = BTreeMap::new();
        inputs.insert(DataId::from("goal".to_owned()), dummy_input());
        let mut outputs = BTreeSet::new();
        outputs.insert(DataId::from("feedback".to_owned()));
        let err = validate_ros2_config(&NodeId::from("n".to_owned()), &config, &inputs, &outputs)
            .unwrap_err();
        assert!(err.to_string().contains("action_type"));
    }

    #[test]
    fn validate_action_client_no_inputs() {
        let config = action_config(Ros2Role::Client);
        let mut outputs = BTreeSet::new();
        outputs.insert(DataId::from("feedback".to_owned()));
        let err = validate_ros2_config(
            &NodeId::from("n".to_owned()),
            &config,
            &BTreeMap::new(),
            &outputs,
        )
        .unwrap_err();
        assert!(err.to_string().contains("input"));
    }

    #[test]
    fn validate_action_client_no_outputs() {
        let config = action_config(Ros2Role::Client);
        let mut inputs = BTreeMap::new();
        inputs.insert(DataId::from("goal".to_owned()), dummy_input());
        let err = validate_ros2_config(
            &NodeId::from("n".to_owned()),
            &config,
            &inputs,
            &BTreeSet::new(),
        )
        .unwrap_err();
        assert!(err.to_string().contains("output"));
    }

    #[test]
    fn validate_action_server_ok() {
        let config = action_config(Ros2Role::Server);
        let mut inputs = BTreeMap::new();
        inputs.insert(DataId::from("feedback".to_owned()), dummy_input());
        let mut outputs = BTreeSet::new();
        outputs.insert(DataId::from("goal".to_owned()));
        validate_ros2_config(&NodeId::from("n".to_owned()), &config, &inputs, &outputs).unwrap();
    }

    #[test]
    fn validate_action_server_no_inputs() {
        let config = action_config(Ros2Role::Server);
        let mut outputs = BTreeSet::new();
        outputs.insert(DataId::from("goal".to_owned()));
        let err = validate_ros2_config(
            &NodeId::from("n".to_owned()),
            &config,
            &BTreeMap::new(),
            &outputs,
        )
        .unwrap_err();
        assert!(err.to_string().contains("input"));
    }

    #[test]
    fn validate_action_server_no_outputs() {
        let config = action_config(Ros2Role::Server);
        let mut inputs = BTreeMap::new();
        inputs.insert(DataId::from("feedback".to_owned()), dummy_input());
        let err = validate_ros2_config(
            &NodeId::from("n".to_owned()),
            &config,
            &inputs,
            &BTreeSet::new(),
        )
        .unwrap_err();
        assert!(err.to_string().contains("output"));
    }

    #[test]
    fn validate_bad_type_format() {
        let config = Ros2BridgeConfig {
            service: Some("/svc".into()),
            service_type: Some("invalid_no_slash".into()),
            role: Some(Ros2Role::Client),
            ..Default::default()
        };
        let mut inputs = BTreeMap::new();
        inputs.insert(DataId::from("request".to_owned()), dummy_input());
        let mut outputs = BTreeSet::new();
        outputs.insert(DataId::from("response".to_owned()));
        let err = validate_ros2_config(&NodeId::from("n".to_owned()), &config, &inputs, &outputs)
            .unwrap_err();
        assert!(err.to_string().contains("package/TypeName"));
    }

    #[test]
    fn validate_type_rejects_special_chars() {
        let config = Ros2BridgeConfig {
            service: Some("/svc".into()),
            service_type: Some("pkg-bad/Evil".into()),
            role: Some(Ros2Role::Client),
            ..Default::default()
        };
        let mut inputs = BTreeMap::new();
        inputs.insert(DataId::from("request".to_owned()), dummy_input());
        let mut outputs = BTreeSet::new();
        outputs.insert(DataId::from("response".to_owned()));
        let err = validate_ros2_config(&NodeId::from("n".to_owned()), &config, &inputs, &outputs)
            .unwrap_err();
        assert!(err.to_string().contains("alphanumeric"));
    }

    #[test]
    fn validate_qos_bad_durability() {
        let config = Ros2BridgeConfig {
            service: Some("/svc".into()),
            service_type: Some("a/B".into()),
            role: Some(Ros2Role::Client),
            qos: adora_message::descriptor::Ros2QosConfig {
                durability: Some("persistent".into()),
                ..Default::default()
            },
            ..Default::default()
        };
        let mut inputs = BTreeMap::new();
        inputs.insert(DataId::from("request".to_owned()), dummy_input());
        let mut outputs = BTreeSet::new();
        outputs.insert(DataId::from("response".to_owned()));
        let err = validate_ros2_config(&NodeId::from("n".to_owned()), &config, &inputs, &outputs)
            .unwrap_err();
        assert!(err.to_string().contains("durability"));
    }

    #[test]
    fn validate_qos_keep_last_out_of_range() {
        let config = Ros2BridgeConfig {
            service: Some("/svc".into()),
            service_type: Some("a/B".into()),
            role: Some(Ros2Role::Client),
            qos: adora_message::descriptor::Ros2QosConfig {
                keep_last: Some(100_000),
                ..Default::default()
            },
            ..Default::default()
        };
        let mut inputs = BTreeMap::new();
        inputs.insert(DataId::from("request".to_owned()), dummy_input());
        let mut outputs = BTreeSet::new();
        outputs.insert(DataId::from("response".to_owned()));
        let err = validate_ros2_config(&NodeId::from("n".to_owned()), &config, &inputs, &outputs)
            .unwrap_err();
        assert!(err.to_string().contains("keep_last"));
    }

    // --- Type annotation tests ---

    fn parse_dataflow(yaml: &str) -> Descriptor {
        serde_yaml::from_str(yaml).expect("test YAML should parse")
    }

    #[test]
    fn type_check_no_annotations_no_warnings() {
        let dataflow = parse_dataflow("nodes:\n  - id: a\n");
        let reg = TypeRegistry::new();
        let warnings = check_type_annotations(&dataflow, &reg);
        assert!(warnings.is_empty());
    }

    #[test]
    fn type_check_valid_output_type() {
        let dataflow = parse_dataflow(
            "nodes:\n  - id: camera\n    outputs:\n      - image\n    output_types:\n      image: std/media/v1/Image\n",
        );
        let reg = TypeRegistry::new();
        let warnings = check_type_annotations(&dataflow, &reg);
        assert!(warnings.is_empty());
    }

    #[test]
    fn type_check_output_types_key_not_in_outputs() {
        let dataflow = parse_dataflow(
            "nodes:\n  - id: camera\n    output_types:\n      image: std/media/v1/Image\n",
        );
        let reg = TypeRegistry::new();
        let warnings = check_type_annotations(&dataflow, &reg);
        assert_eq!(warnings.len(), 1);
        assert!(warnings[0].message.contains("not found in outputs"));
    }

    #[test]
    fn type_check_unknown_urn_with_suggestion() {
        let dataflow = parse_dataflow(
            "nodes:\n  - id: camera\n    outputs:\n      - image\n    output_types:\n      image: std/media/v1/Imag\n",
        );
        let reg = TypeRegistry::new();
        let warnings = check_type_annotations(&dataflow, &reg);
        assert_eq!(warnings.len(), 1);
        assert!(warnings[0].message.contains("unknown type"));
        assert!(warnings[0].message.contains("did you mean"));
    }

    #[test]
    fn type_check_matching_edge_types() {
        let dataflow = parse_dataflow(
            "\
nodes:
  - id: sender
    outputs:
      - data
    output_types:
      data: std/core/v1/Float32
  - id: receiver
    inputs:
      data: sender/data
    input_types:
      data: std/core/v1/Float32
",
        );
        let reg = TypeRegistry::new();
        let warnings = check_type_annotations(&dataflow, &reg);
        assert!(warnings.is_empty());
    }

    #[test]
    fn type_check_mismatched_edge_types() {
        let dataflow = parse_dataflow(
            "\
nodes:
  - id: sender
    outputs:
      - data
    output_types:
      data: std/core/v1/Float32
  - id: receiver
    inputs:
      data: sender/data
    input_types:
      data: std/media/v1/Image
",
        );
        let reg = TypeRegistry::new();
        let warnings = check_type_annotations(&dataflow, &reg);
        assert_eq!(warnings.len(), 1);
        assert!(warnings[0].message.contains("type mismatch"));
        assert!(warnings[0].message.contains("Float32"));
        assert!(warnings[0].message.contains("Image"));
    }

    // --- Phase 1: strict mode tests ---

    #[test]
    fn strict_types_parses_in_yaml() {
        let dataflow = parse_dataflow("nodes:\n  - id: a\nstrict_types: true\n");
        assert_eq!(dataflow.strict_types, Some(true));
    }

    #[test]
    fn strict_mode_warns_on_unannotated_upstream() {
        let dataflow = parse_dataflow(
            "\
nodes:
  - id: sender
    outputs:
      - data
  - id: receiver
    inputs:
      data: sender/data
    input_types:
      data: std/core/v1/Float32
",
        );
        let reg = TypeRegistry::new();
        let result = check_type_annotations_full(&dataflow, &reg, true);
        assert!(!result.warnings.is_empty());
        assert!(result.warnings[0].message.contains("no type annotation"));
    }

    #[test]
    fn non_strict_no_warning_on_unannotated_upstream() {
        let dataflow = parse_dataflow(
            "\
nodes:
  - id: sender
    outputs:
      - data
  - id: receiver
    inputs:
      data: sender/data
    input_types:
      data: std/core/v1/Float32
",
        );
        let reg = TypeRegistry::new();
        let result = check_type_annotations_full(&dataflow, &reg, false);
        assert!(result.warnings.is_empty());
    }

    // --- Phase 3: type inference tests ---

    #[test]
    fn inference_from_annotated_upstream() {
        let dataflow = parse_dataflow(
            "\
nodes:
  - id: sensor
    outputs:
      - reading
    output_types:
      reading: std/core/v1/Float64
  - id: processor
    inputs:
      reading: sensor/reading
",
        );
        let reg = TypeRegistry::new();
        let result = check_type_annotations_full(&dataflow, &reg, false);
        assert!(result.warnings.is_empty());
        assert_eq!(result.inferences.len(), 1);
        assert_eq!(result.inferences[0].inferred_urn, "std/core/v1/Float64");
        assert_eq!(result.inferences[0].port_id, "reading");
    }

    #[test]
    fn no_inference_when_both_annotated() {
        let dataflow = parse_dataflow(
            "\
nodes:
  - id: sender
    outputs:
      - data
    output_types:
      data: std/core/v1/Float32
  - id: receiver
    inputs:
      data: sender/data
    input_types:
      data: std/core/v1/Float32
",
        );
        let reg = TypeRegistry::new();
        let result = check_type_annotations_full(&dataflow, &reg, false);
        assert!(result.inferences.is_empty());
    }

    #[test]
    fn no_inference_when_neither_annotated() {
        let dataflow = parse_dataflow(
            "\
nodes:
  - id: sender
    outputs:
      - data
  - id: receiver
    inputs:
      data: sender/data
",
        );
        let reg = TypeRegistry::new();
        let result = check_type_annotations_full(&dataflow, &reg, false);
        assert!(result.inferences.is_empty());
    }

    // --- Phase 4: compatibility tests ---

    #[test]
    fn compat_uint8_to_uint32_edge() {
        let dataflow = parse_dataflow(
            "\
nodes:
  - id: sender
    outputs:
      - data
    output_types:
      data: std/core/v1/UInt8
  - id: receiver
    inputs:
      data: sender/data
    input_types:
      data: std/core/v1/UInt32
",
        );
        let reg = TypeRegistry::new();
        let warnings = check_type_annotations(&dataflow, &reg);
        assert!(warnings.is_empty(), "UInt8 -> UInt32 should be compatible");
    }

    #[test]
    fn compat_any_to_bytes_edge() {
        let dataflow = parse_dataflow(
            "\
nodes:
  - id: sender
    outputs:
      - data
    output_types:
      data: std/media/v1/Image
  - id: receiver
    inputs:
      data: sender/data
    input_types:
      data: std/core/v1/Bytes
",
        );
        let reg = TypeRegistry::new();
        let warnings = check_type_annotations(&dataflow, &reg);
        assert!(
            warnings.is_empty(),
            "anything -> Bytes should be compatible"
        );
    }

    #[test]
    fn compat_user_defined_rule_in_yaml() {
        let dataflow = parse_dataflow(
            "\
type_rules:
  - from: std/core/v1/UInt8
    to: std/core/v1/String
nodes:
  - id: sender
    outputs:
      - data
    output_types:
      data: std/core/v1/UInt8
  - id: receiver
    inputs:
      data: sender/data
    input_types:
      data: std/core/v1/String
",
        );
        let reg = TypeRegistry::new();
        let warnings = check_type_annotations(&dataflow, &reg);
        assert!(
            warnings.is_empty(),
            "user-defined rule should make this compatible"
        );
    }

    // --- Phase 5: metadata tests ---

    #[test]
    fn metadata_pattern_resolves() {
        let dataflow = parse_dataflow(
            "\
nodes:
  - id: srv
    pattern: service-server
    outputs:
      - response
",
        );
        let reg = TypeRegistry::new();
        let warnings = check_type_annotations(&dataflow, &reg);
        assert!(warnings.is_empty());
    }

    #[test]
    fn metadata_unknown_pattern() {
        let dataflow = parse_dataflow(
            "\
nodes:
  - id: srv
    pattern: unknown-pattern
    outputs:
      - response
",
        );
        let reg = TypeRegistry::new();
        let warnings = check_type_annotations(&dataflow, &reg);
        assert_eq!(warnings.len(), 1);
        assert!(warnings[0].message.contains("unknown pattern"));
    }

    #[test]
    fn metadata_output_key_not_in_outputs() {
        let dataflow = parse_dataflow(
            "\
nodes:
  - id: srv
    output_metadata:
      missing_port: [request_id]
    outputs:
      - response
",
        );
        let reg = TypeRegistry::new();
        let warnings = check_type_annotations(&dataflow, &reg);
        assert_eq!(warnings.len(), 1);
        assert!(warnings[0].message.contains("output_metadata key"));
    }

    // --- Timer auto-inject test ---

    #[test]
    fn timer_input_type_mismatch() {
        let dataflow = parse_dataflow(
            "\
nodes:
  - id: node
    inputs:
      tick: adora/timer/millis/100
    input_types:
      tick: std/media/v1/Image
",
        );
        let reg = TypeRegistry::new();
        let warnings = check_type_annotations(&dataflow, &reg);
        assert!(!warnings.is_empty());
        assert!(warnings[0].message.contains("type mismatch"));
    }

    #[test]
    fn validate_qos_negative_lease_duration() {
        let config = Ros2BridgeConfig {
            service: Some("/svc".into()),
            service_type: Some("a/B".into()),
            role: Some(Ros2Role::Client),
            qos: adora_message::descriptor::Ros2QosConfig {
                lease_duration: Some(-1.0),
                ..Default::default()
            },
            ..Default::default()
        };
        let mut inputs = BTreeMap::new();
        inputs.insert(DataId::from("request".to_owned()), dummy_input());
        let mut outputs = BTreeSet::new();
        outputs.insert(DataId::from("response".to_owned()));
        let err = validate_ros2_config(&NodeId::from("n".to_owned()), &config, &inputs, &outputs)
            .unwrap_err();
        assert!(err.to_string().contains("lease_duration"));
    }

    // --- Wiring validation tests ---

    #[test]
    fn wiring_valid_dataflow() {
        let yaml = r#"
nodes:
  - id: source
    path: source.py
    outputs:
      - data
  - id: sink
    path: sink.py
    inputs:
      data: source/data
"#;
        let descriptor: Descriptor = serde_yaml::from_str(yaml).unwrap();
        check_wiring(&descriptor).unwrap();
    }

    #[test]
    fn wiring_rejects_nonexistent_source_node() {
        let yaml = r#"
nodes:
  - id: sink
    path: sink.py
    inputs:
      data: nonexistent/data
"#;
        let descriptor: Descriptor = serde_yaml::from_str(yaml).unwrap();
        let err = check_wiring(&descriptor).unwrap_err();
        assert!(
            err.to_string().contains("nonexistent"),
            "expected error about missing node, got: {err}"
        );
    }

    #[test]
    fn wiring_rejects_nonexistent_output() {
        let yaml = r#"
nodes:
  - id: source
    path: source.py
    outputs:
      - data
  - id: sink
    path: sink.py
    inputs:
      data: source/typo
"#;
        let descriptor: Descriptor = serde_yaml::from_str(yaml).unwrap();
        let err = check_wiring(&descriptor).unwrap_err();
        assert!(
            err.to_string().contains("typo"),
            "expected error about missing output, got: {err}"
        );
    }
}
