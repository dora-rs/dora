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
    use adora_message::config::{Input, InputMapping};
    use adora_message::descriptor::{Ros2BridgeConfig, Ros2Role};
    use std::time::Duration;

    fn dummy_input() -> Input {
        Input {
            mapping: InputMapping::Timer {
                interval: Duration::from_secs(1),
            },
            queue_size: None,
            input_timeout: None,
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
}
