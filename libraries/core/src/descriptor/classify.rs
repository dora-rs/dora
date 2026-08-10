//! Node field classifier: type detection + whitelist-based field validation.
//!
//! Every node passes through `classify()` (or `check_module_fields` for
//! modules) before resolution. The function determines the node kind,
//! checks every field against a whitelist for that kind, and either
//! returns a `NodeClass` or collects all unrecognized fields into a
//! single error.
//!
//! ## Adding a new field to `Node`
//!
//! When you add a field to `dora_message::descriptor::Node`, decide which
//! node kinds should accept it and add it to the appropriate whitelist(s)
//! in this file. Fields not in any whitelist are rejected for all kinds.

use dora_message::descriptor::{GitRepoRev, Node, NodeSource, RestartPolicy};
use eyre::{Result, bail};

// ── Public interface ──────────────────────────────────────────────

#[derive(Debug)]
pub(super) enum NodeClass {
    Standard { source: NodeSource },
    Custom,
    Runtime,
    Operator,
    Ros2Bridge,
}

/// Classify a non-module node: determine kind, validate fields against
/// the kind's whitelist, return a `NodeClass` for resolution.
pub(super) fn classify(node: &Node) -> Result<NodeClass> {
    match classify_inner(node)? {
        NodeClassOrModule::Class(kind) => Ok(kind),
        NodeClassOrModule::Module => {
            bail!(
                "module node `{}` must be expanded before resolution — \
                 call expand_modules() first",
                node.id
            )
        }
    }
}

/// Validate module node fields against the module whitelist.
/// Called from `expand.rs` before module expansion.
pub(super) fn check_module_fields(node: &Node) -> Result<()> {
    // Module nodes are checked with their own whitelist
    check_module(node)
}

// ── Internal ───────────────────────────────────────────────────────

enum NodeClassOrModule {
    Class(NodeClass),
    Module,
}

/// Build a `NodeSource` from Standard node fields.
fn standard_source(node: &Node) -> Result<NodeSource> {
    match (&node.git, &node.branch, &node.tag, &node.rev) {
        (None, None, None, None) => Ok(NodeSource::Local),
        (Some(repo), branch, tag, rev) => {
            let rev = match (branch, tag, rev) {
                (None, None, None) => None,
                (Some(branch), None, None) => Some(GitRepoRev::Branch(branch.clone())),
                (None, Some(tag), None) => Some(GitRepoRev::Tag(tag.clone())),
                (None, None, Some(rev)) => Some(GitRepoRev::Rev(rev.clone())),
                other @ (_, _, _) => {
                    bail!("only one of `branch`, `tag`, and `rev` are allowed (got {other:?})")
                }
            };
            Ok(NodeSource::GitBranch {
                repo: repo.clone(),
                rev,
            })
        }
        (None, _, _, _) => {
            bail!("`git` source required when using branch, tag, or rev")
        }
    }
}

/// Detect the node kind from the discriminator fields directly.
///
/// This mirrors the match in `NodeExt::kind()` but WITHOUT the hub
/// pre-check, so the classifier can still report the precise
/// kind-specific errors for other node types instead of bailing out
/// early for every node that carries a `hub:` field.
///
/// NOTE: hub resolution validation lives in `Node::kind()`'s pre-check
/// (mod.rs), not here. kind() rejects hub-only nodes (no path, no other
/// discriminator) early because hub references must be resolved by
/// `dora build` before descriptor resolution. For nodes with a kind
/// discriminator (like Standard with path), hub is a valid field and
/// proceeds through the whitelist normally.
fn classify_inner(node: &Node) -> Result<NodeClassOrModule> {
    match (
        &node.path,
        &node.operators,
        &node.custom,
        &node.operator,
        &node.ros2,
        &node.module,
    ) {
        (None, None, None, None, None, None) => {
            bail!(
                "node `{}` requires a `path`, `custom`, `operators`, `ros2`, or `module` field",
                node.id
            )
        }
        (None, None, None, Some(_), None, None) => {
            check_operator(node)?;
            Ok(NodeClassOrModule::Class(NodeClass::Operator))
        }
        (None, None, Some(_), None, None, None) => {
            check_custom(node)?;
            Ok(NodeClassOrModule::Class(NodeClass::Custom))
        }
        (None, Some(_), None, None, None, None) => {
            check_runtime(node)?;
            Ok(NodeClassOrModule::Class(NodeClass::Runtime))
        }
        (Some(_), None, None, None, None, None) => {
            check_standard(node)?;
            Ok(NodeClassOrModule::Class(NodeClass::Standard {
                source: standard_source(node)?,
            }))
        }
        (None, None, None, None, Some(_), None) => {
            check_ros2(node)?;
            Ok(NodeClassOrModule::Class(NodeClass::Ros2Bridge))
        }
        (None, None, None, None, None, Some(_)) => {
            check_module(node)?;
            Ok(NodeClassOrModule::Module)
        }
        _ => bail!(
            "node `{}` has multiple exclusive fields set, only one of `path`, `custom`, `operators`, `operator`, `ros2`, and `module` is allowed",
            node.id
        ),
    }
}

// ── Whitelist check helpers ───────────────────────────────────────

/// Fields shared by ALL node types (consumed at ResolvedNode construction).
const SHARED_FIELDS: &[&str] = &["id", "name", "description", "env", "deploy"];

/// All non-discriminator `Node` fields that are classified per node kind.
///
/// Keep this list exhaustive for `dora_message::descriptor::Node`: a field
/// missing here is never checked against the per-kind whitelists.
const ALL_CHECKABLE_FIELDS: &[&str] = &[
    "path",
    "path_sha256",
    "args",
    "build",
    "git",
    "hub",
    "branch",
    "tag",
    "rev",
    "outputs",
    "output_types",
    "output_framing",
    "inputs",
    "input_types",
    "output_metadata",
    "pattern",
    "send_stdout_as",
    "send_logs_as",
    "min_log_level",
    "max_log_size",
    "max_rotated_files",
    "shared_memory_pool_size",
    "restart_policy",
    "max_restarts",
    "restart_delay",
    "max_restart_delay",
    "restart_window",
    "health_check_timeout",
    "finish_grace_secs",
    "cpu_affinity",
    "params",
];

fn is_set(field: &str, node: &Node) -> bool {
    match field {
        "path" => node.path.is_some(),
        "path_sha256" => node.path_sha256.is_some(),
        "args" => node.args.is_some(),
        "build" => node.build.is_some(),
        "git" => node.git.is_some(),
        "hub" => node.hub.is_some(),
        "branch" => node.branch.is_some(),
        "tag" => node.tag.is_some(),
        "rev" => node.rev.is_some(),
        "outputs" => !node.outputs.is_empty(),
        "output_types" => !node.output_types.is_empty(),
        "output_framing" => !node.output_framing.is_empty(),
        "inputs" => !node.inputs.is_empty(),
        "input_types" => !node.input_types.is_empty(),
        "output_metadata" => !node.output_metadata.is_empty(),
        "pattern" => node.pattern.is_some(),
        "send_stdout_as" => node.send_stdout_as.is_some(),
        "send_logs_as" => node.send_logs_as.is_some(),
        "min_log_level" => node.min_log_level.is_some(),
        "max_log_size" => node.max_log_size.is_some(),
        "max_rotated_files" => node.max_rotated_files.is_some(),
        "shared_memory_pool_size" => node.shared_memory_pool_size.is_some(),
        "restart_policy" => !matches!(node.restart_policy, RestartPolicy::Never),
        "max_restarts" => node.max_restarts != 0,
        "restart_delay" => node.restart_delay.is_some(),
        "max_restart_delay" => node.max_restart_delay.is_some(),
        "restart_window" => node.restart_window.is_some(),
        "health_check_timeout" => node.health_check_timeout.is_some(),
        "finish_grace_secs" => node.finish_grace_secs.is_some(),
        "cpu_affinity" => node.cpu_affinity.is_some(),
        "params" => !node.params.is_empty(),
        // kind discriminator fields — always checked by kind(), never set
        // alongside a different kind, so they are never "set" here
        "custom" | "operators" | "operator" | "ros2" | "module" => false,
        _ => false,
    }
}

fn validate_against_whitelist(node: &Node, allowed: &[&str], kind_name: &str) -> Result<()> {
    let mut unknown: Vec<&str> = Vec::new();
    for field in ALL_CHECKABLE_FIELDS {
        if is_set(field, node) && !allowed.contains(field) {
            unknown.push(field);
        }
    }

    if unknown.is_empty() {
        return Ok(());
    }

    let mut msg = format!(
        "node `{}` has fields that are not allowed on {} nodes: {}\n\
         hint: ",
        node.id,
        kind_name,
        unknown.join(", ")
    );

    // Custom-specific hint for misplaced fields
    if kind_name == "Custom" {
        msg.push_str(
            "these fields should be placed inside the `custom:` block \
             (e.g. `custom.outputs`, `custom.restart_policy`)",
        );
    } else {
        msg.push_str("these fields are not consumed by this node kind; remove them from the node");
    }

    bail!(msg)
}

// ── Per-kind whitelists and checks ────────────────────────────────

/// Standard node whitelist:
/// path, git, branch, tag, rev, hub, build, path_sha256, args,
/// inputs, outputs, output_types, input_types, output_framing,
/// shared_memory_pool_size, restart_policy, max_restarts,
/// restart_delay, max_restart_delay, restart_window,
/// health_check_timeout, finish_grace_secs,
/// send_stdout_as, send_logs_as, min_log_level, max_log_size, max_rotated_files,
/// output_metadata, pattern, cpu_affinity
const STANDARD_ALLOWED: &[&str] = &[
    "path",
    "git",
    "branch",
    "tag",
    "rev",
    "hub",
    "build",
    "path_sha256",
    "args",
    "inputs",
    "outputs",
    "output_types",
    "input_types",
    "output_framing",
    "shared_memory_pool_size",
    "restart_policy",
    "max_restarts",
    "restart_delay",
    "max_restart_delay",
    "restart_window",
    "health_check_timeout",
    "finish_grace_secs",
    "send_stdout_as",
    "send_logs_as",
    "min_log_level",
    "max_log_size",
    "max_rotated_files",
    "output_metadata",
    "pattern",
    "cpu_affinity",
];

fn check_standard(node: &Node) -> Result<()> {
    let mut allowed = SHARED_FIELDS.to_vec();
    allowed.extend(STANDARD_ALLOWED);
    validate_against_whitelist(node, &allowed, "Standard")
}

/// Custom node whitelist:
/// custom, cpu_affinity (+ shared: id/name/description/env/deploy)
///
/// Fields that belong inside `custom:` are rejected with a hint
/// pointing to the correct location.
const CUSTOM_ALLOWED: &[&str] = &["custom", "cpu_affinity"];

fn check_custom(node: &Node) -> Result<()> {
    let mut allowed = SHARED_FIELDS.to_vec();
    allowed.extend(CUSTOM_ALLOWED);
    validate_against_whitelist(node, &allowed, "Custom")
}

/// Runtime node whitelist:
/// operators, cpu_affinity (+ shared)
const RUNTIME_ALLOWED: &[&str] = &["operators", "cpu_affinity"];

fn check_runtime(node: &Node) -> Result<()> {
    let mut allowed = SHARED_FIELDS.to_vec();
    allowed.extend(RUNTIME_ALLOWED);
    validate_against_whitelist(node, &allowed, "Runtime")
}

/// Operator (single) node whitelist:
/// operator, cpu_affinity (+ shared)
const OPERATOR_ALLOWED: &[&str] = &["operator", "cpu_affinity"];

fn check_operator(node: &Node) -> Result<()> {
    let mut allowed = SHARED_FIELDS.to_vec();
    allowed.extend(OPERATOR_ALLOWED);
    validate_against_whitelist(node, &allowed, "Operator")
}

/// ROS2 bridge node whitelist:
/// ros2, args, inputs, outputs, output_types, input_types,
/// output_framing, shared_memory_pool_size,
/// restart_policy, max_restarts, restart_delay, max_restart_delay,
/// restart_window, health_check_timeout, finish_grace_secs,
/// send_stdout_as, send_logs_as, min_log_level, max_log_size, max_rotated_files,
/// output_metadata, pattern, cpu_affinity (+ shared)
const ROS2_ALLOWED: &[&str] = &[
    "ros2",
    "args",
    "inputs",
    "outputs",
    "output_types",
    "input_types",
    "output_framing",
    "shared_memory_pool_size",
    "restart_policy",
    "max_restarts",
    "restart_delay",
    "max_restart_delay",
    "restart_window",
    "health_check_timeout",
    "finish_grace_secs",
    "send_stdout_as",
    "send_logs_as",
    "min_log_level",
    "max_log_size",
    "max_rotated_files",
    "output_metadata",
    "pattern",
    "cpu_affinity",
];

fn check_ros2(node: &Node) -> Result<()> {
    let mut allowed = SHARED_FIELDS.to_vec();
    allowed.extend(ROS2_ALLOWED);
    validate_against_whitelist(node, &allowed, "ROS2 bridge")
}

/// Module node whitelist:
/// module, inputs, params (+ shared)
/// Note: module is the kind discriminator; params is compile-time substitution.
const MODULE_ALLOWED: &[&str] = &["module", "inputs", "params"];

fn check_module(node: &Node) -> Result<()> {
    let mut allowed = SHARED_FIELDS.to_vec();
    allowed.extend(MODULE_ALLOWED);
    validate_against_whitelist(node, &allowed, "Module")
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    fn parse_node(yaml: &str) -> Node {
        serde_yaml::from_str(yaml).expect("test node should parse")
    }

    fn classify_error(yaml: &str) -> String {
        let node = parse_node(yaml);
        format!(
            "{:#}",
            classify(&node).expect_err("node should fail classification")
        )
    }

    #[test]
    fn valid_nodes_for_each_kind_pass_field_classification() {
        for yaml in [
            r#"
id: standard
path: ./node
inputs:
  in: source/out
outputs: [out]
output_metadata:
  out: [request_id]
pattern: service-server
"#,
            r#"
id: custom
custom:
  path: ./node.py
  source: Local
  inputs:
    in: source/out
  outputs: [out]
"#,
            r#"
id: runtime
operators:
  - id: op
    python: op.py
    inputs:
      in: source/out
    outputs: [out]
"#,
            r#"
id: operator
operator:
  python: op.py
  inputs:
    in: source/out
  outputs: [out]
"#,
            r#"
id: bridge
ros2:
  topic: /odom
  message_type: nav_msgs/msg/Odometry
  direction: subscribe
outputs: [odom]
output_metadata:
  odom: [request_id]
pattern: service-server
"#,
        ] {
            let node = parse_node(yaml);
            classify(&node).expect("valid node should classify");
        }

        let module = parse_node(
            r#"
id: nav
module: modules/nav.yml
inputs:
  pose: localization/pose
params:
  speed: "2.0"
"#,
        );
        check_module_fields(&module).expect("valid module node should classify");
    }

    #[test]
    fn rejected_fields_are_reported_for_each_kind() {
        for (yaml, expected_kind, expected_field) in [
            (
                r#"
id: standard
path: ./node
params:
  speed: "2.0"
"#,
                "Standard",
                "params",
            ),
            (
                r#"
id: custom
custom:
  path: ./node.py
  source: Local
outputs: [out]
"#,
                "Custom",
                "outputs",
            ),
            (
                r#"
id: runtime
operators:
  - id: op
    python: op.py
outputs: [out]
"#,
                "Runtime",
                "outputs",
            ),
            (
                r#"
id: operator
operator:
  python: op.py
outputs: [out]
"#,
                "Operator",
                "outputs",
            ),
            (
                r#"
id: bridge
ros2:
  topic: /odom
  message_type: nav_msgs/msg/Odometry
  direction: subscribe
git: https://github.com/example/node.git
"#,
                "ROS2 bridge",
                "git",
            ),
        ] {
            let error = classify_error(yaml);
            assert!(error.contains(expected_kind), "{error}");
            assert!(error.contains(expected_field), "{error}");
        }

        let module = parse_node(
            r#"
id: nav
module: modules/nav.yml
build: cargo build
"#,
        );
        let error = format!(
            "{:#}",
            check_module_fields(&module).expect_err("module build should be rejected")
        );
        assert!(error.contains("Module"), "{error}");
        assert!(error.contains("build"), "{error}");
    }

    #[test]
    fn all_node_fields_are_classified_or_marked_shared() {
        let schema = schemars::schema_for!(Node);
        let schema = serde_json::to_value(schema).expect("schema should serialize");
        let properties = schema
            .pointer("/$defs/Node/properties")
            .or_else(|| schema.pointer("/definitions/Node/properties"))
            .or_else(|| schema.pointer("/properties"))
            .and_then(serde_json::Value::as_object)
            .expect("Node schema should expose properties");

        let mut actual: BTreeSet<_> = properties.keys().map(String::as_str).collect();
        // `deploy` is a real `Node` field, but is intentionally skipped in the
        // generated schema because it uses the unstable `_unstable_deploy`
        // YAML surface.
        actual.insert("deploy");
        let mut classified: BTreeSet<_> = SHARED_FIELDS.iter().copied().collect();
        classified.extend(ALL_CHECKABLE_FIELDS.iter().copied());
        classified.extend(["custom", "operators", "operator", "ros2", "module"]);

        assert_eq!(actual, classified);
    }
}
