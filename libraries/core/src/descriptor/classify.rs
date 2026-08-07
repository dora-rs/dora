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
/// pre-check: the `hub:`/`path` resolution check belongs to
/// `check_standard()` only, so the classifier can still report the
/// precise kind-specific errors for other node types instead of bailing
/// out early for every node that carries a `hub:` field.
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
    // All fields that could be set on a Node (not kind discriminators)
    let all_checkable: &[&str] = &[
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

    let mut unknown: Vec<&str> = Vec::new();
    for field in all_checkable {
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
             (e.g. `custom.run_config.outputs`, `custom.restart_policy`)",
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
    // `hub:` is desugared into a concrete git node by `dora build` /
    // `dora run` / `dora validate` before any kind dispatch — an
    // unresolved reference reaching this point means a flow that
    // skipped resolution.
    if node.hub.is_some() && node.path.is_none() {
        bail!(
            "node `{}` uses an unresolved `hub:` reference — run `dora build` \
             first (`dora start` requires a prior build for hub nodes)",
            node.id
        );
    }
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
/// cpu_affinity (+ shared)
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
