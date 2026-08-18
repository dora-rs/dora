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

use super::{NodeExt, NodeKind};
use dora_message::descriptor::{GitRepoRev, Node, NodeSource, RestartPolicy};
use eyre::{Result, bail};

// ── Public interface ──────────────────────────────────────────────

#[derive(Debug)]
pub(super) enum NodeClass {
    Standard { source: NodeSource },
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

fn classify_inner(node: &Node) -> Result<NodeClassOrModule> {
    match node.kind()? {
        NodeKind::Operator(_) => {
            check_operator(node)?;
            Ok(NodeClassOrModule::Class(NodeClass::Operator))
        }
        NodeKind::Runtime(_) => {
            check_runtime(node)?;
            Ok(NodeClassOrModule::Class(NodeClass::Runtime))
        }
        NodeKind::Standard(_) => {
            check_standard(node)?;
            Ok(NodeClassOrModule::Class(NodeClass::Standard {
                source: standard_source(node)?,
            }))
        }
        NodeKind::Ros2Bridge(_) => {
            check_ros2(node)?;
            Ok(NodeClassOrModule::Class(NodeClass::Ros2Bridge))
        }
        NodeKind::Module(_) => {
            check_module(node)?;
            Ok(NodeClassOrModule::Module)
        }
    }
}

// ── Whitelist check helpers ───────────────────────────────────────

/// Fields shared by ALL node types (consumed at ResolvedNode construction).
const SHARED_FIELDS: &[&str] = &["id", "name", "description", "env", "deploy"];

struct CheckableField {
    name: &'static str,
    is_set: fn(&Node) -> bool,
}

/// All non-discriminator `Node` fields that are classified per node kind.
///
/// Keep this table exhaustive for `dora_message::descriptor::Node`: a field
/// missing here is never checked against the per-kind whitelists.
const ALL_CHECKABLE_FIELDS: &[CheckableField] = &[
    CheckableField {
        name: "path",
        is_set: |node| node.path.is_some(),
    },
    CheckableField {
        name: "path_sha256",
        is_set: |node| node.path_sha256.is_some(),
    },
    CheckableField {
        name: "args",
        is_set: |node| node.args.is_some(),
    },
    CheckableField {
        name: "build",
        is_set: |node| node.build.is_some(),
    },
    CheckableField {
        name: "git",
        is_set: |node| node.git.is_some(),
    },
    CheckableField {
        name: "hub",
        is_set: |node| node.hub.is_some(),
    },
    CheckableField {
        name: "branch",
        is_set: |node| node.branch.is_some(),
    },
    CheckableField {
        name: "tag",
        is_set: |node| node.tag.is_some(),
    },
    CheckableField {
        name: "rev",
        is_set: |node| node.rev.is_some(),
    },
    CheckableField {
        name: "outputs",
        is_set: |node| !node.outputs.is_empty(),
    },
    CheckableField {
        name: "output_types",
        is_set: |node| !node.output_types.is_empty(),
    },
    CheckableField {
        name: "output_framing",
        is_set: |node| !node.output_framing.is_empty(),
    },
    CheckableField {
        name: "inputs",
        is_set: |node| !node.inputs.is_empty(),
    },
    CheckableField {
        name: "input_types",
        is_set: |node| !node.input_types.is_empty(),
    },
    CheckableField {
        name: "output_metadata",
        is_set: |node| !node.output_metadata.is_empty(),
    },
    CheckableField {
        name: "pattern",
        is_set: |node| node.pattern.is_some(),
    },
    CheckableField {
        name: "send_stdout_as",
        is_set: |node| node.send_stdout_as.is_some(),
    },
    CheckableField {
        name: "send_logs_as",
        is_set: |node| node.send_logs_as.is_some(),
    },
    CheckableField {
        name: "min_log_level",
        is_set: |node| node.min_log_level.is_some(),
    },
    CheckableField {
        name: "max_log_size",
        is_set: |node| node.max_log_size.is_some(),
    },
    CheckableField {
        name: "max_rotated_files",
        is_set: |node| node.max_rotated_files.is_some(),
    },
    CheckableField {
        name: "shared_memory_pool_size",
        is_set: |node| node.shared_memory_pool_size.is_some(),
    },
    CheckableField {
        name: "restart_policy",
        is_set: |node| !matches!(node.restart_policy, RestartPolicy::Never),
    },
    CheckableField {
        name: "max_restarts",
        is_set: |node| node.max_restarts != 0,
    },
    CheckableField {
        name: "restart_delay",
        is_set: |node| node.restart_delay.is_some(),
    },
    CheckableField {
        name: "max_restart_delay",
        is_set: |node| node.max_restart_delay.is_some(),
    },
    CheckableField {
        name: "restart_window",
        is_set: |node| node.restart_window.is_some(),
    },
    CheckableField {
        name: "health_check_timeout",
        is_set: |node| node.health_check_timeout.is_some(),
    },
    CheckableField {
        name: "finish_grace_secs",
        is_set: |node| node.finish_grace_secs.is_some(),
    },
    CheckableField {
        name: "cpu_affinity",
        is_set: |node| node.cpu_affinity.is_some(),
    },
    CheckableField {
        name: "params",
        is_set: |node| !node.params.is_empty(),
    },
];

fn validate_against_whitelist(node: &Node, allowed: &[&str], kind_name: &str) -> Result<()> {
    let mut unknown: Vec<&str> = Vec::new();
    for field in ALL_CHECKABLE_FIELDS {
        if (field.is_set)(node) && !allowed.contains(&field.name) {
            unknown.push(field.name);
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

    msg.push_str("these fields are not consumed by this node kind; remove them from the node");

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
    fn hub_only_node_keeps_resolution_error() {
        let error = classify_error(
            r#"
id: yolo
hub: dora-yolo@^0.5
"#,
        );

        assert!(error.contains("unresolved `hub:` reference"), "{error}");
        assert!(error.contains("run `dora build`"), "{error}");
    }

    #[test]
    fn every_checkable_field_has_set_detector() {
        let field_cases = [
            ("path", "id: x\npath: ./node\n"),
            ("path_sha256", "id: x\npath: ./node\npath_sha256: abc123\n"),
            ("args", "id: x\npath: ./node\nargs: --foo\n"),
            ("build", "id: x\npath: ./node\nbuild: cargo build\n"),
            (
                "git",
                "id: x\npath: ./node\ngit: https://github.com/example/node.git\n",
            ),
            ("hub", "id: x\npath: ./node\nhub: dora-yolo@^0.5\n"),
            (
                "branch",
                "id: x\npath: ./node\ngit: https://github.com/example/node.git\nbranch: main\n",
            ),
            (
                "tag",
                "id: x\npath: ./node\ngit: https://github.com/example/node.git\ntag: v1.0.0\n",
            ),
            (
                "rev",
                "id: x\npath: ./node\ngit: https://github.com/example/node.git\nrev: abc123\n",
            ),
            ("outputs", "id: x\npath: ./node\noutputs: [out]\n"),
            (
                "output_types",
                "id: x\npath: ./node\noutput_types:\n  out: string\n",
            ),
            (
                "output_framing",
                "id: x\npath: ./node\noutput_framing:\n  out: raw\n",
            ),
            ("inputs", "id: x\npath: ./node\ninputs:\n  in: source/out\n"),
            (
                "input_types",
                "id: x\npath: ./node\ninput_types:\n  in: string\n",
            ),
            (
                "output_metadata",
                "id: x\npath: ./node\noutput_metadata:\n  out: [request_id]\n",
            ),
            ("pattern", "id: x\npath: ./node\npattern: service-server\n"),
            (
                "send_stdout_as",
                "id: x\npath: ./node\nsend_stdout_as: stdout\n",
            ),
            ("send_logs_as", "id: x\npath: ./node\nsend_logs_as: logs\n"),
            (
                "min_log_level",
                "id: x\npath: ./node\nmin_log_level: INFO\n",
            ),
            ("max_log_size", "id: x\npath: ./node\nmax_log_size: 1024\n"),
            (
                "max_rotated_files",
                "id: x\npath: ./node\nmax_rotated_files: 3\n",
            ),
            (
                "shared_memory_pool_size",
                "id: x\npath: ./node\nshared_memory_pool_size: 1048576\n",
            ),
            (
                "restart_policy",
                "id: x\npath: ./node\nrestart_policy: on-failure\n",
            ),
            ("max_restarts", "id: x\npath: ./node\nmax_restarts: 1\n"),
            ("restart_delay", "id: x\npath: ./node\nrestart_delay: 1.0\n"),
            (
                "max_restart_delay",
                "id: x\npath: ./node\nmax_restart_delay: 5.0\n",
            ),
            (
                "restart_window",
                "id: x\npath: ./node\nrestart_window: 60.0\n",
            ),
            (
                "health_check_timeout",
                "id: x\npath: ./node\nhealth_check_timeout: 10.0\n",
            ),
            (
                "finish_grace_secs",
                "id: x\npath: ./node\nfinish_grace_secs: 2.5\n",
            ),
            ("cpu_affinity", "id: x\npath: ./node\ncpu_affinity: [0]\n"),
            ("params", "id: x\npath: ./node\nparams:\n  speed: fast\n"),
        ];

        let expected: BTreeSet<_> = ALL_CHECKABLE_FIELDS
            .iter()
            .map(|field| field.name)
            .collect();
        let actual: BTreeSet<_> = field_cases.iter().map(|(name, _)| *name).collect();
        assert_eq!(actual, expected);

        for (field_name, yaml) in field_cases {
            let node = parse_node(yaml);
            let field = ALL_CHECKABLE_FIELDS
                .iter()
                .find(|field| field.name == field_name)
                .expect("test field should exist in ALL_CHECKABLE_FIELDS");
            assert!(
                (field.is_set)(&node),
                "field `{field_name}` should be detected as set"
            );
        }
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
        classified.extend(ALL_CHECKABLE_FIELDS.iter().map(|field| field.name));
        classified.extend(["operators", "operator", "ros2", "module"]);

        assert_eq!(actual, classified);
    }
}
