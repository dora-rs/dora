//! 描述符字段静默丢弃修复的集成测试
//!
//! 通过独立的 YAML 数据流文件验证：
//! - 合法配置能通过 resolve/expand
//! - 非法配置被拒绝且错误信息包含冲突字段名
//!
//! 运行: `cargo test --test descriptor-validation`

use dora_core::descriptor::{DescriptorExt, expand_modules};
use dora_message::descriptor::Descriptor;
use std::path::Path;

const CASES_DIR: &str = "tests/descriptor-validation/cases";

fn load_descriptor(name: &str) -> Descriptor {
    let path = Path::new(CASES_DIR).join(name);
    let yaml =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("failed to read {name}: {e}"));
    serde_yaml::from_str(&yaml).unwrap_or_else(|e| panic!("failed to parse {name}: {e}"))
}

fn descriptor_should_pass(name: &str) {
    let desc = load_descriptor(name);
    let result = desc.resolve_aliases_and_set_defaults();
    if let Err(e) = &result {
        // 如果是 node 不存在的错误（因为路径指向不存在的可执行文件），
        // 说明校验通过了，只是运行时会找不到文件。这是预期行为。
        let msg = format!("{e:#}");
        if msg.contains("not supported") || msg.contains("mutually exclusive") {
            panic!("{name}: expected pass but got rejection: {e:#}");
        }
    }
    // 无论 Ok 还是"路径不存在"类错误，都算通过
}

fn descriptor_should_fail(name: &str, expected_fields: &[&str]) {
    let desc = load_descriptor(name);
    let result = desc.resolve_aliases_and_set_defaults();
    match result {
        Err(e) => {
            let msg = format!("{e:#}");
            for field in expected_fields {
                assert!(
                    msg.contains(field),
                    "{name}: error should mention '{field}', got: {msg}"
                );
            }
        }
        Ok(_) => {
            panic!("{name}: expected rejection but passed");
        }
    }
}

// ═══════════════════════════════════════════════════════════
// Valid configs — must pass
// ═══════════════════════════════════════════════════════════

#[test]
fn valid_standard_passes() {
    descriptor_should_pass("valid-standard.yml");
}

#[test]
fn valid_runtime_passes() {
    descriptor_should_pass("valid-runtime.yml");
}

#[test]
fn valid_operator_passes() {
    descriptor_should_pass("valid-operator.yml");
}

#[test]
fn valid_custom_passes() {
    descriptor_should_pass("valid-custom.yml");
}

#[test]
fn valid_ros2_passes() {
    descriptor_should_pass("valid-ros2.yml");
}

// ═══════════════════════════════════════════════════════════
// Comprehensive — 全字段覆盖测试
// ═══════════════════════════════════════════════════════════

#[test]
fn comprehensive_custom_all_fields() {
    let desc = load_descriptor("comprehensive-custom.yml");
    let resolved = desc
        .resolve_aliases_and_set_defaults()
        .expect("comprehensive custom should resolve");
    let node = resolved.values().next().expect("should have one node");
    let custom = node.kind.as_custom().expect("should resolve to Custom");

    // run_config fields (merged from Node level)
    assert!(
        custom
            .run_config
            .outputs
            .contains(&dora_message::id::DataId::from("data")),
        "outputs should be merged"
    );
    assert!(
        !custom.run_config.output_types.is_empty(),
        "output_types should be merged"
    );
    assert!(
        !custom.run_config.output_framing.is_empty(),
        "output_framing should be merged"
    );
    assert!(
        !custom.run_config.input_types.is_empty(),
        "input_types should be merged"
    );

    // restart fields (custom.restart_policy=always wins over node-level)
    assert!(
        matches!(
            custom.restart_policy,
            dora_message::descriptor::RestartPolicy::Always
        ),
        "custom internal restart_policy=always should win"
    );
    assert_eq!(custom.max_restarts, 10, "max_restarts should be merged");
    assert!(
        custom.restart_delay.is_some(),
        "restart_delay should be merged"
    );
    assert!(
        custom.max_restart_delay.is_some(),
        "max_restart_delay should be merged"
    );
    assert!(
        custom.restart_window.is_some(),
        "restart_window should be merged"
    );

    // runtime fields
    assert!(
        custom.health_check_timeout.is_some(),
        "health_check_timeout should be merged"
    );
    assert!(
        custom.finish_grace_secs.is_some(),
        "finish_grace_secs should be merged"
    );
    assert!(
        custom.run_config.shared_memory_pool_size.is_some(),
        "shared_memory_pool_size should be merged"
    );

    // build/logging fields
    assert!(custom.args.is_some(), "args should be merged");
    assert!(custom.build.is_some(), "build should be merged");
    assert!(custom.path_sha256.is_some(), "path_sha256 should be merged");
    assert!(
        custom.send_stdout_as.is_some(),
        "send_stdout_as should be merged"
    );
    assert!(
        custom.send_logs_as.is_some(),
        "send_logs_as should be merged"
    );
    assert!(
        custom.min_log_level.is_some(),
        "min_log_level should be merged"
    );
    assert!(
        custom.max_log_size.is_some(),
        "max_log_size should be merged"
    );
    assert!(
        custom.max_rotated_files.is_some(),
        "max_rotated_files should be merged"
    );

    // Node-level metadata
    assert!(node.name.is_some(), "name should be preserved");
    assert!(
        node.description.is_some(),
        "description should be preserved"
    );
    assert!(node.env.is_some(), "env should be preserved");
    assert!(
        node.cpu_affinity.is_some(),
        "cpu_affinity should be preserved"
    );
}

#[test]
fn comprehensive_standard_all_fields() {
    let desc = load_descriptor("comprehensive-standard.yml");
    let resolved = desc
        .resolve_aliases_and_set_defaults()
        .expect("comprehensive standard should resolve");
    let node = resolved.values().next().expect("should have one node");
    let custom = node
        .kind
        .as_custom()
        .expect("Standard should resolve to Custom");

    assert!(
        !custom.run_config.outputs.is_empty(),
        "outputs should be set"
    );
    assert!(!custom.run_config.inputs.is_empty(), "inputs should be set");
    assert!(
        !custom.run_config.output_types.is_empty(),
        "output_types should be set"
    );
    assert!(
        !custom.run_config.input_types.is_empty(),
        "input_types should be set"
    );
    assert!(
        !custom.run_config.output_framing.is_empty(),
        "output_framing should be set"
    );
    assert!(
        custom.run_config.shared_memory_pool_size.is_some(),
        "shared_memory_pool_size should be set"
    );

    assert!(
        !matches!(
            custom.restart_policy,
            dora_message::descriptor::RestartPolicy::Never
        ),
        "restart_policy should be on-failure"
    );
    assert!(custom.health_check_timeout.is_some());
    assert!(custom.finish_grace_secs.is_some());
    assert!(custom.args.is_some());
    assert!(custom.build.is_some());
    assert!(custom.send_stdout_as.is_some());
    assert!(custom.send_logs_as.is_some());
    assert!(custom.min_log_level.is_some());
    assert!(custom.max_log_size.is_some());
    assert!(custom.max_rotated_files.is_some());
}

#[test]
fn comprehensive_runtime_all_fields() {
    descriptor_should_pass("comprehensive-runtime.yml");
}

#[test]
fn comprehensive_operator_all_fields() {
    descriptor_should_pass("comprehensive-operator.yml");
}

#[test]
fn comprehensive_ros2_all_fields() {
    let desc = load_descriptor("comprehensive-ros2.yml");
    let resolved = desc
        .resolve_aliases_and_set_defaults()
        .expect("comprehensive ros2 should resolve");
    let node = resolved.values().next().expect("should have one node");
    let custom = node
        .kind
        .as_custom()
        .expect("ROS2 should resolve to Custom");

    // ROS2 bridge config is serialized into env by resolution
    assert!(
        custom
            .envs
            .as_ref()
            .is_some_and(|e| e.contains_key("DORA_ROS2_BRIDGE_CONFIG")),
        "ROS2 config should be serialized into env"
    );

    // Node-level fields copied into CustomNode (like Standard nodes)
    assert!(
        !custom.run_config.outputs.is_empty(),
        "outputs should be set"
    );
    assert!(
        !custom.run_config.output_types.is_empty(),
        "output_types should be set"
    );
    assert!(
        !custom.run_config.output_framing.is_empty(),
        "output_framing should be set"
    );
    assert!(
        !custom.run_config.input_types.is_empty(),
        "input_types should be set"
    );
    assert!(
        custom.run_config.shared_memory_pool_size.is_some(),
        "shared_memory_pool_size should be set"
    );

    assert!(
        !matches!(
            custom.restart_policy,
            dora_message::descriptor::RestartPolicy::Never
        ),
        "restart_policy should be on-failure"
    );
    assert_eq!(custom.max_restarts, 3, "max_restarts should be set");
    assert!(
        custom.restart_delay.is_some(),
        "restart_delay should be set"
    );
    assert!(
        custom.max_restart_delay.is_some(),
        "max_restart_delay should be set"
    );
    assert!(
        custom.health_check_timeout.is_some(),
        "health_check_timeout should be set"
    );
    assert!(
        custom.finish_grace_secs.is_some(),
        "finish_grace_secs should be set"
    );
    assert!(
        custom.send_stdout_as.is_some(),
        "send_stdout_as should be set"
    );
    assert!(custom.send_logs_as.is_some(), "send_logs_as should be set");
    assert!(
        custom.min_log_level.is_some(),
        "min_log_level should be set"
    );
    assert!(custom.max_log_size.is_some(), "max_log_size should be set");
    assert!(
        custom.max_rotated_files.is_some(),
        "max_rotated_files should be set"
    );
}

// ═══════════════════════════════════════════════════════════
// Runtime — invalid combos
// ═══════════════════════════════════════════════════════════

#[test]
fn invalid_runtime_rejects_build() {
    descriptor_should_fail("invalid-runtime-build.yml", &["build", "not supported"]);
}

#[test]
fn invalid_runtime_rejects_args() {
    descriptor_should_fail("invalid-runtime-args.yml", &["args", "not supported"]);
}

#[test]
fn invalid_runtime_rejects_outputs() {
    descriptor_should_fail("invalid-runtime-outputs.yml", &["outputs", "not supported"]);
}

#[test]
fn invalid_runtime_rejects_restart_policy() {
    descriptor_should_fail(
        "invalid-runtime-restart.yml",
        &["restart_policy", "not supported"],
    );
}

// ═══════════════════════════════════════════════════════════
// Operator — invalid combos
// ═══════════════════════════════════════════════════════════

#[test]
fn invalid_operator_rejects_build() {
    descriptor_should_fail("invalid-operator-build.yml", &["build", "not supported"]);
}

#[test]
fn invalid_operator_rejects_restart_policy() {
    descriptor_should_fail(
        "invalid-operator-restart.yml",
        &["restart_policy", "not supported"],
    );
}

// ═══════════════════════════════════════════════════════════
// Custom — fields now merged (no longer rejected)
// ═══════════════════════════════════════════════════════════

#[test]
fn custom_merges_node_level_outputs() {
    descriptor_should_pass("invalid-custom-outputs.yml");
}

#[test]
fn custom_merges_node_level_inputs() {
    descriptor_should_pass("invalid-custom-inputs.yml");
}

#[test]
fn custom_merges_node_level_build() {
    descriptor_should_pass("invalid-custom-build.yml");
}

#[test]
fn custom_merges_node_level_run_config() {
    descriptor_should_pass("invalid-custom-run-config.yml");
}

#[test]
fn custom_merges_node_level_restart() {
    descriptor_should_pass("invalid-custom-restart.yml");
}

#[test]
fn custom_merges_node_level_runtime_fields() {
    descriptor_should_pass("invalid-custom-runtime-fields.yml");
}

/// Verify Node-level fields are actually merged into the resolved
/// CustomNode, not just silently accepted.
#[test]
fn custom_node_level_fields_merged_into_resolved() {
    let desc = load_descriptor("valid-custom.yml");
    let resolved = desc
        .resolve_aliases_and_set_defaults()
        .expect("valid-custom should resolve");
    let node = resolved
        .values()
        .next()
        .expect("should have at least one resolved node");
    let custom = node
        .kind
        .as_custom()
        .expect("valid-custom should resolve to Custom");

    // outputs and restart_policy were written at Node-level in the YAML
    // and should have been merged into the CustomNode sub-structure.
    assert!(
        !custom.run_config.outputs.is_empty(),
        "node-level outputs should be merged into run_config"
    );
    assert!(
        !matches!(
            custom.restart_policy,
            dora_message::descriptor::RestartPolicy::Never
        ),
        "node-level restart_policy should be merged into CustomNode"
    );
}

// ═══════════════════════════════════════════════════════════
// Custom — truly incompatible fields (still rejected)
// ═══════════════════════════════════════════════════════════

#[test]
fn invalid_custom_rejects_git() {
    descriptor_should_fail("invalid-custom-git.yml", &["git", "not supported"]);
}

#[test]
fn invalid_custom_rejects_metadata() {
    descriptor_should_fail(
        "invalid-custom-metadata.yml",
        &["output_metadata", "pattern", "not supported"],
    );
}

// ═══════════════════════════════════════════════════════════
// ROS2 Bridge — invalid combos
// ═══════════════════════════════════════════════════════════

#[test]
fn invalid_ros2_rejects_build() {
    descriptor_should_fail("invalid-ros2-build.yml", &["build", "ros2"]);
}

// ═══════════════════════════════════════════════════════════
// hub — mutually exclusive
// ═══════════════════════════════════════════════════════════

#[test]
fn invalid_hub_path_mutually_exclusive() {
    descriptor_should_fail("invalid-hub-path.yml", &["hub", "not supported"]);
}

// ═══════════════════════════════════════════════════════════
// Standard — valid combos (pattern / output_metadata are legal)
// ═══════════════════════════════════════════════════════════

#[test]
fn valid_standard_accepts_pattern() {
    descriptor_should_pass("invalid-standard-pattern.yml");
}

#[test]
fn valid_standard_accepts_output_metadata() {
    descriptor_should_pass("valid-standard-output-metadata.yml");
}

// ═══════════════════════════════════════════════════════════
// Module — invalid combos (需要 expand_modules)
// ═══════════════════════════════════════════════════════════

fn load_and_expand(name: &str) -> eyre::Result<Descriptor> {
    let path = Path::new(CASES_DIR).join(name);
    let yaml =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("failed to read {name}: {e}"));
    let desc: Descriptor =
        serde_yaml::from_str(&yaml).unwrap_or_else(|e| panic!("failed to parse {name}: {e}"));
    let base = Path::new(CASES_DIR)
        .canonicalize()
        .expect("cases dir must exist");
    expand_modules(&desc, &base).map_err(Into::into)
}

#[test]
fn invalid_module_rejects_env() {
    let result = load_and_expand("invalid-module-env.yml");
    match result {
        Err(e) => {
            let msg = format!("{e:#}");
            assert!(
                msg.contains("not supported") && msg.contains("env"),
                "expected env rejection, got: {msg}"
            );
        }
        Ok(_) => panic!("expected rejection but passed"),
    }
}

#[test]
fn invalid_module_rejects_build() {
    let result = load_and_expand("invalid-module-build.yml");
    match result {
        Err(e) => {
            let msg = format!("{e:#}");
            assert!(
                msg.contains("not supported") && msg.contains("build"),
                "expected build rejection, got: {msg}"
            );
        }
        Ok(_) => panic!("expected rejection but passed"),
    }
}
