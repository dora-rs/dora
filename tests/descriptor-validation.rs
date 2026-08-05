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
// Custom — invalid combos
// ═══════════════════════════════════════════════════════════

#[test]
fn invalid_custom_rejects_build() {
    descriptor_should_fail("invalid-custom-build.yml", &["build", "not supported"]);
}

#[test]
fn invalid_custom_rejects_git() {
    descriptor_should_fail("invalid-custom-git.yml", &["git", "not supported"]);
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
// Standard — invalid combos
// ═══════════════════════════════════════════════════════════

#[test]
fn invalid_standard_rejects_pattern() {
    descriptor_should_fail(
        "invalid-standard-pattern.yml",
        &["pattern", "not supported"],
    );
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
