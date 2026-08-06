# BUG-005: Node 级字段静默丢弃 — 问题总结

## 背景

Dora 的 YAML 描述符中，节点有 6 种类型（Standard / Runtime / Operator / Custom / ROS2 Bridge / Module）。
每种类型通过不同的 YAML 字段区分（`path:` / `operators:` / `operator:` / `custom:` / `ros2:` / `module:`）。

部分类型有子结构（如 `custom:` 块是 `CustomNode`，`operators:` 是 `RuntimeNode`），
子结构内部也有 `outputs`、`inputs`、`restart_policy` 等字段。
这些字段和 Node 级的对应字段**同名但属于不同 Rust struct**。

## 问题

当用户把字段写在子结构外部（Node 级）时，serde 反序列化到 `Node` struct 的同名字段，
但解析函数 `resolve_aliases_and_set_defaults_in_topology` 对不同节点类型处理方式不同：

| 节点类型   | 解析方式       | Node 级字段处理      |
|-----------|---------------|---------------------|
| Standard  | 逐字段复制到 CustomNode | ✅ 有效      |
| ROS2 Bridge | 同 Standard  | ✅ 有效              |
| Custom    | `.clone()`    | ❌ 静默丢弃 → bug-005 |
| Runtime   | `.clone()`    | ❌ 静默丢弃（早有校验） |
| Operator  | `.clone()`    | ❌ 静默丢弃（早有校验） |
| Module    | 展开阶段独立校验 | ❌ 被拦截             |

关键代码路径 (`libraries/core/src/descriptor/mod.rs`):

```rust
// 第 396-426 行：Standard 节点 → 复制 Node 级字段
NodeKindMut::Standard { path, source, .. } => CoreNodeKind::Custom(CustomNode {
    run_config: NodeRunConfig {
        inputs: node.inputs,       // ← 读 Node 级
        outputs: node.outputs,     // ← 读 Node 级
        ...
    },
    restart_policy: node.restart_policy,  // ← 读 Node 级
    ...
}),

// 第 427 行：Custom 节点 → 只克隆子结构
NodeKindMut::Custom(node) => CoreNodeKind::Custom(node.clone()),
//                                              ↑ node.inputs、node.outputs 等被丢弃
```

## 当前修复方案：deny-list

在 `validate_node_fields_for_kind` 中为每种节点类型手动列出冲突字段，
在解析前拦截并报错。已覆盖 15+ 字段（`outputs`、`inputs`、`restart_policy` 等）。

## 已知回归（PR #2911 审查指出，尚未修复）

Standard 节点的 `pattern` 和 `output_metadata` 被误加入 deny-list：

```rust
NodeKind::Standard(_) => {
    if !node.output_metadata.is_empty() { conflicts.push("output_metadata"); } // ← 回归！
    if node.pattern.is_some()            { conflicts.push("pattern"); }        // ← 回归！
    if node.hub.is_some()                { conflicts.push("hub"); }            // ✅ 正确
}
```

- `pattern` 和 `output_metadata` 对 Standard 节点是**合法字段**，
  被 `check_metadata_annotations` (`validate.rs:964`) 消费
- `hub` 是正确的新增检查（Standard + hub 应报错）

测试 `invalid_standard_rejects_pattern` 把回归锁进了测试套件。

## 设计问题：deny-list vs allow-list

### deny-list（当前）

```
优点：实现简单，错误消息精确
缺点：易遗漏，新增 Node 字段不会自动拦截
```

### allow-list

```
优点：不漏，新字段默认被拒绝
缺点：需遍历所有 struct 字段，实现复杂，错误消息可能不够精确
```

### 源头方案

```
让 serde 反序列化时拒绝歧义字段（同级不可重复）
理想但兼容性风险大（Standard 节点的 flat 结构依赖 Node 级字段）
```

## 涉及文件

| 文件 | 角色 |
|------|------|
| `libraries/message/src/descriptor.rs` | Node / CustomNode / RuntimeNode struct 定义 |
| `libraries/core/src/descriptor/mod.rs` | `resolve_aliases_and_set_defaults_in_topology` 解析逻辑 + `validate_node_fields_for_kind` 校验 |
| `libraries/core/src/descriptor/expand.rs` | Module 展开 + `module_validate_node_fields_for_kind` 校验 |
| `tests/descriptor-validation.rs` | 集成测试（24 个用例） |
| `tests/descriptor-validation/cases/*.yml` | 测试用的 YAML 描述符 |

## 待解决

1. **立即修**：Standard 节点的 `pattern`/`output_metadata` 误拒绝回归
2. **设计决策**：deny-list 继续用还是换 allow-list？
3. **长期**：是否在 serde 层消除字段歧义？
