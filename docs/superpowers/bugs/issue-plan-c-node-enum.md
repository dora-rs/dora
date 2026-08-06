## Summary

Refactor `Node` from a flat struct into an enum with per-kind variants, so YAML fields
can only land where they're supposed to.

## Problem

`Node` is a single flat struct with ~30 optional fields covering 6 node kinds:

```rust
pub struct Node {
    pub path: Option<String>,
    pub operators: Option<RuntimeNode>,
    pub custom: Option<CustomNode>,
    pub operator: Option<SingleOperatorDefinition>,
    pub ros2: Option<Ros2BridgeConfig>,
    pub module: Option<String>,
    pub outputs: BTreeSet<DataId>,       // valid for Standard, merging for Custom
    pub restart_policy: RestartPolicy,   // valid for Standard, merging for Custom
    // ... etc
}
```

Because the same field name (`outputs`, `restart_policy`, etc.) exists on both `Node`
and sub-structures like `CustomNode.run_config`, serde silently routes them to
different Rust fields depending on YAML indentation — with no error for the wrong
choice:

```yaml
nodes:
  - id: my_custom
    custom:
      outputs: [a]    # → CustomNode.run_config.outputs  ✅ read
    outputs: [b]       # → Node.outputs                   ❌ dropped (BUG-005)
```

The current fix (merge of #2911) works around this by merging `Node.*` into
sub-structures at resolution time, but the root cause persists: the type system
allows fields to end up in the wrong struct.

## Proposal

Replace `Node` with an enum where each variant carries only its legal fields:

```rust
pub enum NodeDef {
    Standard {
        id: NodeId,
        path: String,
        source: NodeSource,
        outputs: BTreeSet<DataId>,
        restart_policy: RestartPolicy,
        // ...
    },
    Custom {
        id: NodeId,
        custom: CustomNode,        // contains ALL config inside
        // no flat outputs, restart_policy etc. — they're in CustomNode
    },
    Runtime {
        id: NodeId,
        operators: RuntimeNode,    // contains ALL config inside
    },
    // ... Operator, Ros2Bridge, Module
}
```

With a custom `Deserialize` that rejects fields at the wrong level:

```yaml
# ❌ serde error: "unexpected field `outputs` on Custom node —
# did you mean to put it inside `custom:`?"
nodes:
  - id: my_custom
    custom:
      path: ./node.py
    outputs: [a]
```

## Benefits

- **Impossible to silently drop** — wrong-level fields rejected at parse time
- **Self-documenting** — each variant's shape makes valid config obvious
- **Removes validation code** — no more deny-list / merge logic
- **Better IDE support** — schema / autocomplete per kind

## Risks / Considerations

- Large refactor — every descriptor read path changes
- Backward compatibility — if users relied on Node-level fields for
  Standard nodes, the migration must preserve that (Standard variant
  keeps them flat)
- Module expansion currently mutates `Node` fields — needs redesign
- `CoreNodeKind` (Runtime/Custom) already exists and could be unified

## Related

- PR #2911 — current incremental fix
- BUG-005 — silent field drop on Custom nodes
