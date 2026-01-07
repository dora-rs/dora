# Metadata Use Log Example

This example demonstrates advanced features of Dora's metadata system:
- **Metadata dependencies (herds)**: Reuse node definitions from external packages
- **Workflow-type nodes**: Use entire dataflows as nodes in parent dataflows

## Features Demonstrated

### 1. Simple Herd Nodes
Reference individual nodes from external packages:
```yaml
- id: log_data
  proto: "out_log/log1"  # Single node from herd
  inputs:
    tick: send_data/data
```

### 2. Workflow-Type Nodes
Use complete dataflows as nodes:
```yaml
- id: log_data2
  proto: "out_log/flow_log"  # Entire dataflow as a node
  inputs:
    tick: send_data/data
```

## How It Works

### Herd Dependencies (dora.yaml)
```yaml
dependencies:
  out_log:                    # Custom herd name
    package: log              # Original package name (optional)
    path: ../metadata-log     # Path to herd directory
    nodes: ["log1", "log2", "flow_log"]  # Which nodes to import
```

### Workflow-Type Nodes
When a node's `entry` points to a `.yaml` file, it's treated as a workflow node:

**In the herd's dora.yaml:**
```yaml
- name: flow_log
  export: true
  entry: ./dataflow.yaml     # Points to a dataflow
  inputs:
    - name: tick
      type: number
      required: true
      target: log_data/tick  # Which graph node receives this input
```

The `target` field specifies which node inside the sub-dataflow receives the parent's input.

**Result:**
- The sub-dataflow is expanded inline
- All graph nodes get prefixed IDs (e.g., `log_data2_send_data`, `log_data2_log_data`)
- Internal connections are preserved
- External inputs are routed to specified targets

## Getting Started

```bash
# Build the dataflow
dora build dataflow.yaml --uv

# Run the dataflow
dora run dataflow.yaml --uv
```

## Benefits

- **Reusability**: Share node implementations and complete workflows
- **Composition**: Build complex dataflows from smaller, tested components
- **Namespacing**: Custom herd names prevent conflicts
- **Selective Import**: Choose which nodes to import from a herd
- **Encapsulation**: Workflow nodes hide internal implementation details
