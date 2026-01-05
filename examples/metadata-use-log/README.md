# Metadata Use Log Example

This example demonstrates how to use **metadata dependencies** (also called "herds") in Dora dataflows. Herds allow you to reuse node definitions from external packages, making it easy to compose dataflows from shared components.

## Overview

This dataflow uses:
- A local node (`sender`) defined in the local `dora.yaml`
- A node from an external herd (`log/log1`) defined in `../metadata-log/dora.yaml`

## File Structure

### dataflow.yaml
Defines the dataflow graph using `proto` references:
```yaml
graph:
  - id: send_data
    proto: "sender"        # Local node from dora.yaml
    inputs:
      tick: dora/timer/millis/10

  - id: log_data
    proto: "out_log/log1"  # Node from herd dependency
    inputs:
      tick: send_data/data
```

### dora.yaml
Defines local nodes and declares dependencies:
```yaml
nodes:
  - name: sender
    lang: python
    entry: ./send_data.py
    build: pip install numpy pyarrow
    outputs:
      - name: data
        type: number

dependencies:
  out_log:                    # Custom herd name (used in proto references)
    package: log              # Original herd package name (optional, for documentation)
    path: ../metadata-log     # Path to the herd directory
    nodes: ["log1", "log2"]   # Which nodes to import (empty = all exported nodes)
```

## How It Works

1. **Dependencies Map**: The `dependencies` field in `dora.yaml` is a map where:
   - **Key** (`out_log`): The custom name used as prefix in proto references (e.g., `out_log/log1`)
   - **Value**: Contains:
     - `package`: Original herd name (optional, for clarity)
     - `path`: Relative or absolute path to the herd package
     - `nodes`: List of node names to import (empty means all exported nodes)

2. **Proto References**: 
   - Local nodes: Use simple names like `"sender"`
   - Herd nodes: Use qualified names like `"out_log/log1"` (custom_name/node_name)

3. **Path Resolution**: When a node comes from a herd, its `entry` path is resolved relative to the herd's directory, not the consumer's directory.

## Getting Started

```bash
# Build the dataflow
dora build dataflow.yaml --uv

# Run the dataflow
dora run dataflow.yaml --uv
```

## Benefits

- **Reusability**: Share node implementations across multiple dataflows
- **Versioning**: Pin to specific versions using git references in herd metadata
- **Namespacing**: Custom herd names prevent naming conflicts
- **Selective Import**: Choose which nodes to import from a herd
