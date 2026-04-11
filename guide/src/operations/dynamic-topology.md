# Dynamic Topology

Add and remove nodes from running dataflows without restarting.

## CLI Commands

```bash
# Add a node from a YAML definition
dora node add --from-yaml new-node.yml --dataflow my-app

# Remove a node (stops process + cleans up mappings)
dora node remove my-app filter-node

# Connect two nodes (add a live mapping)
dora node connect --dataflow my-app sender/value filter/input

# Disconnect two nodes (remove a mapping)
dora node disconnect --dataflow my-app sender/value filter/input
```

## Node YAML Definition

Dynamic nodes are defined in standalone YAML files with the same format
as a single entry in the `nodes:` list:

```yaml
# filter-node.yml
id: filter
path: filter.py
outputs:
  - output
```

After adding, wire inputs explicitly with `dora node connect`.

## Examples

- [dynamic-add-remove](../../../examples/dynamic-add-remove/) — basic add/remove/connect pipeline
- [dynamic-agent-tools](../../../examples/dynamic-agent-tools/) — AI agent with dynamically-added tools

## Current Limitations

- Daemon-side node spawning for `AddNode` is pending (coordinator dispatch works, daemon logs a warning)
- Cross-daemon dynamic topology not yet supported
- Dynamic nodes are not persisted across dataflow restart

See the [Dynamic Topology Plan](../../../docs/plan-dynamic-topology.md) for the full design.
