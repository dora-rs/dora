# Modules (Reusable Sub-Dataflows)

Modules let you define reusable sub-graphs of nodes in separate YAML files and compose them into larger dataflows. Modules are expanded at compile time -- the runtime never sees them.

## Quick Start

**Module file** (`modules/transform_module.yml`):

```yaml
module:
  name: transform_pipeline
  inputs: [raw_data]
  outputs: [filtered]

nodes:
  - id: doubler
    path: doubler.py
    inputs:
      data: _mod/raw_data
    outputs:
      - doubled

  - id: filter
    path: filter_even.py
    inputs:
      data: doubler/doubled
    outputs:
      - filtered
```

**Dataflow file** (`dataflow.yml`):

```yaml
nodes:
  - id: sender
    path: sender.py
    outputs:
      - value

  - id: pipeline
    module: modules/transform_module.yml
    inputs:
      raw_data: sender/value

  - id: receiver
    path: receiver.py
    inputs:
      filtered: pipeline/filtered
```

After expansion, `pipeline` becomes two nodes: `pipeline.doubler` and `pipeline.filter`, with all wiring resolved automatically.

## Module Definition File

A module file has two sections:

### `module:` header

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `name` | string | yes | Module name (metadata only) |
| `inputs` | list | no | Required input port names |
| `inputs_optional` | list | no | Optional input ports (silently skipped if not wired) |
| `outputs` | list | no | Output port names exposed to the parent dataflow |

### `nodes:` list

Standard node definitions, with one special syntax: **`_mod/port_name`** references a module input port. When expanded, `_mod/port_name` is replaced with whatever the parent wired to that port.

```yaml
module:
  name: my_module
  inputs: [camera_feed]
  outputs: [detections]

nodes:
  - id: detector
    path: detect.py
    inputs:
      image: _mod/camera_feed    # resolved to parent's wiring
    outputs:
      - detections
```

### Module-level build

Modules can have a top-level `build:` command that runs before any inner node builds:

```yaml
module:
  name: ml_pipeline
  inputs: [image]
  outputs: [result]

build: pip install -r requirements.txt

nodes:
  - id: model
    path: model.py
    inputs:
      image: _mod/image
    outputs:
      - result
```

## Using Modules

Reference a module in a dataflow node using the `module:` field instead of `path:`:

```yaml
- id: nav_stack
  module: modules/navigation.module.yml
  inputs:
    goal_pose: localization/goal
```

The module node's `inputs:` map wires parent outputs to module input ports. External nodes reference module outputs as `<module_id>/<output_name>` (e.g., `nav_stack/cmd_vel`).

## Parameters

Pass configuration values to modules via `params:`:

```yaml
- id: fast_pipeline
  module: modules/transform_module.yml
  inputs:
    raw_data: sender/value
  params:
    speed: "2.0"
    mode: turbo
```

Inside the module, reference params in `args:` using `$PARAM_<UPPERCASE_KEY>`:

```yaml
nodes:
  - id: processor
    path: processor.py
    args: --speed $PARAM_SPEED --mode $PARAM_MODE
    inputs:
      data: _mod/raw_data
    outputs:
      - result
```

Parameters are also injected as environment variables (`PARAM_SPEED`, `PARAM_MODE`) into every node inside the module.

## Expansion Rules

1. Load the module YAML file and validate its header
2. Prefix all internal node IDs with `{module_id}.` (e.g., `nav_stack.planner`)
3. Replace `_mod/port_name` references with the actual sources from the parent's input map
4. Rewrite internal cross-references (e.g., `planner/path` becomes `nav_stack.planner/path`)
5. Map module-declared outputs to internal node outputs, so `nav_stack/cmd_vel` resolves to `nav_stack.controller/cmd_vel`
6. Replace the module node with the expanded flat nodes
7. Substitute `params:` values in `args:` fields and inject as env vars

Use `adora expand` to see the result:

```bash
adora expand dataflow.yml
```

## Nested Modules

Modules can reference other modules. The expansion is recursive with a depth limit of 8 levels:

```yaml
# outer_module.yml
module:
  name: outer
  inputs: [data]
  outputs: [result]

nodes:
  - id: inner
    module: inner_module.yml
    inputs:
      raw: _mod/data

  - id: postprocess
    path: postprocess.py
    inputs:
      data: inner/processed
    outputs:
      - result
```

After expansion, node IDs are fully qualified: `outer.inner.some_node`.

## Optional Inputs

Declare inputs as optional when a module should work with or without certain connections:

```yaml
module:
  name: flexible_processor
  inputs: [data]
  inputs_optional: [config]
  outputs: [result]

nodes:
  - id: processor
    path: processor.py
    inputs:
      data: _mod/data
      config: _mod/config    # silently dropped if not wired
    outputs:
      - result
```

When the parent doesn't wire `config`, the input is simply omitted from the expanded node.

## Visualization

`adora graph` renders module boundaries as Mermaid subgraphs, making it easy to see which nodes came from which module:

```bash
adora graph dataflow.yml --open
```

## Validation

Validate a standalone module file without a full dataflow:

```bash
adora expand --module modules/transform_module.yml
```

This checks:
- Valid YAML structure
- Module header is present with `name`, `inputs`, `outputs`
- All `_mod/` references correspond to declared inputs or optional inputs
- No duplicate node IDs
- Internal wiring is consistent

## Security

- **Path confinement**: Module file paths must resolve within the dataflow's base directory. Absolute paths and directory traversal (`../`) outside the base are rejected.
- **File size limit**: Module files are capped at 1 MB.
- **Depth limit**: Recursive nesting is capped at 8 levels.
- **Param key validation**: Parameter keys must be alphanumeric with underscores only.

## Example

See [`examples/module-dataflow/`](../examples/module-dataflow/) for a complete working example with a sender, transform module (doubler + filter), and receiver.

```bash
adora run examples/module-dataflow/dataflow.yml
```
