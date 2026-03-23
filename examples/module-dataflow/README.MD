# Module Dataflow

Demonstrates **reusable sub-graphs** (modules) — one of Adora's most
powerful features for composing large dataflows from smaller, self-contained
pieces.

## Architecture

```
sender --> value --> [pipeline module] --> filtered --> receiver
                           │
                           ├── pipeline.doubler    (multiplies each value × 2)
                           └── pipeline.filter     (keeps only even numbers)
```

The `pipeline` node in `dataflow.yml` is not a single node — at build time
it expands into two nodes: `pipeline.doubler` and `pipeline.filter`, with
all wiring resolved automatically.

## Nodes

**sender** (`sender.py`) — Emits integers 0–19 on the `value` output, one
every 50 ms, then exits.

**pipeline** (`modules/transform_module.yml`) — A reusable module containing
two internal nodes:
- **pipeline.doubler** (`modules/doubler.py`): receives each value and sends `value × 2`
- **pipeline.filter** (`modules/filter_even.py`): passes through only even numbers,
  drops odd ones silently

**receiver** (`receiver.py`) — Logs every value it receives on the `filtered`
input.

## Run

```bash
pip install adora-rs pyarrow
adora run dataflow.yml
```

Expected output (abridged):

```
INFO  pipeline.doubler: Doubled [0] -> [0]
INFO  pipeline.filter:  Passed through even values: [0]
INFO  receiver:         Received [filtered]: [0]

INFO  pipeline.doubler: Doubled [1] -> [2]
INFO  pipeline.filter:  Passed through even values: [2]
INFO  receiver:         Received [filtered]: [2]

INFO  pipeline.doubler: Doubled [2] -> [4]
INFO  pipeline.filter:  Passed through even values: [4]
INFO  receiver:         Received [filtered]: [4]
...
INFO  pipeline.doubler: Doubled [19] -> [38]
INFO  pipeline.filter:  Passed through even values: [38]
INFO  receiver:         Received [filtered]: [38]
```

All 20 inputs (0–19) reach the receiver because doubling any integer always
produces an even number — the filter never drops anything in this example.

## Inspect the Expansion

Use `adora expand` to see the flat dataflow after module expansion:

```bash
adora expand dataflow.yml
```

The `pipeline` node disappears and is replaced by `pipeline.doubler` and
`pipeline.filter` with fully qualified IDs and resolved wiring. This is
exactly what the daemon receives at runtime — modules have zero runtime
overhead.

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `module:` field instead of `path:` | `dataflow.yml` |
| Module input/output declarations | `modules/transform_module.yml` |
| `_mod/port_name` to reference module inputs | `modules/transform_module.yml` |
| Compile-time node ID prefixing (`pipeline.doubler`) | `adora expand dataflow.yml` |
| Internal node wiring inside a module | `doubler → filter` |
| Zero runtime overhead | modules expand before spawn |

## Key Concept: Modules

Instead of wiring `doubler` and `filter` into every dataflow that needs them,
define them once in a module file:

```yaml
# modules/transform_module.yml
module:
  name: transform_pipeline
  inputs: [raw_data]
  outputs: [filtered]

nodes:
  - id: doubler
    path: doubler.py
    inputs:
      data: _mod/raw_data    # _mod/ refers to the module's declared inputs
    outputs:
      - doubled

  - id: filter
    path: filter_even.py
    inputs:
      data: doubler/doubled
    outputs:
      - filtered
```

Then reference it anywhere with a single line:

```yaml
- id: pipeline
  module: modules/transform_module.yml
  inputs:
    raw_data: sender/value   # wire the module's input port
```

Downstream nodes reference module outputs as `pipeline/filtered` — they
never see the internal structure.
