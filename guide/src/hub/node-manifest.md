# Writing a Node Manifest

A **node manifest** (`dora-node.yml`) is the typed, versioned description of a
reusable dora node: its entry point, build command, typed input/output
contracts, and configuration surface. It is what turns a folder of code into a
node that other people can discover and wire up with one line of YAML:

```yaml
nodes:
  - id: detector
    hub: dora-yolo@^0.5      # ← resolved from a manifest, no PATH knowledge
    inputs:
      image: camera/image
    outputs:
      - bbox
```

The manifest holds only what a language's native manifest cannot: dora
contracts and dora wiring. Name, version, and dependencies stay in
`pyproject.toml` / `Cargo.toml`; the manifest never duplicates them.

> `hub:` is an **unstable feature** — its behaviour may change in a future
> release. The manifest format itself (`apiVersion: 1`) is stable and useful on
> its own: a `dora-node.yml` next to a plain `path:` node is picked up for
> contract validation today, with no index involved (see
> [Local development](#local-development-no-index)).

## A complete manifest

```yaml
# dora-node.yml — lives next to pyproject.toml / Cargo.toml
apiVersion: 1                 # manifest schema version (currently 1)
name: dora-yolo               # defaults to [project].name / [package].name
namespace: dora-rs            # index namespace (== a GitHub org/user)
description: YOLO object detection on camera frames
categories: [ml-inference]    # from the fixed category list
keywords: [vision, detection, yolo]

runtime: python               # python | rust | c | cpp
entrypoint: dora-yolo         # what to run, relative to the node's working dir
build: pip install .          # how to build it (run in the working dir)

platforms: []                 # optional allowlist, e.g. [linux-x86_64,
                              # linux-aarch64]; empty = all
dora: ">=0.4"                 # optional supported dora version range

inputs:
  image:
    type: std/media/v1/Image  # type URN (see Type Annotations)
    required: true
    description: BGR frame to run detection on
outputs:
  bbox:
    type: std/vision/v1/BoundingBox
    description: detected bounding boxes
  # a node may have zero inputs (a source) or zero outputs (a sink)

env:                          # configuration surface, documented + typed
  MODEL:
    default: yolov8n.pt
    description: model weights file
  CONFIDENCE:
    type: float
    default: 0.4

types: {}                     # optional custom type definitions, keyed by
                              # type URN (see below); omit when there are none

example: |                    # snippet shown by `dora hub info`
  - id: detector
    hub: dora-yolo@^0.5
    inputs:
      image: camera/image
    outputs:
      - bbox
```

### Field reference

| Field | Required | Notes |
|---|---|---|
| `apiVersion` | yes | manifest schema version; currently `1` |
| `name` | defaultable | falls back to `[project].name` / `[package].name`; PEP 503-normalized |
| `namespace` | yes | index key is `namespace/name`; equals a GitHub org/user |
| `runtime` | yes | `python` \| `rust` \| `c` \| `cpp` |
| `entrypoint` | yes | a **relative** path within the working dir — no absolute, no `..` (see below) |
| `build` | recommended | the build command, run in the working dir (`pip install .`, `cargo build --release`, a cmake invocation) |
| `inputs`, `outputs` | yes, **may be empty** | sinks have no outputs; sources no inputs |
| `inputs.*.type` etc. | recommended | type URNs; omit to leave a port untyped (validation skips it) |
| `platforms`, `dora` | recommended | platform allowlist (validated for `<os>-<arch>` shape; drives `dora hub search --platform` filtering) + dora compat range |
| `description`, `categories`, `keywords` | recommended | drive `dora hub search` |
| `env`, `types`, `requirements`, `example` | optional | configuration, shipped types, declared needs, doc snippet |

The manifest schema ships as JSON Schema at
`libraries/core/dora-node-schema.json` for editor autocompletion.

### The entrypoint

`entrypoint` is **validated as a relative path** within the node's working
directory — absolute paths and `..` components are rejected:

- **Python**: a console script on the managed-env `PATH` (the `[project.scripts]`
  name), e.g. `dora-yolo`.
- **Rust / C / C++**: the build output, e.g. `target/release/dora-yolo` or
  `build/lidar`.

When resolved from a hub package, the entrypoint is looked up **only** inside
the node's own working dir or managed environment — the ambient `$PATH` is not
searched, so a typo fails loudly instead of silently running a host binary.

## Typed contracts

Input and output types use dora's existing
[type URN system](../concepts/types.md): `std/<category>/v1/<TypeName>`, nested
struct schemas, parameterized types, and a widening-compatibility graph. When a
`hub:` (or adjacent `path:`) node is composed, the manifest's `inputs`/`outputs`
types surface as the node's `input_types` / `output_types`, and the **normal**
validation pipeline applies:

```bash
dora validate dataflow.yml   # producer/consumer URN compatibility, field diffs
dora build dataflow.yml      # same checks, before anything runs
```

A mismatch — wiring a `std/vision/v1/BoundingBox` output into an input expecting
`std/media/v1/Image` — fails at compose time with the producer/consumer URNs,
before any process starts. The dataflow author's own annotations always win over
the manifest's.

### Shipping custom types

A node may ship its own type definitions (same YAML shape as
`types/std/*/v1.yml`). They must live **under the package's own namespace** —
`std/` and other namespaces are rejected:

```yaml
namespace: acme
types:
  acme/lidar/v1/PointCloud:
    arrow: Struct
    fields:
      - name: x
        type: Float32
      - name: y
        type: Float32
outputs:
  cloud:
    type: acme/lidar/v1/PointCloud
```

Shipped types are loaded into the dataflow's type registry at resolution time,
so a consumer can reference them in its own port checks.

## Scaffolding with `dora hub init`

`dora hub init` writes a starter `dora-node.yml` next to your `pyproject.toml`
or `Cargo.toml`, pre-filling what it can read from there (name, entrypoint from
`[project.scripts]` / the binary target, description):

```bash
cd my-node
dora hub init               # writes ./dora-node.yml
dora hub init path/to/node  # or target a specific directory
```

Fill in your `inputs`/`outputs` types and `env`, then validate the manifest on
its own:

```bash
dora validate --node-manifest dora-node.yml
```

## Local development (no index)

You do **not** need to publish anything to get contract validation. A
`dora-node.yml` sitting next to a plain `path:` node is picked up automatically:

```yaml
nodes:
  - id: detector
    path: target/release/detector   # dora-node.yml lives at the node's root
    inputs:
      image: camera/image
```

`dora build` / `dora validate` read the adjacent manifest, inject its typed
ports, and check your wiring — the same validation a hub consumer gets. This is
the recommended inner loop while authoring a node's contracts.

## Build-location guidance

A node built from source means **something compiles** on the machine that runs
it. dora's multi-daemon flow has each daemon build its own nodes — fine on a
laptop, but slow-to-impossible on a constrained device (a Jetson compiling a
heavy node can thermal-throttle or run out of memory).

- **Compile-on-target** (the default): acceptable for capable machines and
  light nodes.
- **Build-on-a-host-and-ship** (recommended for embedded): build for the target
  architecture on a capable machine and deploy the result via the offline cache
  flow, so the robot never compiles.

As a rule of thumb: **don't run heavy builds on constrained daemons.**
