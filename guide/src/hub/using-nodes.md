# Using Hub Nodes

This chapter is the consumer side: finding a node, wiring it into a dataflow, and
understanding how the version you wrote resolves to the version you run. To
*author* a node, see [Writing a Node Manifest](node-manifest.md).

## Referencing a node

Add a `hub:` field to a node instead of `path:` or `git:`:

```yaml
nodes:
  - id: detector
    hub: dora-yolo@^0.5
    inputs:
      image: camera/image
    outputs:
      - bbox

  - id: camera
    hub: opencv-video-capture@^0.3
    outputs:
      - image
```

`hub:` is mutually exclusive with `path:`, `git:`, `build:`, and the
`branch`/`tag`/`rev` fields on the same node — the Hub provides all of those.
You still write the node's `inputs`/`outputs` wiring as usual; the *types* of
those ports come from the published manifest.

### The reference syntax

```
[<namespace>/]<name>@<version-requirement>
```

| Form | Meaning |
|---|---|
| `dora-yolo@^0.5` | name in the **official** `dora-rs` namespace |
| `dora-rs/dora-yolo@^0.5` | the same, fully qualified |
| `acme/lidar@~2.1` | a third-party namespace |
| `dora-vad` | no `@` → `*`, any version (the latest non-yanked) |

A bare name (no `namespace/`) means the official `dora-rs` namespace. A namespace
maps to a GitHub org or user and resolves against exactly one index (see
[Custom & Enterprise Indexes](indexes.md)).

### Version requirements

The part after `@` is a standard semver/Cargo version requirement:

| Requirement | Matches |
|---|---|
| `^0.5` | `>=0.5.0, <0.6.0` (caret; `0.x` treats minor as breaking) |
| `^1.2` | `>=1.2.0, <2.0.0` |
| `~2.1` | `>=2.1.0, <2.2.0` |
| `>=0.5, <0.7` | an explicit range |
| `=1.2.3` | exactly that version |
| `*` (or omitted) | any version |

Resolution picks the **highest non-yanked** version that satisfies the
requirement. Pre-release versions (e.g. `0.6.0-rc1`) are only matched by a
requirement that itself names a pre-release, following the Cargo convention.

Pin your requirement, then lock it for reproducibility — see [Reproducible
Builds](reproducible-builds.md). Without a lockfile, each `dora build`
re-resolves to the latest matching version.

## Finding nodes

### Search the catalog

```bash
dora hub search yolo                    # name / description / keyword match
dora hub search --category ml-inference # browse a category
dora hub search lidar --platform linux-x86_64
dora hub search                         # list everything
```

Search spans every configured index and prints `namespace/name version —
description (categories)`, using each package's latest version.

### Inspect a package

`dora hub info` shows a package's typed contracts, configuration, and a ready-to-
paste example — read this before wiring a node so you know its ports and types:

```bash
dora hub info dora-yolo            # latest version
dora hub info dora-yolo@0.5.2      # a specific version
dora hub info acme/lidar@^2        # a requirement
```

It prints the version (and a `(yanked)` marker if applicable), description,
categories, supported platforms, runtime, the **typed inputs and outputs**, env
vars, and the example snippet.

Add `--offline` to any of these to use only the local cache without hitting the
network.

## Building and validating

Resolution happens at `dora build` / `dora validate` / `dora run` time — there is
no separate fetch step:

```bash
dora validate dataflow.yml   # resolve hub nodes, check wiring + types, no build
dora build dataflow.yml      # resolve, pin, and build every node
dora run dataflow.yml        # build + run (local)
```

Because a published node ships its input/output **types** in its manifest, the
normal type-checking pipeline applies to it: wiring a `BoundingBox` output into
an input that expects an `Image` fails at compose time with the producer/consumer
URNs, before any process starts. Your own annotations in the dataflow always take
precedence over the manifest's.

## What gets built

A `hub:` node desugars to a pinned `git:` node: the resolver clones the node's
source at the exact published commit and runs its `build` command (e.g.
`pip install .`, `cargo build --release`) on the machine that builds it. For a
node that ships a prebuilt binary for your platform, the resolver downloads and
sha256-verifies that artifact instead of cloning — see [Binary
distribution](indexes.md#binary-distribution).

Heavy builds run wherever the node runs, so on constrained devices prefer
build-on-a-host-and-ship (see the manifest chapter's [build-location
guidance](node-manifest.md#build-location-guidance) and the offline
[`dora hub fetch`](reproducible-builds.md#offline-and-air-gapped-builds) flow).

## Overriding a node locally

While iterating on a node you consume, point a single package at a local checkout
without editing the YAML or publishing anything:

```bash
dora build dataflow.yml --hub-override dora-yolo=../my-fork/dora-yolo
```

This is a local-build-only inner-loop tool (it cannot combine with
`--write-lockfile` or distributed builds). See [Reproducible
Builds](reproducible-builds.md) for the lockfile interactions.
