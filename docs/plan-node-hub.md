# Dora Hub — Node Packaging & Distribution Spec

> **Status:** Draft for review — review window: 14 days from the spec PR
> opening; open questions (§15) adopt their stated defaults on silence.
> **Discussion:** https://github.com/orgs/dora-rs/discussions/2095
> **Umbrella issue:** https://github.com/dora-rs/dora/issues/2097
> **Credit:** Builds on @LeonRust's Dora Hub proposal (#2095) and the
> architecture review posted there. Most of the original's decision matrix is
> adopted unchanged; the deltas and their reasons are in
> [Appendix A](#appendix-a--delta-vs-the-original-proposal). Roadmap items
> marked *(seeking owner)* are open for anyone to claim — contributions from
> the original proposal's author are explicitly invited on the items closest
> to that design (P2.3 index bootstrap, category curation, P4.1 web).

Dora Hub is dora's node packaging, discovery, and distribution system: node
authors publish typed, versioned nodes once; dataflow authors find and use
them with one line of YAML. This spec is the implementation contract — work
proceeds as small PRs against the roadmap in [§14](#14-roadmap), tracked under
the umbrella issue #2097.

The architecture in one sentence: **node source lives in git (in the
`dora-hub` repo or the author's own); dora maintains a typed manifest and a
commit pin in a catalog (`dora-hub/node-index`); `hub: name@version`
desugars to the `git:` source machinery dora already ships, plus contracts
and discovery on top.**

---

## 1. Motivation

Today a dataflow author who wants a camera driver either writes one or finds a
`dora-*` package on PyPI by word of mouth, then wires it up by copying a YAML
snippet from a README. There is:

- no way to **search** for existing nodes (`README.md` tables, maintained by hand),
- no **typed contract**: a node's inputs/outputs are prose in a README; wiring
  mistakes surface at runtime,
- no **versioned reference** in the dataflow (`build: pip install dora-yolo`
  floats to the latest release),
- no **reproducibility**: the same dataflow builds differently next month,
- no story for **Rust / C / C++ nodes** at all, and
- no **enterprise/offline** path.

At the same time, dora already has more of the solution in-tree than is
commonly known, and this spec deliberately builds on it instead of inventing
parallel machinery:

| Existing capability | Where | Role in Hub |
|---|---|---|
| Type URNs + registry (`std/core/v1/UInt64`, nested structs, user types, compat graph) | `libraries/core/src/types.rs`, `types/std/` | The contract language of the manifest |
| Descriptor type annotations + validation (`input_types`/`output_types`, `strict_types`) | `descriptor.rs`, `validate.rs` | Compose-time contract checking |
| Git node sources with commit pinning (`git:`/`branch:`/`tag:`/`rev:`) | `NodeSource::GitBranch`, `GitManager` | **What `hub:` desugars to** — the whole source-fetch path |
| URL binary fetch + sha256 (`path: <url>`) | `dora-download`, `source_is_url`/`resolve_path` | The opt-in binary form (§8.2) |
| Build lockfile (`<stem>.dora-lock.yaml`, `--locked`, `--write-lockfile`) | `binaries/cli/src/command/build/lockfile.rs` | Extended to pin hub packages |
| Multi-daemon build distribution (CLI resolves, coordinator routes, daemons build) | `build/distributed.rs`, `coordinator/handlers.rs`, daemon `build_dataflow` | Template for per-daemon hub fetch |
| Managed Python envs (`.dora/python-envs/<node>` via uv) | `build_command.rs` | Install target for Python packages |
| HTTP download + sha256 verification | `dora-download` (already a daemon dependency) | Base for release-asset/index fetching |
| ~60 published Python nodes with a working convention (PyPI dist name == console-script == `path:`) | `dora-rs/dora-hub` repo | The migration corpus and packaging convention |
| Per-user config dir + token file conventions | `libraries/message/src/auth.rs` | Hub config location |

## 2. Goals and non-goals

### Goals

1. **Publish**: a node author can publish a typed, versioned node with one
   command — an index entry pinning their source to a git commit (no upload;
   the source already lives in a git repo).
2. **Discover**: `dora hub search camera` works offline-cheap and needs no
   hosted service.
3. **Use**: `hub: dora-yolo@^0.5` in `dataflow.yml` fetches, builds, and
   runs — on a laptop or across a multi-daemon cluster.
4. **Validate**: input/output contracts are declared in the manifest and
   checked when a dataflow is composed (`dora validate` / `dora build`),
   before anything runs.
5. **Reproduce**: the lockfile pins the **source commit** (or, for the binary
   form, the artifact `sha256`), so `--locked` rebuilds from byte-identical
   source. The transitive dependency closure is pinned only as tightly as the
   node's own `build:` pins it — the same reproducibility model `git:` nodes
   have today (§10.3), not a Hub-specific closure lock.
6. **Enterprise & offline**: a private index is just a private git repo; an
   offline robot runs from a pre-populated, hash-verified cache.
7. **Migrate**: the existing ~60 PyPI nodes become hub packages by adding
   manifests — no re-publishing.

### Non-goals (for this spec; some are explicitly later phases)

- **Hosted registry service** (Postgres/S3/web dashboard/API tokens). The
  index is a git repo; everything a hosted service would add (search API,
  download stats, web UI) is Phase 4 and optional — built *on top of* the
  same index data, never replacing it.
- **A dora-specific archive format.** Wheels, crates, OCI artifacts, and
  checksummed release assets already exist; dora pins them, it does not host
  them.
- **Runtime QoS / shape enforcement.** The manifest reuses the existing type
  system; deepening *enforcement* (per-message validation, tensor shapes,
  metadata contracts at runtime) is a parallel type-system workstream
  ([§6.4](#64-known-type-system-gaps-out-of-scope-here)), not a Hub
  deliverable.
- **Dataflow orchestration changes, visual editors, in-browser testing,
  license enforcement** (unchanged from the original proposal's non-goals).

## 3. Use cases

Each use case is an acceptance test for a roadmap phase (noted in brackets).

**UC1 — Find and use a node** *(Phase 2)*
Ana builds a perception pipeline. `dora hub search detection` lists
`dora-yolo 0.5.2 — object detection (ml-inference)` with its license and
platforms. `dora hub info dora-yolo` prints the typed
inputs/outputs and an example snippet. She adds to her `dataflow.yml`:

```yaml
- id: detector
  hub: dora-yolo@^0.5      # bare name == official `dora-rs/` namespace (§7.2)
  inputs:
    image: camera/image
  outputs:
    - bbox
```

`dora build dataflow.yml --uv` resolves `^0.5` against the index to a pinned
commit, clones that commit, runs the node's `build:` (for a Python node,
`pip install .` into the managed env — its wheel deps come from PyPI as
always), and writes the pin to `dataflow.dora-lock.yaml`. No `pip install`
line in the dataflow, no PATH knowledge.

**UC2 — Contract mismatch caught at compose time** *(Phase 1+2)*
Ana wires `detector/bbox` (declared `std/vision/v1/BBox2D` in the manifest)
into a node expecting `std/media/v1/Image`. `dora validate` (and `dora build`)
fails with the producer/consumer URNs and the field-level diff — before any
process starts. This works because manifest contracts surface as the existing
`input_types`/`output_types` annotations.

**UC3 — Publish a Python node** *(Phase 3)*
Ben has a LiDAR driver in his own git repo. `dora hub init` scaffolds
`dora-node.yml` next to his `pyproject.toml`; he fills in outputs and types
(`build: pip install .`). `dora hub publish --dry-run` validates the manifest,
checks the entry point, and verifies contracts. `dora hub publish` then
resolves his tag to a commit and opens a PR adding
`ben-robotics/dora-lidar@1.0.0` to the index, with `source` pointing at his
repo + that commit. Index CI validates the entry and the bot auto-merges
(§7.5) — discoverable minutes later, no human in the loop for routine version
adds, and no PyPI publish required (his repo *is* the source-of-record).

**UC4 — Publish a Rust node** *(Phase 2/3)*
Same flow; the `source` stanza points at the node's git repo + commit (its
`build:` is `cargo build --release`). For a node too heavy to compile on a
robot, the publisher may additionally attach prebuilt per-platform binaries
(§8.2) with a `fallback-git` so uncovered arches still compile from source.

**UC5 — Reproducible team builds** *(Phase 2)*
Carol commits `dataflow.yml` + `dataflow.dora-lock.yaml`. CI runs
`dora build --locked`: resolution is skipped, each node is rebuilt from its
**pinned commit** (binary form: the pinned `sha256` is verified), and a moved
tag or a new release cannot alter what source is built. This is the same
guarantee the existing v2 lockfile already gives `git:` sources — extended to
carry the hub provenance. (Transitive-dep pinning is as tight as each node's
`build:` makes it, §10.3 — Hub doesn't add or subtract from that.)

**UC6 — Multi-machine deployment** *(Phase 2)*
Dataflow nodes deploy to three daemons on mixed architectures. The CLI
resolves `hub:` ranges to pins once (centrally, like git rev→commit today);
the coordinator routes per-daemon node sets; **each daemon clones the pinned
commit and builds its own nodes** for its own platform (§10.2 — exactly the
existing `git:` multi-daemon flow). For the opt-in binary form, the
per-platform artifact selects at fetch time.

**UC7 — Offline robot** *(Phase 2)*
A field robot has no internet. `dora hub fetch dataflow.yml --target-dir
./hub-cache` (run where there is connectivity) pre-clones each pinned commit
(and downloads any binary artifacts) named in the lockfile; on the robot,
`dora build --locked --offline` builds from the cached source (binary form:
re-verifies the `sha256`) and fails loudly on any miss. (The transitive
Python dep closure is *not* pre-fetched unless the node's `build:` /
`uv.lock` makes it so — same offline caveat `git:` + `pip install` nodes have
today; tightening that is the general build-reproducibility workstream, not
Hub.)

**UC8 — Enterprise private index** *(Phase 2)*
A company keeps proprietary nodes in `git@github.com:acme/dora-nodes.git`
(source) and a private index repo `git@github.com:acme/dora-index.git`.
`~/.config/dora/hub.toml` adds the index **bound to the `acme/` namespace**
(§7.3) — namespace↔index binding makes dependency-confusion attacks
structurally impossible. Descriptor entries written `hub: acme/lidar-fusion@2.1`
resolve only against the acme index, whose `source` points at the private
source repo. No dora-side server, SSO, or token system — git auth (the
company's existing credentials) gates both repos.

**UC9 — Migrating the existing node-hub** *(Phase 3)*
A script generates draft `dora-node.yml` manifests for the ~60 active nodes
in `dora-rs/dora-hub`; typing the contracts is farmed out as one
good-first-issue per node (§12). Index entries pin the wheels already on
PyPI. Nothing is re-published; `build: pip install dora-yolo` keeps working
forever, `hub: dora-yolo@^0.5` becomes the better way.

**UC10 — Yank a broken release** *(Phase 3)*
A published version corrupts data. The maintainer runs `dora hub yank`, which
opens the flag-flip PR; yank-only diffs from the namespace owner auto-merge
(§7.5), so the window is minutes, not review latency. Index admins can yank
over an unresponsive owner. New resolutions skip yanked versions; existing
lockfiles that pin one keep working but `dora build` prints a warning.
History stays in git.

**UC11 — Node author's inner loop** *(Phase 2)*
While developing `dora-yolo 0.6`, Ben tests his local checkout against a real
consumer dataflow without publishing anything:
`dora build consumer.yml --hub-override dora-rs/dora-yolo=../dora-yolo`
substitutes the local package (manifest read from the checkout, contracts
still validated, env built from local source). Hermetic end-to-end testing of
the resolve→fetch→install pipeline uses a `path = ./fixtures/index` index
alias in `hub.toml` — the same mechanism the integration tests use. A
`dora-node.yml` sitting next to a plain `path:` node is also picked up for
contract injection, so local development gets the same validation as hub
consumption.

## 4. Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│ dataflow.yml            hub: dora-yolo@^0.5                      │
│ dataflow.dora-lock.yaml pins: commit hash (git) / url+sha256     │
└──────────────┬───────────────────────────────────────────────────┘
               │ resolve via index (CLI)        clone+build (per daemon)
┌──────────────▼─────────────────────────────────────────────────┐
│ dora-hub  (one repo)                                            │
│   node-index/<ns>/<name>/<ver>.yml = manifest + source pointer  │
│   node-hub/<name>/                  = source of dora-rs's nodes │
│   (private/enterprise = a separate repo, namespace-bound §7.3)  │
└──────────────┬─────────────────────────────────────────────────┘
               │ source pointer → git repo+commit (or binary URL)
┌──────────────▼─────────────────────────────────────────────────┐
│ SOURCE: dora-hub/node-hub, OR the author's own git repo         │
│ (opt-in: a prebuilt binary URL for heavy nodes — §8.2)          │
└─────────────────────────────────────────────────────────────────┘
               ▲ publish = PR adding a node-index entry (bot-merged §7.5)
               │           — points at source wherever it is hosted
```

Four layers:

1. **Manifest** (`dora-node.yml`): the typed, dora-specific description of a
   node — contracts, entry point, env/config. Lives in the node's source,
   *copied into* the index entry at publish time so discovery never needs to
   fetch source.
2. **Index** (`dora-hub/node-index`): the catalog mapping
   `namespace/name@version` → manifest + a `source` pointer (§8.1), with
   machine-enforced append-only rules and bot-merged publishing (§7.5). The
   index never hosts bytes — it points at them. Living inside `dora-hub`
   lets a node's source and its index entry land in one PR. Auth, hosting,
   and review tooling are git's; private indexes are separate repos composed
   via namespace binding.
3. **Source**: the node's bytes, fetched and built through the **existing**
   `git:` source machinery (`GitManager` clone at a pinned commit + the
   node's `build:`), or downloaded via the **existing** `path: <url>` +
   `dora-download` path for the opt-in binary form. No new fetch stack.
4. **CLI + runtime** (`dora hub` + `hub:` in `dora build`/`run`/`validate`):
   the single user surface. A resolved `hub:` node *is* a git-sourced node,
   so it flows through the existing build/spawn/multi-daemon pipeline
   unchanged (§10).

## 5. The node manifest — `dora-node.yml`

YAML, not JSON: every other authored artifact in dora (dataflows, type
definitions) is YAML, and the manifest embeds type URNs and YAML snippets from
those files. (The original proposal's `node.json` maps field-for-field;
see Appendix A.)

The manifest holds only what native package manifests *cannot*: dora
contracts and dora wiring. Name, version, license, and dependencies are read
from `pyproject.toml`/`Cargo.toml` at publish time where possible and
mirrored into the index entry; the manifest may override but should not
duplicate.

```yaml
# dora-node.yml — lives next to pyproject.toml / Cargo.toml
apiVersion: 1
name: dora-yolo               # defaults: [project].name / [package].name
namespace: dora-rs            # index namespace (== a GitHub org/user, §7.4)
description: YOLO object detection on camera frames
categories: [ml-inference]    # from the fixed category list, §7.6
keywords: [vision, detection, yolo]

runtime: python               # python | rust | c | cpp
entrypoint: dora-yolo         # bare command name — no path separators (§11);
                              # resolved only inside the node's managed env /
                              # hub cache, then spawned like `path:` today
platforms: []                 # optional allowlist, e.g. [linux-x86_64,
                              # linux-aarch64, macos-aarch64]; empty = all.
                              # Surfaced in search/info; checked pre-dispatch.
dora: ">=0.4"                 # optional supported dora version range

inputs:
  image:
    type: std/media/v1/Image  # existing type URN system, §6
    required: true
    description: BGR frame to run detection on
outputs:
  bbox:
    type: std/vision/v1/BBox2D
    description: detected bounding boxes
  # nodes may have zero inputs (sources) or zero outputs (sinks)

env:                          # configuration surface, documented + typed.
  MODEL:                      # security-sensitive names (LD_PRELOAD, DYLD_*,
    default: yolov8n.pt       # PYTHONPATH, PATH) are rejected at validation.
    description: model weights file or hub id
  CONFIDENCE:
    type: float
    default: 0.4

types: []                     # optional: custom type defs shipped with the
                              # node, loaded into the TypeRegistry (§6.3)

requirements:                 # informational: surfaced by `hub info` and at
  hardware: []                # build start. v1 actively probes only `cuda`;
  system: {}                  # everything else is documentation, never
                              # auto-installed.

example: |                    # snippet shown by `dora hub info`
  - id: detector
    hub: dora-yolo@^0.5
    inputs:
      image: camera/image
    outputs:
      - bbox
```

Field rules:

| Field | Required | Notes |
|---|---|---|
| `apiVersion` | yes | manifest schema version, starts at 1 |
| `name`, `namespace` | yes (name defaultable) | index key is `namespace/name`; pypi names PEP 503-normalized |
| `runtime`, `entrypoint` | yes | entrypoint is a bare command name (validated) |
| `inputs`, `outputs` | yes, **may be empty** | sinks have no outputs; sources no inputs (fixes the original proposal's ≥1 requirement) |
| `inputs.*.type` etc. | recommended | type URNs; omitted = untyped (validation skips) |
| `platforms`, `dora` | recommended | platform facet for search + pre-dispatch check; dora compat range |
| `description`, `categories`, `keywords` | recommended | drive search |
| `env`, `types`, `requirements`, `example` | optional | |

The manifest schema is published as JSON Schema (like `dora-schema.json`) for
editor support, and `dora hub init` scaffolds it (extending the `dora new`
template machinery in `binaries/cli/src/template/`).

## 6. Typed contracts

### 6.1 Reuse, don't invent

Contracts use the **existing** type system: URNs
(`std/<category>/v1/<TypeName>[params]`) resolved by
`TypeRegistry` (`libraries/core/src/types.rs`), including nested struct
schemas, parameterized types (`std/media/v1/AudioFrame[sample_type=f32]`), the
widening compatibility graph, and user-defined types. The original proposal's
flat scalar `type: uint8` + separate `shape` field is dropped in favor of
URNs (Appendix A, D10/D11).

### 6.2 How manifest contracts reach validation

When `dora build`/`validate` resolves a `hub:` node, the manifest's
`inputs`/`outputs` types are injected as the node's `input_types` /
`output_types` descriptor annotations (unless the dataflow author overrides
them). From there the **existing** validation pipeline applies unchanged:
URN resolution with typo suggestions, producer/consumer edge compatibility,
struct field diffs, type inference for unannotated neighbors, and
`strict_types` escalation (`libraries/core/src/descriptor/validate.rs`,
`check_type_annotations_full`).

Ordering note for implementers: hub manifest resolution runs **after module
expansion and before type-checking** in `build()`/`validate()` — earlier than
git rev resolution, which today runs after the type-check block. `dora
validate` on a `hub:` dataflow therefore uses the cached index (and respects
`--offline`); it is no longer a fully offline command for hub dataflows.

The same injection applies to a plain `path:` node with a `dora-node.yml`
sitting next to it (UC11), so local development and hub consumption validate
identically.

This is the single highest-leverage integration in the spec: Hub gets
compose-time contract checking for free, and every improvement to the type
system automatically improves Hub nodes.

### 6.3 Custom types shipped with nodes

A node may ship type definitions (`types:` in the manifest, same YAML shape as
`types/std/*/v1.yml`). At resolution time these are materialized into the hub
cache and added to the `TypeRegistry` load path for that dataflow (today the
registry loads `<working_dir>/types`; one new search path). Namespace rule:
shipped types must live under the package's index namespace
(`acme/lidar/v1/PointCloud`), `std/` is rejected (already enforced by
`load_from_dir`).

### 6.4 Known type-system gaps (out of scope here)

Runtime enforcement is today scalar-only, first-message-only, warn-only, and
output-side unchecked; tensor shapes and runtime metadata contracts don't
exist. These gaps predate Hub, apply equally to hand-wired dataflows, and are
tracked as a parallel type-system workstream — Hub depends only on what
exists today.

## 7. The index

### 7.1 Format

The catalog is a directory **in the `dora-rs/dora-hub` repo** (`node-index/`),
not a separate repo — so a node's source (`node-hub/<name>/`) and its index
entry (`node-index/<ns>/<name>/<ver>.yml`) can land in one PR. One file per
published version:

```
dora-hub/
  node-hub/                # source of dora-rs's own nodes (as today)
    dora-yolo/  …
  node-index/              # the catalog (this section)
    dora-rs/
      dora-yolo/
        package.yml        # namespace-level: description, repo, OWNERS list
        0.5.1.yml          # manifest snapshot + source pointer (+ yanked flag)
        0.5.2.yml
    acme/
      lidar-fusion/
        2.1.0.yml          # entry points at acme's OWN repo (§8.1) — source
                           # need not live in dora-hub
```

A version file is the manifest (§5) plus a `source` stanza (§8.1). For the
default git form:

```yaml
# node-index/dora-rs/dora-yolo/0.5.2.yml
manifest: { ...dora-node.yml contents, verbatim... }
source:
  git: https://github.com/dora-rs/dora-hub   # or the author's own repo
  rev: 9f4c1ae…                              # commit hash (resolved from tag)
  subdir: node-hub/dora-yolo
published: 2026-07-01T12:00:00Z
yanked: false                                # UC10
```

The `source` is the single source-of-record for a version; the binary form
(§8.1) records per-platform `url` + `sha256` instead, and adding late
platform binaries to an existing version is an *additive append* PR — see
§7.5 for what the rules machine-enforce.

### 7.2 Names and resolution

`hub: [<namespace>/]<name>@<semver-req>`.

- **Bare names are official-namespace shorthand**: `hub: dora-yolo@^0.5` means
  exactly `dora-rs/dora-yolo@^0.5`. Deterministic — a third party publishing
  `their-ns/dora-yolo` can never change what a bare name resolves to. All
  non-official packages are always written namespaced.
- Resolution = pick the highest non-yanked version satisfying the requirement
  from the index that the namespace is bound to (§7.3). Resolved pins
  (including namespace and index) go to the lockfile; `--locked` skips
  resolution entirely.
- Full semver requirement syntax via the `semver` crate (already a
  `dora-message` dependency; promoted to a workspace dependency when hub code
  first uses it).

### 7.3 Multi-index configuration and namespace binding

`~/.config/dora/hub.toml` (the established `dirs::config_dir()/dora/`
location):

```toml
[[index]]
alias = "official"
git = "https://github.com/dora-rs/dora-hub"
path = "node-index"              # the catalog dir within the repo
# official carries every namespace not claimed by another index entry

[[index]]
alias = "acme"
git = "git@github.com:acme/dora-index.git"
namespaces = ["acme"]            # binding: acme/* resolves ONLY here

[[index]]
alias = "fixtures"               # UC11 / integration tests
path = "./tests/fixtures/index"
namespaces = ["test"]
```

**A namespace resolves against exactly one index.** Explicit `namespaces`
bindings win; the official index is the default for unbound namespaces; a
namespace bound by two index entries is a configuration error. This makes the
classic dependency-confusion attack (a public index entry shadowing or being
shadowed by a private package) structurally impossible — there is no "search
order" to race.

A missing config file means `official` only. Dataflows may additionally
declare required index bindings (so a repo is self-describing) under a
`hub_indexes:` dataflow key.

Index transport: blobless **sparse** clone into
`~/.cache/dora/index/<alias>/`, scoped to the `path` dir
(`git clone --filter=blob:none --sparse … && git sparse-checkout set
node-index`) so fetching the catalog stays cheap even though it lives in the
larger `dora-hub` repo — the clone pulls only the index tree, not the 60+
node source trees. Refreshed at most once per `dora build`/`hub` invocation;
`--offline` skips. Refresh requires **fast-forward**: the CLI records the
last-seen index commit and warns loudly on non-fast-forward or regression
(freeze/rollback protection — a stale or replayed index could otherwise hide
yanks; full TUF-style signing is out of scope for v1 and noted as residual
risk in §11).

### 7.4 Namespace governance

Namespaces are claimed by a PR adding `node-index/<ns>/` with a `package.yml`
listing the owner GitHub accounts. Machine-checked at PR time (index CI, not
reviewer eyeballs):

- the namespace **equals the claiming PR author's GitHub login, or an org
  they publicly belong to** (verified via the GitHub API);
- it is not on the reserved list (`dora`, `dora-rs`, `dora-hub`, `std`,
  `official`, `hub`, …);
- an edit-distance / confusable check against existing namespaces flags
  lookalikes (`d0ra-rs`, `acrne`) for mandatory human review.

New-namespace PRs always get a human reviewer; routine publishing within a
namespace does not (§7.5). Transfer and dispute policy (owner unresponsive
for 6 months, trademark conflicts) is a one-page document landed with the
index bootstrap (P2.3) — written before the first dispute, not during it.

### 7.5 Machine-enforced index rules and bot merging

Publish-via-PR only scales with automation. Index CI enforces, structurally
(not by convention):

1. **Append-only**: a PR may *add* new version files. The only permitted
   modifications to an existing version file are (a) flipping `yanked` (plus
   a reason), and (b) *adding* entries to `dist.artifacts` (late platform
   wheels). Any other diff to an existing version file fails CI and requires
   an index admin.
2. **Owner check**: the PR author must be in the target namespace's
   `package.yml` OWNERS list.
3. **Schema + pin validation**: entry validates against the schema; artifact
   URLs are reachable and their hashes match (spot-checked at PR time,
   re-verified by every installing client).

PRs passing all three **auto-merge via a bot** — publishing latency is CI
latency, minutes not review queues (the winget/conda-forge model; UC3).
Human review is reserved for: new namespaces (§7.4), OWNERS changes,
yank-by-non-owner, and changes to the index CI/bot itself. The same applies
to the `dora-rs/` namespace — its OWNERS are the dora maintainers.

### 7.6 Categories

Fixed enum, extended by index PR + discussion (start from the proposal's
seven plus what the existing node-hub README actually uses):
`sensor, actuator, robot, transform, filter, ml-inference, llm, speech,
communication, recorder, visualization, simulator, debug`.

## 8. Where node bytes live — source by default

Hub does not invent a distribution system. dora already fetches, pins, and
builds nodes from git, and downloads-and-runs binaries from a URL. Hub adds a
catalog and contracts on top; the bytes flow through the mechanisms that ship
today. **An index entry never hosts the bytes — it points at them.**

### 8.1 The source pointer

Every index version file carries a `source` stanza — the only thing the
resolver needs to obtain the node. Two forms, both backed by existing dora
machinery:

| Form | Points at | Pin | Fetch + build | Reuses |
|---|---|---|---|---|
| **git** (default) | a git repo + commit + subdir — the node's source-of-record (in `dora-rs/dora-hub` itself for official nodes, or the author's own repo) | repo URL + **commit hash** | clone at the pinned commit, run the manifest's `build:`, resolve the entrypoint | `NodeSource::GitBranch` → `ResolvedNodeSource::GitCommit`, `GitManager`, the v2 lockfile's commit pinning |
| **binary** (opt-in, heavy nodes) | a prebuilt artifact URL (GitHub Release asset, etc.) per platform | `url` + `sha256` | HTTP download, hash-verify, run | the existing `path: <url>` + `dora-download` path — plus one small addition: threading the pinned `sha256` into `download_file`'s already-present `expected_sha256` (today the URL `path:` carries no checksum; §10.1) |

```yaml
# index/dora-rs/dora-yolo/0.5.2.yml — git source (the normal case)
manifest: { ...dora-node.yml contents... }
source:
  git: https://github.com/dora-rs/dora-hub
  rev: 9f4c1ae…                       # commit hash (resolved from a tag at publish)
  subdir: node-hub/dora-yolo          # where in the repo the node lives
published: 2026-07-01T12:00:00Z
yanked: false
```

```yaml
# opt-in binary form, for a node too heavy to compile on a robot
source:
  binary:
    - platform: linux-aarch64
      url: https://github.com/acme/dora-lidar/releases/download/v2.1.0/dora-lidar-linux-aarch64
      sha256: "ab12…"
    - platform: linux-x86_64
      url: https://github.com/acme/dora-lidar/releases/download/v2.1.0/dora-lidar-linux-x86_64
      sha256: "cd34…"
  fallback-git:                       # optional: source build for uncovered arches
    git: https://github.com/acme/dora-lidar
    rev: 7d80c…
    subdir: .
```

The dropped backends from the prior draft (`pypi`, `crates-io`, `oci`,
`github-release` as distinct registry integrations, the `HubBackend` trait
with N implementations) are gone. Two reasons: dora already has both paths
(git source + URL binary), and source-by-git covers every language uniformly.
A Python node is just a git source whose `build:` is `pip install .` (or `uv
pip install`); its heavy binary deps (torch, opencv) still come as wheels from
PyPI at build time — that is the language's job, not Hub's, and is unchanged
from how the ~60 existing nodes already work. A Rust node is a git source
whose `build:` is `cargo build --release`. C/C++ is a git source with a cmake
`build:`. One mechanism, every language.

### 8.2 Binary is opt-in and stays graceful

Source-by-git is the default and the source-of-record. The `binary` form
exists only for nodes where compile cost on the target is genuinely
prohibitive (the ROS2 bridge is the canonical case). It is layered, not
exclusive (the cargo-binstall model): a `binary` entry **should** carry a
`fallback-git` so a platform with no prebuilt artifact degrades to a source
build instead of a hard "unsupported platform" wall. Partial arch coverage is
therefore safe — you ship the binaries you have, everything else compiles.

Producing per-arch binaries is the publisher's job (their CI), never a dora
build farm. dora already maintains an 8-target cross-compile matrix for its
own CLI; a reusable `dora-rs/build-node` GitHub Action can template that so
authors get the matrix without standing up their own — but that Action is a
Phase-4 convenience, out of v1 scope. **v1 Hub publishes git sources only;**
the `binary` form is reserved in the schema so it can be added without a
format change.

### 8.3 Build location — the one real embedded decision

With git sources, *something compiles*. dora's existing multi-daemon flow has
each daemon build its own nodes (§10.2) — fine on a laptop, slow-to-impossible
on a constrained daemon (a Jetson compiling a heavy node can thermal-throttle
or OOM). This is a deployment decision, separate from the distribution format:

- **Compile-on-target** (simplest; what `git:` does today): the daemon
  compiles. Acceptable for capable machines and light nodes.
- **Build-on-a-host-and-ship** (recommended for embedded): build for the
  target arch on a capable machine and deploy the result via the existing
  `dora hub fetch` / offline cache flow (UC7), so the robot never compiles.

Hub does not force either; it inherits whatever the build/deploy pipeline
already does for `git:` nodes. The guidance — *don't run heavy builds on
constrained daemons* — belongs in the guide chapter (P1.5).

### 8.4 Cache and integrity

`<base_working_dir>/hub/<ns>/<name>/<commit-or-version>/` (per-workspace) and
`~/.cache/dora/hub/` (user-level, shared), keyed by the pinned commit (git
form) or `version` (binary form). git sources reuse `GitManager`'s existing
commit-addressed clone cache (the commit hash *is* the integrity guarantee).
Binary artifacts are content-addressed and **re-verified against the lockfile
`sha256` on download and cache reuse** — which is exactly the checksum-
threading addition from §10.1 (`download_file`'s `expected_sha256`), without
which a `path: <url>` desugar would silently drop the hash. Offline transfer
(UC7) ships the cloned source / downloaded binaries + the lockfile, re-verified
on the target.

## 9. CLI surface

New nested subcommand (`binaries/cli/src/command/hub/`, following the
`cluster` module pattern):

```
dora hub search <query> [--category C] [--platform P]   # search index manifests
dora hub info <pkg>[@<ver>]                             # manifest, contracts, example
dora hub list [--dataflow F]                            # hub packages in lockfile/cache
dora hub init [--manifest-only]                         # scaffold dora-node.yml
dora hub publish [--dry-run] [--index A]                # §9.1 (open index PR)
dora hub yank <pkg>@<ver> [--undo]                      # automates the flag-flip PR
dora hub fetch <dataflow.yml | pkg@ver> [--target-dir D]  # pre-populate cache (UC7)
dora hub outdated [--dataflow F]                        # lockfile pins vs index
dora hub update [<pkg>] [--dataflow F]                  # re-resolve + rewrite lockfile
```

`login`/`logout` from the original proposal are deliberately absent (no
hosted service; git carries its own auth). There is
also no `dora hub install` into a global store — packages are resolved
per-dataflow by `dora build`, cargo-style. Because ~60 nodes' worth of users
have `pip install dora-yolo` muscle memory, `dora hub install` ships as an
**error stub** that explains the per-dataflow model and points at
`hub:` + `dora build` / `dora hub fetch`.

Resolution lives in `dora build` / `dora run` / `dora validate` (`dora run`
builds internally, so it inherits hub support; `dora start` requires a prior
`dora build`, exactly as git sources do today):

```
dora build dataflow.yml                 # resolve ranges, fetch, build, write lockfile
dora build dataflow.yml --locked        # verify pins, no resolution (CI)
dora build dataflow.yml --offline       # cache only, fail loudly on miss
dora build dataflow.yml \
  --hub-override dora-rs/dora-yolo=../dora-yolo    # UC11 local substitution
```

### 9.1 Publish flow

Publishing is just *adding an index entry that points at your source* — there
is no separate "upload the bytes" step, because the bytes already live in a
git repo (yours or `dora-hub`):

```
dora hub publish
  1. Validate: manifest schema, entrypoint, type URNs resolve, contract
     sanity, env-var deny-list (dry-run stops here)
  2. Resolve the source pin: the tag/branch you name → a commit hash, against
     the git repo the manifest points at (defaults to the repo you publish
     from; `dora-rs/dora-hub` for official nodes). For the opt-in binary
     form, capture per-platform url + sha256 from the release assets.
  3. Open the index PR adding node-index/<ns>/<name>/<ver>.yml (via `gh` if
     available, else print the entry + path); bot-merged when CI passes
     (§7.5). For a dora-rs node, the same PR can add the source under
     node-hub/ and the index entry together.
```

No native-registry publish, no upload, no build farm. The publisher's only
obligation is that the named commit stays reachable in the named repo — which
git already guarantees once the index pins the hash.

## 10. Descriptor and runtime integration

### 10.1 Descriptor

```yaml
nodes:
  - id: detector
    hub: dora-yolo@^0.5        # NEW field; mutually exclusive with path/git/build
    inputs:
      image: camera/image
    outputs:
      - bbox
```

- New `hub: Option<String>` on the raw `Node`
  (`libraries/message/src/descriptor.rs`, sibling of `git:`/`path:`).
- **The CLI desugars `hub:` to a concrete git node *before dispatch*.** This
  is the load-bearing mechanism, and it matters because the existing
  `git_sources` wire state is only `{ repo, commit_hash }` — it carries no
  subdir and no entrypoint, and the builder uses the clone root as the git
  node's working dir. So the index entry's `subdir` and the manifest's
  `entrypoint`/`build` cannot ride the existing protocol as-is. Instead, at
  resolve time the CLI rewrites a `hub:` node into an ordinary git-sourced
  `Node`:

  | hub entry → | synthesized git node field |
  |---|---|
  | `source.git` + resolved `commit` | `git:` + pinned rev (→ `GitManager` clone) |
  | `source.subdir` + manifest `entrypoint` | `path: <subdir>/<entrypoint>` (relative to the clone root — `resolve_path` already joins working_dir + path, so a subdir path resolves with no new field) |
  | manifest `build` | `build:` |
  | manifest `inputs/outputs` types | `input_types`/`output_types` (§6.2) |

  After this rewrite the coordinator and daemon see a **normal git node** —
  the `{repo, commit_hash}` wire state and `resolve_path` are unchanged. The
  new code is one CLI-side desugaring step (the CLI already resolves git
  rev→commit and expands modules centrally; this is the same stage), not a
  protocol change or a bespoke daemon spawner. The lockfile additionally
  records the hub provenance (name@version) over the pinned commit so
  `dora hub` commands and `--locked` drift detection work (§10.3).
- The binary form (§8.1) desugars to a `path: <url>` node — but the existing
  URL `path:` carries no checksum, so this needs **one small addition**: the
  pinned `sha256` must travel to the spawn site and be passed to
  `download_file`'s already-present `expected_sha256` parameter (today the
  call sites pass `None`). That is the only new plumbing the binary form
  requires; it is deferred with the binary form itself (P2.8).
- `path:`/`git:`/`build:` are forbidden together with `hub:` (the index
  entry supplies the source and build); `inputs`/`outputs` in the dataflow
  must be a subset of the manifest's declared ports (validated).
- `input_types`/`output_types` are injected from the manifest (§6.2).
- The parser accepts `hub:` under its final name from day one, gated by an
  "unstable feature" warning until Phase 2 exits — *not* an
  `_unstable_hub:` key that would leak into 60+ index `example:` blocks and
  every tutorial, then need a rename.

### 10.2 Multi-daemon flow (it *is* the git-source flow)

Because the CLI rewrites `hub:` into a concrete git node before dispatch
(§10.1), the multi-daemon path is the one dora already runs for `git:` nodes:

1. **CLI (central):** resolve each `hub:` range against the index → a pinned
   commit, then **synthesize the git node** (git/rev/path/build, §10.1).
   Write/verify the lockfile.
2. **Coordinator:** the synthesized node travels in the existing
   `git_sources` map of `BuildDataflowNodes` — it *is* a git node by the time
   the coordinator sees it, so no new wire type. One additive change: a
   daemon capability flag so the coordinator can give a clear "upgrade this
   daemon" error if a daemon predates `hub:` support (it would otherwise
   never receive a `hub:` node, since desugaring is central — the flag is
   belt-and-suspenders for mixed-version clusters).
3. **Daemon:** clones the pinned commit, runs the node's `build:` for its own
   platform, records working dir + env in `BuildInfo`, and spawns via
   `resolve_path` against the synthesized `path` (`<subdir>/<entrypoint>`) —
   *byte-for-byte the existing git-node path*. The binary form instead
   downloads + `sha256`-verifies the per-platform URL via `dora-download`
   (the new checksum-threading from §10.1); everything downstream is
   identical.

A daemon whose platform has no prebuilt binary and no `fallback-git` (§8.2)
fails with a clear per-node, per-daemon error. With `fallback-git` it compiles
from source instead.

### 10.3 Lockfile

`BuildLockfile` extends its existing `git_sources` to record the hub
provenance alongside the pinned commit — minimal additions, because a hub
node *is* a git node once resolved:

```yaml
hub_sources:                         # provenance layer over git_sources
  detector:
    name: dora-rs/dora-yolo          # index key (namespace/name)
    version: 0.5.2                    # resolved from the @^0.5 range
    source:
      git: https://github.com/dora-rs/dora-hub
      rev: 9f4c1ae…                   # the pinned commit (the actual lock)
      subdir: node-hub/dora-yolo
    # binary form instead records: url + sha256 per platform
```

- `--locked`: fingerprint match + the pinned commit (or binary `sha256`) is
  used verbatim, no index resolution. Identical guarantee to today's
  `git:` + lockfile reproducibility.
- `--offline`: no network — the cache must already hold the cloned commit /
  downloaded binary (populated by `dora hub fetch`, UC7).
- **Dependency-closure pinning is the same gap `git:` nodes have today** and
  is *not* a Hub deliverable: a Python node's transitive wheels (numpy,
  torch) are pinned only as tightly as its own `build:` pins them (a
  committed `uv.lock` / `requirements.txt` with hashes). Hub doesn't make
  this worse and doesn't try to solve it; tightening it (optional `uv
  export` capture) is a general build-reproducibility improvement that
  benefits `git:` and `hub:` nodes equally, tracked outside this spec.

## 11. Security model

| Property | Mechanism |
|---|---|
| Version immutability | **machine-enforced** append-only index CI (§7.5): existing version files accept only `yanked` flips and artifact *additions*; anything else needs an index admin. Git history audits everything |
| Source/artifact integrity | the source is pinned to a **commit hash** in the index entry + lockfile (git's own content-addressing); the opt-in binary form pins `sha256`, re-verified on every run (§8.4). Only the pinned commit/binary is used |
| Build-time code execution | installing a node runs its `build:` (cargo build scripts, pip install) — arbitrary code at build time, identical to a `git:` node today. `--locked` binds it to the reviewed, commit-pinned source. Heavy binary deps (torch) come as wheels from PyPI, the language ecosystem's trust boundary, unchanged |
| Malicious manifest fields | `entrypoint` restricted to bare command names resolved inside the managed env/cache (no absolute paths, no traversal — the executed binary cannot escape the pinned artifact); `env:` deny-list for loader-hijack variables (LD_PRELOAD, DYLD_*, PYTHONPATH, PATH) |
| Index integrity | git over HTTPS/SSH; protected branch; bot merge only on rule-conformant diffs; fast-forward-only refresh with rollback warning (§7.3) |
| Namespace integrity | CI-verified GitHub identity match, reserved list, confusable-name screening, human review for new namespaces (§7.4) |
| Publisher auth | delegated to git: the source lives in a git repo the author controls (or `dora-hub`), index PRs are GitHub-authenticated — dora stores no credentials. The dora-rs org **requires 2FA** for `dora-hub` write access (= index/source committers) |
| Yank | owner yank-flip PRs auto-merge (minutes, UC10); index admins can yank over an unresponsive owner; mass-yanks from one namespace trigger second review; never deletes; lockfiles keep working |

**Residual risks, stated plainly:** (1) a compromised namespace-owner GitHub
account can publish malicious *new* versions until noticed — pins and
`--locked` protect existing consumers; detection relies on the community and
yank. This is the same trust level as PyPI/crates.io account compromise.
(2) Without index signing (TUF-style), a compromised git host could serve a
frozen index state within the fast-forward constraint; signing is a Phase 4
candidate. (3) installing a node runs its `build:` (cargo build scripts, pip
install) — arbitrary code at build time, exactly as a `git:` node does today;
`--locked` bounds it to the reviewed, commit-pinned source.

## 12. Migration of the existing node-hub

Scope: the **62 active** nodes in `dora-rs/dora-hub/node-hub/`; the 8
`node-archive/` entries are not migrated.

1. Script (`scripts/hub/gen-manifests.py`, lives in `dora-rs/dora-hub`)
   drafts `dora-node.yml` per node from `pyproject.toml` (name, entrypoint
   from `[project.scripts]`, description) — and **audits std/ URN coverage**,
   emitting a gap list of message shapes that have no std type yet (this
   feeds the type-system workstream, §6.4).
2. Contract typing is **62 labeled good-first-issues** (one node, one PR) —
   the project's best community-onboarding surface — with a named
   coordinator *(seeking owner)*. Launch gate for P3.5: the 20
   most-downloaded nodes fully typed; the rest may publish untyped (their
   manifests still carry entrypoint/description/category, so search and
   install work; `hub info` marks contracts as not yet declared).
3. Bulk index PR adds a `node-index/` entry per node, `source` pointing at the
   **`dora-hub` repo itself** (`subdir: node-hub/<name>`) at the commit of
   each node's current release tag. The source already lives in the repo, so
   the entry and (for future versions) the source can co-evolve in one place.
   No re-publishing, no version bumps — just a pin.
4. `dora-hub` repo CI gains a `dora hub publish` step keyed to its existing
   release process (per-node version detection from the release diff —
   specced in the P3.3 issue, since dora-hub is a monorepo with a single
   release event today).
5. The README package table becomes generated output from the index (badge
   links unchanged).
6. `build: pip install dora-X` keeps working indefinitely — `hub:` is sugar
   plus contracts plus a commit pin, not a breaking change. (PyPI publishing
   of these packages can continue independently; Hub doesn't require or
   replace it — it pins the source.)

## 13. Web & hosted layer (Phase 4, optional)

A static site generated from the index (search via client-side index, node
pages from manifests, README rendering from the source repos) gives the
original proposal's §XIII web platform without servers — and is the natural
first-refusal item for the original proposer *(seeking owner)*. Download
counts come from pypistats / crates.io / ghcr APIs. Only if scale ever
demands it does a hosted search API or richer dashboard get built — on top
of the same index data, so nothing in Phases 1–3 is throwaway.

## 14. Roadmap

Phases gate on each other; items within a phase are mostly parallel,
deliberately sized as **one small PR each** unless marked *(workstream)*.
Tracking: umbrella issue #2097 holds the live checklist; each item becomes
its own issue when its phase opens. Items are unowned until claimed —
*(seeking owner)* applies to all of them.

### Phase 0 — Spec (this document)

- **P0.1** Land this spec via PR; 14-day review window on discussion #2095.

### Phase 1 — Manifest & contracts (no network, no index; immediately useful)

- **P1.1** `dora-node-manifest` parsing + JSON Schema in `dora-core`
  (serde types, schema generation alongside `dora-schema.json`).
- **P1.2** `dora hub init` scaffolding + `dora new` integration. *Depends: P1.1.*
- **P1.3** Manifest validation: entrypoint bare-name rule, env deny-list,
  type-URN resolution against `TypeRegistry`, port sanity. Core of
  `dora hub publish --dry-run` and `dora validate --node-manifest <file>`.
  *Depends: P1.1.*
- **P1.4** Manifest→descriptor contract injection, including for `path:`
  nodes with an adjacent `dora-node.yml` (§6.2; testable without any index).
  *Depends: P1.1.*
- **P1.5** Guide chapter: writing a node manifest. *Depends: P1.1–P1.3.*

### Phase 2 — Index & consumption (the `hub:` field works end-to-end)

- **P2.1** Index format + resolver crate (`dora-hub-client`): parse index
  trees, semver resolution, yank handling, namespace binding (pure functions
  + fixtures). *Depends: P1.1.*
- **P2.2** Index transport & cache: clone/refresh (fast-forward check),
  `path =` local indexes, `hub.toml`, `--offline`. *Depends: P2.1.*
- **P2.3** Bootstrap the `node-index/` catalog **in `dora-rs/dora-hub`**:
  directory layout, path-scoped index CI (append-only enforcement,
  schema/pin validation, namespace identity + confusable checks) that does
  *not* gate the human-reviewed `node-hub/` source path, auto-merge bot
  scoped to `node-index/**`, namespace policy page, PR templates.
  *Depends: P2.1 (format). Larger than one PR for the policy/bot pieces —
  tracked as 2–3 PRs in its issue.*
- **P2.4** `dora hub search` / `info` / `list`. *Depends: P2.2, P2.3.*
- **P2.5** Descriptor: `hub:` field (unstable-warning gate), `NodeSource::Hub`
  that **desugars to `ResolvedNodeSource::GitCommit`** (§10.1), hub arm in
  `Node::kind()`/`node_kind_mut()`, port-subset validation. Smaller than the
  prior `HubSource` design — it threads a git commit, not a new fetch type.
  *Depends: P1.4.*
- **P2.6** Lockfile: extend `git_sources` with hub provenance (name/version
  over the pinned commit), fingerprint extension, `--locked`/`--offline`
  semantics. No new closure-capture machinery — same reproducibility model
  as `git:` nodes today (§10.3). *Depends: P2.1, P2.5.*
- **P2.7** Source fetch+build wiring: a resolved `hub:` git source flows
  through the existing `GitManager` clone + `build:` + `resolve_path` spawn
  (the bulk is *reuse*, not new code). *Depends: P2.2, P2.5, P2.6.*
- **P2.8** Binary form (§8.1/§8.2): per-platform `url`+`sha256` via the
  existing `path: <url>` + `dora-download` path, with `fallback-git`
  degrade-to-source. *Depends: P2.7. (Deferred unless a heavy node needs it
  in v1 — git source covers all current nodes.)*
- **P2.10** Multi-daemon: a resolved hub node *is* a git node, so it rides
  the existing `git_sources` flow (§10.2). The only new work is the daemon
  capability flag + clear too-old-daemon error. *Depends: P2.7.*
- **P2.11** `dora hub fetch` (pre-clone/pre-download, UC7), `--hub-override`
  (UC11), smoke tests in `tests/example-smoke.rs` against a **local
  fixture index** (no dependency on the live catalog), and an
  `examples/hub-dataflow`. *Depends: P2.7.*
- **P2.12** Custom-type shipping (§6.3): registry load path from hub cache.
  *Depends: P2.7; parallel-safe.*

### Phase 3 — Publishing & migration (ecosystem goes live)

- **P3.1** `dora hub publish` (validate → resolve tag→commit pin → open the
  node-index PR; `--index`). *Depends: P1.3, P2.3.*
- **P3.2** `dora hub yank` / `outdated` / `update`. *Depends: P2.6, P3.1.*
- **P3.3** node-hub migration *(workstream, not a PR)*: manifest generator +
  URN gap audit, 62 per-node typing issues, bulk index PR, dora-hub release
  CI integration (UC9, §12). *Depends: P3.1.*
- **P3.4** node-index CI hardening: pin reachability re-checks, integrity
  spot-audit, confusable screening tuning. *Depends: P2.3.*
- **P3.5** Stabilize: remove the unstable-feature warning from `hub:`, guide
  "publishing" chapter, announce. Gate: top-20 migrated nodes fully typed
  (§12). *Depends: everything above + a soak period.*

### Phase 4 — Discovery & hardening (optional, demand-driven)

- **P4.1** Static site from the index *(seeking owner — first refusal to the
  original proposer)*. **P4.2** download-stats aggregation.
- **P4.3** index signing / provenance (TUF-lite, sigstore fields).
- **P4.4** hosted search API *only if* the static index stops scaling.

### Dependency sketch

```
P0.1 ─► P1.1 ─► {P1.2, P1.3, P1.4, P1.5}
        P1.1 ─► P2.1 ─► {P2.2, P2.3} ─► P2.4
        P1.4 ─► P2.5 ─► P2.6 ──┐
                 P2.2 ─────────┼─► P2.7 ─► {P2.8 (opt), P2.10,
                               │           P2.11, P2.12}
        {P1.3, P2.3} ─► P3.1 ─► {P3.2, P3.3}
        P2.3 ─► P3.4            all ─► P3.5 ─► P4.*
```

## 15. Open questions

Defaults are stated; silence during the 14-day review window adopts the
default.

1. **Manifest filename**: `dora-node.yml` vs embedding under a `[tool.dora]`
   table in `pyproject.toml` / `Cargo.toml` metadata. *Default: separate
   file — uniform across languages, C/C++ has no native manifest.*
2. **Where hub network code lives**: new `dora-hub-client` crate under
   `libraries/` vs inside `dora-cli`. *Default: separate crate — the daemon
   needs the fetch path (§10.2) and the CLI stays a thin shell.*
3. **Binary form in v1?** Ship the opt-in binary `source` (§8.1/§8.2) now, or
   defer until a heavy node needs it. *Default: defer — git source covers
   every current node; the schema reserves the `binary` form so adding it is
   not a format change.*
4. **Dependency-closure capture for non-uv Python** (plain pip): support or
   require uv for hub nodes? *Default: hub Python nodes require `--uv`
   (managed envs are uv-based already); plain-pip support can follow.*
5. **Subgraph/module packages**: dora has reusable subgraphs ("modules");
   should the index host those too? *Default: out of v1; manifest
   `apiVersion` leaves room.*
6. **Review window length & launch gates** (§12's top-20 typed rule, the
   14-day window): maintainer judgment calls — adjust freely in the spec PR.

---

## Appendix A — Delta vs the original proposal

**Adopted unchanged** (the majority): semver + yank semantics (D6/D7),
deprecation fields (D8), explicit transform nodes over implicit conversion
(D12), category enum + keywords (D13), independent forks (D14), hardware
requirements declared by publisher / supplied by deployer (D15), per-node
examples (D16), fail-fast system deps with no auto-install (D19), search
ranking by downloads first (D20), quality signals without subjective scoring
(D29), and the Hub/Runtime separation that frames the whole design.

Rows that change, and why:

| # | Original decision | This spec | Why |
|---|---|---|---|
| D2 | Hosted multi-registry (FQDN), Postgres+S3 | Git-backed multi-index with namespace binding | No 24/7 service to fund/secure; enterprises reuse git+registry auth (UC8); namespace binding kills dependency confusion; hosted layer deferred to demand (Phase 4) |
| D3/D5 | GitHub OAuth + token dashboard | None (delegated auth) | No service → no tokens; PyPI/crates.io/gh credentials already exist |
| D9 | inputs and outputs both required ≥1 | May be empty | Sinks/sources exist |
| D10/D11 | Flat Arrow scalar + `shape` + wildcard matching | Existing type-URN system | dora already ships URNs, nested schemas, params, compat graph, and validation — reuse beats parallel invention |
| D17 | `dora hub test` 4-step incl. runtime smoke | `publish --dry-run` = static validation; local runtime testing via `--hub-override` (UC11) | Runtime smoke harness is separable; the override gives authors a real pre-publish runtime path; revisit a packaged smoke harness in Phase 3 |
| D18 | Metadata+source (build-on-install) for Py/Rust; 3-tier C/C++ | **Source-by-git for every language** (commit-pinned), reusing dora's existing `git:` machinery; opt-in prebuilt binary form for heavy nodes | Source is uniform, portable across the heterogeneous/embedded targets dora runs on, and already built; no per-ecosystem backend stack to write or host; binary is a per-platform fast-path with a source fallback |
| D22–25 | Runtime-layer venv/conflict/offline items | Managed uv envs already exist; offline via closure-complete cache + lockfile | Most of it shipped in dora already (`--uv` envs) |
| D26 | Self-hosted registry binary | Private git index, namespace-bound | Strictly less to operate |
| D28 | Official = no prefix; promotion strips prefix | Bare name = `dora-rs/` shorthand; everything else always namespaced | Keeps the original's ergonomic instinct without renames-on-promotion breaking references |
| §7.1 | Dual publish path (CLI + tag webhook → Hub CI) | Single path: native publish + bot-merged index PR | One source of truth; no CI farm running untrusted builds; bot merging keeps publish latency in minutes |
| §IX | REST API v1 | None in v1 | The index *is* the API; resolution is local |
