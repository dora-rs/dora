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

The architecture in one sentence: **artifact bytes live on each ecosystem's
native registry (PyPI, crates.io, OCI/GitHub Releases); dora maintains a typed
manifest and an immutable pin in a git-backed index; one CLI surface
(`dora hub`) and one descriptor field (`hub:`) hide the difference.**

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
| Git node sources with commit pinning (`git:`/`branch:`/`tag:`/`rev:`) | `NodeSource::GitBranch`, `GitManager` | Template for the `hub:` source; escape-hatch backend |
| Build lockfile (`<stem>.dora-lock.yaml`, `--locked`, `--write-lockfile`) | `binaries/cli/src/command/build/lockfile.rs` | Extended to pin hub packages |
| Multi-daemon build distribution (CLI resolves, coordinator routes, daemons build) | `build/distributed.rs`, `coordinator/handlers.rs`, daemon `build_dataflow` | Template for per-daemon hub fetch |
| Managed Python envs (`.dora/python-envs/<node>` via uv) | `build_command.rs` | Install target for Python packages |
| HTTP download + sha256 verification | `dora-download` (already a daemon dependency) | Base for release-asset/index fetching |
| ~60 published Python nodes with a working convention (PyPI dist name == console-script == `path:`) | `dora-rs/dora-hub` repo | The migration corpus and packaging convention |
| Per-user config dir + token file conventions | `libraries/message/src/auth.rs` | Hub config location |

## 2. Goals and non-goals

### Goals

1. **Publish**: a node author can publish a typed, versioned node with one
   command, reusing their language's native packaging (wheel / crate / OCI
   artifact / release asset).
2. **Discover**: `dora hub search camera` works offline-cheap and needs no
   hosted service.
3. **Use**: `hub: dora-yolo@^0.5` in `dataflow.yml` fetches, builds, and
   runs — on a laptop or across a multi-daemon cluster.
4. **Validate**: input/output contracts are declared in the manifest and
   checked when a dataflow is composed (`dora validate` / `dora build`),
   before anything runs.
5. **Reproduce**: the lockfile pins exact artifacts — the node artifact *and
   its resolved dependency closure* (§10.3) — so `--locked` builds consume
   byte-identical inputs.
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
`dora-yolo 0.5.2 — object detection (ml-inference)` with its license,
platforms, and backend. `dora hub info dora-yolo` prints the typed
inputs/outputs and an example snippet. She adds to her `dataflow.yml`:

```yaml
- id: detector
  hub: dora-yolo@^0.5      # bare name == official `dora-rs/` namespace (§7.2)
  inputs:
    image: camera/image
  outputs:
    - bbox
```

`dora build dataflow.yml --uv` resolves `^0.5` against the index, installs the
pinned wheels into the node's managed env, and writes the pins to
`dataflow.dora-lock.yaml`. No `pip install` line, no PATH knowledge.

**UC2 — Contract mismatch caught at compose time** *(Phase 1+2)*
Ana wires `detector/bbox` (declared `std/vision/v1/BBox2D` in the manifest)
into a node expecting `std/media/v1/Image`. `dora validate` (and `dora build`)
fails with the producer/consumer URNs and the field-level diff — before any
process starts. This works because manifest contracts surface as the existing
`input_types`/`output_types` annotations.

**UC3 — Publish a Python node** *(Phase 3)*
Ben has a LiDAR driver as a Python package. `dora hub init` scaffolds
`dora-node.yml` next to his `pyproject.toml`; he fills in outputs and types.
`dora hub publish --dry-run` validates the manifest, checks the entry point,
and verifies contracts. His release CI publishes the wheel to PyPI as today;
`dora hub publish --skip-native` then opens a PR against the index adding
`ben-robotics/dora-lidar@1.0.0` with the wheel hashes. Index CI validates the
entry and the bot auto-merges (§7.5) — the node is discoverable minutes after
the wheels land, with no human in the loop for routine version adds.

**UC4 — Publish a Rust node** *(Phase 2/3)*
Same flow; the backend stanza points at crates.io (`cargo install --locked`
source build, `.crate` checksum pinned) or at prebuilt binaries on a GitHub
Release / OCI registry for heavy nodes.

**UC5 — Reproducible team builds** *(Phase 2)*
Carol commits `dataflow.yml` + `dataflow.dora-lock.yaml`. CI runs
`dora build --locked`: resolution is skipped, every pinned artifact —
including transitive Python dependencies — is hash-verified, and a changed
upstream release cannot alter the build. This extends the existing v2
lockfile that already does this for git sources.

**UC6 — Multi-machine deployment** *(Phase 2)*
Dataflow nodes deploy to three daemons on mixed architectures. The CLI
resolves `hub:` ranges to pins once (centrally, like git rev→commit today);
the coordinator routes per-daemon node sets; **each daemon fetches and
installs its own nodes** for its own platform from self-sufficient pins
(§10.2 — daemons need no hub configuration). Platform-specific artifacts
(wheels, binaries) select per-daemon at fetch time.

**UC7 — Offline robot** *(Phase 2)*
A field robot has no internet. `dora hub fetch dataflow.yml --target-dir
./hub-cache` (run where there is connectivity) downloads the full pinned
closure — node artifacts and transitive dependencies — from the lockfile;
on the robot, `dora build --locked --offline` re-verifies every archive
against its lockfile hash and fails loudly on any miss.

**UC8 — Enterprise private index** *(Phase 2)*
A company keeps proprietary nodes in `git@github.com:acme/dora-index.git` and
artifacts in their Artifactory (PyPI-compatible) and Harbor (OCI) instances.
`~/.config/dora/hub.toml` adds the index **bound to the `acme/` namespace**
(§7.3) — namespace↔index binding makes dependency-confusion attacks
structurally impossible. Descriptor entries written `hub: acme/lidar-fusion@2.1`
resolve only against the acme index. No dora-side server, SSO, or token
system — git and registry auth are the company's existing credentials.

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
│ dataflow.dora-lock.yaml pins: version + hashes + dep closure     │
└──────────────┬───────────────────────────────────────────────────┘
               │ resolve (CLI, central)          fetch/install (per daemon)
┌──────────────▼───────────────┐   ┌──────────────────────────────┐
│ INDEX (git repo)             │   │ BACKENDS (artifact bytes)    │
│  dora-rs/node-index          │   │  PyPI wheels   (Python)      │
│  <ns>/<name>/<ver>.yml       │   │  crates.io     (Rust src)    │
│   = manifest + backend pin   │   │  OCI / GH Release (binaries) │
│  + private indexes (git),    │   │  git+commit    (escape hatch)│
│    namespace-bound           │   │                              │
└──────────────────────────────┘   └──────────────────────────────┘
               ▲                                  ▲
               │ publish = native publish + index PR (bot-merged)
┌──────────────┴───────────────────────────────────┴──────────────┐
│ NODE PACKAGE: pyproject.toml/Cargo.toml + dora-node.yml manifest │
└──────────────────────────────────────────────────────────────────┘
```

Five layers, five independent failure domains:

1. **Manifest** (`dora-node.yml`): the typed, dora-specific description of a
   node — contracts, entry point, env/config. Lives in the node's repo,
   *copied into* the index at publish time so discovery never needs artifact
   downloads.
2. **Index**: a git repo mapping `namespace/name@version` → manifest + backend
   pin, with machine-enforced append-only rules and bot-merged publishing
   (§7.5). Auth, hosting, CDN, and review tooling are delegated to git
   hosting. Multiple indexes compose via namespace binding.
3. **Backends**: where bytes live. Uniform *pin* abstraction (exact version +
   content hashes + resolved artifact URLs), per-backend fetch/install
   dispatch.
4. **CLI** (`dora hub` + `hub:` handling inside `dora build`/`run`/`validate`):
   the single user surface. Uniform UX does not require a uniform format.
5. **Runtime integration**: resolved hub nodes flow through the *existing*
   build/spawn pipeline (managed envs, working dirs, multi-daemon routing) —
   the daemon learns one new source kind and nothing else.

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

A git repository (official: `dora-rs/node-index`) with one file per published
version:

```
index/
  dora-rs/
    dora-yolo/
      package.yml          # namespace-level: description, repo, OWNERS list
      0.5.1.yml            # manifest snapshot + backend pin (+ yanked flag)
      0.5.2.yml
  acme/
    lidar-fusion/
      2.1.0.yml
```

A version file is the manifest (§5) plus a `dist` stanza:

```yaml
# index/dora-rs/dora-yolo/0.5.2.yml
manifest: { ...dora-node.yml contents, verbatim... }
dist:
  backend: pypi
  package: dora-yolo               # PEP 503 normalized
  version: 0.5.2
  artifacts:                       # the ONLY installable artifacts for this
    - file: dora_yolo-0.5.2-py3-none-any.whl
      url: https://files.pythonhosted.org/…/dora_yolo-0.5.2-py3-none-any.whl
      sha256: "ab12…"
published: 2026-07-01T12:00:00Z
yanked: false                      # UC10
```

Artifacts not listed here are **not installable** through Hub, even if the
backend later grows more files for the same version (e.g. late platform
wheels uploaded to an existing PyPI release). Late artifacts are added by an
*additive append* PR — see §7.5 for what the rules machine-enforce.

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
git = "https://github.com/dora-rs/node-index"
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

Index transport: shallow clone / `git fetch --depth 1` into
`~/.cache/dora/index/<alias>/`, refreshed at most once per
`dora build`/`hub` invocation; `--offline` skips. Refresh requires
**fast-forward**: the CLI records the last-seen index commit and warns loudly
on non-fast-forward or regression (freeze/rollback protection — a stale or
replayed index could otherwise hide yanks; full TUF-style signing is out of
scope for v1 and noted as residual risk in §11).

### 7.4 Namespace governance

Namespaces are claimed by a PR adding `index/<ns>/` with a `package.yml`
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

## 8. Backends and pins

| Backend | For | Pin | Fetch/install |
|---|---|---|---|
| `pypi` | Python nodes (incl. maturin mixed Rust/Python) | dist name (PEP 503) + version + per-wheel `url`+`sha256` | new uv-pip step into the node's managed env (env prepared by the existing `prepare_managed_python_env`); install via a generated requirements file with hashes; **wheel-only** (`--no-build` — sdists are never built); transitive closure from the lockfile (§10.3) |
| `crates-io` | Rust nodes, source install | crate + version + `.crate` sha256 (the sparse-index `cksum`) | download + verify `.crate`, then `cargo install --locked` (fails loudly if the crate ships no `Cargo.lock`) |
| `oci` | Prebuilt binaries, C/C++, large/mixed artifacts | reference + top-level index digest | pull via OCI distribution API (digest-addressed), select platform manifest, verify, extract (§8.1) |
| `github-release` | Prebuilt binaries, simple cases | release asset `url` + sha256 | HTTP download + hash check via `dora-download` (reqwest + sha2, already a daemon dependency used for URL `path:` nodes) |
| `git` | Escape hatch / unpublished | repo + commit | the **existing** `GitBranch`/`GitManager` machinery, unchanged |

Backend dispatch is a trait (`HubBackend: resolve, fetch, install_into`) in a
new `dora-hub-client` library crate (usable by both CLI and daemon), one
implementation per row, each a separate small PR. Platform selection (wheel
tags, per-target release assets, OCI platform manifests) happens at fetch
time **on the machine that will run the node** (§10.2).

**Pins are self-sufficient**: they carry resolved artifact URLs and hashes,
so daemons never consult an index or read `hub.toml` (mirroring how
`GitSource` carries the full repo URL today). Endpoint rewriting for
air-gapped mirrors, if needed, is daemon-side configuration (a daemon flag /
env var), specced with P2.10.

**Cache trust model.** Two caches:
`<base_working_dir>/hub/<backend>/<ns>/<name>/<version-or-digest>/`
(per-workspace) and `~/.cache/dora/hub/` (user-level, shared). Downloaded
*archives* (wheels, `.crate` files, OCI blobs, release assets) are
content-addressed and **re-verified against the lockfile hash before every
install/copy**, including cache hits. Build *outputs* (compiled crates,
populated envs) are trusted within the local user boundary once built from
verified inputs. Offline transfer (UC7) ships archives + lockfile — verified
again on the target — never built outputs.

### 8.1 OCI artifact layout

A dora node as an OCI artifact (full details settle in the P2.13 issue; the
shape is fixed here):

- Top level: an OCI image index (multi-platform), `artifactType:
  application/vnd.dora.node.v1`. The pin is this index's digest.
- Per platform: an image manifest whose config blob is the `dora-node.yml`
  manifest (`application/vnd.dora.node.manifest.v1+yaml`) and whose single
  layer is a `tar+gzip` of the node payload, entrypoint at archive root.
- Fetch: select platform manifest → verify digests → extract layer into the
  hub cache → spawn entrypoint from there.

Pushing this layout is `oras push`-compatible; `dora hub publish` wraps it.

## 9. CLI surface

New nested subcommand (`binaries/cli/src/command/hub/`, following the
`cluster` module pattern):

```
dora hub search <query> [--category C] [--platform P]   # search index manifests
dora hub info <pkg>[@<ver>]                             # manifest, contracts, example
dora hub list [--dataflow F]                            # hub packages in lockfile/cache
dora hub init [--manifest-only]                         # scaffold dora-node.yml
dora hub publish [--dry-run] [--skip-native] [--index A]  # §9.1
dora hub yank <pkg>@<ver> [--undo]                      # automates the flag-flip PR
dora hub fetch <dataflow.yml | pkg@ver> [--target-dir D]  # pre-populate cache (UC7)
dora hub outdated [--dataflow F]                        # lockfile pins vs index
dora hub update [<pkg>] [--dataflow F]                  # re-resolve + rewrite lockfile
```

`login`/`logout` from the original proposal are deliberately absent (no
hosted service; git and native registries carry their own auth). There is
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

```
dora hub publish
  1. Validate: manifest schema, entrypoint exists and is a bare name,
     type URNs resolve, contract sanity, env-var deny-list
     (dry-run stops here)
  2. Native publish (skippable with --skip-native when release CI already
     published): uv build+publish / cargo publish / oras push / gh release
  3. Capture pins: artifact URLs + hashes from the backend (PyPI JSON API,
     crates sparse-index cksum, OCI digest, asset sha256). If the release
     CI publishes platform artifacts asynchronously, publish either waits
     for the manifest-declared platform set or records what exists now —
     late artifacts are added by additive-append PRs (§7.5).
  4. Open the index PR (via `gh` if available, else print the entry + URL);
     bot-merged when CI passes (§7.5)
```

One publish path, not two: tag-webhook publishing (original proposal §7.1) is
dropped; release CI that already publishes natively just runs
`dora hub publish --skip-native` (UC3) — the index PR is the single source of
truth either way.

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
  (`libraries/message/src/descriptor.rs`, sibling of `git:`/`path:`) and a
  new `NodeSource::Hub` variant.
- **`hub:` replaces `path:`** — unlike `git:`, which annotates a
  path-bearing node. This means `Node::kind()` (which today bails without
  `path`) and `node_kind_mut()` (which hard-requires it) gain an explicit
  hub arm, and the spawn path receives the entrypoint through the resolved
  pin rather than the descriptor: the pinned
  `HubSource { package, version, backend, integrity, artifact_urls,
  entrypoint, env_defaults }` travels in the lockfile and in the
  coordinator→daemon build message, and the daemon synthesizes the spawn
  command from it (today's `CustomNode.path` consumer in
  `spawn/command.rs`). P2.5/P2.10 are sized for this — it touches
  dora-message, descriptor parsing, and daemon spawn, not just an enum.
- `path:`/`build:` are forbidden together with `hub:` (the manifest supplies
  both); `inputs`/`outputs` in the dataflow must be a subset of the
  manifest's declared ports (validated).
- `input_types`/`output_types` are injected from the manifest (§6.2).
- The parser accepts `hub:` under its final name from day one, gated by an
  "unstable feature" warning until Phase 2 exits — *not* an
  `_unstable_hub:` key that would leak into 60+ index `example:` blocks and
  every tutorial, then need a rename.

### 10.2 Multi-daemon flow (mirrors git sources)

1. **CLI (central):** resolve every `hub:` range → pinned, self-sufficient
   `HubSource` (§10.1); write/verify lockfile. (Parallel of rev→commit
   resolution, though it runs earlier in `build()` — see §6.2.)
2. **Coordinator:** `BuildDataflowNodes` carries `hub_sources` per daemon
   next to `git_sources` (`libraries/message/src/coordinator_to_daemon.rs`).
   **Mixed-version compatibility:** daemons advertise hub support in their
   registration handshake; the coordinator fails dispatch of a hub dataflow
   to a daemon that lacks it, with a per-daemon error naming the upgrade —
   never silent field-dropping.
3. **Daemon:** fetches the pinned artifact *for its own platform* (pin
   carries all per-platform artifact URLs + hashes; manifest `platforms:`
   was already checked pre-dispatch), installs into the node's env/cache,
   records working dir + env in `BuildInfo` exactly as today. Spawn resolves
   the entrypoint **only inside the node's managed env / hub cache dir**
   (§11) via the existing managed-PATH logic in `spawn/command.rs`.

A daemon whose platform has no pinned artifact fails the build with a clear
per-node, per-daemon error.

### 10.3 Lockfile

`BuildLockfile` v3 adds, next to `git_sources` (the descriptor fingerprint
scheme is generalized to cover hub stanzas, so `--locked` detects hub drift):

```yaml
hub_sources:
  detector:
    package: dora-rs/dora-yolo
    version: 0.5.2
    backend: pypi
    index: official
    entrypoint: dora-yolo
    artifacts:                       # the node's own artifacts (all platforms)
      dora_yolo-0.5.2-py3-none-any.whl:
        url: https://files.pythonhosted.org/…
        sha256: "ab12…"
    environment:                     # FULL resolved dependency closure for the
      - name: numpy                  # node's env (uv-exported, with hashes) —
        version: 2.4.6               # without this, "reproducible" and UC7
        wheels: { … sha256 per platform wheel … }   # offline are fiction
      - name: opencv-python
        …
```

- `--locked`: fingerprint match + every install consumes only
  lockfile-listed, hash-verified artifacts (uv requirements-with-hashes
  semantics require the full closure — which the lockfile has).
- `--offline`: additionally, no network — cache must hold the closure
  (populated by `dora hub fetch`, UC7).
- crates-io nodes record the `.crate` cksum; their *transitive* build still
  follows the crate's own `Cargo.lock` via `cargo install --locked` —
  crates without a committed `Cargo.lock` fail with a clear error rather
  than silently floating.

## 11. Security model

| Property | Mechanism |
|---|---|
| Version immutability | **machine-enforced** append-only index CI (§7.5): existing version files accept only `yanked` flips and artifact *additions*; anything else needs an index admin. Git history audits everything |
| Artifact integrity | sha256/digest pinned in the index entry and lockfile; re-verified on **every** install, including cache hits (§8 cache trust model). Only index-listed artifacts are installable |
| Build-time code execution | wheels only for pypi (sdists never built); `cargo install --locked` for crates (build.rs of *pinned* deps still runs — stated plainly); installing a node runs its packaging-level code paths exactly like `pip install` today. `--locked` + hashes bound execution to reviewed, pinned versions |
| Malicious manifest fields | `entrypoint` restricted to bare command names resolved inside the managed env/cache (no absolute paths, no traversal — the executed binary cannot escape the pinned artifact); `env:` deny-list for loader-hijack variables (LD_PRELOAD, DYLD_*, PYTHONPATH, PATH) |
| Index integrity | git over HTTPS/SSH; protected branch; bot merge only on rule-conformant diffs; fast-forward-only refresh with rollback warning (§7.3) |
| Namespace integrity | CI-verified GitHub identity match, reserved list, confusable-name screening, human review for new namespaces (§7.4) |
| Publisher auth | delegated: PyPI/crates.io/ghcr accounts for bytes, GitHub for index PRs — dora stores no credentials. The dora-rs org **requires 2FA** for index-repo collaborators; Trusted Publishing/OIDC recommended for official packages' native publishing |
| Yank | owner yank-flip PRs auto-merge (minutes, UC10); index admins can yank over an unresponsive owner; mass-yanks from one namespace trigger second review; never deletes; lockfiles keep working |

**Residual risks, stated plainly:** (1) a compromised namespace-owner GitHub
account can publish malicious *new* versions until noticed — pins and
`--locked` protect existing consumers; detection relies on the community and
yank. This is the same trust level as PyPI/crates.io account compromise.
(2) Without index signing (TUF-style), a compromised git host could serve a
frozen index state within the fast-forward constraint; signing is a Phase 4
candidate. (3) crates-io transitive build scripts execute at install time,
bounded by the crate's own lockfile.

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
3. Bulk index PR pins the **already-published** wheels (versions, URLs +
   sha256 from PyPI's JSON API). No re-publishing, no version bumps.
4. `dora-hub` repo CI gains a `dora hub publish --skip-native` step keyed to
   its existing release process (per-node version detection from the release
   diff — specced in the P3.3 issue, since dora-hub is a monorepo with a
   single release event today).
5. The README package table becomes generated output from the index (badge
   links unchanged).
6. `build: pip install dora-X` keeps working indefinitely — `hub:` is sugar
   plus contracts plus pins, not a breaking change.

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
- **P2.3** Bootstrap `dora-rs/node-index`: repo layout, index CI
  (append-only enforcement, schema/pin validation, namespace identity +
  confusable checks), auto-merge bot, namespace policy page, PR templates.
  *Depends: P2.1 (format). Larger than one PR for the policy/bot pieces —
  tracked as 2–3 PRs in its issue.*
- **P2.4** `dora hub search` / `info` / `list`. *Depends: P2.2, P2.3.*
- **P2.5** Descriptor: `hub:` field (unstable-warning gate), `NodeSource::Hub`,
  hub arm in `Node::kind()`/`node_kind_mut()`, port-subset validation, and
  the `HubSource` pin type carrying entrypoint/env (§10.1). *Depends: P1.4.*
- **P2.6** Lockfile v3: `hub_sources` incl. dependency-closure `environment`
  capture (uv export), fingerprint extension, `--locked`/`--offline`
  semantics. *Depends: P2.1, P2.5.*
- **P2.7** Backend: `pypi` — resolve/fetch, wheel-only hash-checked install
  into managed envs via generated requirements file. *Depends: P2.2, P2.5,
  P2.6.*
- **P2.8** Backend: `crates-io` (cksum pinning, `cargo install --locked`).
  *Depends: P2.7 (trait shape settles).*
- **P2.9** Backend: `github-release` (via `dora-download`). *Depends: P2.7.*
- **P2.10** Multi-daemon, split: **a)** protocol — `hub_sources` in
  `BuildDataflowNodes` + capability handshake + mixed-version error path;
  **b)** daemon fetch/install path (reusing `dora-hub-client`);
  **c)** per-platform artifact selection + pre-dispatch `platforms:` check +
  error surfaces. *Depends: P2.7.*
- **P2.11** `dora hub fetch` (closure pre-fetch, UC7), `--hub-override`
  (UC11), smoke tests in `tests/example-smoke.rs` against a **local
  fixture index** (no dependency on the live node-index), and an
  `examples/hub-dataflow`. *Depends: P2.7.*
- **P2.12** Custom-type shipping (§6.3): registry load path from hub cache.
  *Depends: P2.7; parallel-safe.*
- **P2.13** Backend: `oci` (§8.1 layout; `oci-client` dependency audit).
  *Depends: P2.7.*

### Phase 3 — Publishing & migration (ecosystem goes live)

- **P3.1** `dora hub publish` (validate → native publish → pin capture →
  index PR, `--index`, async-artifact handling). *Depends: P1.3, P2.3.*
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
                 P2.2 ─────────┼─► P2.7 ─► {P2.8, P2.9, P2.10a→b→c,
                               │           P2.11, P2.12, P2.13}
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
3. **OCI pull dependency**: minimal hand-rolled distribution-API client vs
   the `oci-client` crate. *Default: `oci-client`, audited via the existing
   supply-chain gates.*
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
| D18 | Metadata+source (build-on-install) for Py/Rust; 3-tier C/C++ | Pinned native artifacts (wheel/crate/OCI/release), wheel-only for Python | Defines the missing package format by delegation; immutable pins + closure locking fix the mutable-tag and floating-dependency holes |
| D22–25 | Runtime-layer venv/conflict/offline items | Managed uv envs already exist; offline via closure-complete cache + lockfile | Most of it shipped in dora already (`--uv` envs) |
| D26 | Self-hosted registry binary | Private git index, namespace-bound | Strictly less to operate |
| D28 | Official = no prefix; promotion strips prefix | Bare name = `dora-rs/` shorthand; everything else always namespaced | Keeps the original's ergonomic instinct without renames-on-promotion breaking references |
| §7.1 | Dual publish path (CLI + tag webhook → Hub CI) | Single path: native publish + bot-merged index PR | One source of truth; no CI farm running untrusted builds; bot merging keeps publish latency in minutes |
| §IX | REST API v1 | None in v1 | The index *is* the API; resolution is local |
