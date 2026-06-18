# Overview

The **Dora Hub** lets you use a reusable node in a dataflow with one line of
YAML, instead of vendoring its source or hand-writing a `path:`/`git:` entry:

```yaml
nodes:
  - id: detector
    hub: dora-yolo@^0.5      # ← resolved from the Hub, pinned, contract-checked
    inputs:
      image: camera/image
    outputs:
      - bbox
```

`dora build` resolves `dora-yolo@^0.5` to a specific published version, pins it
to an exact commit, injects its typed input/output contracts into validation,
and builds it — the same way `cargo` or `npm` resolve a dependency.

> The `hub:` feature is **unstable**: its behaviour may change in a future
> release. The node manifest format itself (`apiVersion: 1`) is stable and
> useful without the Hub — see [Writing a Node Manifest](node-manifest.md).

## The model

The Hub is a **package manager for dora nodes**, deliberately cargo-style:

- **Per-dataflow resolution, no global install.** There is no `dora hub install`
  that mutates a machine. A node is a dependency of *a dataflow*; you add `hub:`
  to the YAML and `dora build` resolves it. (`dora hub install` exists only to
  point you at this.)
- **A git-backed index, not an artifact upload.** Publishing adds a small
  *index entry* that points at a git commit of the node's source; it does not
  upload a tarball. The default index is the public catalog at
  [`dora-rs/dora-hub`](https://github.com/dora-rs/dora-hub).
- **Source-distributed by default.** A resolved node is cloned at its pinned
  commit and built on the machine that runs it. A node may *optionally* ship a
  prebuilt binary for a platform (see [Custom & Enterprise
  Indexes](indexes.md#binary-distribution)).
- **Reproducible.** `dora build --write-lockfile` records the exact commits;
  `dora build --locked` rebuilds them byte-for-byte. See [Reproducible
  Builds](reproducible-builds.md).
- **Typed end-to-end.** A published node carries its dora input/output contracts
  in its manifest, so wiring it wrong fails at `dora build`/`dora validate` time,
  before anything runs.

## A package reference

A `hub:` value is `[<namespace>/]<name>@<version-requirement>`:

```yaml
hub: dora-yolo@^0.5          # official namespace (dora-rs) — bare name
hub: dora-rs/dora-yolo@^0.5  # the same, written out
hub: acme/lidar@~2.1         # a third-party namespace
hub: dora-vad                # no @req → any version (the latest non-yanked)
```

A **bare name** is shorthand for the official `dora-rs/` namespace. The version
requirement is a standard semver/Cargo range (`^0.5`, `~2.1`, `>=0.5, <0.7`,
`=1.2.3`, `*`); resolution picks the **highest non-yanked** version that
satisfies it. See [Using Hub Nodes](using-nodes.md) for the full syntax.

## What this section covers

- **[Using Hub Nodes](using-nodes.md)** — reference a node, search the catalog,
  inspect a package's contracts.
- **[Writing a Node Manifest](node-manifest.md)** — turn your own code into a
  node others can use with one line.
- **[Reproducible Builds](reproducible-builds.md)** — lockfiles, `--locked`,
  checking for and applying updates, offline builds, and mirroring sources.
- **[Publishing a Node](publishing.md)** — add your node to a catalog, manage
  versions, and the namespace/ownership model.
- **[Custom & Enterprise Indexes](indexes.md)** — private/mirrored catalogs via
  `hub.toml`, namespace binding, and binary distribution.

## How it relates to `path:` and `git:`

`hub:` is a fourth node source, mutually exclusive with `path:`, `git:`, and
`build:` on the same node. Under the hood the Hub **desugars** a `hub:` node into
an ordinary pinned `git:` node (or a verified binary download) before the build
runs — so everything you know about git nodes still applies. The Hub's job is the
*resolution* (name + version → commit + contracts), nothing more.
