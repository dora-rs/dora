# Publishing a Node

Publishing makes your node installable with `hub: <namespace>/<name>@<version>`.
It does **not** upload code: it adds a small, append-only *index entry* that
points at a git commit of your node's source. The source stays in your repo; the
index just records "version `X` of this node lives at commit `Y`, and here is its
manifest."

## Prerequisites

1. A committed [`dora-node.yml`](node-manifest.md) at your node's root, with a
   `namespace`, `name`, and version (the version is read from
   `[package].version` / `[project].version` at the pinned commit).
2. The node's source pushed to a git remote (the index entry points at it).
3. Your `namespace` matches a GitHub org or user you control (see
   [Namespaces & ownership](#namespaces--ownership)).

## Preview the entry

```bash
cd my-node
dora hub publish --dry-run
```

This resolves the commit, reads and strictly validates the manifest as committed,
and prints the exact `<namespace>/<name>/<version>.yml` index entry it would add —
without writing anything or opening a PR. Run this first.

Useful flags:

| Flag | Default | Purpose |
|---|---|---|
| `--rev <ref>` | `HEAD` | the git commit/ref to pin |
| `--repo <url>` | the `origin` remote | the source repo URL recorded in the entry |
| `--version <semver>` | the native manifest's version at the commit | override the published version |
| `--index <alias>` | the index bound to your namespace | target a specific `hub.toml` index |
| `--dry-run` | — | validate + print, don't write |

The version, name, and manifest are all read **at the pinned commit** (via
`git show <commit>:…`), not from your working tree — so an uncommitted manifest
or version bump is an error, not a silently-published entry. Commit first.

## Publishing to the official catalog

The default index is the public catalog at
[`dora-rs/dora-hub`](https://github.com/dora-rs/dora-hub), a git repo. Because
it is git-backed, publishing to it is a **pull request**:

```bash
dora hub publish        # prints the entry + PR instructions for a git index
```

`dora hub publish` against a git-backed index prints the entry and the steps to
open a PR that adds it under `node-index/<namespace>/<name>/<version>.yml`. The
catalog's CI then validates the entry (schema, full-commit pin, path/namespace
consistency, append-only), and a conformant publish by a namespace **owner**
auto-merges — the publishing latency is just CI latency, the winget/conda-forge
model.

## Versioning and yanking

Published version files are **immutable** — the index is append-only. To release
a new version, publish a new entry; to mark a bad version, **yank** it:

```bash
dora hub yank dora-yolo@0.5.1 --reason "panics on empty input"
dora hub yank dora-yolo@0.5.1 --undo        # restore it
```

Yanking flips the one mutable flag on an entry. A yanked version is **skipped by
new resolution** (so `^0.5` picks the next-highest good version), but a lockfile
that already pinned it keeps working with a warning — so yanking never breaks
existing reproducible builds, it just stops new ones from selecting it. Like
publishing, yanking a git-backed index is a small flag-flip PR.

## Namespaces & ownership

- A **namespace** is a GitHub org or user (`dora-rs`, `acme`). The index key is
  `namespace/name`.
- A namespace resolves against **exactly one** index (no search order, so no
  dependency-confusion). The official `dora-rs` namespace always resolves to the
  official catalog.
- Each package carries an **owners** list (`package.yml`). The catalog's
  auto-merge only applies to a routine publish authored by an owner of the
  package's namespace; a new namespace, an owners change, or a yank by a
  non-owner is held for human review. A newly-claimed namespace must also match
  the claiming author's GitHub identity (their login, or an org they belong to) —
  claiming someone else's username is rejected.
- The `std` namespace and a short reserved list (`dora`, `dora-rs`, `dora-hub`,
  `hub`, `official`, …) are reserved.

## Local indexes

For testing or a private catalog on disk, `dora hub publish` against a
**local** (`path =`) index appends the entry directly (atomically, refusing to
overwrite an existing version) instead of printing PR instructions. See
[Custom & Enterprise Indexes](indexes.md) for configuring one.

## Checklist

```bash
dora hub init                      # scaffold dora-node.yml (if you haven't)
dora validate --node-manifest dora-node.yml   # manifest is well-formed
# commit dora-node.yml + the version bump, push
dora hub publish --dry-run         # preview the exact entry
dora hub publish                   # print the entry + PR instructions (or append, for a local index)
```
