# Publishing a Node

Publishing makes your node installable with `hub: <namespace>/<name>@<version>`.
By default `dora hub publish` uploads the committed source archive and metadata
to the registry at <https://dora.pub> (override with `DORA_HUB_REGISTRY_URL`),
which stores an append-only version and a pinned source commit. A `hub.toml`
namespace **explicitly bound to a local/git index** keeps the legacy
index-entry workflow.

The registry never accepts a mutable branch/tag as a pin: the version records a
full source commit and the archive checksum, so consumers resolve immutable
bytes.

## Prerequisites

1. A committed [`dora-node.yml`](node-manifest.md) at your node's root with a
   `namespace` and `name`. The **version is not part of the manifest** — it is
   read from `[package].version` (Cargo) / `[project].version` (pyproject) at the
   pinned commit, or supplied with `--version`. (Adding `version:` to the
   manifest is rejected — it uses `deny_unknown_fields`.)
2. A registry account and a publish token:

   ```bash
   dora hub login                 # prompts for account/password; stores a token
   dora hub token                 # inspect the saved token
   ```

   Or create a token in the registry website and set `DORA_HUB_TOKEN`.
3. A git repository so `publish` can package the committed tree (a dirty
   working tree is intentionally ignored).
4. Your `namespace` is owned by your account (the registry enforces this).

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
| `--index <alias>` | the index bound to your namespace | target a specific `hub.toml` index (legacy) |
| `--registry <url>` | `DORA_HUB_REGISTRY_URL` or <https://dora.pub> | upload to a specific registry |
| `--token <token>` | `DORA_HUB_TOKEN` or saved token | registry API token |
| `--description`, `--license`, `--repository` | manifest / empty | registry metadata |
| `--dry-run` | — | validate + preview, don't upload |

The version, name, and manifest are all read **at the pinned commit** (via
`git show <commit>:…`), not from your working tree — so an uncommitted manifest
or version bump is an error, not a silently-published entry. Commit first.

## Publishing to the official registry

The default registry is <https://dora.pub> (not live yet). Until it launches,
point the CLI at a local/test registry:

```bash
export DORA_HUB_REGISTRY_URL=http://127.0.0.1:8080
dora hub publish                  # uploads the committed source archive
```

`dora hub publish` validates the manifest at the pinned commit, packages the
committed subtree, and uploads it with the token. The registry returns the
version, archive checksum and source pin; it is immediately available to
`hub:` resolution and to `dora hub search/info`.

For a namespace explicitly bound to a **local path index** in `hub.toml`, the
command appends the legacy index entry instead (atomic, append-only). A
git-backed index still prints the entry/PR instructions.

## Versioning and yanking

Published version files are **immutable** — the index is append-only. To release
a new version, publish a new entry; to mark a bad version, **yank** it:

```bash
dora hub yank dora-yolo@0.5.1 --reason "panics on empty input"
dora hub yank dora-yolo@0.5.1 --undo        # restore it
```

For a registry package this calls the registry's `yank`/`unyank` API with the
saved token; for a configured local index it flips the flag in place.

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
  official registry.
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
# commit dora-node.yml + the version bump
dora hub login                     # create/save a publish token
dora hub publish --dry-run         # validate + preview
dora hub publish                   # upload to the registry (or append to a local index)
```
