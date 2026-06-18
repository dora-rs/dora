# Custom & Enterprise Indexes

By default the Hub resolves every namespace from the public catalog at
[`dora-rs/dora-hub`](https://github.com/dora-rs/dora-hub). With **no config at
all**, that is the only index. A `hub.toml` lets you add private or mirrored
catalogs and bind namespaces to them — for an internal node registry, an
air-gapped mirror, or an enterprise fork.

## `hub.toml`

Config lives at `~/.config/dora/hub.toml` (override the path with the
`DORA_HUB_CONFIG` environment variable). It is a list of `[[index]]` entries:

```toml
# An internal git-backed catalog for the `acme` namespace.
[[index]]
alias = "acme"
git = "https://git.acme.internal/dora-index"
namespaces = ["acme", "acme-labs"]

# A local on-disk catalog (useful for testing or air-gapped setups).
[[index]]
alias = "local"
path = "/srv/dora-index"
namespaces = ["sandbox"]
```

Each `[[index]]` has:

| Key | Meaning |
|---|---|
| `alias` | required; unique (case-insensitive); also the cache directory name |
| `git` | the catalog's git remote URL … |
| `path` | … **or** a local catalog directory (a subpath within a git clone defaults to `node-index`) |
| `namespaces` | the namespaces this index exclusively serves |

## Namespace binding (no dependency confusion)

Resolution is **not** a search order — it's an exclusive mapping, which is what
prevents dependency-confusion attacks:

- A namespace listed under an index resolves **only** there.
- Any namespace **not** bound to a custom index resolves from the official
  catalog.
- Binding the same namespace to two indexes is a **config error**.
- The official `dora-rs` namespace always resolves to the official catalog; a
  custom index cannot claim it.

So `hub: acme/lidar` in the config above resolves from the `acme` index, while
`hub: dora-yolo` still resolves from the official catalog — there is exactly one
place each can come from.

### Mirroring the official index

Point the reserved `official` alias at an internal mirror to serve even the
`dora-rs` namespace from your own infrastructure:

```toml
[[index]]
alias = "official"
git = "https://git.acme.internal/dora-hub-mirror"
```

## Binary distribution

A published node can ship a **prebuilt binary** for a platform instead of
requiring every consumer to compile from source — the cargo-binstall model. The
index entry's source gains a `binary` list:

```yaml
source:
  binary:
    - platform: linux-x86_64
      url: https://artifacts.acme.internal/lidar-2.1-linux-x86_64.tar.gz
      sha256: <64-hex>
  fallback-git:          # optional: build from source where no binary matches
    git: https://git.acme.internal/lidar
    rev: <full-commit-hash>
    subdir: nodes/lidar
```

At resolution time:

- if the entry ships a binary for the **current** platform (`<os>-<arch>`, exact
  match), the node becomes a sha256-verified URL download — **no clone, no
  build**;
- else if a `fallback-git` source is present, the node builds from that;
- else resolution fails with a clear "no binary for this platform" error.

The artifact `url` must be `https://` and the `sha256` (64 hex chars) is
re-verified after download. A binary pin is recorded in the lockfile so
`--locked` reproduces the exact artifact, and the **daemon** fetches and verifies
it at node spawn time.

> `dora hub fetch` mirrors git sources but **not** binary artifacts. For an
> air-gapped binary node, host the artifact `url` somewhere the target can reach.

## Catalog layout

A catalog (the official one or your own) is a directory tree:

```
node-index/
  <namespace>/
    <name>/
      package.yml        # description, repo, owners
      <version>.yml      # one immutable entry per published version
```

Each `<version>.yml` holds the node's manifest verbatim plus its `source`
(git pin or binary), `published` timestamp, and `yanked` flag. The catalog is
append-only — the only permitted mutation to a published version is the `yanked`
flag. See [Publishing a Node](publishing.md) for adding entries.

## Security notes

The Hub treats index entries as untrusted input:

- git source URLs are scheme-validated (`https`/`ssh`/`file`/`git@`) before any
  `git` subprocess runs; cleartext `git://` is rejected. Untrusted index entries
  may not point at local paths unless `DORA_HUB_ALLOW_LOCAL_SOURCES=1`.
- a `rev` must be a full 40/64-char hex commit — no branches, tags, or
  abbreviations (those are mutable).
- an entry's declared `namespace` must match the namespace it was fetched under,
  and catalog reads/writes are confined to the catalog root (no symlink escapes).
- binary artifacts are sha256-verified after download.
