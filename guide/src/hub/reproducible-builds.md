# Reproducible Builds

A `hub:` requirement like `^0.5` is a *range*. Without pinning, each `dora build`
re-resolves it to the latest matching version, so two builds can differ. A
**lockfile** records the exact commit (and, for binary nodes, the exact
artifact) each node resolved to, so a build is reproducible across machines and
over time — the same model as `Cargo.lock` or `package-lock.json`.

## Writing a lockfile

```bash
dora build dataflow.yml --write-lockfile
```

This resolves every `hub:` (and `git:`) node, builds the dataflow, and writes a
lockfile next to the dataflow named `<dataflow-stem>.dora-lock.yaml`. Commit it
alongside the dataflow.

The lockfile stores, per node:

- the resolved **git source** — repo, exact `commit_hash`, and `subdir`; plus
  the hub **provenance** (package `name`, resolved `version`, and a
  `manifest_digest`), or
- the resolved **binary source** — `platform`, `url`, and `sha256` — for a node
  that shipped a prebuilt artifact.

It also records a `descriptor_fingerprint` of the dataflow's source graph, used
to detect when the YAML changed out from under the lockfile.

## Building from the lockfile

```bash
dora build dataflow.yml --locked
```

`--locked` is **strict**: every `hub:` node must be present in the lockfile and
still valid, or the build fails rather than resolving live. It rebuilds the exact
pinned commits/artifacts. Use it in CI and on deploy targets so what you ship is
what you tested.

`--locked` also re-checks integrity:

- the dataflow's fingerprint must match the lockfile (the YAML hasn't drifted);
- a binary node's artifact is re-verified against its `sha256`;
- if a published index entry's manifest was rewritten since you locked
  (its `manifest_digest` no longer matches), `--locked` **fails** — the
  contract you locked against changed. (A rewritten *source pin* warns and uses
  the lockfile pin.)

> `dora validate` is **best-effort**, not strict: when a lockfile exists it uses
> its pins, but a missing or stale pin falls back to live resolution with a note
> instead of failing — so validating a half-updated dataflow never fails harder
> than building it without `--locked`. Reproducibility is enforced by
> `build --locked`, not by `validate`.

## Inspecting and updating pins

### What's pinned

```bash
dora hub list dataflow.yml
```

Lists each hub package pinned in the lockfile with its version and short commit
(or `binary <platform>` for a binary pin). Errors if there is no lockfile,
pointing you at `--write-lockfile`.

### What's outdated

```bash
dora hub outdated dataflow.yml
```

Compares each locked pin against the latest **non-yanked** version in the index
(ignoring the YAML range) and reports `node: ns/name <pinned> -> <latest>
(newer available)`. It exits non-zero if any package couldn't be checked, so it
composes in CI.

### Apply updates

```bash
dora hub update dataflow.yml            # re-resolve + rewrite the lockfile
dora hub update dataflow.yml --dry-run  # show what would change, write nothing
```

`update` re-resolves every node within its YAML requirement and rewrites the
lockfile — but compiles nothing. It produces a lockfile byte-identical to
`dora build --write-lockfile` (it re-resolves non-hub git refs and runs the
contract/type checks too), just without the build. Review the lockfile diff and
commit it.

All of these accept `--offline` to use only the cached index.

## Offline and air-gapped builds

Two independent "offline" mechanisms:

- **`--offline`** (on `build`, `validate`, and the `hub` query commands) skips
  the network index refresh and uses the local cache, failing loudly on a cache
  miss instead of silently going stale.

- **`dora hub fetch`** mirrors the pinned *sources* into a directory so a machine
  with no network (or no git access to the source repos) can still build:

  ```bash
  dora hub fetch dataflow.yml                      # mirror the lockfile's pins
  dora hub fetch dataflow.yml --target-dir vendor  # into ./vendor
  dora hub fetch dora-yolo@0.5.2                    # mirror one package, live-resolved
  ```

  A dataflow target reads pins from the lockfile (validated against the current
  YAML, like `--locked`); a bare `name@version` resolves live against the index.
  Sources are blobless-cloned at their pinned commit into `<dir>/<commit>`.

  > `fetch` mirrors **git** sources. Binary-node artifacts are **not**
  > pre-downloaded — the daemon fetches and sha256-verifies them at spawn time.
  > For a fully air-gapped binary node, host the artifact `url` somewhere the
  > target can reach.

## Recommended workflow

1. Develop with plain `dora build` (live resolution within your `^`/`~` ranges).
2. Before sharing or deploying: `dora build --write-lockfile`, commit the
   `*.dora-lock.yaml`.
3. In CI and on deploy targets: `dora build --locked` (and `--offline` if the
   index cache is pre-warmed).
4. Periodically: `dora hub outdated`, then `dora hub update` + review the diff.
