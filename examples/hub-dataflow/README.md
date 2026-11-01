# Hub dataflow example

Demonstrates using a node published to the [Dora Hub](../../docs/plan-node-hub.md)
with the `hub:` field — no `path:`, no `git:`, no manual `build:`.

By default `dora` resolves from <https://dora.pub>; until that site launches,
point it at a local registry:

```bash
export DORA_HUB_REGISTRY_URL=http://127.0.0.1:8080
```

```yaml
# dataflow.yml
nodes:
  - id: detector
    hub: dora-yolo@^0.5      # bare name == the official `dora-rs/` namespace
    inputs:
      image: camera/image
    outputs:
      - bbox
```

`dora build dataflow.yml` resolves `^0.5` against the index to a pinned commit,
clones that commit, runs the node's `build:`, and writes the pin to
`dataflow.dora-lock.yaml`. `dora run dataflow.yml` then builds and runs it.

## Discovering and inspecting nodes

```bash
dora hub search detection            # find nodes by name/keyword/category
dora hub info dora-yolo              # typed inputs/outputs + example snippet
dora hub list dataflow.yml           # hub packages pinned in the lockfile
dora hub login                       # create/save a registry publish token
dora hub fetch dataflow.yml --target-dir ./hub-cache   # warm cache + mirror sources
```

`fetch` populates the index cache (so `--offline` *resolution* works) and
clones each pinned source into `--target-dir`. Full air-gapped *source* reuse
by `dora build` is a follow-up (#2097); today `--offline` builds resolve from
the cache and reuse `dora build`'s own clone cache.

## Reproducible and offline builds

```bash
dora build dataflow.yml --write-lockfile   # pin commits
dora build dataflow.yml --locked           # CI: rebuild from the pins, no resolution
dora build dataflow.yml --locked --offline # robot: build from the cache, fail on miss
```

## Notes

- `hub:` is an unstable feature; its behavior may change.
- The example references `dora-yolo`, which becomes available once the official
  registry is populated. Until then, set `DORA_HUB_REGISTRY_URL` to a local
  registry (see the hub-registry project) or point `~/.config/dora/hub.toml` at
  a local fixture index — see `tests/hub-smoke.rs` for a self-contained setup.
