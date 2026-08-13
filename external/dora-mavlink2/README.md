# dora-mavlink2 — parked MAVLink 2 bridge

MAVLink 2 ↔ Apache Arrow conversion (common dialect) plus a daemon-spawnable
bridge node speaking TCP, UDP, and direct serial. **Extracted from
`dora-rs/dora` before the 1.0 release and parked here.** It is not built by a
dora build and dora 1.0 ships no MAVLink support.

This directory is staged for lifting into its own repository. Nothing here is
compiled by the workspace — the crates are listed in the root `Cargo.toml`
`[workspace] exclude`, not `members`.

---

## Why this was parked, and why it is easy to unpark

Unlike the memory-pool extraction, this one is **not** a warning about
entanglement. MAVLink was already well-isolated: it touched dora only through
the public node API, and removing it from the tree changed no dora source file
outside the workspace manifest and the smoke-test harness.

It was parked for scope, not for design. At 1.0 dora commits to semver on
everything it publishes, and a domain-specific protocol bridge — one MAVLink
dialect, one autopilot ecosystem, roughly two months old at the freeze — is a
poor thing to freeze. Shipping it on its own version line lets it move at the
pace of the flight-controller ecosystem instead of dora's.

**So the seam already exists.** Reinstating this does not require designing
one; it requires a repository, a release, and a dependency edge pointing the
other way.

## Layout

| Path | Former location | Crate |
|---|---|---|
| `bridge/` | `libraries/extensions/mavlink2-bridge` | `dora-mavlink2-bridge` |
| `bridge-node/` | `binaries/mavlink2-bridge-node` | `dora-mavlink2-bridge-node` |
| `examples/` | `examples/mavlink2-bridge` | 3 example node crates + 3 dataflow YAMLs |
| `examples-sitl-mission/` | `examples/mavlink2-bridge-sitl-mission` | ArduCopter SITL mission (no crates) |

## What it depended on

- `dora-node-api` — the public node API, nothing internal.
- `mavlink 0.18` with `std`, `serde`, `dialect-common`, and the
  `transport-{tcp,udp,direct-serial}` features. This dependency was removed
  from the root `[workspace.dependencies]`; the extracted crates must declare
  it themselves.
- `arrow` — via `dora-node-api`'s re-export.

## To stand this up as its own repository

1. `git subtree split` (or `git filter-repo`) this directory to preserve the
   history of all four subtrees.
2. Turn the four directories into a workspace: add a root `Cargo.toml` with
   `members = ["bridge", "bridge-node", "examples/*"]`, and move the `mavlink`
   dependency spec above into its `[workspace.dependencies]`.
3. Replace `dora-node-api = { workspace = true }` with a crates.io version
   requirement — `dora-node-api = "1"` once dora 1.0 is published.
4. Port the smoke coverage. The four smoke tests deleted from
   `tests/example-smoke.rs` (`smoke_mavlink2_bridge_rust`,
   `smoke_local_mavlink2_bridge_rust`, and the two `_python` variants) and the
   `scripts/smoke-all.sh` entries are the starting point; both used the UDP
   simulator rather than a real autopilot, so they are CI-portable as-is.
5. Keep the fixed-port caveat in mind: the bridge binds `udp:14550`, so the
   smoke harness needs the orphan-reaping `pkill` that was removed from
   `scripts/smoke-all.sh`, or successive runs fail with "Address already in
   use".

## Known loose ends carried over

- `deny.toml` and `.cargo/audit.toml` still carry a `RUSTSEC-2026-0194`
  (`quick-xml` quadratic attribute check) ignore whose justification names
  `mavlink-bindgen` as one of two build-time sources. With MAVLink gone the
  remaining source is `self_update`; the ignore entry is still needed but its
  comment now over-explains. Not worth churning until the ignore expires
  (review date 2026-09).
- `_typos.toml` retains an allow-list entry introduced for flight-controller
  vocabulary in the MAVLink example. Harmless, and cheap to keep in case the
  bridge returns.
- The SITL mission example was never in CI (it needs an external ArduPilot
  SITL on `udp:14550`) and remains manual-only.
