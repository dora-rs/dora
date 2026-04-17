# Migration Guide: dora 0.x → 1.0

> **Status (2026-04-16):** minimum-viable scaffold. The full migration guide with per-API before/after examples is tracked as follow-up work.
>
> Previous filename: `dora-compatibility.md` (which documented a fork→upstream compat layer that is obsolete under the tree-takeover consolidation strategy).
>
> **Cross-reference caveat:** the audit evidence files named below (`phase--1-audit-2026-04-16.md`, `audit-2026-03-21-closure.md`, `ownership-verification-2026-04-16.md`, `downstream-user-assessment-2026-04-16.md`) currently live on the `docs/consolidation-plan-review` branch (PR #286) and are not yet on `main`. They land with that PR. Filenames are written as plain text below so no link 404s if the merge order flips.

## What changed

dora 1.0 is the consolidation of the fork tree into the upstream `dora-rs/dora` repository. See [`plan-dora-1.0-consolidation.md`](plan-dora-1.0-consolidation.md) for the full context.

**Headline:** 1.0 is a **hard break** from 0.x. You cannot mix-and-match 0.x and 1.0 components in a running cluster. Plan a full-cluster restart for the upgrade.

## Before you upgrade — know the breaking changes

| Surface | 0.x behaviour | 1.0 behaviour | Evidence |
|---|---|---|---|
| Wire protocol (CLI ↔ coordinator) | tarpc over TCP with JSON framing | WebSocket with new message shapes | `phase--1-audit-2026-04-16.md` §4 (lands with PR #286) |
| Message-enum variants (`NodeEvent`, `DaemonCommunication`) | 0.x variant order | New variants inserted (`InputRecovered`, `NodeRestarted`, `ParamUpdate`, `ParamDeleted`, `Shmem`) — bincode tags shifted | `phase--1-audit-2026-04-16.md` §3 (lands with PR #286) |
| CLI handshake | Optional `get_version()` RPC | Mandatory `ControlRequest::Hello` with semver check | `libraries/message/src/cli_to_coordinator.rs` |
| Recording file extension | `.adorec` | `.drec` (magic bytes were already `DORAREC\x00` / `DORAEND\x00`) | `libraries/recording/src/lib.rs` |
| Request-reply communication layer | `libraries/communication-layer/request-reply/` | Replaced by `send_service_request()` / `send_service_response()` helpers and service/action patterns | [`docs/patterns.md`](patterns.md) |
| Auth | Query-parameter token | Bearer header token, constant-time comparison | `libraries/message/src/auth.rs` |

Additional fixes and hardening items from the 2026-03-21 audit are closed in 1.0; see `audit-2026-03-21-closure.md` (lands with PR #286) for the full per-finding record.

## Upgrade path

1. **Update `Cargo.toml` dependencies.** Bump `dora-node-api`, `dora-operator-api`, `dora-cli` to `1.0`. The crate names are unchanged; no rename.
2. **Rebuild every component.** Daemons, coordinators, CLI, and every node/operator binary must be rebuilt against the 1.0 crates. Do not run 0.x binaries against 1.0 daemons.
3. **Update YAML descriptors if needed.** Most 0.x descriptors work unchanged, but a few fields have new defaults or renames — see [`yaml-spec.md`](yaml-spec.md).
4. **Rename existing recordings.** `mv capture.adorec capture.drec` — the file format itself is unchanged, only the extension differs.
5. **Update CLI usage.** Any tooling that pipes files into `dora replay` should use `.drec` going forward. Shell completions and aliases referencing `*.adorec` need updating.
6. **Coordinate the restart.** In a distributed deployment, bring down all 0.x daemons before starting 1.0 daemons. A fork CLI connecting to an upstream 0.x coordinator (or vice versa) will fail at the TCP handshake with a low-level framing error, not a graceful version-mismatch message.

## New features in 1.0

Non-exhaustive; see the full 1.0 release notes for the complete list.

- Service pattern: `send_service_request()` / `send_service_response()` with correlation IDs
- Action pattern: goal / feedback / result metadata with cancellation
- Streaming chunks: `send_stream_chunk()` + `StreamSegment`
- Structured logging: `node.log()` / `node.log_info()` bridged from Python `logging`
- Restart awareness: `is_restart()` / `restart_count()`
- Input health: `InputTracker` / `InputState`
- Fault-tolerance snapshots + coordinator state catch-up
- `DoraNodeBuilder` with custom daemon port (upstream PR #1591 compatible)
- CUDA IPC via ctypes (no more `numba` dependency)

## Still to do (this guide)

- Per-API before/after code snippets for the 10 most-used 0.x APIs
- Dedicated section on the C/C++ API (headers stable, call signatures unchanged)
- Dedicated section on the Python API
- `dora migrate` subcommand usage (if shipped — scope is still open, see consolidation plan §297)

## Related documents

- [`plan-dora-1.0-consolidation.md`](plan-dora-1.0-consolidation.md) — the full consolidation plan (on main)
- `phase--1-audit-2026-04-16.md` — wire-protocol audit evidence (lands with PR #286)
- `audit-2026-03-21-closure.md` — security/correctness closure (lands with PR #286)
- `ownership-verification-2026-04-16.md` — publish-path readiness (lands with PR #286)
- `downstream-user-assessment-2026-04-16.md` — who this matters for (lands with PR #286)
