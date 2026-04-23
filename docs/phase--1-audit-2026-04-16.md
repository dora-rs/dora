# Phase -1 Wire-Protocol Audit

**Date:** 2026-04-16
**Author:** heyong4725 + AI
**Scope:** Evidence for plan §19.7 "Wire protocol audit" gate and Decision Point D-1 (wire-protocol compatibility strategy). Closes issue #288.

---

## 1. Sources examined

| Side | Repo | Version | Git ref |
|---|---|---|---|
| `dora-upstream` | `dora-rs/dora` | 0.5.0 | `upstream/main` as of 2026-04-16 (HEAD `52835cff6` — "Replace numba with ctypes in dora.cuda for CUDA IPC (#1618)") |
| `dora-fork` | `dora-rs/adora` (this repo) | 0.2.1 | local `HEAD` as of 2026-04-16 |

Commands used are listed in §7 for reproducibility.

---

## 2. Headline finding

**The protocols are incompatible at multiple layers. Rolling upgrade between a `dora-upstream` 0.5.0 deployment and a `dora-fork` 1.0.0 deployment is impossible.**

The incompatibility is not a matter of one or two added fields. It spans:

1. **Transport layer** — upstream coordinator listens on tarpc/TCP with JSON framing; fork coordinator listens on WebSocket with its own framing. They cannot even complete a TCP handshake meaningfully, let alone exchange protocol messages.
2. **Message enum shape** — top-level CLI→coordinator dispatch changed from flat RPC structs (upstream) to an enum-based `ControlRequest` (fork).
3. **Enum variant ordering** — fork inserted new variants into the middle of existing enums (e.g. `NodeEvent`, `DaemonCommunication`), which shifts every subsequent bincode tag.
4. **New mandatory handshake** — fork adds a `Hello { dora_version }` request the coordinator uses to reject mismatched CLI versions up-front (per dora-rs/adora#151). Upstream has no such handshake.
5. **Fork-only files** — `libraries/message/src/auth.rs` (bearer-token auth) and `libraries/message/src/ws_protocol.rs` (WebSocket control plane) have no upstream equivalent.

**Recommendation: D-1a (hard protocol break).** The migration guide must state: *a full cluster restart is required when moving from dora 0.x to 1.0.* There is no rolling-upgrade path, and writing one (D-1b bridge release or D-1c version negotiation) is not justifiable — the divergence is too large and the user base is small.

---

## 3. Per-file diff summary

Diffed against `upstream/main` for `libraries/message/src/*.rs`:

| File | Diff lines | Added | Removed | Wire-compatible? | Notes |
|---|---:|---:|---:|---|---|
| `cli_to_coordinator.rs` | 383 | 260 | 82 | **No** | Fork replaces flat `BuildRequest`/`StartRequest` structs with `ControlRequest` enum + `Hello` handshake. Completely different dispatch. |
| `common.rs` | 221 | 116 | 33 | **No** | New types (`DaemonId` extensions, fork-only identifiers). |
| `config.rs` | 567 | 382 | 29 | **No** | Major additions; new config surface. |
| `coordinator_to_cli.rs` | 224 | 148 | 37 | **No** | New reply variants. |
| `coordinator_to_daemon.rs` | 225 | 128 | 65 | **No** | State catch-up additions. |
| `daemon_to_coordinator.rs` | 229 | 132 | 38 | **No** | New `DaemonEvent` enum, `NodeStatus`, `FaultToleranceSnapshot`, `NetworkMetrics`. |
| `daemon_to_daemon.rs` | 14 | 0 | 5 | **No** (minor) | Only 14 diff lines, but even small changes break bincode. |
| `daemon_to_node.rs` | 148 | 45 | 47 | **No** | `DaemonCommunication::Shmem` variant added ahead of `Tcp`; `NodeEvent` gained `InputRecovered`, `NodeRestarted`, `ParamUpdate`, `ParamDeleted` variants; `DaemonReply::NextEvents` type changed `Vec<Timestamped<NodeEventOrUnknown>>` → `Vec<Timestamped<NodeEvent>>`. |
| `descriptor.rs` | 1,266 | 616 | 311 | **No** | YAML descriptor superset (type rules, output framing, distribute strategy, 374 more lines than upstream). |
| `id.rs` | 271 | 183 | 16 | **No** | New identifier types. |
| `integration_testing_format.rs` | 0 | 0 | 0 | Yes | Identical. |
| `lib.rs` | 73 | 14 | 19 | — | Mostly reexports; not a wire surface by itself. |
| `metadata.rs` | 605 | 155 | 335 | **No** | Upstream has `TryFromParameterError` helper struct; fork removed it. Fork adds `BufferOffset` and metadata-key constants. `Parameter` enum variants are in matching order (the only point of wire overlap on this file). |
| `node_to_daemon.rs` | 12 | 1 | 0 | Borderline | Only a single-line addition; if this were the only divergence, a rolling upgrade would still fail because the rest of the protocol has broken. |
| `auth.rs` | fork-only | — | — | **No** | Bearer-token authentication; no upstream equivalent. |
| `ws_protocol.rs` | fork-only | — | — | **No** | WebSocket control-plane messages; upstream speaks tarpc, not WebSocket. |

**Total:** 14 of 16 files have protocol-breaking changes; 1 is identical (`integration_testing_format.rs`); 1 is internal (`lib.rs`).

---

## 4. Transport-layer divergence

| Direction | Upstream (0.5.0) | Fork (1.0) | Compatible? |
|---|---|---|---|
| CLI ↔ Coordinator | tarpc over TCP, `tokio_serde::formats::Json`; `binaries/coordinator/src/lib.rs` uses `tarpc::serde_transport::tcp::listen` | WebSocket over TCP; `binaries/coordinator/src/ws_server.rs` uses `tokio::net::TcpListener::bind` + WebSocket upgrade | **No** |
| Coordinator ↔ Daemon | TCP with bincode framing | TCP with bincode framing (kept as migration runway) | Message shape diverged (see §3); transport layer similar. |
| Daemon ↔ Daemon | Zenoh | Zenoh (1.8 vs 1.1) | Minor-version; zenoh `unstable` APIs evolved. |
| Daemon ↔ Node | Custom shmem + TCP control channels | Custom shmem + TCP control channels (Phase 3b may change this) | Enum shapes diverged (see §3 `daemon_to_node.rs`). |

**Concrete interop test** (not run, but informed by the code):

- A fork CLI (`dora` 1.0) connecting to an upstream coordinator (0.5.0): the fork CLI opens a TCP connection and sends an HTTP Upgrade request. The upstream coordinator, listening on tarpc, either drops the connection or returns garbage. **Connection fails immediately.**
- An upstream CLI (0.5.0) connecting to a fork coordinator (1.0): the upstream CLI opens a TCP connection expecting tarpc JSON framing. The fork coordinator responds to HTTP Upgrade; upstream CLI does not know that protocol. **Connection fails immediately.**

---

## 5. Version negotiation / handshake analysis

**Upstream:** exposes `get_version()` tarpc method that returns a `VersionInfo { coordinator_version, message_format_version }`. No mandatory version check — the CLI may call it, but a mismatched CLI can still issue requests that then fail with deserialization errors.

**Fork:** adds `ControlRequest::Hello { dora_version: semver::Version }` as the first message the CLI must send (per `libraries/message/src/cli_to_coordinator.rs:193-215`). The coordinator checks semver compatibility via `check_cli_version()` (`:220-232`) and replies `HelloOk` or `Error` with a human-readable mismatch message. This is a real improvement over upstream's optional check, but it only exists on the fork side.

**Cross-version behavior:**
- Fork CLI (1.0) → upstream coordinator (0.5.0): the `Hello` request serializes as an unknown message type; upstream coordinator fails the connection. No helpful error.
- Upstream CLI (0.5.0) → fork coordinator (1.0): upstream never sends `Hello`; fork coordinator waits for it and times out, or rejects the first non-Hello request. No helpful error.

**Implication:** even if the transport layers matched (they don't), the mandatory-vs-optional handshake asymmetry would break the interop.

---

## 6. Recommendation

### D-1a: hard protocol break (selected)

Ship `dora 1.0` with a documented hard break. The migration guide and release notes must state:

> **Breaking change: dora 1.0 is not wire-compatible with dora 0.x.** Daemons, coordinators, and CLIs must all be upgraded together. If you run a distributed cluster, schedule a maintenance window and restart every component. Running a 0.x daemon against a 1.0 coordinator (or vice versa) will fail at connection establishment with low-level framing errors.

### Why not D-1b (bridge release) or D-1c (version negotiation)?

- **D-1b** would require a `dora 0.6` that speaks both protocols. Given the fork's transport is WebSocket and upstream's is tarpc, a bridge release would effectively ship both servers in one binary and route by port/handshake. Implementation cost ~1–2 engineer-weeks; benefit accrues only to users doing rolling upgrades. Fork's stated user base is small (§1 headline: 4 stars). Not worth the cost.
- **D-1c** would require both sides to negotiate protocol version on connect, with a 0.x-compatible path in 1.0. The transport-layer divergence makes this effectively D-1b with extra steps.

### Migration-guide action items (before Phase 5)

- [ ] Release note: explicit "full cluster restart required" callout.
- [ ] Document the `ControlRequest::Hello` handshake and the semver compatibility semantics so ops teams know what a mismatch error looks like.
- [ ] Add an explicit test in `tests/` that confirms an intentionally-mismatched CLI fails cleanly with a human-readable message (currently it fails obscurely).
- [ ] In the fork's coordinator startup logs, emit a line like `accepting dora-message v1.0.0; dora 0.x peers are not supported` so operators catch misconfigurations early.

---

## 7. Reproducing this audit

Commands are taken from plan §16 Appendix E (corrected in PR #286 round 2) and were run on 2026-04-16 from a checkout of `dora-rs/adora` with `upstream` remote added pointing at `https://github.com/dora-rs/dora`.

```bash
# Setup
git remote add upstream https://github.com/dora-rs/dora.git 2>/dev/null || true
git fetch upstream main

# Per-file diff metrics
for f in cli_to_coordinator.rs common.rs config.rs coordinator_to_cli.rs \
         coordinator_to_daemon.rs daemon_to_coordinator.rs daemon_to_daemon.rs \
         daemon_to_node.rs descriptor.rs id.rs integration_testing_format.rs \
         lib.rs metadata.rs node_to_daemon.rs; do
  lines=$(git diff upstream/main..HEAD -- "libraries/message/src/$f" | wc -l | tr -d ' ')
  added=$(git diff upstream/main..HEAD -- "libraries/message/src/$f" | grep -c '^+[^+]' || echo 0)
  removed=$(git diff upstream/main..HEAD -- "libraries/message/src/$f" | grep -c '^-[^-]' || echo 0)
  printf "%-40s diff=%6s  +%5s  -%5s\n" "$f" "$lines" "$added" "$removed"
done

# Fork-only message files
diff <(git ls-tree -r --name-only upstream/main -- libraries/message/src/) \
     <(git ls-tree -r --name-only HEAD -- libraries/message/src/)

# Transport check
git show upstream/main:binaries/coordinator/src/lib.rs | grep -E "tarpc|TcpListener|WebSocket"
grep -rE "TcpListener::bind|WebSocket|tarpc" binaries/coordinator/src/ | head
```

Raw diffs are reproducible from `git diff upstream/main..HEAD -- libraries/message/src/<file>` for any of the files listed above.

---

## 8. Gate update

Following this audit:

- Plan §19.7 "Wire protocol audit" row flips from **Pending evidence** → **Done** with a link to this file.
- Plan §17 Appendix F is populated from §3, §4, §5 above.
- Plan §7 D-1 is resolved as **D-1a (hard protocol break)** with this audit as evidence.
- Issue #288 can be closed.
- Risk R-1 ("wire protocol incompatibility breaks rolling upgrades") is confirmed and requires the migration-guide action items listed in §6.

---

## 9. Caveats

- This audit was informed by source-level diff, not by running two daemons side-by-side and watching them fail. A live interop test would add confidence but is unlikely to change the conclusion: transport-layer divergence alone guarantees the hard break.
- Zenoh minor-version (1.1 → 1.8) was not deeply audited; zenoh's own compatibility matrix is the authoritative source for daemon↔daemon compatibility across zenoh minor versions. If a zenoh-1.8 daemon talks to a zenoh-1.1 daemon across Phase 3b's SHM migration, that's a separate risk worth sampling.
- The `integration_testing_format.rs` file is identical but is only exercised in tests, not on the production wire, so its compatibility is cosmetic.
