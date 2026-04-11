# Zenoh Shared Memory: Should Dora Adopt It?

## Context

dora-rs/dora PR #1378 replaces the custom shared memory implementation with Zenoh's native shared memory. Dora (our fork) currently uses the custom implementation. This document analyzes whether we should adopt the Zenoh SHM approach.

## Current Implementation (Custom SHM)

```
Node A → [allocate POSIX shmem] → [write data] → [send shmem ID to daemon]
Daemon → [map shmem] → [route to subscribers] → [track DropToken]
Node B → [map shmem] → [read data] → [report drop token done]
Daemon → [notify Node A: token released] → Node A recycles region
```

**Per node:** 4 fixed shmem regions (control, events, drop, close) + dynamic output regions
**Dependencies:** `shared_memory_extended`, `raw_sync_2` (custom fork, pinned `=0.1.5`)
**Threshold:** Messages < 4KB go over TCP, >= 4KB use shmem
**Limitations:** Single message per channel, no pipelining, 4 blocking threads per node

## Zenoh SHM Approach (PR #1378)

```
Node A → [allocate from zenoh SHM pool] → [zenoh publish on topic]
Node B → [zenoh subscribe] → [receive ZShm zero-copy ref]
         (same machine: direct SHM pointer, remote: network copy)
```

**Per node:** Own zenoh session + SHM provider (8MB pool default)
**Dependencies:** `zenoh` with `shared-memory` + `unstable` features
**No daemon in data path:** Direct node-to-node via zenoh pub/sub
**Automatic:** Cross-machine fallback handled transparently

## Comparison

| Aspect | Custom SHM | Zenoh SHM |
|--------|-----------|-----------|
| **Latency (>= 64B)** | Baseline | **35% lower** (bypasses daemon hop) |
| **Throughput (>= 2KB)** | Baseline | **3-10x better** (no DropToken overhead) |
| **Throughput (< 2KB)** | TCP (fast) | **55-80% slower** (zenoh per-msg overhead) |
| **Code complexity** | ~1200 lines + DropToken lifecycle | ~540 lines (660 lines deleted) |
| **Dependencies** | `shared_memory_extended` + `raw_sync_2 =0.1.5` (pinned) | `zenoh` (already a dep) |
| **Cross-machine** | Separate Zenoh path | **Unified** (same code path) |
| **Memory management** | Custom cache (20 regions, 256MB) | Zenoh GC + ref counting |
| **Blocking threads** | 4 per shmem node | 0 (zenoh is async) |
| **Unsafe code** | 11 unsafe blocks in channel.rs | 0 (zenoh handles it) |
| **API stability** | Stable (our code) | **`unstable` feature flag** in zenoh |
| **Recording/replay** | Works (data goes through daemon) | **Broken** (ZenohShmInput events skipped) |
| **First message** | Normal | **16x latency spike** (session init) |
| **memlock requirement** | No | **Yes** (`ulimit -l unlimited`) |

## Analysis

### Strong Reasons to Adopt

1. **35% lower latency, 3-10x better throughput** for typical robotics payloads (camera frames, point clouds, tensors are all > 4KB). This directly supports our "10-17x faster than ROS2" claim.

2. **Eliminate 660 lines of DropToken lifecycle code** — the entire `drop_stream.rs`, DropToken tracking in daemon and node, `pending_drop_tokens`, `check_drop_token`. This is the most complex and error-prone part of the current IPC. Our audit found multiple issues here (unbounded drop channels, drop token race conditions).

3. **Remove `raw_sync_2 =0.1.5` pinned dependency** — this is an exact-version pin on a custom fork with no upstream maintenance. It blocks shmem upgrades and is a supply-chain risk.

4. **Remove `shared_memory_extended`** — another niche dependency with limited maintenance.

5. **Eliminate 4 blocking threads per shmem node** — currently each node spawns 4 `spawn_blocking` threads for shmem listener loops. Zenoh is fully async.

6. **Unified local/remote data path** — currently we have separate code paths for local (shmem + daemon routing) and remote (Zenoh pub/sub). With Zenoh SHM, it's one path.

7. **Aligns with upstream dora** — easier to track upstream improvements and merge back.

### Concerns

1. **Zenoh `unstable` feature** — the SHM API is not stabilized. It could change in zenoh 2.0. However, we already depend on zenoh heavily, and the SHM API has been stable since zenoh 0.10 (2023).

2. **Small message regression** — 55-80% slower for < 2KB. Mitigated by keeping the existing threshold (4KB) — messages below it continue via the daemon's TCP/shmem path. Or adopt the `DORA_ZERO_COPY_THRESHOLD` approach from the PR.

3. **Recording broken** — the PR skips ZenohShmInput events to avoid copying SHM data. We'd need to add a recording path that copies data before forwarding, or record at the daemon level (data-less notifications still flow through daemon).

4. **memlock requirement** — deployment environments need `ulimit -l unlimited` or `CAP_IPC_LOCK`. This is already recommended in our `plan-soft-realtime.md` for RT deployments. For `dora run` (local dev), we can handle this gracefully with fallback.

5. **First-message latency spike** — 16x on first message due to zenoh session init. Can be mitigated by pre-warming the session during node startup.

6. **Each node opens own zenoh session** — more OS resources (sockets, threads) per node. Acceptable for typical dataflows (5-50 nodes) but could matter for very large deployments.

## Recommendation: **Adopt, with threshold guard**

The performance gains are compelling (35% lower latency, 3-10x throughput for robotics payloads) and the code simplification is substantial (660 lines of DropToken lifecycle eliminated, 11 unsafe blocks removed). The concerns are all manageable:

- Keep 4KB threshold for small messages (TCP path unchanged)
- Add recording support by copying shmem data at the record-node level
- Handle memlock gracefully with fallback + warning
- Pre-warm zenoh session during node init

## Implementation Plan

### Phase 1: Zenoh SHM for Node-to-Node Data (Port PR #1378)

**Scope:** Replace the data-plane shmem path. Keep the 4 control channels (control, events, drop, close) on the existing shmem for now — they carry small messages and work fine.

1. **Node API:** Add zenoh session + ShmProvider to `DoraNode`
2. **Send path:** `send_output` publishes via zenoh when data >= threshold
3. **Receive path:** Node subscribes to zenoh topics for each input; receives `ZShm` zero-copy
4. **Daemon:** Remove DropToken tracking. Daemon receives data-less notifications for routing awareness.
5. **Delete:** `drop_stream.rs`, DropToken types, `pending_drop_tokens`, `check_drop_token`

### Phase 2: Recording Support

Add a recording interceptor that copies shmem data before it's consumed:
- Record-node subscribes to the same zenoh topics
- Copies `ZShm` to `Vec<u8>` for `.adorec` file
- No change to normal data path (recording is a subscriber like any other)

### Phase 3: Control Channel Migration (Optional, v0.3+)

Replace the 4 per-node shmem control channels with zenoh too:
- Eliminates `shared_memory_extended` and `raw_sync_2` entirely
- Control messages are small — use zenoh without SHM (normal pub/sub)
- Removes all custom shmem code from the codebase

## Key Files to Change

| File | Change |
|------|--------|
| `apis/rust/node/src/node/mod.rs` | Add zenoh session, ShmProvider, publishers |
| `apis/rust/node/src/node/drop_stream.rs` | **Delete** |
| `apis/rust/node/src/event_stream/mod.rs` | Add zenoh subscribers per input |
| `apis/rust/node/src/event_stream/data_conversion.rs` | Add `RawData::ZenohShm(ZShm)` |
| `binaries/daemon/src/lib.rs` | Remove DropToken tracking |
| `libraries/message/src/common.rs` | Remove `DropToken`, simplify `DataMessage` |
| `libraries/message/src/daemon_to_node.rs` | Remove `NodeDropEvent` |
| `libraries/core/src/topics.rs` | Add zenoh topic helpers |

## Risks

- **Zenoh unstable API change** — monitor zenoh releases; pin to `~1.8` (already done)
- **Small message regression** — threshold guard mitigates
- **Breaking change for custom node implementations** — `NodeEvent::Input.data` type changes from `Option<Arc<DataMessage>>` to a new enum including `ZShm`. All downstream consumers need updating.
- **Python API** — PyArrow integration with zenoh SHM needs verification (the PR handles this for dora)

## References

- PR: https://github.com/dora-rs/adora/pull/1378
- Zenoh SHM docs: https://zenoh.io/docs/manual/shmem/
- Benchmarks: In PR comments by phil-opp
