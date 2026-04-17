# Fault Injection Scenarios (T2.4)

**Status**: Design, not yet implemented
**Date**: 2026-04-08
**Scope**: Operational spec for expanding `tests/fault-tolerance-e2e.rs` with 8 new chaos scenarios per `plan-agentic-qa-strategy.md` Section 6.4. Each scenario has a motivation, a concrete setup, an assertion, and an invariant the system must preserve.

---

## 1. Motivation

The existing `tests/fault-tolerance-e2e.rs` has 3 tests:
- `restart_recovers_from_failure` — node panics with restart policy
- `max_restarts_limit_reached` — restart policy bounded
- `input_timeout_closes_stale_input` — timeout semantics

These test **node-level** failures. They do not test:
- **Coordinator crashes** (the single point of control)
- **Daemon crashes mid-operation** (the single point of local routing)
- **Storage corruption** (the persistence layer)
- **Network partitions** (the distributed coordination)

Without coverage in these areas, distributed-system bugs that span process boundaries can only be caught by production incidents or external audits. The 2026-03-21 audit explicitly flagged "auto-recovery broken" as a concern that neither unit tests nor the existing fault-tolerance tests would surface.

## 2. Implementation prerequisites

The existing fault-tolerance tests call `dora_daemon::Daemon::run_dataflow` **in-process** via the library API. This makes them fast but means they cannot test scenarios that require killing a component — the daemon is a library call, not a process.

**Real fault injection requires one of:**

### Option A: Subprocess-based tests (recommended for most scenarios)

Spawn `dora` binary as a subprocess, kill it with SIGKILL at specific points, then bring it back up and verify recovery. Pattern:

```rust
// Scaffolding to add:
mod subprocess {
    pub struct DoraProcess { child: Child, port: u16 }
    impl DoraProcess {
        pub async fn start(...) -> Self { ... }
        pub fn kill(&mut self) { self.child.kill().unwrap() }
        pub async fn wait_ready(&mut self, timeout: Duration) -> bool { ... }
        pub async fn restart(&mut self) { ... }
    }
}
```

This scaffolding is ~100 lines and reusable across all subprocess-based scenarios.

### Option B: Fault injection hooks (for scenarios that need exact timing)

Add conditionally-compiled `#[cfg(feature = "fault-injection")]` hooks at specific points in coordinator/daemon code that panic or delay on demand. Used by tests to trigger failures at exact semantic points (e.g., "right after sending a state catch-up message but before receiving the ack").

Example hook:
```rust
#[cfg(feature = "fault-injection")]
fault_inject::maybe_panic("coordinator::state_catchup::before_ack");
```

Requires one new crate (`dora-fault-inject`) and ~30 lines of conditional code in each target location. Invasive but enables precise scenarios.

### Recommendation

Start with Option A. Most scenarios can be implemented without fault hooks — they only need "kill the process at roughly the right time". Only escalate to Option B for scenarios that genuinely require exact-point injection (scenarios 1, 4, 6 below).

## 3. Scenarios

Each scenario has the form: **Setup → Fault → Expected recovery → Invariant**.

### 3.1 Coordinator crash mid-transaction

**Motivation**: The coordinator tracks running dataflows and daemon state. If it crashes while processing a command (e.g., `start dataflow`), does the next coordinator restart reconstruct consistent state?

**Setup**:
1. Start coordinator A, daemon B, daemon C
2. Send `dora start` for a multi-node dataflow
3. Wait until at least one node is `ready`

**Fault**: `SIGKILL` the coordinator immediately after the first node reports ready (subprocess pattern from Option A; precise timing needs Option B).

**Recovery**:
1. Restart the coordinator
2. Coordinator should reload persisted state from `coordinator-store`
3. Coordinator should re-establish connections to both daemons
4. The in-flight dataflow should either complete successfully OR be cleanly marked as failed

**Invariant**: no "zombie" state — the coordinator's view of running dataflows matches what the daemons actually have running. `dora list` after restart returns the same set of active dataflows as the daemons' local state.

**Status**: Not implemented. Requires Option A scaffolding + Option B hook for precise timing.

### 3.2 Daemon crash mid-replay

**Motivation**: When a daemon reconnects to the coordinator, the coordinator replays state (per PR #130's `handle_pruned_state_catchup_fallback`). If the daemon crashes during the replay, does a subsequent reconnect correctly resume?

**Setup**:
1. Start coordinator, daemon A, run a dataflow with persisted params
2. Kill daemon A gracefully, wait for coordinator to register disconnect
3. Restart daemon A
4. Coordinator begins state catch-up replay
5. At this point the daemon has received some `SetParam` messages but not all

**Fault**: `SIGKILL` daemon A after the third `SetParam` received during replay.

**Recovery**:
1. Restart daemon A again
2. Coordinator re-runs the state catch-up from the last ack sequence
3. Params eventually match the coordinator's state

**Invariant**: no duplicated params, no missing params, no stuck-forever replay loop. Every persisted param ends up on the daemon exactly once.

**Status**: Not implemented. Requires Option B hooks for "panic after Nth SetParam".

### 3.3 Coordinator store corruption

**Motivation**: The `coordinator-store` uses redb for persistence. If the file is corrupted (disk error, incomplete write, malicious edit), does the coordinator fail gracefully or panic?

**Setup**:
1. Start coordinator, run a dataflow that writes params to the store
2. Stop coordinator cleanly
3. Corrupt the store file: write random bytes over a random 64-byte window in the middle of the file
4. Restart coordinator

**Fault**: corruption injected between coordinator runs (file-level, no process kill needed).

**Expected behavior**: coordinator must either
(a) detect the corruption, log a clear error, and refuse to start (with an actionable message about restoring from backup), OR
(b) recover the uncorrupted records and log which ones were lost

**Invariant**: the coordinator must NOT silently accept corrupted data or panic with an unhelpful stack trace. If it starts, every record it returns must be valid.

**Status**: Not implemented. Simplest of the set — no subprocess choreography needed, just file manipulation.

### 3.4 Network partition during state catch-up

**Motivation**: The state catch-up protocol (PR #130) assumes messages eventually arrive. What happens if the network partitions between coordinator and daemon mid-replay?

**Setup**:
1. Coordinator, daemon A (both on localhost for simplicity)
2. Run a dataflow with persisted params
3. Disconnect the daemon (kill the coordinator→daemon TCP connection without killing the daemon process)
4. Coordinator marks daemon as disconnected
5. Re-establish connection → coordinator begins state catch-up

**Fault**: during the state catch-up replay, forcibly close the TCP connection again (mid-replay).

**Recovery**: on the next reconnect, state catch-up should resume from the correct ack sequence. No double-applied params.

**Invariant**: the `daemon_ack_sequence` on the coordinator matches the highest sequence the daemon has actually received.

**Status**: Not implemented. Requires subprocess + TCP-level connection manipulation. Possibly easier with a proxy process that can drop connections on demand.

### 3.5 Clock skew between nodes

**Motivation**: The HLC (hybrid logical clock) from `uhlc` is supposed to handle bounded clock skew. What if one node is 5 seconds ahead of another?

**Setup**:
1. Run two daemons on separate namespaces with `systemd-run --property=Clock=...` or equivalent to offset clocks
2. Exchange messages over Zenoh
3. Verify the HLC converges

**Fault**: pre-existing clock skew (configured at startup, not injected).

**Expected behavior**: HLC timestamps remain monotonic per daemon; cross-daemon ordering uses HLC not wall-clock; no panic or assertion failure from uhlc.

**Invariant**: for any two causally-related messages (`m1 → m2`), `hlc(m2) > hlc(m1)`, regardless of wall-clock skew.

**Status**: Not implemented. Easier on Linux (`faketime` LD_PRELOAD) than macOS.

### 3.6 Disk full during recording

**Motivation**: The `record-node` writes `.drec` files. What if the disk fills up mid-recording?

**Setup**:
1. Mount a small tmpfs (100 MB) for the recording directory
2. Run a dataflow that records to that path
3. Fill the tmpfs until write fails

**Fault**: disk-full error on recording write.

**Expected behavior**: record-node logs an error, stops recording, but does not crash the rest of the dataflow. The other nodes continue running. A partial `.drec` file should still be readable (valid prefix of records).

**Invariant**: a disk-full failure in one node must not affect upstream producers or unrelated consumers.

**Status**: Not implemented. Moderately complex; requires root (or sudo) to mount tmpfs in CI.

### 3.7 OOM kill of a node

**Motivation**: When the OOM killer takes out a node, the daemon should detect, report, and apply the restart policy.

**Setup**:
1. Run a dataflow with a Python node that grows in memory via a leak
2. OS OOM-killer picks the Python process
3. Daemon detects via process exit status

**Fault**: OOM SIGKILL (externally triggered or naturally triggered by memory allocation).

**Expected behavior**: daemon sees `SIGKILL` exit status, reports to coordinator, applies restart policy (if configured). If max_restarts exceeded, dataflow is marked failed cleanly.

**Invariant**: no zombie processes, no leaked shmem regions, no hung flume channels.

**Status**: Not implemented. The easiest way to trigger is `kill -9` from the test, not actual OOM.

### 3.8 Coordinator store write during unexpected power loss

**Motivation**: The `coordinator-store` (redb) claims crash-safe writes. Verify it actually is by simulating power loss mid-write.

**Setup**:
1. Start coordinator
2. Begin a bulk param write (100 params)
3. `SIGKILL` the coordinator mid-write (not graceful shutdown)
4. Restart coordinator
5. Check which params survived

**Fault**: SIGKILL during a transaction.

**Expected behavior**: every committed transaction is either fully present or fully absent after restart. No partial commits.

**Invariant**: redb's ACID guarantee holds in practice. Any transaction that returned `Ok(())` from `commit()` must be durable.

**Status**: Not implemented. Easy once Option A scaffolding exists.

## 4. Implementation priority

If a contractor picks this up, work in this order (easiest first, biggest value ratio):

1. **3.3 Coordinator store corruption** — no scaffolding needed, pure file manipulation. Fastest proof of value.
2. **Option A scaffolding** — the subprocess abstraction. ~100 lines. Unlocks scenarios 3.1, 3.2, 3.4, 3.6, 3.7, 3.8.
3. **3.8 Power loss during store write** — once scaffolding exists, trivial to add.
4. **3.7 OOM kill** — similar difficulty.
5. **3.1 Coordinator crash mid-transaction** — needs timing care.
6. **3.2 Daemon crash mid-replay** — needs Option B hook for precise timing.
7. **3.4 Network partition** — needs TCP-level manipulation, hardest.
8. **3.5 Clock skew** — platform-specific, lowest priority.
9. **3.6 Disk full** — requires CI environment changes (sudo or tmpfs).

**Total estimated effort**: 3-5 days of focused work by a contractor familiar with the codebase.

## 5. CI integration

Fault injection tests are **slow** (each needs ≥10 seconds of real time, and some need minutes). They should not block PRs.

**Recommended policy**:
- **Tier 1 (PR)**: do not run fault injection tests
- **Tier 2 (nightly)**: run the full fault injection suite on the main branch
- **Tier 3 (pre-release)**: re-run as part of the release-readiness report

CI workflow addition (nightly):

```yaml
fault-injection:
  name: Fault injection (nightly)
  runs-on: ubuntu-latest
  if: github.event_name == 'schedule'
  timeout-minutes: 60
  steps:
    - uses: actions/checkout@v4
    - uses: dtolnay/rust-toolchain@master
    - uses: Swatinem/rust-cache@v2
    - name: Build CLI
      run: cargo build -p dora-cli --release
    - name: Install CLI
      run: cp target/release/dora /usr/local/bin/dora
    - name: Run fault injection suite
      run: cargo test --test fault-tolerance-e2e -- --test-threads=1 --include-ignored
```

The `--include-ignored` flag lets us mark scenarios as `#[ignore]` until their implementation lands, then un-ignore as each one is completed.

## 6. Meta note on AI-authored fault tests

Fault injection tests are particularly dangerous to have AI agents write. They are easy to make pass without actually testing the right thing. A "test" that kills the coordinator, restarts it, and asserts "it's alive" does NOT test recovery — it tests that restart succeeds. The harder question — "did the state match what was there before?" — is the one that AI agents reliably fail to assert.

**Guidance for whoever implements these tests (human or agent):**

For every test, write down the **invariant before the fault** and the **invariant after the recovery**. Assert both. Reject any test that only checks "the system is up" without checking "the system is correct". Apply the anti-tautological-test pattern from Section 6 of `plan-agentic-qa-strategy.md` here specifically — fault tests are where the pattern matters most.

Every test in this doc already lists an explicit invariant. Implementers should not delete those sections.

---

## References

- [`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md) Section 6.4
- [`tests/fault-tolerance-e2e.rs`](../tests/fault-tolerance-e2e.rs) — existing 3 tests
- [`docs/plan-coordinator-ha.md`](plan-coordinator-ha.md) — HA design the above tests validate
- [`docs/audit-report-2026-03-21.md`](audit-report-2026-03-21.md) — motivation (auto-recovery broken)
- [`docs/fault-tolerance.md`](fault-tolerance.md) — user-facing fault-tolerance guide
- PR [dora-rs/adora#130](https://github.com/dora-rs/adora/pull/130) — state catch-up protocol tested by 3.2 and 3.4
