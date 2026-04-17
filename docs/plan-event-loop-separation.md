# Event Loop: Control/Data Plane Separation

## Context

The daemon's single `while let Some(event) = events.next().await` loop processes both data-plane events (node messages, Zenoh publish) and control-plane events (metrics, heartbeat, health checks, coordinator commands). The main bottleneck: **Zenoh `publisher.put().await` is called inline** in the hot path. At 1000 msg/sec with 5 remote receivers, that's 5000 Zenoh awaits/sec blocking the entire event loop.

Impact: `dora stop` under load takes 2-5s. Metrics collection causes 50-100ms latency spikes.

## Current Event Loop (line 560, daemon/lib.rs)

```
                        ┌──────────────────────┐
                        │   Single Event Loop   │
                        │                       │
Hot:  SendOut ──────────┤  route local (fast)   │
      InterDaemon ──────┤  route local (fast)   │
      Timer ────────────┤  distribute (fast)    │
                        │                       │
      SendOut ──────────┤  Zenoh pub AWAIT ◀──── BLOCKS everything
                        │                       │
Cold: MetricsInterval ──┤  sysinfo AWAIT ◀────── BLOCKS 30-50ms
      Heartbeat ────────┤  WS send (fast)       │
      HealthCheck ──────┤  scan (fast)          │
      Coordinator ──────┤  spawn/inline (mixed) │
                        └──────────────────────┘
```

## Approach: Offload Two Blockers

Keep the single-threaded event loop for data routing (fast, no locking). Offload the two main blockers to separate tasks.

```
                        ┌──────────────────────┐
                        │   Main Event Loop     │
                        │   (never blocks)      │
Hot:  SendOut ──────────┤  route local (fast)   │
      InterDaemon ──────┤  route local (fast)   │
      Timer ────────────┤  distribute (fast)    │
                        │                       │
      SendOut ──────────┤  try_send to channel ─┼──► Zenoh Publish Task
                        │                       │    (dedicated, bounded 256)
Cold: MetricsInterval ──┤  snapshot + spawn ────┼──► Metrics Task
      HealthCheck ──────┤  scan (fast)          │    (fire-and-forget)
      Coordinator ──────┤  spawn/inline         │
                        └──────────────────────┘
```

## Step 1: Zenoh Publish Channel (highest impact)

**Problem:** `publisher.put(data).await` at line 2371 blocks per-message.

**Fix:** Send to a bounded channel; dedicated task does the actual Zenoh I/O.

### Changes

**`running_dataflow.rs`:** Change publisher storage:
```rust
// Before:
pub publishers: BTreeMap<OutputId, zenoh::pubsub::Publisher<'static>>
// After:
pub publishers: BTreeMap<OutputId, Arc<zenoh::pubsub::Publisher<'static>>>
```

**`lib.rs` — Daemon struct:** Add channel:
```rust
zenoh_tx: mpsc::Sender<ZenohOutbound>,
```

**`lib.rs` — run_general():** Spawn drain task:
```rust
let (zenoh_tx, mut zenoh_rx) = mpsc::channel::<ZenohOutbound>(256);
tokio::spawn(async move {
    while let Some(msg) = zenoh_rx.recv().await {
        if let Err(e) = msg.publisher.put(msg.serialized).await {
            tracing::error!("zenoh publish failed: {e}");
        }
        msg.net_bytes_sent.fetch_add(msg.payload_len, Relaxed);
        msg.net_messages_sent.fetch_add(1, Relaxed);
    }
});
```

**`lib.rs` — send_to_remote_receivers():** Replace inline await:
```rust
// Before:
publisher.put(serialized_event).await?;
// After:
let _ = self.zenoh_tx.try_send(ZenohOutbound {
    publisher: publisher.clone(), // Arc clone = cheap
    serialized: serialized_event,
    ...
});
```

**Backpressure:** `try_send` + drop on `Full` with warning (consistent with local routing). 256-message buffer absorbs bursts.

## Step 2: Fire-and-Forget Metrics (removes 50ms spikes)

**Problem:** `collect_and_send_metrics().await` blocks inline for spawn_blocking sysinfo.

**Fix:** Snapshot PIDs + Arc counters, spawn detached task.

### Changes

**`lib.rs` — MetricsInterval handler:**
```rust
Event::MetricsInterval => {
    if let Some(sys) = self.metrics_system.take() {
        let snapshot = self.snapshot_for_metrics(); // clone Arcs, PIDs
        tokio::spawn(async move {
            // Heavy sysinfo work off event loop
            collect_metrics_background(sys, snapshot, ...).await;
        });
    }
    // If None, previous collection still running — skip this interval
}
```

**`lib.rs` — New event variant:**
```rust
Event::MetricsSystemReturn(sysinfo::System)
```
Spawned task sends the `System` back for reuse (delta CPU calculations need state).

## Step 3: Background Heartbeat (optional, minor)

Move heartbeat serialization + WS send to a spawned interval task. Low priority — already fast via channel push.

## Key Files

| File | Change |
|------|--------|
| `binaries/daemon/src/lib.rs` | Zenoh channel, metrics fire-and-forget, ZenohOutbound struct |
| `binaries/daemon/src/running_dataflow.rs` | `Arc<Publisher>` in publishers map |

## Expected Impact

| Metric | Before | After |
|--------|--------|-------|
| Zenoh publish blocking | 1-10ms per message | 0 (channel push) |
| Metrics latency spike | 50-100ms | 0 (background) |
| `dora stop` under load | 2-5s | <500ms |
| 100ms warnings | frequent at >100 Hz | rare |

## Risks

- **Message ordering**: Single Zenoh drain task preserves FIFO per channel. OK.
- **`Arc<Publisher>`**: Zenoh 1.8 `put(&self)` works through Arc. Verified.
- **Backpressure change**: Implicit (block) -> explicit (drop on Full). Consistent with local routing.
- **Metrics System reuse**: `take`/`put-back` via event variant. If task panics, next interval creates fresh System.

## Verification

1. Existing smoke tests pass (end-to-end data flow unchanged)
2. Benchmark: event loop throughput with simulated Zenoh delay
3. Manual: `dora stop` responds <500ms under 1000 Hz load
4. Manual: no `Daemon took >100ms` warnings during metrics
