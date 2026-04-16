# Fault Tolerance

Dora provides built-in fault tolerance for robotic and AI dataflows. Nodes can automatically restart on failure, detect stale upstream connections, gracefully degrade when inputs are unavailable, and the coordinator can persist state to disk so it survives crashes and restarts.

## Features at a Glance

| Feature | Scope | Config |
|---------|-------|--------|
| Restart policies | Per-node | `restart_policy`, `max_restarts`, `restart_delay`, ... |
| Health monitoring | Per-node | `health_check_timeout`, `health_check_interval` (dataflow-level) |
| Input timeouts | Per-input | `input_timeout` |
| Circuit breaker | Automatic | Triggered by `input_timeout`, auto-recovers |
| NodeRestarted event | Downstream nodes | Automatic when upstream restarts |
| InputTracker API | Rust nodes | `dora_node_api::InputTracker` |
| Observability | Daemon-wide | Atomic counters logged periodically |
| Distributed health | Multi-daemon | Coordinator heartbeat monitoring |
| Coordinator state persistence | Coordinator | `--store redb` (requires `redb-backend` feature) |

---

## Restart Policies

Control what happens when a node exits or crashes.

### Configuration

```yaml
nodes:
  - id: my-node
    path: ./target/debug/my-node
    restart_policy: on-failure  # never | on-failure | always
    max_restarts: 5             # 0 = unlimited (default: 0)
    restart_delay: 1.0          # initial delay in seconds
    max_restart_delay: 30.0     # cap for exponential backoff
    restart_window: 300.0       # reset counter after this many seconds
```

### Policy Types

**`never`** (default) -- Node is not restarted. Failure propagates normally.

**`on-failure`** -- Restart only when the node exits with a non-zero exit code. Clean exits (code 0) are not restarted.

**`always`** -- Restart on any exit, except:
- The dataflow was stopped by the user (`dora stop` or Ctrl-C)
- All inputs were closed and the node exited with a non-zero code

### How Restarts Work Internally

When a node process exits, the daemon evaluates the restart decision in this order:

1. **Policy check**: Does the restart policy allow it?
   - `Never` -> no restart
   - `OnFailure` -> restart only if exit code != 0
   - `Always` -> restart
2. **Disable check**: Has `disable_restart` been set? (set when all inputs close or during manual stop via `stop_all`)
3. **Window check**: If `restart_window` is set and the window has elapsed since the first restart, reset the counter to 0
4. **Limit check**: If `max_restarts > 0` and the window counter exceeds it, give up permanently
5. **Backoff**: If `restart_delay` is set, sleep for the computed delay (re-checking `disable_restart` after waking)
6. **Respawn**: The node process is spawned fresh with the same configuration

The daemon tracks restart state per node instance in the `spawn/prepared.rs` lifecycle loop. Each node runs in its own tokio task, so restarts don't block other nodes.

### Backoff

When `restart_delay` is set, the daemon waits before restarting. The delay doubles on each attempt (exponential backoff) and is capped by `max_restart_delay`.

The backoff exponent is capped at 16 internally to prevent overflow (`2^16 = 65536x multiplier`).

Example with `restart_delay: 1.0` and `max_restart_delay: 10.0`:
```
Attempt 1: wait 1s    (1.0 * 2^0)
Attempt 2: wait 2s    (1.0 * 2^1)
Attempt 3: wait 4s    (1.0 * 2^2)
Attempt 4: wait 8s    (1.0 * 2^3)
Attempt 5: wait 10s   (capped at max_restart_delay)
Attempt 6: wait 10s   (capped)
```

During the backoff sleep, the daemon continuously monitors the `disable_restart` flag. If all inputs close while the node is waiting to restart, the restart is cancelled with the log message: "restart cancelled: inputs closed during backoff wait".

### Restart Window

When `restart_window` is set, the restart counter resets after the window elapses (measured from the first restart in the current window). This enables "N restarts per M seconds" semantics.

Example: `max_restarts: 5`, `restart_window: 300.0` means "at most 5 restarts per 5 minutes". If the window elapses without hitting the limit, the counter resets and the node gets another 5 attempts.

### Restart Disable During Shutdown

When the daemon stops a dataflow (via `stop_all`), it calls `disable_restart()` on every node **before** sending Stop events. This prevents the restart mechanism from fighting the shutdown process. The `disable_restart` flag is an `Arc<AtomicBool>` shared between the daemon event loop and the node's spawn lifecycle task.

### NodeRestarted Event

When a node restarts, the daemon sends a `NodeRestarted` event to all downstream nodes that consume its outputs. This allows downstream nodes to:

- Reset internal state or caches
- Log the upstream recovery
- Re-initialize connections or sessions

The event carries the `NodeId` of the restarting node. Downstream nodes receive it automatically via the event stream:

```rust
match event {
    Event::NodeRestarted { id } => {
        println!("upstream node {id} restarted, resetting state");
        // Clear any cached state from the old node instance
    }
    _ => {}
}
```

The daemon finds downstream nodes via `dataflow.mappings`, which maps each node's outputs to all subscribing (receiver_node, input_id) pairs. Each unique receiver gets one `NodeRestarted` event per restart.

---

## Health Monitoring

Passive monitoring detects hung nodes that stop communicating with the daemon.

```yaml
health_check_interval: 2.0  # seconds (default: 5.0, dataflow-level)
nodes:
  - id: my-node
    path: ./target/debug/my-node
    health_check_timeout: 30.0  # seconds (per-node)
    restart_policy: on-failure
```

### Configurable Health Check Interval

The `health_check_interval` is a **dataflow-level** setting that controls how often the daemon checks node health. Default is 5.0 seconds. Lower values detect hung nodes faster but add more overhead. Set this at the top level of your dataflow YAML, not per-node.

### How It Works Internally

The daemon runs a health check sweep at the configured `health_check_interval` (via a tokio interval stream emitting `Event::NodeHealthCheckInterval`).

Each `RunningNode` has a `last_activity: Arc<AtomicU64>` field storing the timestamp (milliseconds since epoch) of the last communication. This is updated atomically by the node's communication handler (`node_communication/mod.rs`) every time the node sends any request to the daemon (event subscriptions, output sends, etc.).

The health check function (`check_node_health`) iterates all running nodes:

1. Skip nodes without `health_check_timeout` set
2. Skip nodes with `last_activity == 0` (not yet connected)
3. Compute `elapsed_ms = now - last_activity`
4. If `elapsed_ms > timeout_ms`, log a warning and **kill** the node process

After killing, the normal exit handling runs, which evaluates the restart policy. This means `health_check_timeout` combined with `restart_policy: on-failure` automatically recovers hung nodes.

### What Counts as "Activity"

Any message from the node to the daemon counts:
- Event subscription requests
- Output data sends (via shared memory or TCP)
- Timer tick acknowledgments

Normal input data received from other nodes does **not** reset the timer -- the node must actively communicate with the daemon.

---

## Input Timeouts and Circuit Breaker

Per-input timeouts detect when an upstream node stops producing data.

### Configuration

```yaml
nodes:
  - id: downstream-node
    path: ./target/debug/downstream
    inputs:
      sensor_data:
        source: camera-node/frames
        input_timeout: 5.0  # seconds
```

The `input_timeout` is set per input, not per node. Different inputs can have different timeouts.

### How It Works Internally

The daemon maintains an `InputDeadline` for each input with a timeout:

```
struct InputDeadline {
    timeout: Duration,        // configured timeout
    last_received: Instant,   // last time data arrived
}
```

These are stored in `RunningDataflow.input_deadlines` keyed by `(NodeId, DataId)`.

**Timeout detection** runs during the same 5-second health check interval. The `check_input_timeouts` function:

1. Scans all `input_deadlines` entries
2. If `last_received.elapsed() > timeout`, the input is "broken"
3. The `(node_id, input_id)` pair is moved from `input_deadlines` to `broken_inputs`
4. The daemon calls `break_input()` which sends `InputClosed { id }` to the downstream node
5. If all of a node's inputs are now closed (and none are broken/recoverable), `AllInputsClosed` is sent and the node's restart is disabled

**Deadline reset**: Every time data arrives on an input, its `last_received` is reset to `Instant::now()`.

### Circuit Breaker: Auto-Recovery

The circuit breaker tracks broken inputs in `RunningDataflow.broken_inputs`. When new data arrives on a broken input:

1. The data is delivered to the node normally
2. The `broken_inputs` entry is removed
3. The input is re-added to `open_inputs`
4. A new `InputDeadline` is created (restarting the timeout)
5. An `InputRecovered { id }` event is sent to the node
6. The `circuit_breaker_recoveries` counter is incremented

This means recovery is fully automatic. If the upstream node restarts (via restart policy) and begins producing data again, downstream nodes seamlessly resume receiving it.

### Node-Side Handling

In Rust nodes, handle these events in your event loop:

```rust
use dora_node_api::{DoraNode, Event};

let (mut node, mut events) = DoraNode::init_from_env()?;
while let Some(event) = events.recv() {
    match event {
        Event::Input { id, data, .. } => {
            // Normal processing
        }
        Event::InputClosed { id } => {
            // Upstream stopped producing on this input.
            // You can: use cached data, skip processing, alert operator, etc.
        }
        Event::InputRecovered { id } => {
            // Upstream is back online for this input.
            // Resume normal processing.
        }
        Event::Stop(_) => break,
        _ => {}
    }
}
```

---

## InputTracker API (Rust)

The `InputTracker` helper tracks input health and caches the last received value per input, making graceful degradation easy.

```rust
use dora_node_api::{DoraNode, Event, InputTracker, InputState};

let (mut node, mut events) = DoraNode::init_from_env()?;
let mut tracker = InputTracker::new();

while let Some(event) = events.recv() {
    tracker.process_event(&event);

    match event {
        Event::Input { id, data, .. } => {
            // Fresh data available
        }
        Event::InputClosed { id } => {
            // Input timed out -- fall back to cached data
            if let Some(stale_data) = tracker.last_value(&id) {
                // Use stale_data as fallback
            }
        }
        Event::Stop(_) => break,
        _ => {}
    }

    // Check overall health
    if tracker.any_closed() {
        let closed: Vec<_> = tracker.closed_inputs();
        // Log or adjust behavior
    }
}
```

### Internal Design

`InputTracker` maintains two `HashMap`s:

- `states: HashMap<DataId, InputState>` -- current state per input (Healthy or Closed)
- `cache: HashMap<DataId, ArrowData>` -- last received value per input

On `Event::Input`, both maps are updated (state = Healthy, cache = data clone). On `Event::InputClosed`, only state changes (cache is preserved). On `Event::InputRecovered`, state is set back to Healthy. The cache is never cleared, so `last_value()` always returns the most recent data even after the input closes.

Note: `ArrowData` wraps `Arc<dyn arrow::array::Array>`, so the cache clone is reference-counted (cheap).

### API Reference

| Method | Returns | Description |
|--------|---------|-------------|
| `new()` | `InputTracker` | Create empty tracker |
| `process_event(&Event)` | `bool` | Update state. Returns true if event was relevant |
| `state(&DataId)` | `Option<InputState>` | Current state (Healthy or Closed) |
| `is_closed(&DataId)` | `bool` | Check if input is closed |
| `last_value(&DataId)` | `Option<&ArrowData>` | Last received value (available even when closed) |
| `closed_inputs()` | `Vec<&DataId>` | All currently closed inputs |
| `any_closed()` | `bool` | True if any tracked input is closed |

---

## Observability

The daemon tracks fault tolerance events with atomic counters (`FaultToleranceStats`) and logs a summary every 5 seconds during the health check interval.

### Counters

| Counter | Type | Incremented when |
|---------|------|-----------------|
| `restarts` | `AtomicU64` | A node restart is initiated (in spawn lifecycle) |
| `health_check_kills` | `AtomicU64` | A node is killed by the health check (unresponsive) |
| `input_timeouts` | `AtomicU64` | An input timeout fires (circuit breaker trips) |
| `circuit_breaker_recoveries` | `AtomicU64` | Data arrives on a broken input (auto-recovery) |

All counters use `Ordering::Relaxed` since they are informational and don't need strict ordering guarantees.

### Log Output

When any counter is non-zero, the daemon emits a structured log line:

```
INFO fault tolerance stats restarts=3 health_kills=0 input_timeouts=1 cb_recoveries=1
```

These counters are cumulative for the lifetime of the daemon process. They are not reset between dataflows.

---

## Distributed Health

In multi-daemon deployments, the coordinator monitors daemon heartbeats.

### Protocol

- **Heartbeat interval**: 3 seconds (coordinator sends heartbeat to each daemon)
- **Disconnect threshold**: 30 seconds without a response
- **Detection**: On each heartbeat sweep, the coordinator removes daemons that haven't responded within the threshold
- **Notification**: The coordinator broadcasts `PeerDaemonDisconnected { daemon_id }` to all remaining daemons

### DaemonInfo

The `ConnectedMachines` CLI query returns `Vec<DaemonInfo>`:

```rust
pub struct DaemonInfo {
    pub daemon_id: DaemonId,
    pub last_heartbeat_ago_ms: u64,  // milliseconds since last heartbeat
}
```

This allows monitoring tools to detect daemons that are alive but slow to respond.

### Daemon-Side Handling

When a daemon receives `PeerDaemonDisconnected`, it logs a structured warning:

```
WARN peer daemon disconnected daemon_id=machine-B
```

Currently this is informational. Future work may include automatic migration of nodes from the disconnected daemon.

---

## Coordinator State Persistence

By default the coordinator holds all state in memory. If the coordinator process crashes or is restarted, all knowledge of running dataflows is lost -- daemons continue running but become orphaned, and users must manually re-run dataflows.

The **redb store backend** solves this by persisting coordinator state to a single file on disk using [redb](https://docs.rs/redb), a pure-Rust embedded key-value store with copy-on-write B-trees that are crash-safe by design.

### Design: Stateless Coordinator with Stateful Backend

The coordinator itself remains stateless in the K8s sense -- it can be stopped and restarted at any time. All durable state lives in the store backend behind the `CoordinatorStore` trait:

```
Coordinator (stateless process)
    |
    v
CoordinatorStore trait
    |
    +-- InMemoryStore (default, no persistence)
    +-- RedbStore     (persists to ~/.dora/coordinator.redb)
```

This separation means:
- The coordinator event loop never reads from the filesystem during normal operation (only at startup recovery)
- All state mutations are written to the store at well-defined persistence points
- The store can be swapped without changing coordinator logic

### Enabling Persistence

```bash
# Use default path (~/.dora/coordinator.redb)
dora coordinator --store redb

# Use custom path
dora coordinator --store redb:/path/to/coordinator.redb

# Default: in-memory only (no persistence)
dora coordinator --store memory
```

The `redb` backend requires the `redb-backend` Cargo feature, which is enabled in the default CLI build.

### What Is Persisted

The store tracks three record types:

| Record | Key | Persisted Fields |
|--------|-----|-----------------|
| `DataflowRecord` | UUID (16 bytes) | uuid, name, descriptor (JSON), status, daemon IDs, generation counter, created/updated timestamps |
| `BuildRecord` | UUID (16 bytes) | build ID, status, errors, created/updated timestamps |
| `DaemonInfo` | DaemonId (bincode) | daemon ID, machine ID |

Records are serialized with [bincode](https://docs.rs/bincode/2) for compact, fast encoding.

### Dataflow Status Lifecycle

The coordinator persists dataflow status at every state transition:

```
Start command     -->  Pending
All daemons ready -->  Running
Stop command      -->  Stopping
All nodes finish  -->  Succeeded  or  Failed { error }
Spawn failure     -->  Failed { error: "spawn failed: ..." }
```

Each persist call increments the record's `generation` counter, providing a monotonic version for conflict detection.

### Persistence Points

The coordinator writes to the store at these moments in the event loop:

1. **Dataflow started** (`ControlRequest::Start`) -- record created with status `Pending`
2. **Dataflow spawned** (`DataflowSpawnResult` success from all daemons) -- updated to `Running`
3. **Spawn failed** (`DataflowSpawnResult` error) -- updated to `Failed` with the actual error message
4. **Stop requested** (`ControlRequest::Stop` or `StopByName`) -- updated to `Stopping`
5. **All nodes finished** (`DataflowFinishedOnDaemon`) -- updated to `Succeeded` or `Failed` with per-node error details
6. **Graceful shutdown** (Ctrl-C or `Destroy` command) -- all running dataflows marked `Stopping` before stop messages are sent

If a store write fails, the coordinator logs a warning and continues operating with in-memory state. This prevents a store failure from blocking the dataflow lifecycle.

### Startup Recovery

When the coordinator starts with a redb store that contains data from a previous run, it performs recovery:

1. Read all persisted dataflow records via `store.list_dataflows()`
2. For any record with a non-terminal status (`Pending`, `Running`, `Stopping`):
   - Mark it as `Failed { error: "coordinator restarted" }`
   - Increment the generation counter
   - Write the updated record back to the store
3. Terminal records (`Succeeded`, `Failed`) are left unchanged

This ensures that stale dataflows from a crashed coordinator are not confused with actively running ones. The daemons that were running those dataflows will detect the coordinator disconnect independently.

### Error Detail Preservation

When a dataflow fails, the `Failed` status includes the actual per-node error messages rather than a generic string:

```
Failed { error: "node-1: exited with code 137; node-2: failed to spawn node: binary not found" }
```

Errors are collected from `DataflowDaemonResult.node_results` across all daemons, formatted as `node_id: error_message`, and joined with `; `.

### Schema Versioning

The redb database includes a `meta` table with a `schema_version` key. On open:

- If no version exists (fresh database), the current version is written
- If the stored version matches the binary's version, the database opens normally
- If there is a mismatch, the database is rejected with an error

This prevents silent data corruption when the serialization format of stored records changes between Dora versions. The current schema version is `1`.

### File Security

On Unix systems:
- The database file is set to `0600` (owner read/write only) after creation
- The default directory (`~/.dora/`) is set to `0700` (owner only)
- Custom paths provided via `redb:/path` are validated to reject `..` components

### Internal Architecture

```rust
// Store trait (libraries/coordinator-store/src/lib.rs)
pub trait CoordinatorStore: Send + Sync {
    fn put_dataflow(&self, record: &DataflowRecord) -> Result<()>;
    fn get_dataflow(&self, uuid: &Uuid) -> Result<Option<DataflowRecord>>;
    fn list_dataflows(&self) -> Result<Vec<DataflowRecord>>;
    fn delete_dataflow(&self, uuid: &Uuid) -> Result<()>;
    // ... daemon and build methods
}
```

The `RedbStore` implementation uses three redb tables (`daemons`, `dataflows`, `builds`) with UUID-based binary keys and bincode-serialized values. All operations are synchronous (redb is a synchronous library); the coordinator calls them directly from the async event loop since they are fast in-process operations.

A bincode deserialization limit of 64 MiB guards against corrupted data that could encode huge allocation sizes in length prefixes.

---

## Complete YAML Reference

```yaml
# Dataflow-level settings
health_check_interval: 2.0    # health check sweep interval (default: 5.0s)

nodes:
  - id: sensor-node
    path: ./target/debug/sensor
    inputs:
      tick: dora/timer/millis/100
    outputs:
      - frames

  - id: processor
    path: ./target/debug/processor

    # Restart policy
    restart_policy: on-failure    # never | on-failure | always
    max_restarts: 5               # 0 = unlimited
    restart_delay: 1.0            # initial backoff delay (seconds)
    max_restart_delay: 30.0       # max backoff cap (seconds)
    restart_window: 300.0         # reset counter after N seconds

    # Health monitoring
    health_check_timeout: 30.0    # kill if no activity for N seconds

    inputs:
      frames:
        source: sensor-node/frames
        input_timeout: 5.0        # circuit breaker timeout (seconds)
        queue_size: 10            # input buffer size (default: 10)
    outputs:
      - result
```

---

## Use Case Scenarios

### 1. Camera Pipeline with Intermittent Hardware Failures

A camera driver node occasionally crashes due to USB disconnects. The processing pipeline should survive these outages and resume when the camera reconnects.

```yaml
nodes:
  - id: camera-driver
    path: ./target/debug/camera-driver
    restart_policy: on-failure
    max_restarts: 0               # unlimited -- hardware failures are expected
    restart_delay: 2.0            # wait for USB to re-enumerate
    max_restart_delay: 30.0
    inputs:
      tick: dora/timer/millis/33  # ~30 FPS
    outputs:
      - frames

  - id: object-detector
    path: ./target/debug/detector
    inputs:
      frames:
        source: camera-driver/frames
        input_timeout: 5.0        # tolerate 5s camera outage
    outputs:
      - detections

  - id: planner
    path: ./target/debug/planner
    inputs:
      detections:
        source: object-detector/detections
        input_timeout: 10.0       # longer tolerance -- can plan with stale data
      lidar:
        source: lidar-driver/points
        input_timeout: 3.0
```

**What happens when the camera crashes:**

1. `camera-driver` exits with non-zero code
2. Daemon evaluates `on-failure` policy -> restart after 2s backoff
3. During the outage, `object-detector` receives `InputClosed { id: "frames" }` after 5s
4. `planner` receives `InputClosed { id: "detections" }` after 10s
5. Camera restarts, begins producing frames
6. `object-detector` receives new frame data + `InputRecovered { id: "frames" }` (circuit breaker recovers)
7. `planner` receives detections + `InputRecovered { id: "detections" }`

**Node-side handling in the planner:**

```rust
use dora_node_api::{DoraNode, Event, InputTracker};

let (mut node, mut events) = DoraNode::init_from_env()?;
let mut tracker = InputTracker::new();

while let Some(event) = events.recv() {
    tracker.process_event(&event);

    match event {
        Event::Input { id, data, .. } => match id.as_ref() {
            "detections" => plan_with_detections(&data),
            "lidar" => update_lidar_map(&data),
            _ => {}
        },
        Event::InputClosed { id } => match id.as_ref() {
            "detections" => {
                // Camera pipeline down -- plan with lidar only
                plan_lidar_only();
            }
            "lidar" => {
                // LiDAR down -- use last known detection data
                if let Some(stale) = tracker.last_value(&"detections".into()) {
                    plan_with_stale_detections(stale);
                }
            }
            _ => {}
        },
        Event::Stop(_) => break,
        _ => {}
    }
}
```

### 2. ML Inference Node with OOM Crashes

An ML inference node occasionally runs out of memory on large inputs. It should restart quickly but give up after repeated failures (indicating a systemic issue).

```yaml
nodes:
  - id: ml-inference
    path: ./target/debug/ml-inference
    restart_policy: on-failure
    max_restarts: 3
    restart_delay: 0.5
    restart_window: 60.0          # 3 restarts per minute
    health_check_timeout: 60.0    # ML inference can be slow
    inputs:
      images:
        source: preprocessor/images
    outputs:
      - predictions
```

**Behavior:**
- Node crashes from OOM -> restarts after 0.5s
- Crashes again on another large input -> restarts after 1.0s
- Crashes a third time -> restarts after 2.0s
- Crashes a fourth time within 60s -> `max_restarts` exceeded, node fails permanently
- If the node runs stably for 60s after the first crash, the restart window resets and it gets 3 more chances

### 3. Multi-Sensor Fusion with Graceful Degradation

A robot fuses data from multiple sensors. Individual sensors may fail, but the system should continue operating with reduced capability.

```yaml
nodes:
  - id: sensor-fusion
    path: ./target/debug/sensor-fusion
    inputs:
      camera:
        source: camera-node/frames
        input_timeout: 3.0
      lidar:
        source: lidar-node/points
        input_timeout: 3.0
      imu:
        source: imu-node/readings
        input_timeout: 1.0        # IMU is critical, short timeout
      gps:
        source: gps-node/fix
        input_timeout: 10.0       # GPS can be intermittent
    outputs:
      - fused-state
```

**Node-side with InputTracker:**

```rust
use dora_node_api::{DoraNode, Event, InputTracker};

let (mut node, mut events) = DoraNode::init_from_env()?;
let mut tracker = InputTracker::new();

while let Some(event) = events.recv() {
    tracker.process_event(&event);

    match event {
        Event::Input { id, data, .. } => {
            // Process fresh data from any sensor
            update_sensor(&id, &data);
            compute_and_send_fusion(&mut node, &tracker);
        }
        Event::InputClosed { id } => {
            // Sensor went offline -- adjust fusion weights
            eprintln!("sensor {id} offline, degrading");
            compute_and_send_fusion(&mut node, &tracker);
        }
        Event::InputRecovered { id } => {
            // Sensor back online
            eprintln!("sensor {id} recovered");
        }
        Event::Stop(_) => break,
        _ => {}
    }
}

fn compute_and_send_fusion(node: &mut DoraNode, tracker: &InputTracker) {
    // Use fresh data where available, stale cache for degraded sensors
    let camera = tracker.last_value(&"camera".into());
    let lidar = tracker.last_value(&"lidar".into());
    let imu = tracker.last_value(&"imu".into());

    if tracker.is_closed(&"imu".into()) {
        // IMU is critical -- switch to emergency mode
        emergency_stop(node);
        return;
    }

    // Fuse available sensors, weighting active ones higher
    let closed = tracker.closed_inputs();
    let active_count = 4 - closed.len();
    // ... fusion logic using active_count for confidence weighting
}
```

### 4. Long-Running Data Processing Pipeline

A batch processing pipeline runs continuously. The processing node occasionally hangs due to a third-party library bug. Health monitoring detects and recovers from these hangs.

```yaml
nodes:
  - id: data-ingest
    path: ./target/debug/ingest
    restart_policy: always        # always restart -- this is a long-running service
    max_restarts: 0               # unlimited
    restart_delay: 1.0
    inputs:
      tick: dora/timer/millis/1000
    outputs:
      - records

  - id: processor
    path: ./target/debug/processor
    restart_policy: on-failure
    max_restarts: 10
    restart_delay: 0.5
    restart_window: 600.0         # 10 restarts per 10 minutes
    health_check_timeout: 30.0    # kill if hung for 30s
    inputs:
      records: data-ingest/records
    outputs:
      - results

  - id: writer
    path: ./target/debug/writer
    restart_policy: on-failure
    max_restarts: 5
    restart_delay: 2.0            # give DB time to recover
    max_restart_delay: 60.0
    inputs:
      results:
        source: processor/results
        input_timeout: 60.0       # processor may be slow
```

**What happens when the processor hangs:**

1. Processor stops communicating with daemon
2. After 30s, health check detects the hang and kills the process
3. `health_check_kills` counter increments
4. Daemon evaluates `on-failure` -> restart after 0.5s
5. New processor instance starts, resumes consuming from `data-ingest`
6. `writer` may have received `InputClosed` during the 60s timeout -- or may not if the restart was fast enough
7. If `writer` did receive `InputClosed`, it gets `InputRecovered` when new results arrive

### 5. Distributed Deployment with Daemon Failure Detection

A multi-machine deployment where the coordinator monitors daemon health.

```
Machine A (coordinator + daemon):  camera-driver, preprocessor
Machine B (daemon):                ml-inference, postprocessor
Machine C (daemon):                planner, actuator-driver
```

**What happens when Machine B loses network:**

1. Coordinator's heartbeat to Machine B fails
2. After 30s without response, coordinator removes Machine B from active daemons
3. Coordinator broadcasts `PeerDaemonDisconnected { daemon_id: "machine-B" }` to Machine A and Machine C
4. Daemons on A and C log: `WARN peer daemon disconnected daemon_id=machine-B`
5. Nodes on A and C with inputs from Machine B's nodes receive `InputClosed` events (via their input timeouts)
6. CLI queries to `ConnectedMachines` show only A and C with their `last_heartbeat_ago_ms`

### 6. Coordinator Crash Recovery with redb Persistence

A long-running multi-daemon deployment where the coordinator must survive restarts without losing track of dataflow history.

```bash
# Start coordinator with persistent store
dora coordinator --store redb

# In another terminal, start a dataflow
dora start examples/rust-dataflow/dataflow.yml --name my-pipeline --detach

# Coordinator crashes or is killed (e.g., OOM, hardware failure)
# ... time passes ...

# Restart coordinator with the same store
dora coordinator --store redb
```

**What happens on restart:**

1. Coordinator opens `~/.dora/coordinator.redb` and reads persisted dataflow records
2. Finds `my-pipeline` with status `Running`
3. Marks it as `Failed { error: "coordinator restarted" }`, increments generation
4. Logs: `INFO recovering stale dataflow <uuid> ("my-pipeline") -> marking as Failed`
5. `dora list` now shows `my-pipeline` with its final status and timestamps
6. Daemons detect the coordinator disconnect independently and stop their nodes
7. User can start a fresh dataflow -- the coordinator is fully operational

The key benefit: the coordinator retains a complete history of dataflow lifecycle events across restarts. Without `--store redb`, all state would be lost and the operator would have no record of what was running before the crash.

### 7. Periodic Batch Job with Always-Restart

A node that processes batches and exits when done. It should restart to process the next batch.

```yaml
nodes:
  - id: batch-processor
    path: ./target/debug/batch-proc
    restart_policy: always        # restart even on clean exit
    max_restarts: 0               # unlimited
    restart_delay: 10.0           # wait 10s between batches
    max_restart_delay: 10.0       # no exponential growth
    inputs:
      trigger: dora/timer/millis/1  # immediate first trigger
    outputs:
      - batch-result
```

The node processes one batch, exits with code 0, waits 10s, then restarts to process the next. The `always` policy ensures restarts even on success. Setting `restart_delay == max_restart_delay` gives a constant delay.

---

## Best Practices

**Start with `on-failure`**. Use `always` only for nodes that are expected to exit and restart (e.g., periodic batch jobs).

**Set `max_restarts`**. Unlimited restarts can mask bugs. Start with 3-5 and increase if needed. Use `max_restarts: 0` only for nodes where crashes are expected and unavoidable (hardware drivers, external API clients).

**Use `restart_window`**. Prevents permanent restart loops. A window of 60-300 seconds is typical. Without a window, a node that crashes at startup will exhaust its restart budget immediately.

**Tune `restart_delay`**. Start with 0.5-1.0 seconds. Too short causes thrashing; too long delays recovery. Match the delay to your node's typical startup time and the root cause of failures:
- USB/hardware reconnection: 2-5s
- Network service reconnection: 1-3s
- OOM/transient bugs: 0.5-1.0s

**Set `health_check_timeout` generously**. Should be at least 2-3x your node's longest expected processing time. ML inference nodes may need 60s+. If too short, healthy nodes get killed during normal processing.

**Set `input_timeout` per input**. Not all inputs need the same timeout. Use shorter timeouts for high-frequency inputs (IMU, camera) and longer timeouts for slow/bursty sources (GPS, batch results). A good starting point is 3-5x the expected publish interval.

**Use `InputTracker` for critical paths**. When a node must keep running even with degraded inputs, use `InputTracker` to fall back to cached data. This is essential for sensor fusion, planning, and control nodes.

**Use `--store redb` for production deployments**. The redb backend ensures the coordinator retains dataflow history across crashes and restarts. The in-memory default is fine for development but loses all state on exit. The redb file is small (proportional to the number of dataflow records) and adds negligible overhead.

**Known limitation:** when the coordinator disconnects, the daemon currently kills running node processes before reconnecting ([#260](https://github.com/dora-rs/adora/issues/260)). Dataflow *records* survive coordinator restart (via redb), but running *processes* are restarted from scratch. Seamless process reclaim across reconnect is planned.

**Combine features for defense in depth**:
- `restart_policy` + `restart_delay` -> recover from node crashes
- `health_check_timeout` -> recover from hung nodes
- `input_timeout` -> detect stale upstream data
- `InputTracker` -> graceful degradation in node code
- `--store redb` -> survive coordinator crashes
