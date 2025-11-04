# Fault Isolation and System Fault Tolerance for dora-rs

## Problem Statement

Currently, dora-rs lacks comprehensive fault recovery mechanisms. While each node runs in a separate process (providing basic fault isolation), the system has no capability to:

1. **Automatically recover from node failures** - When a node crashes, the dataflow terminates
2. **Prevent cascading failures** - Downstream nodes wait indefinitely for inputs that never arrive
3. **Maintain system availability** - No graceful degradation or optional node paths
4. **Configure fault policies** - No declarative way to specify recovery behavior per node

This makes dora-rs unsuitable for production robotic and AI systems that require high availability and resilience.

## Design Philosophy: Framework Simplicity

**Core Principle**: The framework should focus on **dataflow orchestration**, not **application state management**.

### Analysis: Who Should Manage State?

This proposal initially included comprehensive checkpointing mechanisms in the framework. However, after careful consideration, we concluded that **state management should be the node's responsibility**, not the framework's. Here's why:

#### Arguments for Node-Level State Management ✅

1. **Simplicity**: Framework focuses on orchestration (restart policies, monitoring), not state semantics
2. **Flexibility**: Different nodes have vastly different state needs:
   - **SLAM systems**: Complex spatial maps, octrees, pose graphs → specialized formats (HDF5, ROS bags)
   - **Object trackers**: Simple ID mappings, bounding boxes → JSON, protobuf
   - **Database connectors**: State lives in external system (PostgreSQL, Redis)
   - **Stateless vision models**: No state to restore, just reload model weights
   - **Custom controllers**: PID state, Kalman filters → domain-specific persistence
3. **Domain expertise**: Nodes know best how to checkpoint efficiently
   - What data is critical vs. recomputable?
   - What format is most efficient? (binary, compressed, incremental)
   - When to checkpoint? (time-based, event-based, before risky operations)
4. **Existing solutions**: Many domains already have state management patterns
   - ROS has rosbag for data recording
   - Databases have their own persistence
   - ML models use checkpoint files (PyTorch .pt, TensorFlow SavedModel)
5. **Separation of concerns**:
   - **Framework** = Process lifecycle, failure detection, restart orchestration
   - **Node** = Business logic, domain state, persistence strategy

#### What Successful Systems Do

- **Kubernetes**: Provides persistent volumes and restart policies, but apps manage their own data
- **Erlang OTP supervisors**: Restart processes cleanly, don't save/restore internal state
- **Docker/systemd**: Container/process orchestration, apps handle persistence
- **ROS2 lifecycle nodes**: Framework provides lifecycle hooks, nodes handle state transitions

**Exception**: Apache Flink DOES provide checkpointing, but it's a **stream processing framework** where distributed state IS the core concern. Dora-rs is a **dataflow orchestration framework** - different problem domain.

#### What the Framework SHOULD Provide

Instead of managing state, the framework should provide:

1. ✅ **Graceful shutdown signals** - Give nodes time to save state before kill
2. ✅ **Lifecycle hooks** - Notify nodes of restart/recovery events
3. ✅ **Metadata passing** - Pass context to restarted nodes (previous run ID, failure timestamp)
4. ✅ **Environment configuration** - Pass state directories, recovery flags
5. ✅ **Optional helpers** - Utility libraries for common patterns (but not mandatory)

This keeps dora-rs **simple, fast, and flexible** while still enabling fault tolerance.

### Impact on This Proposal

Based on this analysis, **Phase 4 has been simplified**:
- ❌ **Removed**: Framework-managed checkpointing API, checkpoint storage in daemon
- ✅ **Added**: Graceful shutdown, lifecycle hooks, metadata passing
- ✅ **Documentation**: Best practices and examples for node-level state management

The framework remains focused on what it does best: **fast, reliable dataflow orchestration**.

## Current State Analysis

### What Exists Today

From analyzing the codebase (`binaries/daemon/src/lib.rs`, `binaries/daemon/src/pending.rs`, `libraries/message/src/common.rs`):

#### Fault Detection ✅
- Daemon monitors node process exits via `child.wait()` (spawn.rs:520)
- Tracks exit status types:
  - `NodeExitStatus::Success`
  - `NodeExitStatus::ExitCode(i32)`
  - `NodeExitStatus::Signal(i32)`
  - `NodeExitStatus::IoError(String)`
- Detects "exited before subscribe" (crashed during initialization)

#### Fault Reporting ✅
- `NodeError` struct captures:
  - `exit_status: NodeExitStatus`
  - `cause: NodeErrorCause`
- Error causes tracked:
  - `FailedToSpawn` - spawn/exec failure
  - `Cascading { caused_by_node }` - failure due to upstream node
  - `GraceDuration` - timeout during graceful shutdown
  - `Other { stderr }` - generic failure with logs
- Results propagated: daemon → coordinator → CLI

#### Basic Fault Isolation ✅
- Each node runs in separate process
- Process crash doesn't kill other nodes
- Shared memory reference counting prevents memory corruption

### What's Missing ❌

#### 1. Automatic Recovery
- No restart policies (never/on-failure/always)
- No retry mechanism with backoff
- No maximum restart limits
- No restart delay configuration

#### 2. Cascading Failure Prevention
- Downstream nodes block indefinitely waiting for inputs
- No timeout on expected inputs
- No circuit breaker patterns
- No fallback/alternative data sources

#### 3. Graceful Degradation
- No concept of "optional" nodes
- No partial dataflow execution
- Cannot continue with reduced functionality

#### 4. State Management
- No checkpointing API
- No state recovery after restart
- No way to replay missed inputs
- Stateful nodes (e.g., SLAM, tracking) lose all state on crash

#### 5. Health Monitoring
- No heartbeat mechanism
- No liveness/readiness probes
- Only detect failures after process exit
- Cannot detect "zombie" processes (alive but not processing)

#### 6. Declarative Configuration
- No YAML configuration for fault tolerance
- All behavior hardcoded in daemon
- Cannot specify per-node policies

## Fault Tolerance Design for dora-rs

### Design Principles

Given dora-rs's core values:
- **Performance First**: Zero-copy shared memory, 10-17x faster than ROS2
- **Real-time Focus**: Robotics and AI applications with strict latency requirements
- **Simplicity**: Declarative YAML configuration
- **Distributed**: Coordinator + daemons on multiple machines

Our fault tolerance design must:
1. **Preserve performance** - No overhead in hot path (message passing)
2. **Be optional** - Default behavior unchanged for simple use cases
3. **Be declarative** - Configure via YAML, not code
4. **Support real-time** - Fast recovery times for time-sensitive systems
5. **Handle distributed failures** - Daemon crashes, network partitions

### Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    FAULT TOLERANCE LAYERS                    │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  1. DETECTION LAYER (daemon)                                 │
│     - Process exit monitoring (exists)                       │
│     - Heartbeat mechanism (new)                              │
│     - Output timeout detection (new)                         │
│                                                               │
│  2. POLICY LAYER (daemon + coordinator)                      │
│     - Restart policies from YAML (new)                       │
│     - Dependency analysis (new)                              │
│     - Recovery strategy selection (new)                      │
│                                                               │
│  3. RECOVERY LAYER (daemon)                                  │
│     - Node restart with backoff (new)                        │
│     - State restoration (new)                                │
│     - Input replay (new)                                     │
│                                                               │
│  4. ISOLATION LAYER (daemon + nodes)                         │
│     - Circuit breakers (new)                                 │
│     - Timeout on inputs (new)                                │
│     - Graceful degradation (new)                             │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

## Detailed Design

### 1. Understanding Restart Policies

Restart policies control when the framework automatically restarts a node after it exits. This is a fundamental concept for fault tolerance.

#### Policy Options

##### `restart_policy: disabled` (Default)

**Meaning**: Never automatically restart the node, regardless of how it exits.

**When to use:**
- **Safety-critical nodes**: If a safety controller fails, the entire system should stop for inspection
- **One-shot tasks**: Nodes designed to run once and finish (e.g., map loading, initialization)
- **Development/debugging**: Prevent auto-restart from hiding bugs
- **Default behavior**: Preserves current dora-rs behavior (no surprises for existing users)

**Behavior:**
```
Node exits successfully (code 0)     → Stays stopped ✓
Node crashes (non-zero exit code)    → Stays stopped ✓
Node killed by signal (SIGSEGV, etc) → Stays stopped ✓
```

**Example:**
```yaml
# Safety controller - if it fails, stop the entire dataflow
- id: emergency-brake
  path: safety-controller
  fault_tolerance:
    restart_policy: disabled  # Default, can be omitted
    on_failure:
      action: fail_dataflow
      reason: "Safety system compromised"
```

**Use case**: Emergency brake controller in autonomous vehicle. If it fails, better to stop the vehicle completely than risk operating with a faulty safety system.

---

##### `restart_policy: on-failure-only` (Most Common)

**Meaning**: Restart the node ONLY if it exits with an error (non-zero exit code or signal). If it exits successfully (code 0), let it stay stopped.

**When to use:**
- **Production services**: Camera drivers, perception nodes, controllers that should keep running
- **Recoverable failures**: Hardware glitches, temporary resource exhaustion, transient bugs
- **Long-running processes**: Nodes expected to run continuously but might crash occasionally
- **80% of production nodes**: This is what you want in most real-world systems

**Behavior:**
```
Node exits successfully (code 0)     → Stays stopped ✓ (intentional exit)
Node crashes (non-zero exit code)    → Restarts automatically 🔄
Node killed by signal (SIGSEGV, etc) → Restarts automatically 🔄
Exceeds max_restarts                 → Gives up, stays stopped ✗
```

**Example:**
```yaml
# Camera driver - hardware can glitch, automatically recover
- id: camera
  path: camera-driver
  fault_tolerance:
    restart_policy: on-failure-only
    max_restarts: 10         # Try up to 10 times
    restart_delay: 1s        # Wait 1s before first retry
    max_restart_delay: 30s   # Cap delay at 30s
    restart_backoff_factor: 2.0  # Double delay each time

# YOLO detection - might OOM on GPU, restart with delay
- id: yolo
  path: yolo-node
  fault_tolerance:
    restart_policy: on-failure-only
    max_restarts: 3
    restart_delay: 5s  # Give GPU time to free memory
```

**Use case**: USB camera driver. Cameras can disconnect/reconnect, drivers can crash due to USB errors. Just restart automatically - most of the time it recovers.

**Retry behavior with exponential backoff:**
```
Attempt 1: Crashes → Wait 1s   → Restart
Attempt 2: Crashes → Wait 2s   → Restart (1s * 2.0)
Attempt 3: Crashes → Wait 4s   → Restart (2s * 2.0)
Attempt 4: Crashes → Wait 8s   → Restart (4s * 2.0)
Attempt 5: Crashes → Wait 16s  → Restart (8s * 2.0)
Attempt 6: Crashes → Wait 30s  → Restart (capped at max_restart_delay)
...
Attempt 10: Crashes → Wait 30s → Restart
Attempt 11: Crashes → Give up ✗ (max_restarts exceeded)
```

---

##### `restart_policy: always-restart` (Advanced/Rare)

**Meaning**: Restart the node whenever it exits, regardless of exit status. Even if it exits successfully (code 0), restart it.

**When to use (rare):**
- **Periodic tasks**: Scripts that run, complete, and should run again
- **Watchdog processes**: Designed to repeatedly check something and exit
- **Unreliable nodes on flaky hardware**: Process exits randomly, just keep retrying
- **Stateless workers**: Exit after each task, pick up next task on restart

**Behavior:**
```
Node exits successfully (code 0)     → Restarts anyway 🔄
Node crashes (non-zero exit code)    → Restarts 🔄
Node killed by signal (SIGSEGV, etc) → Restarts 🔄
Exceeds max_restarts                 → Gives up, stays stopped ✗
```

**Example:**
```yaml
# Periodic health monitor - script that checks system and exits
- id: health-monitor
  path: check-system.sh  # Bash script that runs once and exits
  fault_tolerance:
    restart_policy: always-restart
    restart_delay: 30s       # Wait 30s between runs
    max_restarts: 0          # Run forever (unlimited)

# Flaky sensor on unreliable edge device
- id: edge-sensor
  path: sensor-node
  fault_tolerance:
    restart_policy: always-restart  # Keep trying no matter what
    restart_delay: 1s
    max_restart_delay: 60s
    max_restarts: 0  # Never give up
```

**Use case**: System health checker that runs every 30 seconds. The script collects metrics, exits successfully, waits 30s, and runs again. Using `always-restart` turns it into a periodic task.

**⚠️ Warning**: Be careful with `always-restart`! If the node has a bug causing immediate exit, you'll get a rapid restart loop. Always use `restart_delay` to prevent resource exhaustion.

---

#### Comparison Table

| Node Exit Reason              | `disabled` | `on-failure-only` | `always-restart` |
|-------------------------------|------------|-------------------|------------------|
| Success (exit code 0)         | Stop       | Stop              | Restart          |
| Error (non-zero exit code)    | Stop       | Restart           | Restart          |
| Signal (SIGSEGV, SIGKILL)     | Stop       | Restart           | Restart          |
| Exceeds `max_restarts`        | N/A        | Stop              | Stop             |
| **Common Use Case**           | Safety-critical, one-shot tasks | Production services (90% of nodes) | Periodic tasks, watchdogs |

---

#### Decision Flowchart

```
Should this node restart if it crashes?
│
├─ NO → restart_policy: disabled
│       Use cases: Safety-critical, one-shot, debugging
│
└─ YES → Should it restart even on successful exit?
         │
         ├─ NO → restart_policy: on-failure-only  ← MOST COMMON
         │       Use cases: Services, drivers, processing nodes
         │
         └─ YES → restart_policy: always-restart
                  Use cases: Periodic tasks, watchdogs, flaky hardware
```

---

#### Interaction with `max_restarts`

`max_restarts` limits retry attempts to prevent infinite restart loops:

```yaml
fault_tolerance:
  restart_policy: on-failure-only
  max_restarts: 5  # Give up after 5 failed restart attempts
```

- **`max_restarts: 0`**: Unlimited restarts (use with caution!)
- **`max_restarts: N`**: After N restart attempts, mark node as permanently failed
- **Counter resets**: If node runs successfully for a period (e.g., 5 minutes), reset counter to 0

**Why this matters**: Prevents a broken node from consuming resources forever. If a camera driver crashes 5 times in a row, there's likely a hardware problem - better to fail and alert the operator.

---

#### Common Patterns

**Pattern 1: Production Vision System**
```yaml
nodes:
  - id: camera
    fault_tolerance:
      restart_policy: on-failure-only  # Hardware can glitch
      max_restarts: 10

  - id: yolo
    fault_tolerance:
      restart_policy: on-failure-only  # Might OOM
      max_restarts: 3
      restart_delay: 5s

  - id: controller
    fault_tolerance:
      restart_policy: disabled  # Critical - stop if it fails
      on_failure:
        action: fail_dataflow
```

**Pattern 2: Batch Processing**
```yaml
nodes:
  - id: loader
    fault_tolerance:
      restart_policy: disabled  # Runs once, loads data

  - id: processor
    fault_tolerance:
      restart_policy: on-failure-only  # Might crash, retry

  - id: saver
    fault_tolerance:
      restart_policy: disabled  # Runs once, saves results
```

**Pattern 3: Edge Device with Flaky Hardware**
```yaml
nodes:
  - id: sensor
    fault_tolerance:
      restart_policy: always-restart  # Keep retrying forever
      restart_delay: 1s
      max_restarts: 0
```

---

### 2. YAML Configuration Schema

#### Node-Level Configuration

```yaml
nodes:
  - id: camera
    path: camera-node
    inputs:
      tick: dora/timer/millis/30
    outputs:
      - image

    # NEW: Fault tolerance configuration
    fault_tolerance:
      # Restart policy: disabled (default) | on-failure-only | always-restart
      # - disabled: Never restart (current dora-rs behavior)
      # - on-failure-only: Restart only if node crashes/exits with error
      # - always-restart: Restart whenever node exits (even on success)
      restart_policy: on-failure-only

      # Maximum restart attempts (0 = unlimited)
      max_restarts: 5

      # Delay between restarts (with exponential backoff)
      restart_delay: 1s
      max_restart_delay: 30s
      restart_backoff_factor: 2.0

      # Health check configuration
      health_check:
        # Heartbeat interval (0 = disabled)
        heartbeat_interval: 5s
        # Timeout before considering node unhealthy
        heartbeat_timeout: 15s
        # Startup grace period before health checks begin
        startup_grace_period: 10s

      # Failure handling
      on_failure:
        # Action: restart (default) | fail_dataflow | degrade | circuit_break
        action: restart

        # Notify these nodes of failure (send special input)
        notify: [monitoring-node, fallback-camera]

      # Graceful shutdown (give node time to save state before kill)
      graceful_shutdown_timeout: 10s

      # Pass context to help node restore state (node's responsibility)
      restart_environment:
        STATE_DIR: /data/camera-state
        RESTORE_FROM_LATEST: "true"

      # Circuit breaker for this node's outputs
      circuit_breaker:
        enabled: true
        failure_threshold: 3        # Open after N failures
        success_threshold: 2        # Close after N successes
        timeout: 30s                # Half-open after timeout

  - id: object-detection
    path: yolo-node
    inputs:
      image: camera/image
    outputs:
      - bbox

    fault_tolerance:
      restart_policy: on-failure-only
      max_restarts: 3

      # NEW: Input timeout configuration
      input_timeouts:
        image: 1s  # Fail if no image for 1 second

      # Handle upstream (camera) failures
      on_upstream_failure:
        # Action: fail | wait | use_fallback
        action: use_fallback
        fallback_source: fallback-camera/image
        fallback_timeout: 5s

  - id: fallback-camera
    path: camera-node
    outputs:
      - image
    fault_tolerance:
      # This is an optional/fallback node
      optional: true
      # Don't start by default, only when primary camera fails
      start_on_demand: true

  - id: mission-critical-controller
    path: controller-node
    inputs:
      bbox: object-detection/bbox
    outputs:
      - control

    fault_tolerance:
      # Never allow this node to fail, terminate dataflow instead
      restart_policy: disabled
      on_failure:
        action: fail_dataflow
        reason: "Critical safety controller failed"
```

#### Dataflow-Level Configuration

```yaml
# NEW: Global fault tolerance settings
fault_tolerance:
  # Default policy for all nodes (can be overridden per node)
  default_restart_policy: on-failure-only
  default_max_restarts: 3

  # Global circuit breaker
  global_failure_threshold: 10  # Stop dataflow after 10 total failures

  # Cascading failure timeout
  cascading_failure_timeout: 5s

  # Graceful shutdown timeout
  grace_period: 10s

nodes:
  # ... node definitions
```

### 2. Implementation Phases

#### Phase 1: Basic Restart Policies (MVP)

**Goal**: Automatically restart failed nodes with configurable policies

**Changes**:

1. **Extend `dora-core` descriptor parsing** (`libraries/core/src/descriptor/mod.rs`):
   ```rust
   #[derive(Deserialize, Serialize)]
   pub struct FaultToleranceConfig {
       #[serde(default)]
       pub restart_policy: RestartPolicy,
       #[serde(default)]
       pub max_restarts: u32,
       #[serde(default = "default_restart_delay")]
       pub restart_delay: Duration,
       // ... other fields
   }

   #[derive(Deserialize, Serialize, Default)]
   #[serde(rename_all = "kebab-case")]
   pub enum RestartPolicy {
       /// Never restart the node (default, preserves current behavior)
       #[default]
       Disabled,
       /// Restart only if node exits with error (non-zero exit code or signal)
       OnFailureOnly,
       /// Restart whenever node exits, regardless of exit status
       AlwaysRestart,
   }
   ```

2. **Update daemon to track restart counts** (`binaries/daemon/src/lib.rs`):
   ```rust
   struct RunningNode {
       // existing fields...
       restart_count: u32,
       last_restart: Option<Instant>,
       fault_tolerance_config: Option<FaultToleranceConfig>,
   }
   ```

3. **Implement restart logic** in `handle_node_stop_inner`:
   ```rust
   async fn handle_node_stop_inner(&mut self, ...) -> Result<()> {
       // existing error reporting logic...

       // NEW: Check if node should be restarted
       if self.should_restart_node(dataflow_id, node_id, &exit_status)? {
           let delay = self.calculate_restart_delay(dataflow_id, node_id)?;

           tokio::time::sleep(delay).await;

           self.restart_node(dataflow_id, node_id).await?;
           return Ok(());
       }

       // existing cleanup logic...
   }
   ```

4. **Update message protocol** (`libraries/message`):
   - Add `NodeRestarted` event
   - Include restart count in status messages

**Testing**:
- Unit tests for restart policy evaluation
- Integration test: kill node, verify restart
- Integration test: exceed max_restarts, verify failure
- Integration test: exponential backoff timing

**Deliverable**: Nodes can automatically restart on failure based on YAML config

---

#### Phase 2: Health Monitoring & Heartbeats

**Goal**: Detect unhealthy nodes before they fully crash

**Changes**:

1. **Extend Node API** (`apis/rust/node/src/lib.rs`):
   ```rust
   impl DoraNode {
       // NEW: Optional heartbeat support
       pub fn start_heartbeat(&self, interval: Duration) -> HeartbeatHandle {
           // Spawn background task that sends heartbeat to daemon
       }

       // NEW: Report custom health status
       pub fn report_health(&self, healthy: bool, message: Option<String>) -> Result<()> {
           // Send health status to daemon
       }
   }
   ```

2. **Update daemon** (`binaries/daemon/src/lib.rs`):
   ```rust
   struct RunningNode {
       // existing fields...
       last_heartbeat: Option<Instant>,
       heartbeat_config: Option<HeartbeatConfig>,
       health_status: HealthStatus,
   }

   // NEW: Background task to check heartbeats
   async fn monitor_node_health(&mut self) {
       loop {
           tokio::time::sleep(Duration::from_secs(1)).await;

           for (node_id, node) in &self.running_nodes {
               if let Some(config) = &node.heartbeat_config {
                   if node.last_heartbeat.elapsed() > config.timeout {
                       self.handle_unhealthy_node(node_id).await;
                   }
               }
           }
       }
   }
   ```

3. **Add daemon protocol messages** (`libraries/message`):
   ```rust
   pub enum NodeToDaemonRequest {
       // existing...
       Heartbeat { timestamp: Timestamp },
       HealthStatus { healthy: bool, message: Option<String> },
   }
   ```

4. **Python API support** (`apis/python/node`):
   ```python
   class Node:
       def start_heartbeat(self, interval_ms: int) -> HeartbeatHandle:
           """Start automatic heartbeat to daemon"""

       def report_health(self, healthy: bool, message: Optional[str] = None):
           """Report custom health status"""
   ```

**Testing**:
- Test node with heartbeat survives
- Test node without heartbeat triggers restart
- Test custom health reporting
- Test startup grace period

**Deliverable**: Proactive detection of unhealthy nodes via heartbeats

---

#### Phase 3: Input Timeouts & Circuit Breakers

**Goal**: Prevent cascading failures via timeouts and circuit breakers

**Changes**:

1. **Extend Node API** for timeout-aware input reading:
   ```rust
   impl DoraNode {
       // NEW: Receive with timeout
       pub async fn recv_timeout(
           &self,
           timeout: Duration
       ) -> Result<Option<Event>> {
           // Return None on timeout, Some(event) on success
       }

       // Existing recv() behavior unchanged for backwards compatibility
       pub async fn recv(&self) -> Result<Event> {
           self.recv_timeout(Duration::MAX).await?.unwrap()
       }
   }
   ```

2. **Daemon implements input timeout tracking**:
   ```rust
   struct PendingInput {
       input_id: DataId,
       timeout: Duration,
       last_received: Option<Instant>,
   }

   // In event loop, check for timed out inputs
   async fn check_input_timeouts(&mut self) {
       for (node_id, node) in &self.running_nodes {
           for (input_id, pending) in &node.pending_inputs {
               if pending.last_received.elapsed() > pending.timeout {
                   self.send_timeout_event(node_id, input_id).await;
               }
           }
       }
   }
   ```

3. **Circuit breaker implementation**:
   ```rust
   struct CircuitBreaker {
       state: CircuitState,
       failure_count: u32,
       success_count: u32,
       config: CircuitBreakerConfig,
       last_failure: Option<Instant>,
   }

   enum CircuitState {
       Closed,   // Normal operation
       Open,     // Rejecting all requests
       HalfOpen, // Testing recovery
   }

   impl CircuitBreaker {
       fn record_failure(&mut self) {
           self.failure_count += 1;
           if self.failure_count >= self.config.failure_threshold {
               self.state = CircuitState::Open;
               self.last_failure = Some(Instant::now());
           }
       }

       fn should_allow_request(&mut self) -> bool {
           match self.state {
               CircuitState::Closed => true,
               CircuitState::Open => {
                   // Check if timeout elapsed
                   if self.last_failure.elapsed() > self.config.timeout {
                       self.state = CircuitState::HalfOpen;
                       true
                   } else {
                       false
                   }
               }
               CircuitState::HalfOpen => true,
           }
       }
   }
   ```

4. **Add special event types**:
   ```rust
   pub enum Event {
       // existing...
       InputTimeout { id: DataId },
       CircuitBreakerOpen { source_node: NodeId },
       CircuitBreakerClosed { source_node: NodeId },
   }
   ```

**Testing**:
- Test input timeout triggers event
- Test circuit breaker opens after failures
- Test circuit breaker half-open state
- Test downstream nodes handle timeout events

**Deliverable**: Nodes can handle missing inputs gracefully without blocking forever

---

#### Phase 4: Graceful Lifecycle & Node-Managed State

**Goal**: Provide lifecycle hooks for nodes to manage their own state during restart

**Rationale**: After analysis (see "Design Philosophy" section above), we determined that state management should be the **node's responsibility**, not the framework's. The framework should provide lifecycle infrastructure, not state storage.

**Changes**:

1. **Graceful shutdown in daemon** (`binaries/daemon/src/lib.rs`):
   ```rust
   async fn stop_node_gracefully(&mut self, node_id: &NodeId) -> Result<()> {
       // Send Stop event to node
       self.send_stop_event(node_id).await?;

       // Wait for graceful timeout from config
       let timeout = self.get_graceful_timeout(node_id)
           .unwrap_or(Duration::from_secs(5));

       match tokio::time::timeout(timeout, self.wait_for_node_exit(node_id)).await {
           Ok(_) => {
               // Node exited gracefully, it had time to save state
               Ok(())
           }
           Err(_) => {
               // Timeout exceeded, force kill
               warn!("Node {node_id} didn't stop gracefully, force killing");
               self.kill_node(node_id).await
           }
       }
   }
   ```

2. **Lifecycle metadata passing** - Pass context to restarted nodes:
   ```rust
   async fn restart_node(&mut self, node_id: &NodeId) -> Result<()> {
       let restart_info = RestartInfo {
           previous_run_id: self.get_previous_run_id(node_id),
           failure_timestamp: self.get_failure_timestamp(node_id),
           restart_count: self.get_restart_count(node_id),
       };

       // Pass as environment variables
       let mut env = self.node_configs[node_id].env.clone();
       env.insert("DORA_PREVIOUS_RUN_ID".into(), restart_info.previous_run_id.to_string());
       env.insert("DORA_FAILURE_TIMESTAMP".into(), restart_info.failure_timestamp.to_string());
       env.insert("DORA_RESTART_COUNT".into(), restart_info.restart_count.to_string());

       // Also send as special input event after node subscribes
       self.pending_lifecycle_events.insert(
           node_id.clone(),
           Event::Lifecycle {
               event_type: LifecycleEventType::Restarted,
               metadata: restart_info,
           }
       );

       self.spawn_node(node_id, env).await
   }
   ```

3. **New lifecycle events** (`libraries/message`):
   ```rust
   pub enum Event {
       // existing...

       // NEW: Lifecycle events
       Lifecycle {
           event_type: LifecycleEventType,
           metadata: HashMap<String, String>,
       },
   }

   pub enum LifecycleEventType {
       Started,
       Restarted,
       Stopping,
   }
   ```

4. **Node API enhancements** (`apis/rust/node/src/lib.rs`):
   ```rust
   impl DoraNode {
       /// Check if this is a restart (vs. fresh start)
       pub fn is_restart(&self) -> bool {
           std::env::var("DORA_RESTART_COUNT")
               .ok()
               .and_then(|s| s.parse::<u32>().ok())
               .map(|count| count > 0)
               .unwrap_or(false)
       }

       /// Get metadata about previous run (if restarted)
       pub fn get_restart_metadata(&self) -> Option<RestartMetadata> {
           if !self.is_restart() {
               return None;
           }

           Some(RestartMetadata {
               previous_run_id: std::env::var("DORA_PREVIOUS_RUN_ID").ok()?,
               failure_timestamp: std::env::var("DORA_FAILURE_TIMESTAMP").ok()?,
               restart_count: std::env::var("DORA_RESTART_COUNT")
                   .ok()?
                   .parse()
                   .ok()?,
           })
       }
   }
   ```

5. **Python API**:
   ```python
   class Node:
       def is_restart(self) -> bool:
           """Check if this is a restart vs fresh start"""

       def get_restart_metadata(self) -> Optional[RestartMetadata]:
           """Get info about previous run if restarted"""
   ```

**Example: Node-Managed State (SLAM system)**:

```rust
// Node handles its own state persistence
fn main() -> Result<()> {
    let (node, events) = DoraNode::init_from_env()?;

    // Framework tells us if we're restarting
    let mut slam = if node.is_restart() {
        // Node's responsibility: restore state
        let state_dir = env::var("STATE_DIR")?;
        SLAM::restore_from_disk(&state_dir)?
    } else {
        SLAM::new()
    };

    let state_dir = env::var("STATE_DIR").ok();
    let checkpoint_interval = Duration::from_secs(10);
    let mut last_checkpoint = Instant::now();

    for event in events {
        match event {
            Event::Input { id, data, .. } => {
                slam.process(data)?;

                // Node decides when to checkpoint
                if last_checkpoint.elapsed() > checkpoint_interval {
                    if let Some(dir) = &state_dir {
                        slam.save_to_disk(dir)?;
                        last_checkpoint = Instant::now();
                    }
                }
            }
            Event::Stop | Event::Lifecycle { event_type: LifecycleEventType::Stopping, .. } => {
                // Framework gives us time (graceful_shutdown_timeout)
                if let Some(dir) = &state_dir {
                    slam.save_to_disk(dir)?;
                }
                break;
            }
            _ => {}
        }
    }
    Ok(())
}
```

**Testing**:
- Test graceful shutdown timeout
- Test environment variables passed to restarted nodes
- Test lifecycle events delivered correctly
- Test node can detect restart and restore state
- Test force kill after graceful timeout

**Deliverable**:
- Framework provides lifecycle hooks and metadata
- Nodes handle their own state persistence (simpler framework)
- Example patterns documented for common cases (SLAM, tracking, etc.)

---

#### Phase 5: Graceful Degradation & Optional Nodes

**Goal**: Continue dataflow operation with reduced functionality when non-critical nodes fail

**Changes**:

1. **Optional node configuration**:
   ```yaml
   nodes:
     - id: optional-telemetry
       path: telemetry-node
       fault_tolerance:
         optional: true  # Dataflow continues if this node fails
         start_on_demand: false  # Start immediately (default)
   ```

2. **Fallback data sources**:
   ```yaml
   nodes:
     - id: consumer
       inputs:
         # Primary source
         data: primary-producer/output
       fault_tolerance:
         on_upstream_failure:
           action: use_fallback
           fallback_source: backup-producer/output
           fallback_timeout: 5s
   ```

3. **Daemon fallback logic**:
   ```rust
   async fn route_input_to_node(&mut self, target: &NodeId, input_id: &DataId) {
       let config = self.get_node_config(target)?;

       // Check if primary source is available
       if !self.is_node_healthy(&config.primary_source) {
           // Check for fallback
           if let Some(fallback) = config.get_fallback_source(input_id) {
               if self.is_node_healthy(&fallback.source) {
                   // Route from fallback instead
                   return self.route_from_fallback(target, input_id, fallback).await;
               }
           }
       }

       // Normal routing...
   }
   ```

4. **Start-on-demand nodes**:
   ```rust
   async fn handle_upstream_failure(&mut self, failed_node: &NodeId) {
       // Find nodes that have this as fallback
       for (node_id, config) in &self.node_configs {
           if let Some(fallback) = config.fallback_sources.get(failed_node) {
               if fallback.start_on_demand {
                   self.start_fallback_node(fallback).await?;
               }
           }
       }
   }
   ```

**Testing**:
- Test optional node failure doesn't stop dataflow
- Test fallback source activation
- Test start-on-demand nodes
- Test dataflow completes with partial results

**Deliverable**: Non-critical node failures don't terminate the entire dataflow

---

#### Phase 6: Distributed Fault Tolerance

**Goal**: Handle daemon crashes and network partitions in distributed deployments

**Changes**:

1. **Daemon health monitoring** in coordinator:
   ```rust
   struct CoordinatorState {
       // existing...
       daemon_health: HashMap<DaemonId, DaemonHealth>,
   }

   struct DaemonHealth {
       last_heartbeat: Instant,
       missed_heartbeats: u32,
       status: DaemonStatus,
   }

   async fn monitor_daemon_health(&mut self) {
       loop {
           tokio::time::sleep(Duration::from_secs(5)).await;

           for (daemon_id, health) in &mut self.daemon_health {
               if health.last_heartbeat.elapsed() > DAEMON_TIMEOUT {
                   health.missed_heartbeats += 1;

                   if health.missed_heartbeats >= MAX_MISSED_HEARTBEATS {
                       self.handle_daemon_failure(daemon_id).await;
                   }
               }
           }
       }
   }
   ```

2. **Node migration** (optional, advanced):
   ```rust
   async fn handle_daemon_failure(&mut self, failed_daemon: &DaemonId) {
       let nodes = self.get_nodes_on_daemon(failed_daemon);

       for node in nodes {
           // Find another daemon with capacity
           if let Some(new_daemon) = self.find_available_daemon(&node) {
               // Migrate node to new daemon
               self.migrate_node(node, failed_daemon, new_daemon).await?;
           } else {
               // No available daemon, mark node as failed
               self.mark_node_failed(node, "daemon failure").await;
           }
       }
   }
   ```

3. **Coordinator redundancy** (optional, very advanced):
   - Raft or Paxos for coordinator HA
   - Outside scope for initial implementation

**Testing**:
- Test daemon crash detection
- Test node migration (if implemented)
- Test network partition handling

**Deliverable**: Dataflows can survive daemon crashes in distributed deployments

---

### 3. Breaking Changes & Backwards Compatibility

**Backwards Compatibility Strategy**:

1. **All fault tolerance features are OPT-IN**:
   - Default `restart_policy: disabled` (current behavior)
   - No YAML config = no behavior change
   - Existing dataflows run unchanged

2. **Additive API changes**:
   - New methods: `start_heartbeat()`, `report_health()`, `is_restart()`, `get_restart_metadata()`
   - Existing methods unchanged
   - No breaking changes to `DoraNode` interface

3. **Message protocol versioning**:
   - New message types added
   - Old daemons ignore unknown messages
   - Version negotiation on connection

4. **YAML schema extensions**:
   - New `fault_tolerance` section is optional
   - JSON schema validates gracefully
   - Unknown fields ignored (future-proof)

**Migration Path**:

1. **Phase 1-2**: No action required for existing users
2. **Phase 3**: Opt-in by adding `fault_tolerance` to YAML
3. **Phase 4**: Stateful nodes can use `is_restart()` and manage their own state
4. **Phase 5-6**: Enterprise users can enable for production

---

### 4. Performance Considerations

**Zero Overhead in Hot Path**:

1. **Message passing (shared memory) unchanged**:
   - No new checks in data plane
   - Reference counting unchanged
   - Zero-copy semantics preserved

2. **Fault tolerance in control plane only**:
   - Heartbeats: low-frequency (5s interval)
   - Checkpoints: background tasks, async I/O
   - Health checks: out-of-band, separate channel

3. **Benchmarking**:
   - Existing benchmark: `examples/benchmark`
   - Add fault-tolerance-enabled benchmark
   - Ensure <1% overhead with all features enabled

**Memory Overhead**:

1. **Checkpoint storage**:
   - Configurable size limit per node
   - Sliding window (keep last N checkpoints)
   - Optional compression (zstd)

2. **Input replay buffers**:
   - Configurable max size
   - Time-based expiration
   - Disabled by default

**CPU Overhead**:

1. **Health monitoring thread**:
   - 1 thread per daemon
   - Sleep-based, not busy-wait
   - Negligible CPU (<0.1%)

2. **Checkpoint serialization**:
   - User-controlled frequency
   - Background task, doesn't block node
   - Use efficient serialization (bincode)

---

### 5. Testing Strategy

**Unit Tests**:
- Restart policy evaluation logic
- Circuit breaker state machine
- Checkpoint serialization/deserialization
- Timeout calculation (exponential backoff)

**Integration Tests** (new test dataflows):
```rust
// examples/fault-tolerance-test/dataflow.yml
#[tokio::test]
async fn test_node_restart_on_failure() {
    let dataflow = "fault-tolerance-test/restart.yml";

    // Start dataflow
    let daemon = Daemon::run_dataflow(dataflow).await?;

    // Kill a node
    kill_node(&daemon, "test-node").await?;

    // Verify node restarted
    tokio::time::sleep(Duration::from_secs(2)).await;
    assert!(daemon.is_node_running("test-node"));

    // Verify dataflow continues
    assert!(daemon.is_dataflow_running());
}

#[tokio::test]
async fn test_max_restarts_exceeded() {
    // ... similar test, kill node 6 times, verify failure
}

#[tokio::test]
async fn test_heartbeat_timeout() {
    // Start node with heartbeat
    // Pause heartbeat thread
    // Verify daemon detects and restarts node
}

#[tokio::test]
async fn test_stateful_node_checkpoint_restore() {
    // Start node with checkpointing
    // Store state
    // Kill node
    // Verify state restored after restart
}
```

**End-to-End Tests**:
- Realistic robotics dataflow (camera → detection → control)
- Inject faults at different points
- Verify system recovers and continues

**Chaos Engineering**:
- Random node kills
- Network latency injection
- Daemon crashes
- Verify system remains stable

---

### 6. Documentation Requirements

**User Documentation**:

1. **Guide: Fault Tolerance Configuration**
   - Overview of concepts
   - YAML schema reference
   - Common patterns (restart policies, circuit breakers)
   - Best practices

2. **Tutorial: Building Resilient Dataflows**
   - Step-by-step: add restart policy
   - Step-by-step: implement health checks
   - Step-by-step: checkpoint stateful nodes

3. **API Reference Updates**:
   - Rust: `DoraNode` with lifecycle methods (`is_restart()`, `get_restart_metadata()`)
   - Python: `Node` class methods
   - C/C++: new functions

**Developer Documentation**:

1. **Architecture: Fault Tolerance System**
   - Component interaction diagram
   - State machine diagrams (circuit breaker, restart logic)
   - Protocol message flows

2. **Contributing Guide**:
   - How to add new restart policies
   - How to extend checkpoint formats
   - Testing requirements

**Migration Guide**:
- How to upgrade existing dataflows
- Opt-in checklist
- Troubleshooting common issues

---

### 7. Example Use Cases

#### Use Case 1: Vision Pipeline for Robotics

```yaml
# Camera can fail (hardware issue), restart automatically
- id: camera
  path: camera-node
  fault_tolerance:
    restart_policy: on-failure-only
    max_restarts: 10
    health_check:
      heartbeat_interval: 2s

# Object detection is GPU-intensive, can OOM
- id: yolo
  path: yolo-node
  fault_tolerance:
    restart_policy: on-failure-only
    max_restarts: 3
    restart_delay: 5s  # Give GPU time to recover

# Mission-critical controller must never fail
- id: controller
  path: safety-controller
  fault_tolerance:
    restart_policy: disabled
    on_failure:
      action: fail_dataflow
```

#### Use Case 2: SLAM System (Stateful)

```yaml
- id: slam
  path: slam-node
  fault_tolerance:
    restart_policy: on-failure-only
    graceful_shutdown_timeout: 10s  # Give time to save map

    # Node manages its own state, framework just passes info
    restart_environment:
      STATE_DIR: /data/slam-map
      CHECKPOINT_INTERVAL: "5"  # Node decides checkpoint frequency
```

**Node implementation (in slam-node code)**:
```rust
fn main() -> Result<()> {
    let (node, events) = DoraNode::init_from_env()?;

    // Check if we're recovering from a crash
    let mut slam = if node.is_restart() {
        let state_dir = env::var("STATE_DIR")?;
        SLAM::restore_from_disk(&state_dir)?  // Node's responsibility
    } else {
        SLAM::new()
    };

    // Node handles periodic checkpointing
    for event in events {
        // Process inputs, save state periodically...
    }
}
```

#### Use Case 3: Redundant Sensors

```yaml
- id: primary-lidar
  path: lidar-node
  outputs:
    - pointcloud
  fault_tolerance:
    restart_policy: on-failure-only
    max_restarts: 3

- id: backup-lidar
  path: lidar-node
  outputs:
    - pointcloud
  fault_tolerance:
    optional: true
    start_on_demand: true  # Only start if primary fails

- id: perception
  inputs:
    pointcloud: primary-lidar/pointcloud
  fault_tolerance:
    on_upstream_failure:
      action: use_fallback
      fallback_source: backup-lidar/pointcloud
```

#### Use Case 4: Distributed Edge Deployment

```yaml
# Nodes on edge device (unreliable hardware)
- id: edge-camera
  path: camera-node
  _unstable_deploy:
    machine: edge-device
  fault_tolerance:
    restart_policy: on-exit  # Keep trying
    restart_delay: 1s
    max_restart_delay: 60s

# Nodes on cloud server (reliable)
- id: cloud-inference
  path: large-model
  _unstable_deploy:
    machine: cloud-server
  fault_tolerance:
    restart_policy: on-failure-only
    max_restarts: 1  # Fail fast on cloud
```

---

### 8. Open Questions for Discussion

1. **Checkpoint Storage**:
   - Option A: In-memory in daemon (lost on daemon crash)
   - Option B: Persistent storage (disk/database)
   - Option C: Remote storage (S3, object store)
   - **Recommendation**: Start with A, add B in Phase 6

2. **State Recovery Semantics**:
   - Exactly-once processing (like Apache Flink)?
   - At-least-once (simpler, may duplicate)?
   - Best-effort (no guarantees)?
   - **Recommendation**: Best-effort initially, document limitations

3. **Distributed Coordination**:
   - Centralized (coordinator decides everything)?
   - Decentralized (daemons coordinate peer-to-peer)?
   - **Recommendation**: Centralized for simplicity

4. **Configuration Complexity**:
   - Should we provide "profiles" (e.g., `fault_tolerance: resilient`)?
   - Trade-off: simplicity vs. fine-grained control
   - **Recommendation**: Support both (profiles + detailed config)

5. **Observability**:
   - How to expose fault tolerance metrics?
   - OpenTelemetry integration?
   - Dashboard for node health?
   - **Recommendation**: OpenTelemetry metrics + tracing events

---

### 9. Success Metrics

**Functional Goals**:
- ✅ Nodes restart automatically on failure
- ✅ Dataflows survive individual node crashes
- ✅ Stateful nodes restore state after restart
- ✅ Cascading failures prevented via timeouts
- ✅ Optional nodes don't block dataflow completion

**Performance Goals**:
- ✅ <1% overhead with all features enabled
- ✅ <100ms restart latency (from crash to new process)
- ✅ <10MB memory overhead per dataflow
- ✅ Existing benchmarks unchanged

**Usability Goals**:
- ✅ Zero config changes required for existing dataflows
- ✅ Simple YAML config for basic use cases
- ✅ Clear documentation with examples
- ✅ Backwards compatible protocol

---

## Implementation Roadmap

### Timeline

**Phase 1** (4-6 weeks): Basic Restart Policies
- Core restart logic in daemon
- YAML schema extensions
- Basic integration tests
- **Deliverable**: MVP fault tolerance

**Phase 2** (3-4 weeks): Health Monitoring
- Heartbeat mechanism
- Node API extensions
- Python API support
- **Deliverable**: Proactive failure detection

**Phase 3** (4-5 weeks): Timeouts & Circuit Breakers
- Input timeout tracking
- Circuit breaker implementation
- Event API extensions
- **Deliverable**: Cascading failure prevention

**Phase 4** (2-3 weeks): Graceful Lifecycle
- Graceful shutdown with timeout
- Lifecycle events (Started, Restarted, Stopping)
- Restart metadata passing (env vars, events)
- Node API: `is_restart()`, `get_restart_metadata()`
- **Deliverable**: Nodes can manage their own state with framework support

**Phase 5** (3-4 weeks): Graceful Degradation
- Optional nodes
- Fallback sources
- Start-on-demand
- **Deliverable**: Partial dataflow execution

**Phase 6** (6-8 weeks): Distributed Tolerance
- Daemon health monitoring
- Node migration (optional)
- **Deliverable**: Daemon crash survival

**Total**: ~22-30 weeks (5.5-7.5 months) for full implementation

### Phased Releases

- **v0.4.0**: Phase 1-2 (Basic restart + heartbeats)
- **v0.5.0**: Phase 3-4 (Timeouts + lifecycle hooks)
- **v0.6.0**: Phase 5-6 (Degradation + distributed)

---

## Conclusion

This proposal provides a comprehensive fault tolerance system for dora-rs that:

1. **Preserves performance**: Zero overhead in data plane, control plane only
2. **Maintains simplicity**: Declarative YAML configuration, opt-in features
3. **Ensures compatibility**: No breaking changes, additive API
4. **Supports real-world use cases**: Robotics, edge computing, production AI systems
5. **Provides migration path**: Phased rollout over 6-8 months

The design draws from proven patterns in distributed systems (Kubernetes restart policies, Erlang supervision trees, Apache Flink checkpointing) while respecting dora-rs's core values of performance and simplicity.

**Next Steps**:

1. **Community feedback**: Gather input on this proposal
2. **RFC process**: Formalize as dora-rs RFC
3. **Proof of concept**: Implement Phase 1 MVP in feature branch
4. **Iterate**: Refine based on real-world testing

---

## References

- **Current codebase analysis**: `binaries/daemon/src/lib.rs`, `libraries/message/src/common.rs`
- **Similar systems**:
  - Kubernetes restart policies: https://kubernetes.io/docs/concepts/workloads/pods/pod-lifecycle/
  - Erlang OTP supervision trees: https://www.erlang.org/doc/design_principles/sup_princ.html
  - Apache Flink checkpointing: https://nightlies.apache.org/flink/flink-docs-master/docs/dev/datastream/fault-tolerance/checkpointing/
  - ROS2 lifecycle nodes: https://design.ros2.org/articles/node_lifecycle.html
- **Circuit breaker pattern**: https://martinfowler.com/bliki/CircuitBreaker.html
- **Chaos engineering**: https://principlesofchaos.org/
