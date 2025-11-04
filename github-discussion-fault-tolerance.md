# RFC: Fault Isolation and System Fault Tolerance

## Summary

This RFC proposes adding comprehensive fault tolerance and recovery mechanisms to dora-rs, enabling production-grade robotic and AI systems that can automatically recover from node failures while maintaining dora's performance characteristics.

**Design Philosophy**: The framework focuses on **dataflow orchestration** (restart policies, health monitoring, lifecycle management), while **nodes manage their own state**. This keeps the framework simple and flexible.

## Motivation

### Current Limitations

Today, dora-rs provides basic fault isolation (separate processes per node) but lacks recovery mechanisms:

- ❌ **No automatic recovery**: When a node crashes, the dataflow terminates
- ❌ **Cascading failures**: Downstream nodes block indefinitely waiting for inputs
- ❌ **No graceful degradation**: System cannot continue with reduced functionality
- ❌ **No configurability**: Cannot specify per-node fault handling policies

This makes dora-rs unsuitable for production systems requiring high availability (robots, edge AI, autonomous vehicles).

### Current State (Code Analysis)

From reviewing the codebase, dora-rs **does** have basic fault detection:

```rust
// binaries/daemon/src/spawn.rs:520
let exit_status = NodeExitStatus::from(child.wait().await);

// libraries/message/src/common.rs
pub struct NodeError {
    pub cause: NodeErrorCause,  // Cascading, FailedToSpawn, GraceDuration, Other
    pub exit_status: NodeExitStatus,  // Success, ExitCode, Signal, IoError
}
```

But lacks recovery - errors are simply reported and the dataflow terminates.

## Proposed Solution

### Design Principles

1. **Performance first** - Zero overhead in hot path (shared memory communication)
2. **Opt-in** - Existing dataflows work unchanged
3. **Declarative** - Configure via YAML
4. **Real-time friendly** - Fast recovery for time-sensitive systems

### YAML Configuration Example

```yaml
nodes:
  - id: camera
    path: camera-node
    outputs: [image]

    # NEW: Fault tolerance configuration
    fault_tolerance:
      # Restart policy: disabled (default) | on-failure-only | always-restart
      # - disabled: Never restart (current behavior)
      # - on-failure: Restart only if node crashes (most common, 90% of cases)
      # - always-restart: Restart whenever node exits, even on success (rare, periodic tasks)
      restart_policy: on-failure-only
      max_restarts: 5
      restart_delay: 1s

      health_check:
        heartbeat_interval: 5s
        heartbeat_timeout: 15s

      on_failure:
        action: restart  # restart | fail_dataflow | degrade
        notify: [monitoring-node]

  - id: yolo
    path: yolo-node
    inputs:
      image: camera/image

    fault_tolerance:
      restart_policy: on-failure-only

      # Timeout if no input for 1 second
      input_timeouts:
        image: 1s

      # Use fallback if upstream fails
      on_upstream_failure:
        action: use_fallback
        fallback_source: backup-camera/image

  - id: slam
    path: slam-node

    fault_tolerance:
      restart_policy: on-failure-only
      graceful_shutdown_timeout: 10s  # Give node time to save

      # Node manages its own state, framework passes metadata
      restart_environment:
        STATE_DIR: /data/slam-state

  - id: safety-controller
    path: controller

    fault_tolerance:
      # Critical node - terminate dataflow if it fails
      restart_policy: disabled
      on_failure:
        action: fail_dataflow
```

## Implementation Plan

### Phase 1: Basic Restart Policies (MVP) - 4-6 weeks

**Core functionality:**
- Daemon tracks restart counts per node
- Evaluate restart policy on node exit
- Exponential backoff for restart delays
- Update protocol messages for `NodeRestarted` event

**Files changed:**
- `libraries/core/src/descriptor/mod.rs` - Parse fault_tolerance YAML
- `binaries/daemon/src/lib.rs` - Restart logic in `handle_node_stop_inner()`
- `libraries/message/src/` - New protocol messages

**Testing:**
- Unit tests for policy evaluation
- Integration: kill node, verify restart
- Integration: exceed max_restarts, verify failure

### Phase 2: Health Monitoring & Heartbeats - 3-4 weeks

**Core functionality:**
- Node API: `node.start_heartbeat(interval)`
- Daemon monitors heartbeats, restarts unhealthy nodes
- Proactive detection before full crash

**API changes (additive, non-breaking):**
```rust
// Rust
impl DoraNode {
    pub fn start_heartbeat(&self, interval: Duration) -> HeartbeatHandle;
    pub fn report_health(&self, healthy: bool, message: Option<String>) -> Result<()>;
}

// Python
class Node:
    def start_heartbeat(self, interval_ms: int) -> HeartbeatHandle:
    def report_health(self, healthy: bool, message: Optional[str] = None):
```

### Phase 3: Input Timeouts & Circuit Breakers - 4-5 weeks

**Prevents cascading failures:**
- Timeout on expected inputs (don't block forever)
- Circuit breaker pattern for failing nodes
- New event types: `Event::InputTimeout`, `Event::CircuitBreakerOpen`

```rust
impl DoraNode {
    // NEW: Timeout-aware input reading
    pub async fn recv_timeout(&self, timeout: Duration) -> Result<Option<Event>>;
}
```

### Phase 4: Graceful Lifecycle & Node-Managed State - 2-3 weeks

**Philosophy**: State management is the node's responsibility. Framework provides lifecycle infrastructure.

**Framework provides:**
- Graceful shutdown signals (give nodes time to save)
- Lifecycle events (Started, Restarted, Stopping)
- Restart metadata (previous run ID, failure timestamp)

**Node API:**
```rust
impl DoraNode {
    // Check if we're recovering from crash
    pub fn is_restart(&self) -> bool;

    // Get context about previous run
    pub fn get_restart_metadata(&self) -> Option<RestartMetadata>;
}
```

**Example: SLAM node manages its own state**
```rust
fn main() -> Result<()> {
    let (node, events) = DoraNode::init_from_env()?;

    // Node decides how to restore
    let mut slam = if node.is_restart() {
        SLAM::restore_from_disk("/data/slam")?  // Node's code
    } else {
        SLAM::new()
    };

    // Node decides when to checkpoint
    for event in events {
        // Process, periodically save to disk...
    }
}
```

### Phase 5: Graceful Degradation - 3-4 weeks

**Features:**
- Optional nodes (dataflow continues if they fail)
- Fallback data sources
- Start-on-demand nodes (activate when primary fails)

### Phase 6: Distributed Fault Tolerance - 6-8 weeks

**Handle daemon crashes:**
- Coordinator monitors daemon heartbeats
- Optional: Node migration to healthy daemons
- Out of scope for MVP, future work

## Performance Impact

**Hot path (message passing): ZERO overhead**
- No changes to shared memory communication
- No changes to Arrow data format
- Zero-copy semantics preserved

**Control plane: Minimal overhead**
- Heartbeats: 5s interval, negligible CPU
- Lifecycle management: Only during restarts
- Health checks: Separate channel, not in data path

**Benchmarking commitment**:
- Existing `examples/benchmark` must show <1% regression
- Add fault-tolerance-enabled benchmark

## Backwards Compatibility

**100% backwards compatible:**

1. **All features opt-in**:
   - Default `restart_policy: never` (current behavior)
   - No YAML config = no behavior change

2. **Additive API only**:
   - New methods: `start_heartbeat()`, `is_restart()`, `get_restart_metadata()`
   - Existing methods unchanged
   - No breaking changes

3. **Protocol versioning**:
   - New message types added
   - Old daemons ignore unknown messages
   - Forward compatible

## Use Cases

### 1. Vision Pipeline for Mobile Robot
```yaml
# Hardware can fail, restart automatically
- id: camera
  fault_tolerance:
    restart_policy: on-failure-only
    max_restarts: 10
    health_check:
      heartbeat_interval: 2s

# GPU-intensive, can OOM, restart with delay
- id: yolo
  fault_tolerance:
    restart_policy: on-failure-only
    restart_delay: 5s

# Safety-critical, never allow failure
- id: controller
  fault_tolerance:
    restart_policy: never
    on_failure:
      action: fail_dataflow
```

### 2. SLAM System (Stateful)
```yaml
- id: slam
  fault_tolerance:
    restart_policy: on-failure-only
    graceful_shutdown_timeout: 10s
    restart_environment:
      STATE_DIR: /data/slam-map
```

Node code manages state:
```rust
// SLAM node's responsibility
let mut slam = if node.is_restart() {
    SLAM::restore_from_disk(env::var("STATE_DIR")?)?
} else {
    SLAM::new()
};
```

### 3. Redundant Sensors
```yaml
- id: primary-lidar
  fault_tolerance:
    restart_policy: on-failure-only

- id: backup-lidar
  fault_tolerance:
    optional: true
    start_on_demand: true  # Only if primary fails

- id: perception
  fault_tolerance:
    on_upstream_failure:
      action: use_fallback
      fallback_source: backup-lidar/pointcloud
```

## Related Work

- **Kubernetes**: Restart policies (Never/OnFailure/Always), liveness probes
- **Erlang OTP**: Supervision trees with different strategies
- **Apache Flink**: Checkpointing for stateful stream processing
- **ROS2**: Lifecycle nodes with managed states
- **Circuit Breaker**: Martin Fowler's pattern for cascading failure prevention

## Open Questions

1. **Checkpoint storage**:
   - Option A: In-memory (simple, lost on daemon crash)
   - Option B: Persistent disk (survives daemon crash)
   - Option C: Remote storage (S3, distributed)
   - **Proposed**: Start with A, add B later

2. **Recovery semantics**:
   - Exactly-once (complex, like Flink)
   - At-least-once (simpler, may duplicate)
   - Best-effort (no guarantees)
   - **Proposed**: Best-effort, document limitations

3. **Configuration profiles**:
   - Should we support `fault_tolerance: resilient` as shorthand?
   - Trade-off: simplicity vs fine-grained control
   - **Proposed**: Support both

4. **Observability**:
   - How to expose metrics? (restart count, circuit breaker state)
   - OpenTelemetry integration?
   - **Proposed**: Metrics via OpenTelemetry, already integrated

## Success Metrics

**Functional:**
- ✅ Nodes restart automatically on failure
- ✅ Dataflows survive individual crashes
- ✅ Stateful nodes restore state
- ✅ Cascading failures prevented

**Performance:**
- ✅ <1% overhead with all features enabled
- ✅ <100ms restart latency
- ✅ <10MB memory overhead per dataflow

**Usability:**
- ✅ Zero breaking changes
- ✅ Simple YAML for 80% use cases
- ✅ Clear documentation + examples

## Timeline

- **Phase 1-2** (MVP): 7-10 weeks → v0.4.0
- **Phase 3-4**: 6-8 weeks → v0.5.0 (simplified, node-managed state)
- **Phase 5-6**: 9-12 weeks → v0.6.0

**Total**: 5.5-7.5 months for full implementation

## Design Rationale: Why Node-Managed State?

This proposal initially included framework-managed checkpointing (like Apache Flink). However, we reconsidered:

**Why nodes should manage their own state:**

1. **Simplicity**: Framework focuses on orchestration, not application semantics
2. **Flexibility**: Different domains have different needs:
   - SLAM: Complex maps in HDF5
   - Tracking: Simple JSON state
   - Database nodes: State in external DB
   - Vision models: No state, just reload weights
3. **Domain expertise**: Nodes know what's critical vs. recomputable
4. **Existing patterns**: Most domains already have state management (ROS bags, databases, checkpoint files)
5. **Separation of concerns**: Framework = process lifecycle, Node = business logic + state

**What the framework DOES provide:**
- Graceful shutdown (time to save)
- Restart detection (`is_restart()`)
- Metadata passing (failure context)
- Lifecycle events

This keeps dora-rs **simple, fast, and flexible** - aligned with its core philosophy.

## Request for Comments

This is a significant addition to dora-rs. We're seeking feedback on:

1. **Scope**: Is this the right set of features? Missing anything critical?
2. **API design**: Are the proposed APIs intuitive? Rust-idiomatic?
3. **YAML config**: Too complex? Need simpler defaults?
4. **State management approach**: Agree with node-managed state? Or prefer framework checkpointing?
5. **Phasing**: Should we reorder priorities?
6. **Use cases**: What real-world scenarios should we support?

Please share your thoughts, especially if you're:
- Building production systems with dora-rs
- Experienced with fault-tolerant distributed systems
- Maintaining robotics/AI deployments

## Related Issues

- #XXX - Node crash causes dataflow termination
- #XXX - No way to restart failed nodes
- #XXX - Cascading failures in dataflow
- (Please link relevant issues)

---

**Full detailed proposal**: See `fault-tolerance-proposal.md` for complete technical design, implementation details, code examples, and test strategy.
