# Dynamic Topology: Add/Remove Nodes Mid-Execution

## Context

Dora's topology is currently static — all nodes and connections must be declared in the YAML descriptor before the dataflow starts. The `mappings` routing table is built once at `spawn_dataflow()` and never modified. This is the "Static topology" constraint from the audit report.

For AI agent use cases, this means you can't dynamically spin up new LLM agents, add sensor nodes, or remove failed components without restarting the entire dataflow. The goal: enable `dora node add` / `dora node remove` on a running dataflow.

## Approach: Incremental Topology Mutation

Leverage the existing single-threaded daemon event loop — `mappings`, `open_inputs`, and `subscribe_channels` are already mutable HashMap/BTreeMap owned by the event loop. No locking needed. New messages in the coordinator-daemon protocol trigger additive mutations.

## Phase 1: Single-Daemon Add/Remove

### 1.1 New message types

**Files:** `libraries/message/src/cli_to_coordinator.rs`, `coordinator_to_daemon.rs`

Add to `ControlRequest`:
```rust
AddNode { dataflow_id, node: Node }
RemoveNode { dataflow_id, node_id, grace_duration }
AddMapping { dataflow_id, source_node, source_output, target_node, target_input }
RemoveMapping { dataflow_id, source_node, source_output, target_node, target_input }
```

Add to `DaemonCoordinatorEvent`:
```rust
AddNode { dataflow_id, node: ResolvedNode, dataflow_descriptor, uv }
RemoveNode { dataflow_id, node_id, grace_duration }
AddMapping { dataflow_id, source: OutputId, target_node, target_input }
RemoveMapping { dataflow_id, source: OutputId, target_node, target_input }
```

### 1.2 Daemon `add_node_to_dataflow`

**File:** `binaries/daemon/src/lib.rs`

1. Resolve new node's inputs — insert into `dataflow.mappings`
2. Insert into `dataflow.open_inputs`
3. Start timer tasks if node has timer inputs
4. Spawn node via existing `Spawner::spawn_node`
5. On subscribe, register in `subscribe_channels` (existing flow)
6. Mark in `dataflow.running_nodes`

### 1.3 Daemon `remove_node_from_dataflow`

**File:** `binaries/daemon/src/lib.rs`, `running_dataflow.rs`

1. `stop_single_node` (existing)
2. Clean all maps: remove from `mappings` (both as source and receiver), `open_inputs`, `subscribe_channels`, `drop_channels`, `pending_messages`, `running_nodes`, `log_subscribers`, `timers`
3. Send `InputClosed` to downstream nodes that depended on the removed node's outputs

### 1.4 Daemon `add_mapping` / `remove_mapping`

**File:** `binaries/daemon/src/lib.rs`

- `add_mapping`: insert `(target_node, target_input)` into `mappings[OutputId(source)]`, add to `open_inputs[target_node]`
- `remove_mapping`: reverse — remove from `mappings`, send `InputClosed` if was last mapping for that input

### 1.5 Coordinator relay

**File:** `binaries/coordinator/src/handlers.rs`

- Validate no ID collision with existing nodes
- Resolve `Node` to `ResolvedNode`
- Determine target daemon
- Send `DaemonCoordinatorEvent::AddNode` to daemon
- Update coordinator `RunningDataflow.nodes` and `node_to_daemon`

## Phase 2: Cross-Daemon Dynamic Topology

### 2.1 Remote source subscriptions

When a dynamically added node has inputs from nodes on other daemons, declare Zenoh subscribers. Extract existing code from `spawn_dataflow` lines 1837-1895 into reusable `subscribe_remote_outputs()`.

### 2.2 Notify remote daemons

Coordinator sends `AddMapping` to each affected remote daemon so they create Zenoh subscribers for the new node's outputs.

### 2.3 Remote removal

When a node is removed, its Zenoh publishers stop. Existing `OutputClosed` inter-daemon events propagate naturally.

## Phase 3: CLI Integration

### 3.1 CLI commands

**File:** `binaries/cli/src/command/node/add.rs` (new), `remove.rs` (exists, extend)

```
dora node add <dataflow> --from-yaml <node-def.yml>
dora node add <dataflow> --id <id> --path <exe> --input tick=dora/timer/hz/10
dora node remove <dataflow> <node-id> [--grace 5s]
dora node connect <dataflow> <source/output> <target/input>
dora node disconnect <dataflow> <source/output> <target/input>
```

### 3.2 Info enhancement

`dora info` shows `[dynamic]` tag on dynamically added nodes.

## Key Files

| File | Change |
|------|--------|
| `libraries/message/src/cli_to_coordinator.rs` | AddNode/RemoveNode/AddMapping ControlRequest variants |
| `libraries/message/src/coordinator_to_daemon.rs` | AddNode/RemoveNode/AddMapping DaemonCoordinatorEvent variants |
| `binaries/daemon/src/lib.rs` | handle_add_node, handle_remove_node, handle_add_mapping |
| `binaries/daemon/src/running_dataflow.rs` | Mutation methods on RunningDataflow |
| `binaries/coordinator/src/handlers.rs` | Coordinator relay handlers |
| `binaries/coordinator/src/run/mod.rs` | Factor out node resolution logic |
| `binaries/cli/src/command/node/add.rs` | New CLI command |

## Risks

- **Descriptor becomes stale**: `RunningDataflow.descriptor` won't reflect dynamic changes. Accept for now (it's "initial topology").
- **Recording**: `.adorec` header won't include dynamic nodes. Document as limitation.
- **Restart behavior**: `dora restart <dataflow>` should only restart original descriptor nodes. Dynamic nodes are caller's responsibility.
- **No message replay**: New nodes miss messages sent before they joined. Same as Zenoh late-subscribe behavior.

## Verification

1. Unit test: add node to RunningDataflow, verify mappings updated
2. Integration test: start dataflow, `dora node add`, send message, verify new node receives it
3. Integration test: `dora node remove`, verify cleanup (no phantom restarts, no dangling channels)
4. Smoke test: dynamic add in networked mode (dora up + dora node add)

## Open Questions

1. Should dynamically added nodes survive `dora restart <dataflow>`?
2. Max dynamic nodes limit per dataflow?
3. Should `dora node add --from-yaml` support inline node definition or require a file?
