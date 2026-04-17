# Action Example

Demonstrates the **action** communication pattern with **multiple clients**: two clients send goals to a single server, receive intermediate feedback, and get final results. Each client filters responses by its own tracked goal IDs. Supports cancellation of in-progress goals.

## Architecture

```
timer (2s) --> action-client-1 --> goal   --> action-server --> feedback --> action-client-1
                                                            --> result   --> action-client-1
timer (3s) --> action-client-2 --> goal   -->               --> feedback --> action-client-2
                                                            --> result   --> action-client-2
```

Both clients use the same binary -- the YAML wires them as separate nodes with different timers. Feedback and results fan out to all clients; each filters by its own tracked `GOAL_ID`.

## Nodes

**action-client-1 / action-client-2** (Rust, same binary) -- Sends goals at 2s / 3s intervals with a countdown value. Tracks active goals by `GOAL_ID` in a HashMap. Receives feedback (progress updates) and result (final status). Logs are prefixed with the node ID (`DORA_NODE_ID`) to distinguish output.

**action-server** (Rust) -- Manages up to 64 active goals and 128 pending cancellations from any number of clients. On each input event, decrements the countdown for all active goals, sends feedback with current progress. When a goal reaches zero, sends a result with `GOAL_STATUS_SUCCEEDED`. Accepts `cancel` inputs for goal cancellation. Uses prefix matching (`starts_with("goal")` / `starts_with("cancel")`) to handle multiple named inputs.

## Key Concepts

- **Multi-client fan-in/fan-out**: multiple clients share one server; feedback/results fan out, clients correlate by ID
- **Goal ID**: unique identifier created via `DoraNode::new_goal_id()` (UUID v7)
- **Metadata keys**: `GOAL_ID` and `GOAL_STATUS` propagated on all action messages
- **Bidirectional**: server outputs feed back to client inputs
- **Same binary, multiple nodes**: YAML declares distinct nodes pointing to the same executable

See [docs/patterns.md](../../docs/patterns.md#3-action-goalfeedbackresult) for the full pattern specification.

## Run

```bash
cargo build -p action-example-client -p action-example-server
dora run dataflow.yml
```

Or in distributed mode:

```bash
dora up
dora start dataflow.yml --attach
# Ctrl+C to detach, then:
dora stop --all && dora down
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| Multi-client action pattern | Two clients, one server |
| `new_goal_id()` for unique goal IDs | Client |
| `GOAL_ID` / `GOAL_STATUS` metadata | YAML + both nodes |
| Cancel wiring (extensibility) | YAML declares cancel output/input |
| Concurrent goal tracking | Server HashMap with max 64 goals |
| Same binary, multiple nodes | YAML dataflow |
