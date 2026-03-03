# Action Example

Demonstrates the **action** communication pattern: a client sends goals to a server, receives intermediate feedback, and gets a final result. Supports cancellation of in-progress goals.

## Architecture

```
timer (2s) --> action-client --> goal     --> action-server
                              <- feedback <--
                              <- result   <--
               action-client --> cancel   --> action-server
```

## Nodes

**action-client** (Rust) -- Sends a new goal every 2 seconds with a countdown value. Tracks active goals by `GOAL_ID` in a HashMap. Receives feedback (progress updates) and result (final value). A `cancel` output is wired in the YAML for extensibility but is not exercised in the current client code.

**action-server** (Rust) -- Manages up to 64 active goals and 128 pending cancellations. On each input event, decrements the countdown for all active goals, sends feedback with current progress. When a goal reaches zero, sends a result with `GOAL_STATUS_SUCCEEDED`. Accepts `cancel` input for goal cancellation.

## Key Concepts

- **Goal ID**: unique identifier created via `AdoraNode::new_goal_id()` (UUID v7)
- **Metadata keys**: `GOAL_ID` and `GOAL_STATUS` propagated on all action messages
- **Bidirectional**: server outputs feed back to client inputs

See [docs/patterns.md](../../docs/patterns.md#3-action-goalfeedbackresult) for the full pattern specification.

## Run

```bash
cargo build -p action-example-client -p action-example-server
adora run dataflow.yml
```

Or in distributed mode:

```bash
adora up
adora start dataflow.yml --attach
# Ctrl+C to detach, then:
adora stop --all && adora down
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| Action pattern (goal/feedback/result) | Both nodes |
| `new_goal_id()` for unique goal IDs | Client |
| `GOAL_ID` / `GOAL_STATUS` metadata | YAML + both nodes |
| Cancel wiring (extensibility) | YAML declares cancel output/input |
| Concurrent goal tracking | Server HashMap with max 64 goals |
