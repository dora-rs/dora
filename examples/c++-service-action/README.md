# Dora C++ Service and Action Example

This example demonstrates the **Service** (request/reply) and **Action**
(goal/feedback/result) communication patterns from C++, using the primitives
exposed in `dora-node-api.h`. It mirrors the Rust
[`service-example`](../service-example/) and [`action-example`](../action-example/)
so the two can be read side by side.

See [`docs/patterns.md`](../../docs/patterns.md) for the pattern
specification, and dora-rs/dora#2686 for the parity request this example
came from.

## Running

```bash
cargo run --example cxx-service-action
```

Requires `clang++` with C++20 support. The harness builds
`dora-node-api-cxx`, compiles the four nodes into `build/`, and then runs
`dataflow.yml`.

## What each node does

| Node | Source | Role |
|------|--------|------|
| `cxx-service-client` | `nodes/service-client.cc` | Sends `a + b`, blocks for the correlated sum |
| `cxx-service-server` | `nodes/service-server.cc` | Computes the sum, echoes `request_id` back |
| `cxx-action-client` | `nodes/action-client.cc` | Sends a countdown goal, blocks for the terminal result |
| `cxx-action-server` | `nodes/action-server.cc` | Streams countdown feedback, then a `succeeded` result |

## Service: request / reply

`send_service_request` generates a UUID v7 `request_id`, injects it into the
metadata and returns it. You do not set the key yourself:

```cpp
auto request = send_service_request(
    node.send_output, "request", payload, new_metadata());
if (!std::string(request.error).empty()) { /* the send failed */ }
const std::string request_id(request.request_id);
```

On failure `request_id` is empty, so a caller that ignores `error` cannot
end up waiting on a correlation that was never sent.

`recv_service_response` then blocks for that specific reply, with a timeout
and server-restart detection:

```cpp
auto reply = recv_service_response(
    node.events, request_id, "cxx-service-server", 5000);

switch (reply.status) {
case DoraPatternStatus::Matched:         /* reply.event is the response */ break;
case DoraPatternStatus::Timeout:         /* deadline elapsed */            break;
case DoraPatternStatus::ServerRestarted: /* orphaned; retry */             break;
case DoraPatternStatus::StreamEnded:     /* dataflow stopping */           break;
default:                                 /* reply.error has details */     break;
}
```

Events that arrive during the wait but do not match are buffered and
replayed by later `next_event` calls, so the main event loop loses nothing.

The server must read the incoming `request_id` and pass it back. That needs
**`event_as_input_with_metadata`** — the older `event_as_input` returns only
the payload:

```cpp
auto input = event_as_input_with_metadata(std::move(event));
// ... compute the reply ...
send_service_response(
    node.send_output, "response", result, std::move(input.metadata));
```

## Action: goal / feedback / result

Actions correlate on `goal_id`, which the client sets explicitly because the
same id also labels the feedback stream:

```cpp
const std::string goal_id(new_goal_id());
auto metadata = new_metadata();
metadata->set_goal_id(goal_id);
send_output_with_metadata(node.send_output, "goal", payload, std::move(metadata));

auto outcome = recv_action_result(
    node.events, goal_id, "cxx-action-server", 5000);
```

`recv_action_result` returns only on a **terminal** status. The server tags
feedback with `goal_id` alone, and the final message with `goal_id` plus
`goal_status`:

```cpp
// feedback — no goal_status, so the client keeps waiting
metadata->set_goal_id(goal_id);

// terminal result
metadata->set_goal_id(goal_id);
metadata->set_goal_status(goal_status_succeeded());  // or _aborted() / _canceled()
```

Use the `goal_status_*()` accessors rather than string literals: a typo
would leave the client waiting until its timeout instead of failing loudly.

Because feedback arriving during the wait is buffered rather than consumed,
the client's main loop still prints every feedback message after the result
is handled. That replay is visible in this example's output.

## Expected output

Interleaved across the four nodes, roughly:

```
[cxx-service-client] sent request 0199... : 0 + 10
[server] 0199...: 0 + 10 = 10
[cxx-service-client] response 0199...: 0 + 10 = 10
[cxx-action-client] sent goal 0199...: countdown from 3
[server] accepted goal 0199...: countdown from 3
[server] result 0199...: succeeded
[cxx-action-client] result 0199...: succeeded
[cxx-action-client] feedback 0199...: 2
[cxx-action-client] feedback 0199...: 1
[cxx-action-client] feedback 0199...: 0
```

Both clients exit after a fixed number of exchanges, which closes their
outputs and shuts the servers down.
