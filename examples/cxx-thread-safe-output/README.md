# Thread-Safe `send_output` (C++)

This example shows how a C++ Dora node can send output **from worker
threads it spawns itself**, using the thread-safe `SafeOutputSender`
wrapper in the C++ node API.

## The problem

A traditional Dora C++ node runs every handler serially in one
`events->next()` loop. A single slow handler (e.g. 200 ms of image
processing) blocks the whole loop, so high-frequency inputs (IMU,
control ticks) pile up and go stale.

The natural fix is to offload slow work to a worker thread — but the
plain `send_output` is **not** thread-safe. It mutates the node's
internal sender state, so calling it from a worker thread is undefined
behavior. This example demonstrates the supported alternative.

## How it works

1. `init_dora_node()` returns the node, including its `send_output`.
2. `clone_output_sender(dora_node.send_output)` clones the shared node
   handle into a `SafeOutputSender`. It **borrows** the original sender
   rather than consuming it, so `dora_node.send_output` stays valid and
   keeps the full single-threaded API (`log_message`, `close_outputs`,
   `node_config_json`, …) on the main thread. The example calls both to
   show that.
3. On every `tick` (100 ms), the main loop spawns a worker thread that
   simulates a slow task (300 ms sleep) and then calls
   `safe_send_output(*safe_sender, "result", ...)`. Because the work is
   three times the tick interval, several workers are in flight at once
   and genuinely contend for the node lock.
4. The main loop returns to `events->next()` immediately instead of
   blocking, so it stays responsive while workers run.

Both handles point at the same node behind the same lock, so a worker's
send and a main-thread send serialize instead of racing. Workers are
joined before `safe_sender` is dropped, since they borrow it.

If a send ever panics while holding the lock, the node is left poisoned
and every later operation through *either* handle fails with an error
mentioning `poisoned`, rather than reusing a daemon connection that may
be mid-frame. There is no recovery: the node has to be restarted.

## Run

```bash
cargo run --example cxx-thread-safe-output
```

## Expected output

Note how the main loop dispatches new ticks *before* earlier workers
finish — the loop never blocks on the 300 ms work:

```
HELLO FROM C++ (thread-safe output)
[main] received input 'tick' -> dispatching to worker 0
[main] received input 'tick' -> dispatching to worker 1
[worker 0] sent result from worker thread
[worker 1] sent result from worker thread
...
GOODBYE FROM C++ (thread-safe output)
```

## When to use this vs. multiple nodes

Splitting slow and fast work into **separate Dora nodes** is the
idiomatic way to parallelize in Dora, and is usually the right choice —
the daemon schedules nodes across cores for you. Reach for in-node
worker threads + `SafeOutputSender` only when the work must share
in-process state that would be awkward to pass between nodes.
