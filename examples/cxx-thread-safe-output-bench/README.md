# Thread-Safe Output — Latency Benchmark (C++)

Measures **tick-handling latency** under three ways of dealing with a slow
handler, to answer: does offloading slow work to a worker thread (using
`SafeOutputSender`) actually keep a node responsive, and how does it compare to
the idiomatic Dora approach of splitting work across nodes?

## Setup

A `source` node fires two outputs, both driven by built-in timers:

* `tick` every 10 ms — payload is the wall-clock send time (8 bytes), so the
  receiver can measure how long the tick waited before being handled.
* `image` every 1000 ms — the slow-work trigger.

A single contestant binary (`bench-node`) runs in one of four modes, selected by
the `BENCH_MODE` env var in the dataflow YAML:

| Mode | Subscribes to | Behavior |
|---|---|---|
| `blocking` | tick + image | Does the 200 ms image work **inline** (blocks the loop). |
| `worker` | tick + image | Offloads image work to a worker thread that sends via `SafeOutputSender`. |
| `measure` | tick | Tick handler for the two-node config. |
| `slow` | image | Separate image node for the two-node config. |

Three dataflows wire these together: `dataflow-blocking.yml`,
`dataflow-worker.yml`, and `dataflow-twonode.yml` (the idiomatic
split-into-two-nodes approach).

## Run

```bash
cargo run --example cxx-thread-safe-output-bench
```

Each measuring node prints one `BENCH_RESULT ...` line. Compare the `p99_ms`
column — the near-worst latency, which is what matters for real-time work.

## Example results

A representative local run (12 s per configuration, 200 ms image work):

| Contestant | p50 | **p99** | max |
|---|---|---|---|
| `blocking` (inline) | 1.6 ms | **91.7 ms** | 101.0 ms |
| `worker` (worker thread + SafeOutputSender) | 1.6 ms | **3.0 ms** | 5.2 ms |
| `two nodes` (idiomatic Dora) | 1.6 ms | **1.8 ms** | 4.3 ms |

## Interpretation

Both the worker-thread pattern and the split-into-two-nodes approach eliminate
the head-of-line blocking — p99 drops from ~92 ms to a few milliseconds (a ~30×
improvement). The two approaches are in the same ballpark; the two-node version
is marginally better because separate processes don't share a thread at all.

The honest takeaway: `SafeOutputSender` does its job, but it is not faster than
Dora's existing multi-node design. Reach for in-node worker threads when the
slow and fast work must share in-process state; otherwise multiple nodes remain
the recommended default.

> Caveat: the `blocking` max comes out around 100 ms rather than the full
> 200 ms because Dora's input queues drop stale ticks by default instead of
> letting them pile up unbounded — so the real-world cost of blocking is
> *dropped data plus* ~90 ms p99.
