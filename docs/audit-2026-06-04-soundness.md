# Dora Soundness Audit — Hardening Report

Scope: harden the current feature set (fill holes, fix fragility). No new features. `file:line` cited throughout. High findings were adversarially re-verified against source; medium/low are auditor-reported unless marked verified. Generated 2026-06-04 by a multi-agent read-only audit (14 subsystem/cross-cutting auditors → per-finding adversarial verification → synthesis). 74 findings: 8 high, 44 medium, 22 low; 0 dropped as false positive.

---

## 1. Executive Summary

- **The coordinator's daemon-disconnect cleanup is a single point of multiple deadlocks.** `cleanup_disconnected_daemons_from_running_dataflows` (`binaries/coordinator/src/lib.rs:2899-2917`) only logs — it never re-evaluates the readiness barrier, never drains `stop_reply_senders`, and never tears the dataflow down. Three separate verified findings collapse to this one function: ready-barrier deadlock, hung `dora stop`, and orphaned running dataflows. Fixing this one site closes all three.

- **Fault-tolerance behavior contradicts its own stated intent.** The daemon's coordinator-reconnect path (`binaries/daemon/src/lib.rs:1072-1076`) kills every running node on a >20s heartbeat gap despite an inline comment claiming "Running dataflows continue." The transparent-reconnect design (#1996) is silently defeated.

- **Panic-on-malformed-input spans the C/C++ FFI boundary and config parsing.** C (`apis/c/node/src/lib.rs:196`) and C++ (`apis/c++/node/src/lib.rs:435`) node APIs `todo!()`-abort on any non-UInt8 input — a routine cross-language case. Descriptor parsing panics on `dora/timer/hz/<tiny>` (`libraries/message/src/config.rs:285`). Record/replay panics on corrupt files (`libraries/recording/src/lib.rs:160`).

- **Two silent wire-format corruptions in shipped features.** Sub-millisecond (`>1kHz`) timers serialize to `secs/0` and become busy-loops after one roundtrip (`libraries/message/src/config.rs:366`). ROS2 `sequence<bool>` is serialized as a CDR tuple with no length prefix, corrupting the field and everything after it (`.../serialize/sequence.rs:311`); empty struct-sequences fail to deserialize and drop the whole ROS2 message (`.../deserialize/sequence.rs:171`).

- **The Zenoh distributed data plane drops data and control silently.** Cross-daemon `OutputClosed` can be dropped (`binaries/daemon/src/lib.rs:3298`), hanging remote receivers; `queue_policy: backpressure` is bypassed on the zenoh path (`event_stream/mod.rs:355`); empty typed arrays become `NullArray` over zenoh but stay typed over the daemon path (`event_stream/mod.rs:1063`) — cross-transport non-determinism.

- **Security/transport-framing guards are correct but untested**, so a regression would ship silently: coordinator auth (`ws_server.rs:103`) and both sides of the length-prefixed TCP framing DoS guards (`socket_stream_utils.rs`, `daemon_connection/tcp.rs`) have zero tests.

---

## 2. Critical

No findings rise to memory-unsafety/UB-level critical. The verifiers downgraded the candidate FFI-unwind items: cross-`extern "C"` unwind is a *defined forced abort* in current Rust, not UB. The highest-impact items are in High.

---

## 3. High

### Coordinator daemon-disconnect cleanup — one function, three deadlocks (verified)
Root cause: `cleanup_disconnected_daemons_from_running_dataflows` (`binaries/coordinator/src/lib.rs:2899-2917`) mutates sets and logs only.

- **Ready-barrier deadlock** — `binaries/coordinator/src/lib.rs:479-507` / `2899-2916`. If the last pending daemon disconnects after spawn success but before `ReadyOnDaemon`, `pending_daemons` empties via cleanup but `AllNodesReady` never broadcasts; surviving nodes block forever. *Fix:* after cleanup, for each affected dataflow whose `pending_daemons` just emptied AND `daemons` non-empty, broadcast `AllNodesReady` (or terminally fail). Test: 2-daemon dataflow, ready from one, disconnect the other, assert AllNodesReady or failure.
- **`dora stop` waiter hangs** — `binaries/coordinator/src/lib.rs:911` / `614` / `2906`. If all daemons disconnect after a stop is parked, `stop_reply_senders` is never drained. *Fix:* in cleanup, when `daemons` becomes empty, drain `stop_reply_senders` with an error.
- **Orphaned running dataflow** — `binaries/coordinator/src/lib.rs:2899-2917`. Fully-running dataflow whose sole daemon dies lingers as "running" with zero daemons; `dataflow_results`/`spawn_result` never resolve. *Fix:* on `affected && daemons.is_empty()` for a spawned dataflow, mirror `check_spawn_timeouts` teardown (Err results, persist `Failed{terminal:true}`, archive, remove).

One consolidated fix to this cleanup function + one integration test per scenario closes all three.

### Coordinator reconnect kills all running nodes (verified)
`binaries/daemon/src/lib.rs:1072-1076`. `bail!("coordinator heartbeat timeout (20s)")` drops the by-value `Daemon`, dropping each `RunningNode` → `ProcessHandle::drop` submits `Kill` to a still-alive wait task (`spawn/prepared.rs:650-666`), killing every node. Comment "Running dataflows continue" is false. *Fix:* stash `ProcessHandle`s across reconnect so they aren't dropped, OR correct the comment if kill-on-disconnect is intended. Test: assert a spawned node's PID survives a simulated >20s heartbeat gap.

### C and C++ node APIs abort on routine cross-language input (verified)
- C: `apis/c/node/src/lib.rs:195-198` — `read_dora_input_data` hits `todo!()` (forced abort) on any non-UInt8/Null input; no Arrow-FFI escape hatch exists. *Fix:* write null ptr + len 0 and `tracing::error!` the unsupported type. Test: non-UInt8 input → `out_ptr=NULL/out_len=0`, no panic.
- C++: `apis/c++/node/src/lib.rs:434-437` — `event_as_input` `todo!()` aborts via cxx bridge; the fn already returns `Result`. *Fix:* `bail!("unsupported input arrow type {:?}; use event_as_arrow_input ...")`. Test: Int32 Input → Err.

### Sub-millisecond timers corrupt to zero-interval on roundtrip (verified)
`libraries/message/src/config.rs:364-371`. Any interval `<1ms` (`>1kHz`) formats as `secs/0` because `subsec_millis()==0 && as_secs()==0`; descriptors round-trip on the wire, so a configured high-rate timer becomes a 0ms busy-loop on the daemon. *Fix:* emit `millis` only when whole-millisecond, else emit micros/nanos (add unit to parser). Test: `"dora/timer/hz/3000"` survives Display→parse with same Duration.

### ROS2 `sequence<bool>` serialized without CDR length prefix (verified)
`.../ros2-bridge/arrow/src/serialize/sequence.rs:298-318` (`serialize_tuple` at :311) vs the correct `serialize_seq` at :248. Variable-length bool sequences omit the mandatory u32 length prefix; deserialize (`deserialize/sequence.rs:34`) expects one. Silent data corruption of `bool[]`/`sequence<bool>` (occupancy/validity masks) and broken roundtrip. *Fix:* use `serialize_seq(Some(len))` for the variable-length path; keep `serialize_tuple` only for fixed-size arrays. Add a CDR (not serde_assert) roundtrip test.

### Empty ROS2 struct-sequence drops the whole message (verified)
`.../ros2-bridge/arrow/src/deserialize/sequence.rs:160-171`. `arrow::compute::concat` errors on empty input, failing the entire message deserialize (single `StructDeserializer` at `lib.rs:147-149`). Hits common empty fields: `MarkerArray.markers`, `Path.poses`, `DiagnosticArray.status`. *Fix:* special-case `values.is_empty()` → build a length-0 array of the correct child type. Add an empty-`sequence<struct>` deserialize test.

### MAVLink reader thread never exits/reconnects on disconnect (verified)
`binaries/mavlink2-bridge-node/src/main.rs:190-198`. On a fatal TCP disconnect the reader spins forever, sleeping 50ms and emitting a warn (~20/s); no reconnect, no fatal signal — bridge becomes a logging zombie. The same Err arm conflates transient 100ms read timeouts with fatal `Io`/EOF. *Fix:* distinguish `WouldBlock`/`TimedOut` from fatal errors; on fatal, reconnect via `connect_with_retry` or exit and surface a fatal error; rate-limit the warn.

---

## 4. Medium (grouped)

**Coordinator state lifecycle / persistence**
- `dataflow_results` grows unbounded; only `dora clean` evicts (`binaries/coordinator/src/lib.rs:535-538`, removal only at `:1215`). Bound it like `archived_dataflows` (cap 200). (confidence 9)
- One corrupt row makes `list_dataflows` fail, disabling ALL startup recovery + `dora list`/`clean` (verified) — `libraries/coordinator-store/src/redb_store.rs:213-222`; same pattern in `list_daemons` (:159) and `list_builds` (:271). *Fix:* skip-and-warn per-row, return healthy rows.

**Daemon resilience / event loop**
- Main event loop can stall on a backed-up coordinator WS channel (`binaries/daemon/src/coordinator.rs:67-73`, bounded 64). Use `try_send` + drop/log for non-critical events or a writer timeout.
- `dora run` silently drops dynamic-node support on port-in-use (verified) — `binaries/daemon/src/lib.rs:707-716` discards `Ok(None)` from `spawn_listener_loop`; dynamic nodes then fail fast (not hang, per verifier) but the daemon falsely reports success. *Fix:* `bail!` when descriptor has dynamic nodes and port is taken. (#1999, unfixed half)
- Dynamic-node listener busy-spins on persistent `accept()` error (EMFILE) with no backoff/shutdown (`binaries/daemon/src/local_listener.rs:49-67`). Add backoff + watch-shutdown like `node_communication/tcp.rs`.
- Cross-daemon `OutputClosed` can be silently dropped (verified) — `binaries/daemon/src/lib.rs:3276` (`CongestionControl::Drop`), `:3298` (lossy `try_send`); remote receiver never sees `InputClosed`. *Fix:* send control frames on a reliable/blocking path. (Verifier: lower likelihood than data-burst framing implies, but real.)
- Restarted node inherits stale `last_activity`, risking immediate health-check kill before Register (`spawn/prepared.rs:350-355` doesn't reset; `spawner.rs:147` sets it once; `lib.rs:2088-2104`). Reset to `now` on each respawn.

**Node API / Zenoh data plane**
- `queue_policy: backpressure` bypassed on zenoh path (verified) — `event_stream/mod.rs:355-371` `try_send`-and-drop ahead of the scheduler; daemon path uses `blocking_send`. At minimum count drops into `drain_drop_counts()`.
- Empty typed array → `NullArray` over zenoh but typed empty over daemon path (verified) — `event_stream/mod.rs:1063-1066` vs `arrow_utils.rs:148-150`. Cross-transport non-determinism; downstream `as_primitive` panics on one path. *Fix:* return `ArrayData::new_empty(&metadata.type_info.data_type)`.
- `recv_async` drops inputs buffered behind a Stop (verified) — `event_stream/mod.rs:505-507`; scheduler yields Stop first (`scheduler.rs:291-299`) and contradicts its own doc (`scheduler.rs:104-108`). Drain scheduler before short-circuiting, or fix the doc.
- Daemon-delivered raw payload skips per-buffer-offset alignment check the zenoh path enforces (`data_conversion.rs:30` vs `event_stream/mod.rs:1081,1113`). Potential SIGBUS on strict-alignment targets from a buggy peer. Mirror the zenoh alignment check.

**Config / descriptor parsing**
- `dora/timer/hz/<tiny>` panics in `Duration::from_secs_f64` (verified) — `libraries/message/src/config.rs:280-285`. Use `Duration::try_from_secs_f64` and return the existing Err. (Verifier: tokio task panic, not full-process DoS.)
- `parse_byte_size` overflows on large `max_log_size` (panic debug / silent wrap release) — `libraries/core/src/descriptor/validate.rs:325-331`. Use `checked_mul` + bound the float path.
- `resolve_path` returns `Ok` for nonexistent binaries whenever `uv` is installed (verified) — `libraries/core/src/descriptor/mod.rs:364-382`; ignores child exit status, defeating source-existence validation. Capture `.output()`/`.wait()`, only `Ok(path)` on success.
- `DaemonId` with a hyphenated machine_id silently dropped on reconstruct — `libraries/message/src/common.rs:325-331` (Display joins `-`) vs `:62-75` (`splitn(2,'-')`). Hostnames have hyphens. Split on the last `-` (`rsplit_once`).

**Runtime / operators**
- Operator panic payload formatted with Debug → opaque `Any { .. }` — `binaries/runtime/src/lib.rs:175-176`. Downcast `&str`/`String` before bailing.
- `let _ = span.enter()` drops the tracing guard immediately, detaching `on_event` spans + OTel context — `shared_lib.rs:170-171`, `python.rs:191-192`, `python.rs:332/352`. Bind to a named guard.
- Python operator: `reload` latches true forever, swallowing all subsequent `on_event` errors — `python.rs:124,131-132,236-244`. Scope leniency to the immediately-following event.
- Operator `on_event` panic unwinds across `extern "C" fn dora_on_event` — `apis/rust/operator/src/raw.rs:71`, `macros/src/lib.rs:53-62`. Wrap user calls in `catch_unwind`.

**Python / C node + operator APIs**
- `try_recv()`/`drain()`/`is_empty()` no-op (return Empty/`[]`) after `merge_external_events()` (verified) — `apis/python/node/src/lib.rs:818-829, 846-862`. Recoverable via blocking `recv` (intentional, commented), but poll loops silently get nothing. *Fix:* poll the merged stream or raise.
- `drain()` replaces un-convertible events with empty `{}` dicts, losing data; `next()` propagates — `apis/python/node/src/lib.rs:244-253`. Propagate the error or emit an error marker.
- `merge_external_events()` `blocking_lock()` while holding the GIL → deadlock vs suspended `recv_async` — `apis/python/node/src/lib.rs:737-766` / `:757` / `:831`. Wrap in `py.detach` or `try_lock`.
- Python empty list/tuple metadata coerced to String `"[]"` + `println!` spam — `apis/python/operator/src/lib.rs:275-291, 322-325`. Handle empty-list explicitly; use `tracing::warn!`.
- C `dora_read_data` collapses 3 distinct failures to `None` (`apis/rust/operator/types/src/lib.rs:167-173`); C operator example leaks id+data buffers on every message (`examples/c-dataflow/operator.c:52,55,59`). Log failing branch; add `free()`s (or pass the literal directly).

**Coordinator topic injection (rendezvous clusters)**
- `dora topic` publish opens a multicast-only zenoh session that can't reach daemons with multicast disabled — `binaries/coordinator/src/ws_control.rs:465` (`open_zenoh_session(None)`) vs `libraries/core/src/topics.rs:143`. Thread `inter_daemon_peer` into the injection session.

**MAVLink correctness**
- UDP server mode drops outbound frames until first inbound packet (REQUEST_DATA_STREAM + early commands lost, logged as success) — `binaries/mavlink2-bridge-node/src/main.rs:355` (verified). Check the returned byte count; defer/retry stream request after dest is learned.
- Main loop never exits if event stream closes without Stop — `binaries/mavlink2-bridge-node/src/main.rs:386-399`; `None` from `recv_timeout` means both timeout and channel-close. Break into shutdown on detected closure.

**ROS2 codegen robustness**
- Client-side pending-request set unbounded (server side is bounded) — `service.rs:255` / `action.rs:708`; removal only on matching response. Evict on a `REQUEST_TIMEOUT`.
- Generated create fns `Name::new(...).unwrap()` panic on invalid user name/namespace — `service.rs:137,354`, `action.rs:402,882` vs the graceful `message.rs:415`. Replace all four with `.map_err(...)?`.
- Fixed-size string/wstring/struct arrays serialized with no length-equality check — `array.rs:261-275, 277-292, 119-135` (primitive/bool paths check). Add the `array.len() != array_len` guard.
- Scalar WString deser has no `MAX_SEQUENCE_ELEMENTS` cap (sequence path does) — `deserialize/string.rs:72-79` vs `sequence.rs:18-20,134-140`. Apply the same cap.

**Record/replay**
- `RecordingReader::next_entry` panics on corrupt inner length fields (verified, two findings — same root cause) — `libraries/recording/src/lib.rs:159-173`; `node_id_len`/`output_id_len`/`event_bytes_len` sliced unchecked. Reachable from user `DORA_REPLAY_FILE`. Bounds-check each before slicing; add a crafted-record test.

**Test gaps on security/framing (verified, all medium)**
- Coordinator auth `validate_token`/`extract_token` untested — `binaries/coordinator/src/ws_server.rs:91-121`. Add accept/reject/empty-token/disabled cases.
- Daemon TCP framing DoS/overflow/timeout guards untested — `binaries/daemon/src/socket_stream_utils.rs:46-60`. Test via `tokio::io::duplex`.
- Node TCP framing guards untested — `apis/rust/node/src/daemon_connection/tcp.rs:91-106`. Test via `Cursor`.
- Raw-buffer Arrow roundtrip only tested at offset 0 (`arrow_utils.rs:116-123`, test `:338-360`); empty-buffer early-return untested (`arrow_utils.rs:148-150`). Add sliced/offset + nested-empty roundtrip tests.

---

## 5. Quick Wins (high value, low effort)

1. **Operator panic message** — downcast before bailing: `binaries/runtime/src/lib.rs:175-176`. One-liner; restores all operator panic diagnostics.
2. **Tracing span guards** — `let _ = span.enter()` → `let _guard = ...` at `shared_lib.rs:171`, `python.rs:192,352`. Fixes distributed tracing for operators.
3. **ROS2 codegen `.unwrap()` → `?`** — `service.rs:137,354`, `action.rs:402,882`. Mechanical, removes 4 node-abort panics.
4. **Timer `hz` overflow guard** — `Duration::try_from_secs_f64` at `libraries/message/src/config.rs:285`. Turns a panic into the existing Err.
5. **`max_log_size` checked_mul** — `libraries/core/src/descriptor/validate.rs:325-331`.
6. **C operator example `free()`s** — `examples/c-dataflow/operator.c:59` (add `free(out_id_heap); free(out_data);`). Stops a leak copied into every derived project.
7. **`log_to_coordinator` expect → warn+return** — `binaries/daemon/src/log.rs:422`. Matches surrounding error-tolerant style.
8. **`zenoh_publish` unwraps → `ok_or_else(...)?`** — `apis/rust/node/src/node/mod.rs:1050,1081`. Localizes a call-site-only invariant.

---

## 6. Deeper Investments (worth scoping)

1. **Make `cleanup_disconnected_daemons_from_running_dataflows` the authoritative teardown path.** `binaries/coordinator/src/lib.rs:2899-2917` is the root of three High deadlocks (ready-barrier, hung stop, orphaned dataflow). Redesign it to re-evaluate the ready barrier, drain stop senders, resolve results, persist `Failed`, archive, and remove — with integration tests per scenario. Single highest-leverage fix.

2. **Define and enforce daemon behavior on coordinator loss.** Reconcile `binaries/daemon/src/lib.rs:1072-1076` with #1996/#1998: either preserve `ProcessHandle`s across reconnect (transparent), or document kill-on-disconnect and fix the comment. Needs a reconnect integration test asserting node survival.

3. **Audit the C/C++/Python FFI surface for panic-across-boundary.** Verified aborts at `apis/c/node/src/lib.rs:196` and `apis/c++/node/src/lib.rs:435`, plus `apis/rust/operator/src/raw.rs:71`. Establish a convention: no `todo!()`/`unwrap()`/panic in any `extern "C"` or cxx-bridge fn; wrap user callbacks in `catch_unwind`; return error results. One pass across all four language APIs.

4. **Harden ROS2 CDR wire format + add real round-trip tests.** Three verified/strong serialize-deser correctness bugs (`sequence<bool>` tuple, empty struct-sequence concat, fixed-array length checks) and no `tests/` dir in the arrow crate. Add CDR-level (not serde_assert) round-trip tests across primitive/bool/string/struct sequences and fixed arrays; the bugs indicate the test harness never exercised the wire encoding.

5. **Unify empty/typed-array and alignment handling across the daemon and zenoh transports.** Multiple findings (`event_stream/mod.rs:1063`, `data_conversion.rs:30`, `arrow_utils.rs:148`) show the two transports diverge on empty-array typing and offset-alignment validation, producing transport-dependent behavior. Converge on one conversion path with shared empty/alignment handling and cross-transport equivalence tests.

6. **Backpressure + observability for the zenoh data/control plane.** `queue_policy` is honored only on the daemon path; control frames (`OutputClosed`) and overflowed inputs are dropped silently (`event_stream/mod.rs:355`, `binaries/daemon/src/lib.rs:3276,3298`). Scope a reliable control-frame channel and a drop-counter/metric so silent loss becomes observable.

---

*Honest notes:* No critical (memory-unsafety) findings — the FFI items are forced-abort, not UB. The "Critical" section is intentionally empty. Several mediums (coordinator WS stall, listener busy-spin, MAVLink shutdown leak, serial URL parsing, version-prerelease compat, Python timeout-doc drift) are auditor-reported (`verified:false`) and should be confirmed before fixing. The record/replay panic and the recording test-gap finding are the same root cause, deduped here.
