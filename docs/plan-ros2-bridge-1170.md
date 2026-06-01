# Plan: dora GitHub Issue #1170 — "Tracking support for dora-ros2 bridge"

> **Ground-truth note (corrected).** An earlier draft claimed `BridgeServiceType`/`BridgeMessage`/`TypeInfoGuard` live in the python crate's `typed` module. **That was wrong.** Verified line-by-line: `python/src/typed/mod.rs:4` is just `pub use dora_ros2_bridge_arrow::*;`. The types are *defined* in the **`dora-ros2-bridge-arrow`** crate (`arrow/src/lib.rs:71` `TypeInfoGuard`, `:112` `BridgeMessage`, `:157` `BridgeServiceType`, `:177` `BridgeActionType`), and the daemon imports them directly: `use dora_ros2_bridge_arrow::{BridgeActionType, BridgeMessage, BridgeServiceType, TypeInfo, TypeInfoGuard, ...}` (`main.rs:22-26`). The python crate **already depends on `dora-ros2-bridge-arrow` directly** (`python/Cargo.toml:11`). So the import path for the new code is settled: import from `dora_ros2_bridge_arrow`, exactly as the daemon does. Because the prior draft elevated a false premise to "verified," **every "exists / mirrors" citation below was re-checked against `arrow/src/lib.rs` and `binaries/ros2-bridge-node/src/main.rs` before this revision.** A second factual correction surfaced in that re-check and is now reflected throughout: the existing **pub/sub Python path does NOT use `BridgeMessage`/`TypeInfoGuard`** — `publish()` uses `TypedValue` + `Publisher::publish` (`lib.rs:352-362`) and `Subscription` is `Subscription<ArrayData>` driven by a `StructDeserializer` seed (`lib.rs:378,394`). The service/action path therefore introduces the `BridgeServiceType`/`BridgeMessage`/`TypeInfoGuard` mechanism into the Python crate for the first time; it mirrors the **daemon**, not the existing Python pub/sub code.

## 1. Executive summary

The bridge is a **pure-Rust DDS/RTPS stack** (`ros2-client 0.8` + `rustdds =0.11.4`, `libraries/extensions/ros2-bridge/Cargo.toml`) — it never links `rcl`/`rclcpp`. Two distinct surfaces exist for every ROS2 capability: (a) **codegen native APIs** (per-language, compile-time typed structs, generated in `msg-gen/`) and (b) the **dynamic-type bridge daemon** (`binaries/ros2-bridge-node/src/main.rs`, YAML-driven, language-agnostic via Arrow + thread-local `TypeInfo`). The daemon already implements service client/server AND action client/server end-to-end (`run_service_client` `main.rs:265+`, `run_service_server` `:321+`, `run_action_*` `:495+`/`:666+` — verified). So the open tracker items split cleanly:

- **Python service/action surface** is *not* missing core protocol logic — it is missing the **pyo3 wrapper classes** over types that already exist in `dora-ros2-bridge-arrow` (`BridgeServiceType`, `BridgeActionType`, `BridgeMessage`, `TypeInfoGuard`). This is glue, not protocol work, and it's the highest-leverage item. **Caveat:** the glue is *not* a copy of the existing Python pub/sub code (that path uses a different serialization mechanism — see ground-truth note); it is a pyo3-shaped reimplementation of the **daemon's** service loop.
- **Rust action server** turned out to need **no new codegen** (corrected in Phase 4 below): `ros2_client`'s `AsyncActionServer` + the already-generated message structs are sufficient; it is a native example, not codegen work. **C++ service+action server** still require new cxx **codegen** in `msg-gen/src/types/action.rs` + `service.rs` (the harder, XL-leaning work, gated by the nightly Linux+ROS2 CI).
- **Parameter service** is "six `rcl_interfaces` services + a `/parameter_events` topic" — composable on top of the service surface once it exists in the target language.

**Attack strategy:** Drive the whole tracker through the **dynamic bridge first** (cheap, language-agnostic, already battle-tested in the daemon), exposing it per-language, and only invest in codegen-native APIs where typed ergonomics are explicitly wanted. **First PR = Python service client + server pyo3 wrappers**, because all Rust machinery already exists and the reference async logic is adaptable from `main.rs`. Each subsequent item reuses the same recipe and the same macOS Docker test harness. Final gate for every item is the nightly `ros2-bridge` job (`.github/workflows/nightly.yml`, Humble/ubuntu-22.04), reproduced locally via `scripts/qa/ci-nightly-jobs.sh ros2-bridge`.

The canonical **"add a feature" recipe** is:
**codegen (`msg-gen/src/types/<feature>.rs`) → core wrapper (`ros2-bridge/src/lib.rs` over `ros2_client`) → language binding (pyo3 in `python/src/lib.rs`, or cxx in `apis/c++/node/build.rs`) → Rust example → C++/Python example → smoke entry.** For dynamic-bridge / pyo3 items the codegen step is *skipped* (dynamic `TypeInfo` replaces it).

---

## 2. Sub-item assessment table

Difficulty: S≈1-2 days, M≈3-5 days, L≈1-2 wk, XL≈2-4 wk. "Mirrors" cites the existing code an engineer adapts.

| Open sub-item | Diff | Dependencies | Leverage | Mirrors / recipe stage |
|---|---|---|---|---|
| **Python: service client** | **S** | None — `BridgeServiceType`/`BridgeMessage`/`TypeInfoGuard` exist in `dora-ros2-bridge-arrow`; daemon async logic exists | **Highest** — unblocks param-service client + all py service consumers | pyo3 stage only. Adapt `run_service_client` (`main.rs:265-319`) into a blocking pyo3 method |
| **Python: service server** | **S** | Ships with client (same PR) | Highest — unblocks param-service server | Adapt `run_service_server` request-id correlation + **persisted** request stream (`main.rs:321-414`) |
| **Python: action client** | **M** | Service-client PR (pattern only, not code) | High | pyo3 over `ros2_client::action::ActionClient<BridgeActionType>`; mirror `run_action_client` (`main.rs:495+`) |
| **Python: action server** | **M/L** | Action-client PR; owns goal state machine/result cache | High | pyo3 over `ros2_client::action::AsyncActionServer<BridgeActionType>`; mirror `run_action_server` (`main.rs:666+`). **Genuine redesign** of daemon's single-loop into a pull API — see Risks |
| **Parameter service** | **M** | Python (or Rust) service **server** (real code dependency) | Medium — pure composition, no transport | Compose 6 `rcl_interfaces/srv/*` service servers + publish `rcl_interfaces/msg/ParameterEvent`. No new codegen if dynamic bridge used |
| **Rust action server** | **L** | New codegen in `action.rs` | Medium | codegen stage. Mirror existing service-server codegen `service.rs` + action-client codegen `action.rs`; add `cxx_action_server_creation_functions` sibling. Native example mirrors `service_server.rs` |
| **C++ service server** | **L** | cxx codegen path | Medium | codegen + cxx. Service-server codegen already exists in `service.rs`; needs C++ creation fns + header export (`apis/c++/node/build.rs`). Mirror C++ action-client example |
| **C++ action server** | **XL** | Rust action-server codegen first; cxx | Lower | codegen + cxx, hardest — 3 services + 2 topics + goal FSM exposed across FFI. Mirror C++ action-**client** (`examples/ros2-bridge/c++/action-client/`) |
| **Examples aligned w/ ros2/examples** | **M** (incremental) | Each lands with its feature | High (CI gate) | Mirror `examples/ros2-bridge/{rust,python,c++}/turtle` + `yaml-bridge-{service,action}` + smoke entry in `tests/example-smoke.rs` |

---

## 3. Recommended sequencing

**Phase 0 — Test harness (prereq, ~1 day).** Stand up the macOS Docker dev image (Section 5) and reproduce the *current* green nightly `ros2-bridge` job locally. No code yet; this de-risks every later PR by giving a working local ROS2 oracle.

**Phase 1 — PR #1: Python service client + server (S).** *First pick — justified below.* Adds `Ros2ServiceClient`/`Ros2ServiceServer` pyo3 classes + `Ros2Node.create_service_client/server`, a `python/turtle`-style example, and a smoke entry. Unblocks the entire Python service ecosystem and the param-service.

**Phase 2 — PR #2: Python action client + server (M/L).** Reuses PR #1's wrapper *pattern* (not a compile-time dependency — `ActionClient`/`AsyncActionServer` own their own service/topic machinery internally). Mirrors `run_action_*`. The **action server is a genuine redesign** of the daemon's single-loop goal FSM into a pull API (`&mut self` + persisted goal-handle map), not a mechanical copy. Closes the "Python is most behind" gap the maintainer flagged.

**Phase 3 — PR #3: Parameter service (M).** Built in Python on PR #1's service-server class (**real code dependency** — correctly ordered after Phase 1): host the six `rcl_interfaces/srv/*` servers + publish `ParameterEvent`. Pure composition, validated with `ros2 param list/get/set`.

**Phase 4 — PR #4: Rust action server (S, not L — landed-pending).** *Reassessed during implementation: NO new codegen is needed.* `ros2_client::action::AsyncActionServer<A>` + `Node::create_action_server::<A>()` already work with the generated message structs (the same `Fibonacci`/`FibonacciResult`/`FibonacciFeedback` the action-client example uses). The deliverable is therefore just a native example, `examples/ros2-bridge/rust/action-server/`, mirroring `action_client.rs` + the daemon's `run_action_server` FSM (receive → accept → execute → feedback → result).

The example is a **self-contained dora↔dora pair** (a dora action server + a dora action client in one dataflow), because a real rmw-based ROS2 client cannot *discover* a ros2-client-hosted server (ros2-client#4 — `ros2 action info` reports `Action servers: 0`).

**Verification posture (important):** on the macOS **arm64/Docker** harness the server FSM is verified up to `send_result_response` (goal received + accepted + feedback published, `0` cross-RMW deser errors), but the deferred `get_result` service reply does not complete — the documented arm64/Docker service-response limitation ([[cxx-ros2-service-response-arm64-bug]] / #1972 family), independent of this code. The **x86 nightly `ros2-bridge` job** is therefore the oracle: this PR adds the Rust action client + action-server examples to `.github/workflows/nightly.yml` (previously no action example ran in any CI).

**Phase 5 — PR #5/#6: C++ service server, then C++ action server (L → XL).** Service server first (codegen mostly exists), action server last (depends on Phase 4 + hardest cxx surface).

*Phase 5a (C++ service server) — landed-pending.* New cxx codegen `cxx_service_server_creation_functions` in `msg-gen/src/types/service.rs`, mirroring the existing client `cxx_service_creation_functions` in reverse: a per-server request pump receives requests, tags each with a `u64` token (recorded against its ros2-client `RmwRequestId`), and forwards it to the merged event stream; the C++ side does `matches` → `downcast` (→ a `ServiceRequest` event exposing `get_request()` + `get_id()`) → `send_response(id, response)`, which correlates the token back to the `RmwRequestId`. The token avoids exposing `RmwRequestId` across the cxx FFI. Example: `examples/ros2-bridge/c++/service-server/` (`AddTwoInts` server) paired with the rclcpp minimal client. **Unlike the action server, this is fully verifiable on the macOS arm64 harness:** a real rclcpp client discovers the dora server (no ros2-client#4 block for services), the request deserializes, and the example exits `0` (`request: 41 + 1` → `Served 1 service requests`). Added to both `scripts/ros2dev.sh` and the x86 nightly C++ job.

**Why PR #1 (Python service client/server) is first:** It is the only item that is **S** difficulty with **zero new dependencies and highest leverage**. The maintainer's status comment names it highest-leverage. Concretely verified: every Rust type it wraps (`BridgeServiceType`, `BridgeMessage`, `TypeInfoGuard`) already exists and is already a direct dependency of the python crate (`python/Cargo.toml:11`); the request/response/correlation async logic already exists in `binaries/ros2-bridge-node/src/main.rs:265-414`; the pyo3 conversion primitives (PyArrow↔`ArrayData`, GIL handling, module registration) are demonstrated in `publish` / `Subscription.next`. It is the lowest-risk way to prove the per-language recipe end-to-end on the macOS harness before tackling codegen-heavy items, and it directly unblocks Phase 3. **Honest caveat:** "lowest risk" ≠ "trivial copy" — see the service-mapping and late-match risks in §7, which apply from PR #1 onward.

---

## 4. Detailed spec for the FIRST PR — Python service client + server

### Goal
Add Python pyo3 wrappers so a dora Python node can act as a ROS2 service **client** (`call`) and **server** (`take_request` / `send_response`) over the dynamic bridge, with no codegen and no ROS2 dependency in user code beyond pyarrow.

### Verified preconditions (no new core code needed)
- `BridgeServiceType`, `BridgeMessage`, `TypeInfoGuard`, `set_serialize_type_info`/`set_deserialize_type_info`, `TypeInfo` are defined in **`dora-ros2-bridge-arrow`** (`arrow/src/lib.rs:71/112/157`) and re-exported via `python/src/typed/mod.rs` (`pub use dora_ros2_bridge_arrow::*`). The python crate already lists `dora-ros2-bridge-arrow` as a direct dep. **Import them the way the daemon does** (see 4a).
- `Ros2Node.node` is `ros2_client::Node`; `create_client::<BridgeServiceType>(mapping, &Name, &ServiceTypeName, qos, qos)` and `create_server::<BridgeServiceType>(...)` are exactly what `run_service_mode` (`main.rs:188-254`) calls.
- `detect_service_mapping()` reads `RMW_IMPLEMENTATION` at **runtime** (`main.rs:48-66`, called at `:225`/`:242`). It is NOT codegen-time for this dynamic path. Mapping must match the peer's RMW or calls hang (see §7).
- The daemon calls `wait_for_service(&client, &ros_node)` (`main.rs:235`, def `:1111`) **before** issuing client requests. The Python client must do the same (see 4c/4d) — skipping it directly exposes the late-match race.
- QoS: `Ros2QosPolicies: Into<rustdds::QosPolicies>` exists (`python/src/qos.rs:52`), **but its default is `BestEffort`** (`qos.rs:44` `reliable: reliable.unwrap_or(false)`, `:57-64`). ROS2 services require **Reliable** QoS. We must build a Reliable+Volatile default explicitly; we cannot rely on the `Ros2QosPolicies` default.

### Files to modify
**`libraries/extensions/ros2-bridge/python/src/lib.rs`** (primary; ~250 LOC added) plus a 1-function relocation in **`libraries/extensions/ros2-bridge/src/lib.rs`** (see 4b).

#### (a) New imports
Import the service types from the arrow crate, mirroring the daemon (NOT inventing a `typed`-module path):
```rust
use dora_ros2_bridge_arrow::{BridgeServiceType, BridgeMessage, TypeInfo, TypeInfoGuard};
```
(`TypeInfo`/`TypeInfoGuard` may already be in scope via the existing `use typed::{...}`; deduplicate at edit time. The point of truth is the daemon's import block at `main.rs:22-26`.)

#### (b) Shared `detect_service_mapping` (remove duplication)
`detect_service_mapping()` currently lives only in `binaries/ros2-bridge-node/src/main.rs:48`. **Relocate it** to `libraries/extensions/ros2-bridge/src/lib.rs` as `pub fn detect_service_mapping() -> ros2_client::ServiceMapping`, and call it from both the daemon and the pyo3 crate. Surgical 1-function move; removes the duplication the dossier flagged. (Open Q resolved to "yes, relocate.")

#### (c) `Ros2Node` new methods (inside `impl Ros2Node`, after `create_subscription`)
```rust
/// Create a ROS2 service client.
/// :type service_name: str          # e.g. "/add_two_ints"
/// :type service_type: str          # e.g. "example_interfaces/AddTwoInts"
/// :type qos: dora.Ros2QosPolicies, optional   (defaults to Reliable+Volatile)
/// :rtype: dora.Ros2ServiceClient
#[pyo3(signature = (service_name, service_type, qos=None))]
pub fn create_service_client(
    &mut self,
    service_name: &str,
    service_type: String,
    qos: Option<qos::Ros2QosPolicies>,
) -> eyre::Result<Ros2ServiceClient> {
    let (package, type_name) = parse_type(&service_type)?;   // mirror create_topic type-split
    let (req_ti, resp_ti) = service_type_infos(&package, &type_name, &self.messages);
    let q: rustdds::QosPolicies = match qos {
        Some(p) => p.into(),
        None => default_service_qos(),                        // Reliable+Volatile, built explicitly
    };
    let client = self.node.create_client::<BridgeServiceType>(
        dora_ros2_bridge::detect_service_mapping(),
        &ros2_client::Name::new("/", service_name.trim_start_matches('/'))
            .map_err(|e| eyre!("bad service name: {e}"))?,
        &ros2_client::ServiceTypeName::new(&package, &type_name),
        q.clone(), q,
    ).map_err(|e| eyre!("create_client failed: {e:?}"))?;
    Ok(Ros2ServiceClient { client, request_type_info: req_ti, response_type_info: resp_ti,
                            node: /* handle for wait_for_service, see note */ })
}
```
**`wait_for_service` note:** `wait_for_service` needs both the client and the `ros2_client::Node`. Two viable shapes — pick one at implementation time and document it:
1. **Auto-wait in `create_service_client`** (simplest for users): call `client.wait_for_service(&self.node)` with a bounded timeout before returning. Downside: blocks construction until the peer exists.
2. **Explicit `Ros2ServiceClient.wait_for_service(timeout_s)` method** + document that `call` does NOT auto-wait. Requires the client to hold a way to reach the node (store a clone/handle, or have the node drive the wait).

Recommendation: option 2 (explicit), defaulting examples to call it once after creation, because auto-waiting at construction can deadlock a node whose peer starts later in the same dataflow. **This is an open ergonomics question — see §7.**

`create_service_server` mirrors the client signature, calling `self.node.create_server::<BridgeServiceType>(...)`, and constructs the server with its **persisted request stream** (see 4e).

Helpers (file scope):
```rust
fn parse_type(t: &str) -> eyre::Result<(String, String)>;          // mirror create_topic type-split
fn service_type_infos(pkg: &str, name: &str,
    msgs: &Arc<HashMap<String, HashMap<String, Message>>>)
    -> eyre::Result<(TypeInfo<'static>, TypeInfo<'static>)> {       // "_Request"/"_Response", mirror main.rs:200-217
}
fn default_service_qos() -> rustdds::QosPolicies;                   // Reliable + Volatile via QosPolicyBuilder
```

#### (d) `Ros2ServiceClient` pyclass + methods
```rust
#[pyclass] #[non_exhaustive]
pub struct Ros2ServiceClient {
    client: ros2_client::Client<BridgeServiceType>,
    request_type_info: TypeInfo<'static>,
    response_type_info: TypeInfo<'static>,
    // + whatever handle the chosen wait_for_service shape requires
}

#[pymethods]
impl Ros2ServiceClient {
    /// Wait until a matching service server is discovered (recommended before first call).
    /// :type timeout_s: float, optional
    #[pyo3(signature = (timeout_s=None))]
    pub fn wait_for_service(&self, py: Python, timeout_s: Option<f64>) -> eyre::Result<bool> { /* mirror main.rs:1111 */ }

    /// Send request, block for response (default 30s timeout). Does NOT auto wait_for_service.
    /// :type request: pyarrow.Array | pyarrow.StructScalar | dict
    /// :type timeout_s: float, optional
    /// :rtype: pyarrow.Array
    #[pyo3(signature = (request, timeout_s=None))]
    pub fn call(&self, request: Bound<'_, PyAny>, timeout_s: Option<f64>)
        -> eyre::Result<Py<PyAny>>
    {
        let py = request.py();
        let array_data = pyarrow_to_array_data(&request)?;     // EXTRACTED from publish() dict→scalar→array→ArrayData
        let req_id = {
            let _g = TypeInfoGuard::serialize(self.request_type_info.clone());
            self.client.send_request(BridgeMessage(Some(array_data)))
                .map_err(|e| eyre!("send_request: {e:?}"))?
        };
        let _g = TypeInfoGuard::deserialize(self.response_type_info.clone());
        let timeout = Duration::from_secs_f64(timeout_s.unwrap_or(30.0));
        let resp = py.allow_threads(|| futures::executor::block_on(async {  // release GIL while blocking
            let recv = self.client.async_receive_response(req_id);
            futures::pin_mut!(recv);
            match futures::future::select(recv, futures_timer::Delay::new(timeout)).await {
                Either::Left((r,_)) => r.map_err(|e| eyre!("recv response: {e:?}")),
                Either::Right(_) => eyre::bail!("service call timed out after {timeout:?} \
                    (check RMW_IMPLEMENTATION matches the peer; see service-mapping risk)"),
            }
        }))?;
        let data = resp.0.context("service response had no data")?;  // data is ArrayData
        Ok(data.to_pyarrow(py)?.unbind())                            // direct, no make_array round-trip
    }
}
```
**Serialization handling:** `TypeInfoGuard::serialize(req_ti)` is set *only* around `send_request`; `TypeInfoGuard::deserialize(resp_ti)` *only* around the await — exactly the daemon's discipline. `BridgeMessage::serialize` reads the thread-local set by the guard; the guard's `Drop` clears it (RAII, panic-safe — `arrow/src/lib.rs:71-110`). This is the **first** use of this mechanism in the Python crate; the existing pub/sub path uses `TypedValue`/`StructDeserializer` instead and is not a model for it.

**Conversion correction:** `resp.0` is already `ArrayData`, so it goes straight to `.to_pyarrow(py)` — matching `Subscription.next` (`lib.rs:394`, where `value` is `ArrayData` and the code is `value.to_pyarrow(py)?`). No `make_array(...).to_data()` round-trip.

#### (e) `Ros2ServiceServer` pyclass + methods (persisted stream — fixes per-poll bug)
The daemon constructs `receive_request_stream()` **once** and drives it for the server's lifetime (`main.rs:328-352` via `merge_external` + `block_on_stream`). `receive_request_stream()` yields a fresh stream over the DataReader each call; constructing/dropping it per poll risks dropping requests that arrive between polls. So we **persist a `block_on_stream` iterator** as a field:
```rust
#[pyclass] #[non_exhaustive]
pub struct Ros2ServiceServer {
    // Persisted once at creation, mirroring the daemon's single-stream lifetime.
    requests: futures::executor::BlockingStream<Pin<Box<
        dyn Stream<Item = Result<(ros2_client::service::RmwRequestId, BridgeMessage), _>> + Send>>>,
    server_send: /* handle to async_send_response — see note */,
    request_type_info: TypeInfo<'static>,
    response_type_info: TypeInfo<'static>,
    pending: HashMap<u64, ros2_client::service::RmwRequestId>,  // request_id -> rmw id
    next_id: u64,
}
```
**Note on the borrow:** in the daemon, `receive_request_stream()` borrows `&server` and the same `&server` is used for `async_send_response`. Wrapping a self-referential (stream + server) pair in one pyclass field is awkward. Two implementation options, decided at coding time:
1. Hold `Arc<Server<BridgeServiceType>>` (or the ros2-client server behind shared ownership) so both the persisted stream and `send_response` can reference it; OR
2. Wrap the server in the pyclass and build the stream lazily on first `take_request`, then keep it. Either way the stream is built **once**, not per call. **This is the part of PR #1 with real implementation risk** (self-referential borrow across pyo3); flagged in §7.

```rust
#[pymethods]
impl Ros2ServiceServer {
    /// Block for next request. Returns (request_id, request_array) or None on stream end.
    /// :rtype: tuple[int, pyarrow.Array] | None
    pub fn take_request(&mut self, py: Python) -> eyre::Result<Option<(u64, Py<PyAny>)>> {
        let _g = TypeInfoGuard::deserialize(self.request_type_info.clone());
        let item = py.allow_threads(|| self.requests.next());      // pull from the PERSISTED iterator
        let Some(res) = item else { return Ok(None) };
        let (rmw_id, msg) = res.map_err(|e| eyre!("recv request: {e:?}"))?;
        let Some(data) = msg.0 else { return Ok(None) };
        let id = self.next_id; self.next_id += 1;
        self.pending.insert(id, rmw_id);
        Ok(Some((id, data.to_pyarrow(py)?.unbind())))             // data is ArrayData, direct convert
    }

    /// Send response for a previously-taken request_id.
    /// :type request_id: int
    /// :type response: pyarrow.Array | pyarrow.StructScalar | dict
    pub fn send_response(&mut self, request_id: u64, response: Bound<'_, PyAny>)
        -> eyre::Result<()>
    {
        let py = response.py();
        let rmw_id = self.pending.remove(&request_id)
            .context("unknown request_id (already answered or never taken)")?;
        let array_data = pyarrow_to_array_data(&response)?;
        let _g = TypeInfoGuard::serialize(self.response_type_info.clone());
        py.allow_threads(|| futures::executor::block_on(
            /* server */.async_send_response(rmw_id, BridgeMessage(Some(array_data)))
        )).map_err(|e| eyre!("send_response: {e:?}"))?;
        Ok(())
    }
}
```
**Correlation key:** the pyo3 server uses a local `u64 → RmwRequestId` map and echoes the exact `RmwRequestId` back in `send_response` — satisfying §7's "persist the full `rmw_request_id_t` until the response is produced, then echo it exactly." It deliberately does **not** reuse the daemon's String-metadata (`new_request_id()`) correlation, because the pyo3 server never round-trips through dora metadata. `u64` is the single, consistent correlation type for this class.

#### (f) Extract shared PyArrow conversion
Factor the dict→StructScalar→array→`ArrayData` logic currently inline in `publish()` (`lib.rs:337-352`) into `fn pyarrow_to_array_data(data: &Bound<'_, PyAny>) -> eyre::Result<ArrayData>`, and have `publish` call it. Surgical refactor; one source of truth for input conversion.

#### (g) Register classes (`create_dora_ros2_bridge_module`)
```rust
m.add_class::<Ros2ServiceClient>()?;
m.add_class::<Ros2ServiceServer>()?;
```

#### (h) Type stubs
Add the new classes + method signatures to the python `.pyi` stub so editors/typecheckers see them.

### How it reuses existing surface (summary, corrected)
- **Conversion in:** `pyarrow_to_array_data` extracted from `publish():337-352`. **Out:** `ArrayData::to_pyarrow(py)` as in `Subscription.next:394` — `resp.0`/`msg.0` are already `ArrayData`, no `make_array`/`to_data` round-trip.
- **Async + correlation + persisted stream + wait_for_service:** adapted from `run_service_client`/`run_service_server`/`wait_for_service` (`main.rs:265-414`, `:1111`). The daemon is the reference, NOT the Python pub/sub path.
- **Serialization context:** `TypeInfoGuard` + `BridgeMessage` thread-local mechanism from `dora-ros2-bridge-arrow` — newly introduced into the Python crate by this PR.
- **QoS / mapping:** `Ros2QosPolicies: Into<QosPolicies>` exists but defaults to BestEffort; we add an explicit Reliable+Volatile `default_service_qos()`. `detect_service_mapping` relocated to the shared lib.
- **GIL:** wrap every `block_on`/blocking `next()` in `py.allow_threads` (publish is non-blocking so didn't need it; service calls block on the network and must release the GIL).

### Example (lands in same PR)
`examples/ros2-bridge/python/service/` mirroring `examples/ros2-bridge/python/turtle/`: a `dataflow.yml`, a `client_node.py` that calls `wait_for_service()` then `create_service_client("/add_two_ints","example_interfaces/AddTwoInts").call(pa.array([{"a":2,"b":3}]))`, and a `server_node.py` looping `take_request`/`send_response`. The example launch script **exports `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`** to match `detect_service_mapping()`'s Cyclone path (see §5/§7). Plus a driver entry in `libraries/extensions/ros2-bridge/Cargo.toml` mirroring the `python-ros2-dataflow` example wiring.

---

## 5. macOS testing & verification strategy

**Decisive constraint:** the bridge's RustDDS participant does discovery via **default multicast SPDP** with no peers/interface knob. Docker Desktop for Mac has no `--network=host` to macOS and drops multicast across the bridge driver. **Therefore the dora process and the ROS2 node MUST live in the same Linux network namespace.** Never split dora-on-macOS ↔ ROS2-in-container — discovery silently fails.

**Setup (Option A: one container; matches CI exactly):**
1. **Runtime:** Colima with the vz VM type (or Docker Desktop): `colima start --vm-type=vz --cpu 6 --memory 16 --disk 60`.
2. **Image:** `osrf/ros:humble-desktop` (native arm64, includes turtlesim + example-interfaces; **Humble** to match nightly CI). Add rustup pinned to **1.92**, Python 3.12, uv, `clang`, `cmake`, `ros-humble-example-interfaces`, and **`ros-humble-rmw-cyclonedds-cpp`** (required — see RMW default below).
   - `network_mode: host` (= the Linux VM's host net; SPDP loops back in-namespace).
   - Bind-mount repo to `/work`; **named volume** for `CARGO_TARGET_DIR=/cargo-target` (never share `target/` with the macOS-native build — arch fingerprints thrash); named volumes for `/root/.cargo/registry` + `/git`.
   - `.bashrc` sources `/opt/ros/humble/setup.bash` so `AMENT_PREFIX_PATH` is set for **runtime** message loading (see build-vs-runtime note below).
   - `QT_QPA_PLATFORM=offscreen`.
3. **RMW default — corrected.** The compose file MUST default to **`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`** on **both** the ROS2 side and the dora process. An earlier dossier compose used `rmw_fastrtps_cpp`; with services/actions that produces a Cyclone-vs-FastDDS service-mapping mismatch and the exact "hung call" failure §7 warns about, because `detect_service_mapping()` reads `RMW_IMPLEMENTATION` at runtime and dora explicitly implements the Cyclone service mapping. Pin Cyclone everywhere, install `ros-humble-rmw-cyclonedds-cpp`, and assert the env var is set in the example launch scripts. (Topics are mapping-agnostic, so the existing pub/sub examples were unaffected — which is why the wrong default went unnoticed.)

**Build-vs-runtime note (Python crate specifically) — corrected.** The python crate depends on `dora-ros2-bridge { default-features = false }` (disabling `generate-messages`) and parses `.msg`/`.srv` at **runtime** via `get_packages(AMENT_PREFIX_PATH)` (`python/src/lib.rs:68-102`). So **building the pyo3 wheel (`maturin develop`) does NOT require ROS2 message files or `AMENT_PREFIX_PATH`** — that's a runtime dependency of the *running dataflow* only. The `build.rs` codegen re-run discussion (`rerun-if-env-changed` on `AMENT_PREFIX_PATH`) applies to the **native codegen targets**, not this Python target; do not let the Docker loop assume the wheel rebuild depends on ROS2.

**Build-in-container loop:**
```bash
docker compose -f docker-compose.ros2dev.yml up -d --build
docker compose ... exec dora-ros2 bash -lc '
  uv venv --seed -p 3.12 && source .venv/bin/activate &&
  uv pip install -e apis/python/node pyarrow numpy &&
  maturin develop -m libraries/extensions/ros2-bridge/python/Cargo.toml'   # rebuild pyo3 wheel after edits (ROS2-independent)
```
First build is minutes; incremental is fast on persistent volumes.

**Round-trip verification against a real ROS2 node (service example):**
```bash
# Terminal 1 (in container): real ROS2 service server  (Cyclone RMW)
source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run examples_rclpy_minimal_service service   # advertises /add_two_ints
# Terminal 2: dora client node calls it via the new pyo3 wrapper  (same RMW)
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp dora run examples/ros2-bridge/python/service/dataflow-client.yml --uv --stop-after 10s
#   expect node logs: sum == a+b

# Reverse: dora is the SERVER, ros2 CLI is the client
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp dora run examples/ros2-bridge/python/service/dataflow-server.yml --uv &
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 7, b: 5}"
#   expect: response: example_interfaces.srv.AddTwoInts_Response(sum=12)
```
Explicitly test `ros2 service call` issued **immediately after** the dora server starts to exercise the late-match race (§7).

**Action verification (Phase 2):**
```bash
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 5}" --feedback
# vs. dora action server; expect feedback stream + result.sequence == [0,1,1,2,3,5]
```
**Param verification (Phase 3):** `ros2 param list /<node>`, `ros2 param get/set` against the dora-hosted param servers; `ros2 topic echo /parameter_events`.

The local CI parity check (run before every push touching the bridge): `scripts/qa/ci-nightly-jobs.sh ros2-bridge`, which sources Humble and runs `cargo test -p dora-ros2-bridge-python` + the rust/python/cxx `*-ros2-dataflow` examples — identical to the GHA job.

---

## 6. Plan → Spec → Implementation → Testing → Verification loop (per sub-item)

1. **Plan.** Locate the recipe stage(s): codegen / core / binding (pyo3 or cxx) / example / smoke. Most Python items skip codegen. Write success criteria as the exact `ros2 service call` / `ros2 action send_goal` / `ros2 param` command that must round-trip, **with the matching `RMW_IMPLEMENTATION` set**.

2. **Spec.** Enumerate files + signatures (as in Section 4). Name the existing function each new piece mirrors — for service/action items that reference is the **daemon** (`run_service_*`, `run_action_*`, `wait_for_service`), not the Python pub/sub code. Decide serialization (`_Request`/`_Response`/`_Goal`/`_Result`/`_Feedback` `TypeInfo` + `TypeInfoGuard` placement), QoS (Reliable for services), and GIL boundaries (`py.allow_threads` around every blocking call). For the action server, spec `&mut self` + a persisted goal-handle map up front.

3. **Implementation (RED→GREEN→IMPROVE).**
   - **RED — unit:** add a Rust `#[cfg(test)]` test (style of the existing Arrow↔ROS2 round-trip tests in `typed/mod.rs`) asserting request `ArrayData` → CDR → `ArrayData` round-trips through `BridgeServiceType`'s `_Request`/`_Response` `TypeInfo` with the `TypeInfoGuard` discipline. No live ROS2; runs in `cargo test -p dora-ros2-bridge-python`.
   - **GREEN:** implement until the unit test + `maturin develop` import succeed.
   - **IMPROVE:** `cargo clippy -p dora-ros2-bridge-python -- -D warnings`, `cargo fmt --all -- --check`, run `/review` + `/simplify`.

4. **Testing tiers:**
   - **Unit** — type/serialization round-trip, no network.
   - **Integration** — the embedded-Python test target (`dora-ros2-bridge-python`, already run by nightly) exercising client↔server in one process over loopback DDS inside the container.
   - **Smoke** — add entries to `tests/example-smoke.rs`: `run_smoke_test_local("ros2-service", ...)` + networked `run_smoke_test` where applicable. Register the example in `scripts/smoke-all.sh` (ros2 examples stay in the smoke SKIP list for non-ROS2 hosts; they execute only inside the container/nightly).

5. **Verification.** Run the real-ROS2 round-trip commands from Section 5 inside the container, both directions, with matching RMW, including the immediately-after-start late-match probe.

6. **Final gate.** `scripts/qa/ci-nightly-jobs.sh ros2-bridge` locally (Humble), then push; the **nightly `ros2-bridge` job** (ubuntu-22.04 + Humble, non-PR-blocking) is authoritative. Because bridge CI is nightly-only, schedule the merge so a nightly runs within ~24h and watch for the auto-filed `nightly-regression` issue.

---

## 7. Risks & open questions

**Risks**
- **Service-mapping interop (highest).** Fast-DDS (Humble default) uses RTPS `SampleIdentity` for request/reply correlation; Cyclone/RPC-style prepends a `{u64 guid, i64 seq}` body header — **mutually wire-incompatible**. `detect_service_mapping()` reads `RMW_IMPLEMENTATION` at **runtime**, so the peer's RMW must match what dora detects or the call hangs. Mitigation: pin `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` on both sides in the harness, examples, and docs; surface a clear timeout error hinting at mapping mismatch (spec'd in 4d).
- **Late-match race.** A server may receive a request before its reply writer is matched to the client's reply reader, silently dropping the reply. Mitigation: the client calls `wait_for_service` (mirroring the daemon `main.rs:235`); explicitly test `ros2 service call` immediately after server start. **Honest uncertainty:** whether ros2-client buffers a reply across a not-yet-matched reply writer is not proven here — the immediately-after-start probe in §5 is the gate, and if it fails we add a server-side readiness wait.
- **Self-referential server borrow in pyo3 (real PR #1 risk).** Persisting the request stream (built once) while also calling `async_send_response` on the same server is a self-referential borrow that pyo3's `#[pyclass]` (which requires owned/`Send` fields) makes awkward. The two options in 4e (shared `Arc<Server>` vs. lazy-but-persisted stream) are both plausible but **neither is verified to compile against ros2-client 0.8's `Server` API yet**. This is the single most likely place PR #1 takes longer than "S". Resolve by spiking the field layout against the real `Server` type before writing the pyclass.
- **QoS default wrong for services.** `Ros2QosPolicies` defaults to `BestEffort` (`qos.rs:44/57`); services need Reliable. Mitigation: explicit `default_service_qos()` (Reliable+Volatile via `QosPolicyBuilder`); do not lean on the `Into` default.
- **Blocking pyo3 API ergonomics + the `wait_for_service` shape.** `call`/`take_request` are synchronous + GIL-releasing; a user calling `call()` on the dora event-loop thread blocks that node. Acceptable for v1 (matches `Subscription.next`), but must be documented. Coupled open question: should `create_service_client` auto-wait for the service (simple, but can deadlock if the peer starts later in the same dataflow) or expose an explicit `wait_for_service(timeout)` (recommended)? **Unresolved — flagged for maintainer.**
- **GIL deadlock** if any `block_on`/blocking `next()` is *not* wrapped in `py.allow_threads`. Enforce in review.
- **Action server is a genuine redesign, not a copy (Phase 2).** The daemon manages goal state in a single `block_on_stream` loop (`main.rs:666+`); a pull-style pyo3 action server needs `&mut self` + a persisted `HashMap<goal_id, ExecutingGoalHandle>` and must re-implement the goal FSM, result caching (`GetResult` expiry), cancel selection (the 4 `(goal_id,stamp)` cases), and transient-local status publishing outside the loop. This is the highest-defect-surface Python item; do not budget it as mechanical.
- **C++ action server (XL).** Goal FSM + 3 services + 2 topics re-exposed across the cxx FFI. Highest overall defect surface.
- **macOS test fragility.** Only Option A (single container) reliably discovers; any accidental process split breaks silently. Bake the (Cyclone-defaulted) container setup into a committed `docker-compose.ros2dev.yml` so contributors can't misconfigure.

**Open questions**
1. **Native vs dynamic for Python actions:** expose only the dynamic-bridge (`BridgeActionType`) path, or also generate codegen-native typed Python? Recommend dynamic-only (no per-message codegen burden); confirm with maintainer.
2. **`wait_for_service` ergonomics:** auto-wait in `create_service_client`, or explicit `wait_for_service(timeout)` method (recommended)? Affects whether the first `call()` can deadlock against a later-starting peer.
3. **Server field layout vs ros2-client 0.8 `Server` API:** which of the two 4e shapes (shared `Arc<Server>` vs lazy-persisted stream) actually compiles given the `receive_request_stream`/`async_send_response` borrow signatures? Must spike before coding the pyclass.
4. **Param service language:** implement once in Python on PR #1's server (fastest, real dependency on Phase 1), or also in Rust? Recommend Python-first.
5. **Timeout policy:** match the daemon's 30s service / 5min result defaults, or expose per-call `timeout_s` (spec'd as optional arg, daemon values as defaults)? Recommend the latter.
6. **Example alignment scope:** which specific upstream `ros2/examples` packages (e.g. `examples_rclpy_minimal_service`, `examples_rclpy_minimal_action_*`) are the canonical mirror targets, so example dataflow names match upstream exactly? Confirm the list.

*(Resolved from the prior draft: where `BridgeServiceType`/`BridgeMessage`/`TypeInfoGuard` live — `dora-ros2-bridge-arrow`, imported as the daemon does; and whether to relocate `detect_service_mapping` — yes, to the shared `ros2-bridge/src/lib.rs`.)*