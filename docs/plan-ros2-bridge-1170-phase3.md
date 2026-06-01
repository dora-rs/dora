# dora #1170 Phase 3 — Python ROS2 Parameters

**Status:** implemented + verified (this PR).
**Approach correction:** the original design (hand-rolling six `rcl_interfaces`
service servers in Python) was **wrong** and is discarded. Live testing revealed
`ros2-client` **already implements the parameter services natively**, so Phase 3
is a small PyO3 surface over that, not a reimplementation.

---

## 1. Key finding (why the approach changed)

`ros2_client::Node` hosts the full parameter interface itself when
`NodeOptions::start_parameter_services` is set (default `true`):

- It creates all six servers — `Get/GetTypes/List/Set/SetAtomically/Describe
  Parameters` (`ros2-client-0.8.1/src/node.rs:129-134`) — and **drives them from
  the node spinner** the bridge already runs (`node.rs` spin loop).
- It owns the parameter store, validators, set-actions, and publishes
  `/parameter_events`. Default parameter: `use_sim_time`.
- API: `NodeOptions::declare_parameter(name, ParameterValue)`,
  `Node::{set_parameter, get_parameter, list_parameters, has_parameter,
  undeclare_parameter}`, `ParameterValue` enum
  (`NotSet/Boolean/Integer/Double/String` + 5 arrays).

Evidence: a dora node's `list_parameters` service already answered with
`use_sim_time` before any Phase-3 code existed. A hand-rolled
`/<node>/get_parameters` server therefore **conflicts** with the built-in one
(two servers, same name) — which is what the earlier flaky/`use_sim_time`
results were showing.

So: every dora ROS2 node already answers `ros2 param`; it just lacked a Python
way to declare/read/update custom parameters. Phase 3 adds exactly that.

---

## 2. What this PR adds

All in `libraries/extensions/ros2-bridge/python/src/lib.rs` (PyO3), no codegen,
no hand-rolled services:

- **`Ros2Context.new_node(name, namespace, options, parameters=None)`** — the new
  `parameters` dict declares initial parameters via
  `NodeOptions::declare_parameter` before the node is created. Declaring
  `use_sim_time` is rejected (ros2-client re-declares it as `Boolean(false)`
  after user declarations, so a value set here would be silently dropped; set it
  at runtime instead).
- **Type-stability validator** — `new_node` registers a
  `NodeOptions::parameter_validator` that rejects a set whose value type differs
  from the declared one. ros2-client invokes it on **every** set path, so the
  contract holds for the local `set_parameter` *and* the native `rcl_interfaces`
  services the spinner serves to remote `ros2 param set` / rclpy clients — not
  just the Python caller.
- **`Ros2Node.set_parameter(name, value)`**, **`get_parameter(name) -> value|None`**,
  **`list_parameters() -> [str]`**, **`has_parameter(name) -> bool`** — thin
  wrappers over the corresponding `ros2_client::Node` methods. A type-changing
  `set_parameter` raises `RuntimeError` (via the validator above).
- **`py_to_parameter_value` / `parameter_value_to_py`** — convert between Python
  values (`bool/int/float/str/bytes` + homogeneous lists) and
  `ros2_client::ParameterValue`. Scalars are matched on their concrete Python
  type (`bool` before `int`, since Python `bool` is an `int` subclass; `float`
  gated on `PyFloat` so a `numpy.int64` is not silently demoted to `Double`); a
  Python `int` always maps to `Integer` and an out-of-`i64`-range value is
  rejected. Lists take their element type from the first item and require **every**
  element to match it (so `[True, 2]` is rejected, not coerced to `[1, 2]`); empty
  lists are rejected (ambiguous type).
- **`parse_ros2_name`** — independent bridge fix so service/action client/server
  names may be **namespaced** (`/ns/sub/base` → namespace `/ns/sub`, base
  `base`); previously the whole path was forced into the base name, which a ROS2
  `Name` rejects. (Single-segment names like `/fibonacci` are unchanged.) Needed
  by the namespaced parameter-service client in the example, and useful generally.

Stubs added to `apis/python/node/dora/__init__.pyi` for the new methods +
`new_node(parameters=...)`.

Setting a parameter requires it to be declared first (ros2-client default:
undeclared sets are rejected); declare via `new_node(parameters=...)`.

---

## 3. Example — `examples/ros2-bridge/python/parameter/`

A single self-contained `parameter-server` node (`parameter_node.py`,
`dataflow.yml`, `run.rs`):

- declares scalar and list parameters (`speed/name/gain/waypoints/flags`) via
  `new_node(parameters=...)`, prints `list_parameters()`, then asserts
  `get_parameter` (declared values incl. the list round-trips + `missing` →
  `None`), a runtime `set_parameter` round-trip (`speed` → 2.0), that a
  type-changing `set_parameter("gain", "not an int")` is rejected, and that
  declaring `use_sim_time` or a non-homogeneous list (`[1, "two"]`) is rejected —
  printing `PARAM API OK`.
- exercises the **local** parameter API only — no cross-node discovery — so the
  example is deterministic. (The six rcl_interfaces services are still hosted
  natively by ros2-client for an external peer; cross-node assertion was dropped
  because dora↔dora multi-service discovery settles nondeterministically, and a
  real `ros2 param` peer is gated upstream — ros2-client#4.)

Wired as the `python-ros2-dataflow-parameter` Cargo example and added to the
`scripts/ros2dev.sh` release-QA list.

---

## 4. Verification

In the ROS2 Humble container (`scripts/ros2dev.sh`):

```
parameter-server: list_parameters: ['flags', 'gain', 'name', 'speed', 'use_sim_time', 'waypoints']
parameter-server: PARAM API OK   (dataflow exit 0)
```

`cargo clippy -p dora-ros2-bridge-python -- -D warnings` and
`cargo fmt --all -- --check` are clean.

**Discoverability caveat (ros2-client#4):** real-ROS2-client → dora-hosted-server
discovery is gated upstream, so `ros2 param ...` against a dora node is not
reliable today; the parameter API is validated locally and the services
themselves are ros2-client's tested implementation. dora→ROS2 (querying another
node's parameters via a dora `Ros2ServiceClient`) works.

---

## 5. Deferred (v2+)

`floating_point_range` / `integer_range` constraint enforcement, per-parameter
descriptors beyond name/type, and a Python `on_set` validation callback
(`NodeOptions::parameter_validator` / `parameter_set_action` exist in ros2-client
but are not yet exposed through PyO3). None block `ros2 param` / rclpy usage.
