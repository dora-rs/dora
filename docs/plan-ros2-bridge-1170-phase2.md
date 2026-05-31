# §5 — Phase 2: Python ROS2 Action Client + Server (dora #1170)

This section specifies the PyO3 binding that exposes the ROS2 **action**
protocol to Python dataflow nodes, mirroring the Phase-1 service binding
(§4, merged in #1969) and reproducing the proven action correlation logic
of the standalone bridge daemon (`binaries/ros2-bridge-node/src/main.rs`).
The design is a **pull-style** API (poll-and-`block_on`), not the daemon's
stream-merged event loop with per-goal background threads.

---

> **Spike-verified corrections (2026-05-30).** A compile spike on branch
> `spike/ros2-python-action` validated the value-owned layout AND the
> `block_on`-over-`&self` borrow against ros2-client 0.8. Two design
> assumptions below were **wrong** and are corrected here; where the §5.3–5.6
> code blocks still show the old form, the GREEN implementation is authoritative:
> 1. **`GoalId` is `ros2_client::action::GoalId`** (not `action_msgs::`).
> 2. **Action goal handles are `Clone`, NOT `Copy`.** `NewGoalHandle<G>` /
>    `AcceptedGoalHandle<G>` / `ExecutingGoalHandle<G>` derive `Copy` only with a
>    `G: Copy` bound, and `G = BridgeMessage` is `Clone`-not-`Copy`. The FSM
>    methods (`get_new_goal`, `accept_goal`, `start_executing_goal`,
>    `publish_feedback`, `send_result_response`) **consume the handle by value**.
>    Therefore the pull API must **clone** the handle: `get_new_goal(handle.clone())`
>    before `accept_goal(handle)`, and read stored handles as
>    `self.executing.get(id)?.0.clone()` (never `*self.executing.get(id)`) for
>    every `send_feedback` / `send_result` / cancel. Handles are still cheap to
>    clone (goal_id + Arc-backed payload). The value-owned, no-`Arc`/`unsafe`
>    conclusion holds.

## 5.1 Goal

Add `Ros2ActionClient` and `Ros2ActionServer` PyO3 classes plus
`Ros2Node::create_action_client` / `create_action_server`, so a Python node
can:

- **Client**: `send_goal(goal) -> goal_id`, `take_feedback(goal_id, timeout)`,
  `take_result(goal_id, timeout) -> (status, result)`, `cancel(goal_id|None)`.
- **Server**: `take_goal(timeout) -> (goal_id, goal)`, `send_feedback(goal_id, fb)`,
  `send_result(goal_id, result, status)`, `take_cancel(timeout) -> [goal_id]`.

### Verified preconditions (confirmed against source on 2026-05-30)

1. **`BridgeActionType` already exists** in `dora-ros2-bridge-arrow`
   (`arrow/src/lib.rs:177`), implementing `ros2_client::ActionTypes` with
   `GoalType = ResultType = FeedbackType = BridgeMessage`. **Do not redefine it.**
2. **Action goal/result/feedback messages are already indexed** in the Python
   message map by their base names `<Name>_Goal` / `_Result` / `_Feedback`
   (`python/src/lib.rs:117-121`, verified). The synthetic
   `_SendGoal_Request/_Response`, `_GetResult_Request/_Response`, and
   `_FeedbackMessage` wrappers are **not** indexed and are **not needed** — the
   bridge serializes only the Goal/Result/Feedback payloads; `ros2_client`
   builds the service envelopes itself.
3. **`ros2_client` and `rustdds` are imported** (`lib.rs:8`:
   `use ::dora_ros2_bridge::{ros2_client, rustdds};`). All of
   `ros2_client::action::*`, `ros2_client::action_msgs::*`,
   `ros2_client::builtin_interfaces::Time`,
   `ros2_client::unique_identifier_msgs::UUID` resolve.
4. **`pyarrow_to_array_data` (`lib.rs:833`) and the `eyre` macros**
   (`Context`, `ContextCompat`, `eyre`, `bail`, imported at `lib.rs:14`) are
   reusable verbatim.
5. **The service template owns its client/server BY VALUE, not in an `Arc`**
   (`Ros2ServiceClient { client: ros2_client::Client<BridgeServiceType> }`,
   `Ros2ServiceServer { server: ros2_client::Server<BridgeServiceType> }`,
   verified). Every relevant `ActionClient` / `AsyncActionServer` method is
   `&self`; goal handles are `Clone` with no lifetime (see Spike correction 2 —
   they are NOT `Copy` for `BridgeMessage`, and the FSM methods consume them by
   value, so clone per use) — so the action classes own their client/server by
   value too. **No `Arc`, no `ouroboros`, no `unsafe`.**

### Verified compile blockers folded into this spec (from adversarial review)

- **`uuid` is NOT a dependency** (`python/Cargo.toml:8-17`). The client's
  `resolve()` fallback that parsed a UUID string is **dropped**: v1 only
  originates its own goals, so the client tracks them in a
  `HashMap<String, GoalId>` and never parses a string. The server returns
  goal-id strings produced by `goal_id().uuid.to_string()`; cancel ids returned
  to Python are compared as strings against the local map. **No `uuid` dep is
  added.**
- **`dora-message` is NOT a dependency** and `metadata::GOAL_STATUS_*` is not
  imported. The status mapping uses **inline string literals**
  (`"succeeded"`/`"aborted"`/`"canceled"`) exactly equal to the daemon's
  constant values, matching the daemon's behavioral contract without adding a
  dependency. **No `dora-message` dep is added.**
- **`map_status` is written as literal match arms**, not associated-const
  patterns (which are brittle/uncertain to compile).

---

## 5.2 Files to modify

| Path | Change |
|------|--------|
| `libraries/extensions/ros2-bridge/python/src/lib.rs` | **(primary)** Add `Ros2ActionClient`, `Ros2ActionServer` pyclasses + `#[pymethods]`; add `action_type_infos()` helper; add `create_action_client`/`create_action_server` to `Ros2Node`; register both classes in `create_dora_ros2_bridge_module`. Add 3 action constants. |
| `libraries/extensions/ros2-bridge/python/Cargo.toml` | **No change** — `uuid`/`dora-message` deliberately not added (see 5.1). |
| `apis/python/node/dora/__init__.pyi` (and any re-export shim) | Add `Ros2ActionClient`, `Ros2ActionServer` type stubs + the two new `Ros2Node` methods, matching the existing `Ros2ServiceClient`/`Ros2ServiceServer` stubs. |
| `libraries/extensions/ros2-bridge/Cargo.toml` | Add two `[[example]]` targets: `python-ros2-dataflow-action-client` and `python-ros2-dataflow-action-server` (no `required-features`, mirroring the python service examples). |
| `examples/ros2-bridge/python/action-client/` | New: `action_client_node.py`, `dataflow.yml`, `run.rs`. |
| `examples/ros2-bridge/python/action-server/` | New: `action_server_node.py`, `dataflow.yml`, `run.rs`. |
| `scripts/ros2dev.sh` | Add the two new example names to the `EXAMPLES` array (the nightly ros2-bridge CI mirror). **Do NOT** add to `tests/example-smoke.rs` / `scripts/smoke-all.sh` (ROS2 is a hard SKIP there). |
| `tests/example-smoke.rs` | Add a **non-networked** contract/unit smoke only (see 5.7) — the live dataflow is exercised via `ros2dev.sh`. |

### New constants (`lib.rs`, alongside the existing service ones at lib.rs:30-35)

```rust
/// Mirror the standalone bridge daemon's action concurrency cap.
const MAX_CONCURRENT_GOALS: usize = 8;
/// Default timeout for sending a goal and awaiting accept/reject (daemon: 30s).
const ACTION_GOAL_TIMEOUT_S: f64 = 30.0;
/// Default timeout for awaiting a terminal result (daemon: 5 min).
const ACTION_RESULT_TIMEOUT_S: f64 = 300.0;
const ACTION_RESULT_TIMEOUT: Duration = Duration::from_secs(300);
```

---

## 5.3 `Ros2ActionClient`

Owned by value (mirrors `Ros2ServiceClient`). Tracks its own goals in a
`HashMap<String, GoalId>` keyed by the stringified UUID Python sees, so it never
re-parses a string and never needs the `uuid` crate. A simple `in_flight`
counter mirrors `MAX_CONCURRENT_GOALS`, freed on a terminal `take_result` **or**
on `cancel` (the verify fix for the cancel-then-stop-polling wedge).

```rust
/// ROS2 action client. Create via [`Ros2Node::create_action_client`].
///
/// warnings:
/// - dora Ros2 bridge functionality is considered **unstable**. It may be
///   changed at any point without it being considered a breaking change.
/// - There is no `wait_for_action_server` in ros2-client 0.8: the first
///   `send_goal` simply times out (default 30s) if no server is present.
/// - Feedback for concurrent goals shares one subscription; polling feedback
///   for goal A may consume and drop a feedback message for goal B (ros2-client
///   filters by goal_id and silently drops mismatches). Prefer one in-flight
///   goal per client until per-goal feedback buffering is added.
#[pyclass]
#[non_exhaustive]
pub struct Ros2ActionClient {
    client: ros2_client::action::ActionClient<BridgeActionType>,
    goal_type_info: TypeInfo<'static>,
    result_type_info: TypeInfo<'static>,
    feedback_type_info: TypeInfo<'static>,
    /// stringified goal_id (what Python sees) -> Copy GoalId(UUID).
    /// Avoids re-parsing the uuid string and lets us track in-flight slots
    /// without depending on the `uuid` crate.
    goals: HashMap<String, ros2_client::action::GoalId>,
    /// mirrors daemon MAX_CONCURRENT_GOALS=8; freed on terminal take_result or cancel.
    in_flight: usize,
}

#[pymethods]
impl Ros2ActionClient {
    /// Send a goal and block until the server accepts/rejects it.
    /// Returns the goal_id (uuid string) on accept, or None on rejection.
    /// The goal_id is generated CLIENT-side inside `async_send_goal`.
    #[pyo3(signature = (goal, timeout_s=None))]
    pub fn send_goal(
        &mut self,
        goal: Bound<'_, PyAny>,
        timeout_s: Option<f64>,
    ) -> eyre::Result<Option<String>> {
        if self.in_flight >= MAX_CONCURRENT_GOALS {
            eyre::bail!("max concurrent goals ({MAX_CONCURRENT_GOALS}) reached");
        }
        let py = goal.py();
        let array_data = pyarrow_to_array_data(&goal)?;
        let timeout = Duration::from_secs_f64(timeout_s.unwrap_or(ACTION_GOAL_TIMEOUT_S));
        let goal_type_info = self.goal_type_info.clone();
        let (goal_id, resp) = py.detach(|| {
            // serialize: _Goal — guard lives on this OS thread for the whole block_on.
            let _guard = TypeInfoGuard::serialize(goal_type_info);
            futures::executor::block_on(async {
                let send = self.client.async_send_goal(BridgeMessage(Some(array_data)));
                futures::pin_mut!(send);
                let delay = futures_timer::Delay::new(timeout);
                match futures::future::select(send, delay).await {
                    futures::future::Either::Left((r, _)) =>
                        r.map_err(|e| eyre!("failed to send action goal: {e:?}")),
                    futures::future::Either::Right(_) =>
                        eyre::bail!("action goal send timed out after {timeout:?}"),
                }
            })
        })?;
        if !resp.accepted {
            return Ok(None); // daemon main.rs:525 — rejection just skips
        }
        let id = goal_id.uuid.to_string(); // daemon main.rs:682
        self.goals.insert(id.clone(), goal_id);
        self.in_flight += 1;
        Ok(Some(id))
    }

    /// Poll up to `timeout_s` (default 1.0) for one feedback message for
    /// `goal_id`. Returns None on timeout (poll-again semantics, not an error).
    #[pyo3(signature = (goal_id, timeout_s=None))]
    pub fn take_feedback(
        &self,
        py: Python<'_>,
        goal_id: &str,
        timeout_s: Option<f64>,
    ) -> eyre::Result<Option<Py<PyAny>>> {
        let gid = *self
            .goals
            .get(goal_id)
            .with_context(|| format!("unknown goal_id {goal_id:?}"))?;
        let timeout = Duration::from_secs_f64(timeout_s.unwrap_or(1.0));
        let feedback_type_info = self.feedback_type_info.clone();
        let msg = py.detach(|| {
            // deserialize: _Feedback
            let _guard = TypeInfoGuard::deserialize(feedback_type_info);
            futures::executor::block_on(async {
                // feedback_stream(gid) filters by goal_id (client.rs:273). It
                // borrows &self.client but is created, polled once, and dropped
                // INSIDE this block_on, so the borrow never escapes the call.
                let mut s = self.client.feedback_stream(gid);
                let next = s.next();
                futures::pin_mut!(next);
                let delay = futures_timer::Delay::new(timeout);
                match futures::future::select(next, delay).await {
                    futures::future::Either::Left((Some(r), _)) =>
                        r.map(|fb| fb.0).map_err(|e| eyre!("feedback read error: {e:?}")),
                    futures::future::Either::Left((None, _)) => Ok(None), // stream ended
                    futures::future::Either::Right(_) => Ok(None),        // no feedback yet
                }
            })
        })?;
        match msg.flatten() {
            Some(data) => Ok(Some(data.to_pyarrow(py)?.unbind())),
            None => Ok(None),
        }
    }

    /// Request and block for the terminal result of `goal_id`.
    /// Returns (status_str, result_array) on completion, or None on timeout
    /// (the slot is kept so the caller can retry). Frees the in-flight slot on
    /// terminal result. The underlying result request is sent lazily on first
    /// call; feedback arrives independently and is unaffected.
    #[pyo3(signature = (goal_id, timeout_s=None))]
    pub fn take_result(
        &mut self,
        py: Python<'_>,
        goal_id: &str,
        timeout_s: Option<f64>,
    ) -> eyre::Result<Option<(String, Py<PyAny>)>> {
        let gid = *self
            .goals
            .get(goal_id)
            .with_context(|| format!("unknown goal_id {goal_id:?}"))?;
        let timeout = Duration::from_secs_f64(timeout_s.unwrap_or(ACTION_RESULT_TIMEOUT_S));
        let result_type_info = self.result_type_info.clone();
        let out = py.detach(|| {
            // deserialize: _Result
            let _guard = TypeInfoGuard::deserialize(result_type_info);
            futures::executor::block_on(async {
                let req = self.client.async_request_result(gid);
                futures::pin_mut!(req);
                let delay = futures_timer::Delay::new(timeout);
                match futures::future::select(req, delay).await {
                    futures::future::Either::Left((r, _)) =>
                        r.map(Some).map_err(|e| eyre!("result error: {e:?}")),
                    futures::future::Either::Right(_) => Ok(None), // timeout: caller retries
                }
            })
        })?;
        match out {
            Some((status, msg)) => {
                self.goals.remove(goal_id);
                self.in_flight = self.in_flight.saturating_sub(1);
                let status_str = status_enum_to_str(status);
                let data = msg.0.context("action result contained no data")?;
                Ok(Some((status_str, data.to_pyarrow(py)?.unbind())))
            }
            None => Ok(None),
        }
    }

    /// Cancel a goal (or all goals). With `goal_id=None`, cancels ALL goals
    /// (GoalId::ZERO + Time::ZERO). With a goal_id, cancels that goal only
    /// (stamp = Time::ZERO). Returns the CancelGoalResponse return_code as an
    /// int (0=accepted, 1=rejected/nothing-matched, 2=unknown, 3=terminated).
    /// Frees the in-flight slot(s) so a cancel-then-stop-polling pattern cannot
    /// wedge `send_goal`.
    #[pyo3(signature = (goal_id=None, timeout_s=None))]
    pub fn cancel(
        &mut self,
        py: Python<'_>,
        goal_id: Option<&str>,
        timeout_s: Option<f64>,
    ) -> eyre::Result<i8> {
        let timeout = Duration::from_secs_f64(timeout_s.unwrap_or(10.0));
        let gid = match goal_id {
            Some(s) => *self
                .goals
                .get(s)
                .with_context(|| format!("unknown goal_id {s:?}"))?,
            None => ros2_client::action::GoalId::ZERO,
        };
        let stamp = ros2_client::builtin_interfaces::Time::ZERO;
        let code = py.detach(|| {
            futures::executor::block_on(async {
                // async_cancel_goal returns `impl Future + '_` borrowing
                // &self.client; pinned and awaited inside this block_on so the
                // borrow stays local.
                let fut = self.client.async_cancel_goal(gid, stamp);
                futures::pin_mut!(fut);
                let delay = futures_timer::Delay::new(timeout);
                match futures::future::select(fut, delay).await {
                    futures::future::Either::Left((r, _)) =>
                        r.map(|resp| resp.return_code as i8)
                         .map_err(|e| eyre!("cancel error: {e:?}")),
                    futures::future::Either::Right(_) => eyre::bail!("cancel timed out"),
                }
            })
        })?;
        // VERIFY FIX (issue 6): free slot(s) on cancel so a canceled-but-unpolled
        // goal cannot pin a slot forever.
        match goal_id {
            Some(s) => {
                if self.goals.remove(s).is_some() {
                    self.in_flight = self.in_flight.saturating_sub(1);
                }
            }
            None => {
                self.in_flight = self.in_flight.saturating_sub(self.goals.len());
                self.goals.clear();
            }
        }
        Ok(code)
    }
}

// Surface the terminal GoalStatusEnum as a string (the daemon discards it;
// surfacing it is a strict superset). Inline literals — no dora-message dep.
fn status_enum_to_str(status: ros2_client::action_msgs::GoalStatusEnum) -> String {
    use ros2_client::action_msgs::GoalStatusEnum as S;
    match status {
        S::Succeeded => "succeeded",
        S::Aborted => "aborted",
        S::Canceled => "canceled",
        S::Accepted => "accepted",
        S::Executing => "executing",
        S::Canceling => "canceling",
        S::Unknown => "unknown",
    }
    .to_string()
}
```

---

## 5.4 `Ros2ActionServer`

Owned by value (mirrors `Ros2ServiceServer`). Stores Executing handles
(`Copy`, no lifetime) in a bounded `HashMap`. **Verify fix (issue 7):** the
capacity policy mirrors the daemon — abort the **oldest** goal only when at
`MAX_CONCURRENT_GOALS` on insert; the unconditional 300s time-sweep is dropped
(it could silently abort a legitimately long-running action). **Verify fix
(issue / FSM gap 7):** every `abort`/`send_result` is bounded by a `select`
timeout so an absent client cannot freeze the Python thread for 5 minutes; the
abort timeout is shortened from `ACTION_RESULT_TIMEOUT` to a small bound on the
accept hot path.

```rust
/// ROS2 action server. Create via [`Ros2Node::create_action_server`].
///
/// warnings:
/// - dora Ros2 bridge functionality is considered **unstable**.
/// - Goals are auto-accepted and driven to Executing on `take_goal`, so every
///   `send_feedback`/`send_result` is FSM-legal. There is no reject path in v1
///   (mirrors the daemon): reject by accepting then `send_result(status="aborted")`.
/// - `send_result` does NOT return until the client has requested the result
///   (ros2-client buffers out-of-order requests internally). It is bounded by a
///   send timeout so an absent client cannot freeze the Python thread.
#[pyclass]
#[non_exhaustive]
pub struct Ros2ActionServer {
    server: ros2_client::action::AsyncActionServer<BridgeActionType>,
    goal_type_info: TypeInfo<'static>,
    result_type_info: TypeInfo<'static>,
    feedback_type_info: TypeInfo<'static>,
    /// goal_id string -> (Executing handle (Copy), accept time). Bounded like the
    /// service server's `pending`: abort the oldest at MAX_CONCURRENT_GOALS on
    /// insert. Upstream has NO expiry on its goal map, so this is the leak guard.
    executing: HashMap<
        String,
        (ros2_client::action::ExecutingGoalHandle<BridgeMessage>, Instant),
    >,
}

/// Short bound for the accept-path abort of a no-payload / evicted goal, so a
/// client that never requests the result can't freeze take_goal for 5 minutes.
const ACTION_ABORT_TIMEOUT: Duration = Duration::from_secs(5);

#[pymethods]
impl Ros2ActionServer {
    /// Wait up to `timeout_s` (default 1.0) for the next goal. Auto-runs
    /// receive_new_goal -> get_new_goal -> accept_goal -> start_executing_goal
    /// (daemon pipeline, main.rs:666-680), so the stored handle is always
    /// Executing-ready. Returns (goal_id_str, goal_array) or None (timeout, or a
    /// no-payload goal which is auto-aborted).
    #[pyo3(signature = (timeout_s=None))]
    pub fn take_goal(
        &mut self,
        py: Python<'_>,
        timeout_s: Option<f64>,
    ) -> eyre::Result<Option<(String, Py<PyAny>)>> {
        let timeout = Duration::from_secs_f64(timeout_s.unwrap_or(1.0));
        let goal_type_info = self.goal_type_info.clone();
        let taken = py.detach(|| {
            // deserialize: _Goal
            let _guard = TypeInfoGuard::deserialize(goal_type_info);
            futures::executor::block_on(async {
                let recv = self.server.receive_new_goal();
                futures::pin_mut!(recv);
                let delay = futures_timer::Delay::new(timeout);
                let handle = match futures::future::select(recv, delay).await {
                    futures::future::Either::Left((r, _)) =>
                        r.map_err(|e| eyre!("receive_new_goal: {e:?}"))?,
                    futures::future::Either::Right(_) => return Ok(None),
                };
                let data = self.server.get_new_goal(handle); // Option<BridgeMessage>
                let accepted = self.server.accept_goal(handle).await
                    .map_err(|e| eyre!("accept_goal: {e:?}"))?;
                let executing = self.server.start_executing_goal(accepted).await
                    .map_err(|e| eyre!("start_executing_goal: {e:?}"))?;
                Ok::<_, eyre::Report>(Some((executing, data)))
            })
        })?;
        let Some((executing, data)) = taken else { return Ok(None); };
        let goal_id = executing.goal_id().uuid.to_string();
        // No payload => mirror daemon AbortGoal (main.rs:688). Bounded abort.
        let Some(arr) = data.and_then(|m| m.0) else {
            self.abort(py, executing, ACTION_ABORT_TIMEOUT)?;
            return Ok(None);
        };
        // Leak guard (daemon main.rs:805-829): abort the oldest only when at cap.
        // No unconditional time-sweep (verify fix issue 7).
        let now = Instant::now();
        while self.executing.len() >= MAX_CONCURRENT_GOALS {
            let Some(oldest) = self
                .executing
                .iter()
                .min_by_key(|(_, (_, t))| *t)
                .map(|(k, _)| k.clone())
            else {
                break;
            };
            if let Some((h, _)) = self.executing.remove(&oldest) {
                self.abort(py, h, ACTION_ABORT_TIMEOUT)?;
            }
        }
        self.executing.insert(goal_id.clone(), (executing, now));
        Ok(Some((goal_id, arr.to_pyarrow(py)?.unbind())))
    }

    /// Publish one feedback message for an executing goal. Requires Executing
    /// (always true for handles in `executing`). Handle is Copy, read without removal.
    pub fn send_feedback(
        &self,
        goal_id: &str,
        feedback: Bound<'_, PyAny>,
    ) -> eyre::Result<()> {
        let py = feedback.py();
        let (handle, _) = *self
            .executing
            .get(goal_id)
            .with_context(|| format!("unknown/finished goal {goal_id}"))?;
        let array_data = pyarrow_to_array_data(&feedback)?;
        let feedback_type_info = self.feedback_type_info.clone();
        py.detach(|| {
            // serialize: _Feedback
            let _guard = TypeInfoGuard::serialize(feedback_type_info);
            futures::executor::block_on(
                self.server.publish_feedback(handle, BridgeMessage(Some(array_data))),
            )
            .map_err(|e| eyre!("publish_feedback: {e:?}"))
        })?;
        Ok(())
    }

    /// Send the terminal result and retire the goal. `status` maps as the daemon
    /// (main.rs:744-764): succeeded->Succeeded, aborted->Aborted,
    /// canceled->Canceled, unknown->Aborted, None->Succeeded. Bounded by a send
    /// timeout because send_result_response won't resolve until the client
    /// requests the result.
    #[pyo3(signature = (goal_id, result, status=None, timeout_s=None))]
    pub fn send_result(
        &mut self,
        goal_id: &str,
        result: Bound<'_, PyAny>,
        status: Option<&str>,
        timeout_s: Option<f64>,
    ) -> eyre::Result<()> {
        let py = result.py();
        let (handle, _) = self
            .executing
            .remove(goal_id) // terminal: consume the handle
            .with_context(|| format!("unknown/finished goal {goal_id}"))?;
        let end = map_status(status);
        let array_data = pyarrow_to_array_data(&result)?;
        let timeout = Duration::from_secs_f64(timeout_s.unwrap_or(ACTION_RESULT_TIMEOUT_S));
        let result_type_info = self.result_type_info.clone();
        py.detach(|| {
            // serialize: _Result
            let _guard = TypeInfoGuard::serialize(result_type_info);
            futures::executor::block_on(async {
                let send = self.server.send_result_response(
                    handle, end, BridgeMessage(Some(array_data)),
                );
                futures::pin_mut!(send);
                let delay = futures_timer::Delay::new(timeout);
                match futures::future::select(send, delay).await {
                    futures::future::Either::Left((r, _)) =>
                        r.map_err(|e| eyre!("send_result_response: {e:?}")),
                    futures::future::Either::Right(_) =>
                        eyre::bail!("send_result timed out after {timeout:?}"),
                }
            })
        })?;
        Ok(())
    }

    /// Poll up to `timeout_s` (default 1.0) for a cancel request. ros2-client
    /// resolves all four (goal_id, stamp) policies and the {Accepted,Executing}
    /// filter internally. Returns the list of goal_id strings the client asked
    /// to cancel that we still track (the Python handler should then
    /// `send_result(..., status="canceled")` for each), or None on timeout.
    #[pyo3(signature = (timeout_s=None))]
    pub fn take_cancel(
        &self,
        py: Python<'_>,
        timeout_s: Option<f64>,
    ) -> eyre::Result<Option<Vec<String>>> {
        let timeout = Duration::from_secs_f64(timeout_s.unwrap_or(1.0));
        py.detach(|| {
            futures::executor::block_on(async {
                let recv = self.server.receive_cancel_request();
                futures::pin_mut!(recv);
                let delay = futures_timer::Delay::new(timeout);
                let cancel_handle = match futures::future::select(recv, delay).await {
                    futures::future::Either::Left((r, _)) =>
                        r.map_err(|e| eyre!("receive_cancel_request: {e:?}"))?,
                    futures::future::Either::Right(_) => return Ok(None),
                };
                // CancelHandle.goals() is the upstream-resolved candidate set;
                // respond_to_cancel_requests re-filters internally, so passing the
                // whole set is safe.
                let goals: Vec<_> = cancel_handle.goals().collect();
                self.server
                    .respond_to_cancel_requests(&cancel_handle, goals.iter().copied())
                    .await
                    .map_err(|e| eyre!("respond_to_cancel_requests: {e:?}"))?;
                let mine: Vec<String> = goals
                    .iter()
                    .map(|g| g.uuid.to_string())
                    .filter(|s| self.executing.contains_key(s))
                    .collect();
                Ok(Some(mine))
            })
        })
    }
}

impl Ros2ActionServer {
    /// Abort an executing goal by sending an empty Aborted result so the client's
    /// pending result request resolves (daemon abort_executing_goal, main.rs:839).
    /// Bounded by `timeout` so a never-requesting client cannot freeze the thread.
    fn abort(
        &self,
        py: Python<'_>,
        handle: ros2_client::action::ExecutingGoalHandle<BridgeMessage>,
        timeout: Duration,
    ) -> eyre::Result<()> {
        let result_type_info = self.result_type_info.clone();
        py.detach(|| {
            let _guard = TypeInfoGuard::serialize(result_type_info);
            futures::executor::block_on(async {
                let send = self.server.send_result_response(
                    handle,
                    ros2_client::action::GoalEndStatus::Aborted,
                    BridgeMessage(None),
                );
                futures::pin_mut!(send);
                let delay = futures_timer::Delay::new(timeout);
                match futures::future::select(send, delay).await {
                    futures::future::Either::Left((r, _)) =>
                        r.map_err(|e| eyre!("abort send_result_response: {e:?}")),
                    // Best-effort: a client that already gave up leaves the upstream
                    // goal entry pinned; we don't propagate the timeout as an error
                    // on the accept hot path.
                    futures::future::Either::Right(_) => Ok(()),
                }
            })
        })?;
        Ok(())
    }
}

// VERIFY FIX (issue 3): literal match arms (NOT associated-const patterns),
// inline strings (NO dora-message dep). Values equal the daemon's constants.
fn map_status(status: Option<&str>) -> ros2_client::action::GoalEndStatus {
    use ros2_client::action::GoalEndStatus as E;
    match status {
        None | Some("succeeded") => E::Succeeded, // daemon main.rs:763 (None->Succeeded)
        Some("aborted") => E::Aborted,
        Some("canceled") => E::Canceled,
        Some(_) => E::Aborted, // unknown -> Aborted (daemon main.rs:760)
    }
}
```

---

## 5.5 `Ros2Node` constructor methods + helper + module registration

Add an action analogue of `service_type_infos` returning **three** `TypeInfo`s.
Both constructors take `&mut self` on `Ros2Node` (Node creation needs `&mut`),
reuse the single flattened QoS profile across all 5 endpoints (daemon
main.rs:416/442). There is **no `wait_for_action_server`** — unlike
`create_service_client`, neither constructor blocks on discovery (documented).

```rust
/// Parse a `namespace/Action` (or `::`) type string into package + type name
/// and the `_Goal`/`_Result`/`_Feedback` `TypeInfo`s. Mirrors `service_type_infos`.
fn action_type_infos(
    action_type: &str,
    messages: &Arc<HashMap<String, HashMap<String, Message>>>,
) -> eyre::Result<(String, String, TypeInfo<'static>, TypeInfo<'static>, TypeInfo<'static>)> {
    let (package, type_name) =
        match (action_type.split_once('/'), action_type.split_once("::")) {
            (Some(t), None) => t,
            (None, Some(t)) => t,
            _ => eyre::bail!(
                "Expected action type `namespace/Action` or `namespace::Action`, \
                 such as `example_interfaces/Fibonacci`, but got: {action_type}"
            ),
        };
    let mk = |suffix: &str| TypeInfo {
        package_name: package.to_owned().into(),
        message_name: format!("{type_name}_{suffix}").into(),
        messages: messages.clone(),
    };
    Ok((
        package.to_owned(),
        type_name.to_owned(),
        mk("Goal"),
        mk("Result"),
        mk("Feedback"),
    ))
}
```

```rust
// on impl Ros2Node:

/// Create a ROS2 action client. There is no `wait_for_action_server`; the first
/// `send_goal` times out (default 30s) if no server is present.
/// :type action_name: str
/// :type action_type: str
/// :type qos: dora.Ros2QosPolicies
/// :rtype: dora.Ros2ActionClient
pub fn create_action_client(
    &mut self,
    action_name: &str,
    action_type: String,
    qos: qos::Ros2QosPolicies,
) -> eyre::Result<Ros2ActionClient> {
    let (package, type_name, goal_type_info, result_type_info, feedback_type_info) =
        action_type_infos(&action_type, &self.messages)?;
    let q: rustdds::QosPolicies = qos.into();
    let client = self
        .node
        .create_action_client::<BridgeActionType>(
            dora_ros2_bridge::detect_service_mapping(),
            &ros2_client::Name::new("/", action_name.trim_start_matches('/'))
                .map_err(|e| eyre!("failed to parse action name: {e}"))?,
            &ros2_client::ActionTypeName::new(&package, &type_name),
            ros2_client::action::ActionClientQosPolicies {
                goal_service: q.clone(),
                result_service: q.clone(),
                cancel_service: q.clone(),
                feedback_subscription: q.clone(),
                status_subscription: q,
            },
        )
        .map_err(|e| eyre!("failed to create action client: {e:?}"))?;
    Ok(Ros2ActionClient {
        client,
        goal_type_info,
        result_type_info,
        feedback_type_info,
        goals: HashMap::new(),
        in_flight: 0,
    })
}

/// Create a ROS2 action server. Drive it by polling `take_goal`, then
/// `send_feedback` / `send_result` (and optionally `take_cancel`).
/// :rtype: dora.Ros2ActionServer
pub fn create_action_server(
    &mut self,
    action_name: &str,
    action_type: String,
    qos: qos::Ros2QosPolicies,
) -> eyre::Result<Ros2ActionServer> {
    let (package, type_name, goal_type_info, result_type_info, feedback_type_info) =
        action_type_infos(&action_type, &self.messages)?;
    let q: rustdds::QosPolicies = qos.into();
    let server = self
        .node
        .create_action_server::<BridgeActionType>(
            dora_ros2_bridge::detect_service_mapping(),
            &ros2_client::Name::new("/", action_name.trim_start_matches('/'))
                .map_err(|e| eyre!("failed to parse action name: {e}"))?,
            &ros2_client::ActionTypeName::new(&package, &type_name),
            ros2_client::action::ActionServerQosPolicies {
                goal_service: q.clone(),
                result_service: q.clone(),
                cancel_service: q.clone(),
                feedback_publisher: q.clone(),
                status_publisher: q,
            },
        )
        .map_err(|e| eyre!("failed to create action server: {e:?}"))?;
    let server = ros2_client::action::AsyncActionServer::new(server);
    Ok(Ros2ActionServer {
        server,
        goal_type_info,
        result_type_info,
        feedback_type_info,
        executing: HashMap::new(),
    })
}
```

**Module registration** — add to `create_dora_ros2_bridge_module` (lib.rs:854),
alongside the existing `add_class` calls:

```rust
m.add_class::<Ros2ActionClient>()?;
m.add_class::<Ros2ActionServer>()?;
```

(`Ros2Node::create_action_client/server` are picked up automatically as they
live in the existing `#[pymethods] impl Ros2Node`.)

---

## 5.6 Pull-API semantics: FSM, cancel, expiry, feedback, status

### Goal FSM
- **Server acceptance**: `take_goal` runs `receive_new_goal → get_new_goal →
  accept_goal → start_executing_goal` inside one `block_on`. Because the handle
  reaches **Executing before** it is stored, every later `send_feedback`
  (requires Executing) and `send_result` (valid from Accepted|Executing|
  Canceling) is FSM-legal. Goal_id is generated **client-side** (random v4
  inside `async_send_goal`); duplicate goal_ids are silently discarded upstream.
  No-payload goals are auto-aborted (bounded empty `Aborted` result) and never
  surfaced — mirroring the daemon's `AbortGoal`.
- **Client acceptance**: `send_goal` checks `resp.accepted`; rejection returns
  `None`, no map entry, no `in_flight` increment.
- **No reject path in v1** (matches the daemon, which auto-accepts). Reject by
  accept-then-`send_result(status="aborted")`. Documented FSM gap, not a bug.

### The four cancel (goal_id, stamp) cases — all resolved **inside ros2-client**
1. `goal_id==ZERO && stamp==ZERO` → cancel ALL. Client: `cancel(None)`.
2. `goal_id==ZERO && stamp!=ZERO` → cancel all with `accepted_time < stamp`.
   **Not exposed in v1** (no Python `Time` plumbed). Documented gap.
3. `goal_id!=ZERO && stamp==ZERO` → cancel exactly that goal. Client:
   `cancel("<uuid>")`.
4. `goal_id!=ZERO && stamp!=ZERO` → that goal OR any with `accepted_time <
   stamp`. **Not exposed in v1.**
   Server-side, `take_cancel` calls `receive_cancel_request` (applies all four
   policies + the {Accepted,Executing} filter) and `respond_to_cancel_requests`
   (transitions to **Canceling**, replies `return_code=Rejected` if nothing
   matched), then returns the canceled ids we still hold so the handler can
   `send_result(status="canceled")` (legal from Canceling). A canceled goal the
   handler never finishes stays `Canceling` upstream and pins its `executing`
   slot until the next `take_goal` capacity-abort reclaims it. Documented.

### Result request / caching / expiry
Inherited from ros2-client's `AsyncActionServer`: `send_result_response`
internally drains the result-service request stream and buffers other goals'
requests in `result_requests: BTreeMap<GoalId, RmwRequestId>`, so out-of-order
finishing answers the correct `RmwRequestId`. **The Python layer keeps no result
cache and never reads the raw result request.** Upstream has **no** time-based
expiry; the only leak guard is the local `executing` map's abort-oldest-at-cap
policy. Client-side, `in_flight` is freed on terminal `take_result` **or** on
`cancel`; on `take_result` timeout the slot is kept so the caller can retry.

### Feedback streaming (pull, lossy for concurrent goals)
`take_feedback` builds an ephemeral `feedback_stream(gid)` (filters by goal_id),
drives one `.next()` under `select(.., Delay)`, then drops it — all inside one
`block_on`. **Confirmed**: both `feedback_stream` and `receive_feedback` read
the single shared subscription and silently drop (debug-log) feedback for a
non-matching goal_id. So with concurrent goals, polling feedback for A can drop
B's feedback — an inherent ros2-client 0.8 limitation. **v1 is safe for one
in-flight goal per client; documented in the docstring.**

### Status publishing
The server publishes `GoalStatusArray` automatically on every transition
(private `publish_statuses`); there is no public manual-status method and the
pull API exposes none (matches the daemon, which never consumes the status
topic). The client does not subscribe to status; instead `take_result`
**surfaces** the terminal `GoalStatusEnum` as a string — a superset of the
daemon, which discards it. Intermediate `Executing`/`Canceling` transitions are
not observable from Python in v1.

### TypeInfoGuard placement (the #1 trap)
Each guard is created **inside** the `py.detach(|| { let _guard = ...; block_on(...) })`
closure so set/use/clear stay on one OS thread (matches the service template):
- **serialize**: `_Goal` in `send_goal`; `_Feedback` in `send_feedback`;
  `_Result` in `send_result` and `abort`.
- **deserialize**: `_Feedback` in `take_feedback`; `_Result` in `take_result`;
  `_Goal` in `take_goal`. (`take_cancel` does no payload (de)serialization — no
  guard.)

### GIL boundaries
Every blocking ROS2 call is wrapped in `py.detach(|| futures::executor::block_on(...))`
so it never holds the GIL while blocking. `pyarrow_to_array_data`/`to_pyarrow`
run while attached (before/after `detach`).

---

## 5.7 Example + smoke wiring

Directory layout mirrors `examples/ros2-bridge/python/service-client/`:

```
examples/ros2-bridge/python/action-client/{action_client_node.py, dataflow.yml, run.rs}
examples/ros2-bridge/python/action-server/{action_server_node.py, dataflow.yml, run.rs}
```

**`action_client_node.py`** (sketch): build `Ros2Context → Ros2Node →
Ros2QosPolicies(reliable=True)`; `client = node.create_action_client("/fibonacci",
"example_interfaces/Fibonacci", qos)`; on each `Node().next()` tick:
`gid = client.send_goal(pa.array([{"order": 5}]))`; loop `take_feedback(gid,
timeout_s=0.5)` then `take_result(gid)`; assert result, then
`print("PYTHON ACTION CLIENT OK", flush=True)`. Drives against an `rclcpp`/`rclpy`
Fibonacci **action server** spawned by `run.rs`.

**`action_server_node.py`** (sketch): `server = node.create_action_server(
"/fibonacci", "example_interfaces/Fibonacci", qos)`; loop: `g = server.take_goal(
timeout_s=0.5)`; if `g`: unpack `(gid, goal)`, publish a couple of
`send_feedback(gid, ...)`, then `send_result(gid, pa.array([{"sequence": [...]}]),
status="succeeded")`; also poll `take_cancel(timeout_s=0.0)` and finish canceled
ids with `status="canceled"`; print `"PYTHON ACTION SERVER OK"`. Drives against
an `rclcpp`/`rclpy` Fibonacci **action client**.

**`dataflow.yml`** (per example): one python node, `path: ./<node>.py`,
`inputs: { tick: dora/timer/millis/500 }`, `outputs: []`, no `build:` (uv).

**`run.rs`** (per example, mirror `python/service-client/run.rs` exactly,
including the `.join("../../../").join(file!())` working-dir form and
`uv: true`, `dora_run("dataflow.yml", true)` detach=true): build the dataflow,
spawn it on a thread, spawn the ROS2 counterpart via `process_wrap`
`CommandWrap::with_new("ros2", run <pkg> <node>)` under `ProcessGroup::leader()`,
join the dataflow then `.kill()` the ROS2 node. **Set `RMW_IMPLEMENTATION`** in
the spawned ROS2 process env (and document it in the node docstring) to match
the dev harness (e.g. `rmw_cyclonedds_cpp`), since DDS discovery between the
bridge and the counterpart requires a common RMW.

**`Cargo.toml` (`libraries/extensions/ros2-bridge/Cargo.toml`)** — two new
`[[example]]` targets, no `required-features` (mirroring the python service
examples at :77-79):

```toml
[[example]]
name = "python-ros2-dataflow-action-client"
path = "../../../examples/ros2-bridge/python/action-client/run.rs"

[[example]]
name = "python-ros2-dataflow-action-server"
path = "../../../examples/ros2-bridge/python/action-server/run.rs"
```

**`scripts/ros2dev.sh`** — add both names to the `EXAMPLES` array (the nightly
ros2-bridge CI mirror). **Do NOT** add them to `tests/example-smoke.rs` or
`scripts/smoke-all.sh` (ROS2 is a hard SKIP there).

**`tests/example-smoke.rs`** — add only a **non-networked contract test** (no
DDS): assert `action_type_infos("example_interfaces/Fibonacci", &messages)`
yields `_Goal`/`_Result`/`_Feedback` `TypeInfo`s that resolve in a loaded
message map, and that `map_status`/`status_enum_to_str` round-trip the four
status strings. Keep it in the same `contract_*` family the PR CI runs.

---

## 5.8 TDD plan

**RED** (in `dora-ros2-bridge-python`, `#[cfg(test)]` — no DDS, no GIL network):
1. `map_status`: `None`/`"succeeded"` → `Succeeded`; `"aborted"` → `Aborted`;
   `"canceled"` → `Canceled`; `"bogus"` → `Aborted`. (Pure fn; write first,
   watch it fail to compile until `map_status` exists.)
2. `status_enum_to_str`: each `GoalStatusEnum` variant → expected lowercase
   string; assert `"succeeded"/"aborted"/"canceled"` match the daemon's constant
   *values*.
3. `action_type_infos`: given a hand-built `messages` map containing
   `Fibonacci_Goal/_Result/_Feedback`, assert it returns
   `(package, type_name)` plus three `TypeInfo`s whose `message_name`s are the
   three suffixed names; and that an unknown/`badformat` type errors.

**GREEN**: implement `map_status`, `status_enum_to_str`, `action_type_infos`,
then the two pyclasses + the two `Ros2Node` methods + module registration until
(a) the unit tests pass and (b) `maturin build` of `dora-ros2-bridge-python`
succeeds (cargo excludes this crate — see 5.10).

**IMPROVE**: run `cargo clippy -p dora-ros2-bridge-python -- -D warnings` (works
even though it's excluded from `--all`; the Python feature gates are compile-only
here), `cargo fmt --all -- --check`. Then a **live** loop on the ROS2 dev harness
(`scripts/ros2dev.sh`) running `python-ros2-dataflow-action-client` and
`-action-server` against rclcpp counterparts until the `... OK` sentinels print.
Verify the `in_flight`/`executing` bookkeeping under: reject, cancel-then-stop,
no-payload goal, and result-before-finish ordering.

---

## 5.9 Risks / open items (spike before coding)

1. **`block_on` on a multi-thread tokio runtime + `Send` `#[pyclass]`** —
   *Medium confidence.* The borrow analysis is verified: every relevant
   `AsyncActionServer`/`ActionClient` method is `&self`; handles are `Clone`/no
   lifetime (cloned per use — Spike correction 2); the only `+ '_` borrowers
   (`feedback_stream`, `async_cancel_goal`)
   are consumed inside one `block_on`; `AsyncActionServer`'s internal
   `std::sync::Mutex` is never held across `.await`. The residual uncertainty is
   purely *whether the structs are `Send`* for `#[pyclass]`. **Spike:** add the
   two field-only structs and the two constructors, then `maturin build` (or
   `cargo build -p dora-ros2-bridge-python` invoked directly, bypassing the
   workspace exclude) to confirm `#[pyclass]`'s `Send` bound is satisfied before
   writing the method bodies. This is the single highest-value pre-coding check.
2. **`map_status` literal-arm form** — *High confidence it compiles* (plain
   `&str` literal patterns), chosen specifically to avoid the brittle
   associated-const-pattern form the adversarial pass flagged.
3. **`abort` inside `take_goal` blocking the accept hot path** — bounded to
   `ACTION_ABORT_TIMEOUT` (5s) and made best-effort (timeout → `Ok(())`), so a
   client that never requests an aborted/evicted goal's result cannot freeze
   `take_goal` for 5 minutes. Residual: the upstream `goals` entry for such a
   goal still leaks (no upstream expiry) — acceptable for v1, documented.
4. **Concurrent-goal feedback loss** — inherent ros2-client 0.8 limitation;
   v1 contract is one in-flight goal per client. Not a code risk; an API
   limitation to call out in docs/PR.
5. **Cancel cases 2 & 4 unreachable in v1** — needs a Python `Time` type; out
   of scope. Documented.
6. **RMW interop in the example** — the live example only validates if the
   bridge and the rclcpp counterpart share an RMW; set `RMW_IMPLEMENTATION`
   explicitly in `run.rs`. Per project memory, the *rust* action-client example
   was noted as "rotted" — verify the harness path end-to-end before relying on
   it as a template.
7. **`detect_service_mapping()` / `ActionTypeName` / `ActionClientQosPolicies`
   field names** — verified against daemon `main.rs:416-459`; reconfirm the
   exact field identifiers compile during the spike (they are the load-bearing
   names the constructors fill).

---

## 5.10 Build note

`dora-ros2-bridge-python` is **excluded** from `cargo build --all` / clippy /
test (it's a PyO3 cdylib built with **maturin**). Verify changes via either
`maturin build` of this crate **or** a direct `cargo build -p
dora-ros2-bridge-python` (which works for the spike). The standalone daemon
(`binaries/ros2-bridge-node`) and `dora-ros2-bridge-arrow` remain in the normal
cargo build and exercise the same underlying action types, so a regression in
`BridgeActionType` would surface there.

---

## 5.11 Note: the live #1170 checklist is stale

The public #1170 tracking checklist still lists the Python **service**
client/server as outstanding, but they landed in **PR #1969** (the
`Ros2ServiceClient`/`Ros2ServiceServer` pyclasses this Phase-2 design mirrors).
The checklist should be updated to mark Python service client/server **done
(#1969)** and to add the two Phase-2 items specified here (Python action client
and server), so the roadmap reflects reality before this PR opens.
