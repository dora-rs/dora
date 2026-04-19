# Capability-to-Test Matrix

Where each Dora capability is covered, at what tier, and how strongly.

Answers three questions that the test-type-oriented
[`testing-matrix.md`](testing-matrix.md) and the command-oriented
[`testing-guide.md`](testing-guide.md) leave implicit:

- If I change this feature, what tests should I run?
- Is this documented capability actually validated anywhere?
- Where are the known coverage gaps?

Addresses [#1633](https://github.com/dora-rs/dora/issues/1633); parent
[#1628](https://github.com/dora-rs/dora/issues/1628).

---

## Legend

- **Tier**: `PR`, `Nightly`, `Manual` — per
  [`testing-matrix.md`](testing-matrix.md)'s tier policy.
- **Strength**:
  - **Contract** — asserts a specific behavioral outcome (exact
    markers, counts, orderings). A regression in the feature will
    fail the test.
  - **Smoke** — proves the code path ran to completion without an
    obvious error. A regression that silently changes behavior may
    still pass.
  - **Structural** — formatting / lint / supply chain / etc. Enforces
    repo invariants, not feature behavior.

---

## CLI lifecycle & validation

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| `dora --help` + per-subcommand `--help` | `test` job argparse block (ci.yml) | PR | Contract |
| `dora validate` (basic + `--strict-types`) | `test` job validate block | PR | Contract |
| `dora expand`, `dora graph` | `test` job CLI smoke block | PR | Contract |
| `dora new` templates (rust, python) on 3 platforms | `cli` job (ci.yml) | PR | Smoke |
| `dora new` templates (c, cxx, cmake) on Linux | `cli` job (ci.yml) | PR | Smoke |
| `dora run` local mode, wide set of examples | `examples` job (3 platforms) | PR | Smoke |
| `dora up`/`start`/`stop`/`down` lifecycle | `smoke_*` in `example-smoke.rs` | Nightly | Smoke |
| `dora run --stop-after` contract on 4 examples | `contract_*` in `example-smoke.rs` | PR | Contract |
| `dora self update --check-only` (read-only path) | `topic-and-top-smoke` job | Nightly | Smoke |

Known gap: `dora self update` destructive swap path (tracked in
[#215](https://github.com/dora-rs/dora/issues/215)).

## Dataflow execution modes

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| `dora run` (embedded coord + daemon) | `smoke_local_*`, `contract_*` | PR (contract) / Nightly (smoke) | Contract + Smoke |
| `dora up` + `dora start --detach` (full network mode) | `smoke_*` (networked siblings) | Nightly | Smoke |
| Module expansion (`module:` inline inclusion) | `smoke_module_dataflow`, `smoke_local_module_dataflow` | Nightly | Smoke |
| Module loading from URL | `smoke_rust_dataflow_url` | Nightly | Smoke |
| Dynamic dataflow (`dora run` dynamic variant) | `smoke_rust_dataflow_dynamic` | Nightly | Smoke |
| Python builder API | no automated coverage | — | Gap |

## Service pattern (request / reply)

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| request_id correlation, arithmetic contract | `contract_service_example_correlates_requests_and_responses` | PR | Contract |
| Two clients fan-in to one server | `smoke_service_example`, `smoke_local_service_example` | Nightly | Smoke |
| Timeout / retry paths | no automated coverage | — | Gap (#1630 follow-up) |

## Action pattern (goal / feedback / result)

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| feedback precedes terminal `succeeded` | `contract_action_example_feedback_precedes_success_result` | PR | Contract |
| Two clients fan-in, countdown | `smoke_action_example`, `smoke_local_action_example` | Nightly | Smoke |
| Cancellation path | no automated coverage | — | Gap (#1630 follow-up) |

## Streaming pattern (session / segment / chunk)

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| `fin=true` triggers session reassembly | `contract_streaming_example_reassembles_session_on_fin` | PR | Contract |
| Multi-session token streams | `smoke_streaming_example`, `smoke_local_streaming_example` | Nightly | Smoke |
| `flush=true` + interruption semantics | no automated coverage | — | Gap (#1630 follow-up) |

## Validated pipeline (deterministic source → transform → sink)

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| Source emits 10, transform doubles, sink asserts match | `contract_validated_pipeline_produces_exactly_ten_doubled_outputs` | PR | Contract |
| Liveness smoke on same fixture | `smoke_validated_pipeline`, `smoke_local_validated_pipeline` | Nightly | Smoke |

## Topic / trace / top inspection

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| `dora top --once` (JSON snapshot) | `topic-and-top-smoke` | Nightly | Contract |
| `dora topic list --format json` (NDJSON) | `topic-and-top-smoke` | Nightly | Contract |
| `dora topic info --duration N` (≥10 msgs on 10 Hz fixture) | `topic-and-top-smoke` | Nightly | Contract |
| `dora topic echo --count N` (N frames matched) | `topic-and-top-smoke` | Nightly | Contract |
| `dora topic hz --duration N` (≥10 samples on 10 Hz fixture) | `topic-and-top-smoke` | Nightly | Contract |
| `dora topic pub --count N` | `topic-and-top-smoke` | Nightly | Contract |
| `dora trace list` / `trace view` empty-state + prefix errors | `topic-and-top-smoke` | Nightly | Contract |
| `dora top` interactive TUI mode | no automated coverage | — | Gap (needs expect harness) |

## Parameter operations

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| WS protocol happy-path CRUD (`SetParam`, `GetParam`, `GetParams`, `DeleteParam`) | `cli_param_set_get_list_delete` in `ws-cli-e2e.rs` | PR | Contract |
| WS protocol JSON-type coverage (int, float, string, bool, array, object) | `cli_param_set_json_types` | PR | Contract |
| WS protocol error paths (unknown target, missing key) | `cli_param_set_rejects_unknown_target`, `cli_param_get_nonexistent`, `cli_param_delete_rejects_unknown_target` | PR | Contract |
| `dora param` CLI subprocess CRUD + output format | `cli_param_crud_via_subprocess` in `ws-cli-e2e.rs::param_cli` | PR | Contract |
| `dora param set` JSON parse-error path | `cli_param_set_rejects_non_json_value` | PR | Contract |

## Record / replay

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| `dora record` → `.drec` file + `dora replay` round-trip (existence + size) | `record-replay` job | Nightly | Smoke |
| Semantic replay equivalence (validated-pipeline SUCCESS marker after record → replay round-trip) | `contract_record_replay_reproduces_validated_pipeline` | PR | Contract |

## Fault tolerance

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| `restart_policy: on-failure` recovery | `restart_recovers_from_failure` | PR | Contract |
| `max_restarts` exhaustion marks node failed | `max_restarts_exhaustion_marks_node_failed` | PR | Contract |
| `restart_policy: always` re-spawns on clean exit | `restart_policy_always_restarts_on_clean_exit` | PR | Contract |
| `restart_window` counter reset | `restart_window_resets_restart_counter` | PR | Contract |
| `input_timeout` delivers `InputClosed` | `input_timeout_delivers_input_closed_to_downstream` | PR | Contract |
| `health_check_timeout` SIGKILLs unresponsive node | `health_check_timeout_sigkills_unresponsive_node` | PR | Contract |
| `NodeRestarted` delivery to downstream | `node_restarted_is_delivered_to_downstream` | PR | Contract |
| `InputRecovered` delivery after break | `input_recovered_is_delivered_after_broken_input_receives_data` | PR | Contract |
| Full kill→respawn→kill cycle under health check | no automated coverage | — | Gap (#1631 follow-up) |

## Cluster lifecycle

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| `dora cluster status` (lists daemons + dataflows) | `cluster-smoke` | Nightly | Contract |
| `dora cluster down` tears everything cleanly | `cluster-smoke` | Nightly | Contract |
| `dora cluster up` (SSH-backed) | no automated coverage | — | Manual (infra-dependent) |
| `dora cluster install/uninstall/upgrade` | no automated coverage | — | Manual (systemd + SSH) |

## redb persistence & state reconstruction

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| redb store writes dataflow record + survives coord restart | `redb-backend-smoke` | Nightly | Contract |
| Running → Recovering transition on coord restart | `state-reconstruction-smoke` | Nightly | Contract |
| Daemon auto-reconnect after coord freeze | `daemon-reconnect-smoke` (Linux only) | Nightly | Contract |
| Full reconciliation back to Running post-restart | no automated coverage | — | Gap (aspirational per `state-reconstruction-smoke` docstring) |

## Cross-language interop

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| Rust → Python dataflow | `smoke_cross_language_rust_to_python`, `smoke_local_cross_language_rust_to_python` | Nightly | Smoke |
| Python → Rust dataflow | `smoke_cross_language_python_to_rust`, `smoke_local_cross_language_python_to_rust` | Nightly | Smoke |
| Semantic Rust → Python payload delivery (exact count + value validation) | `contract_cross_language_rust_to_python_delivers_all_ten_values` | PR | Contract |
| Semantic Python → Rust payload delivery (exact count + value validation) | `contract_cross_language_python_to_rust_delivers_all_ten_values` | PR | Contract |

## Python operator path

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| Python node template + `dora run` | `cli` job (ci.yml) | PR | Smoke |
| Python async / drain / echo / log / multiple-arrays / concurrent-rw examples | `smoke_python_*` + `smoke_local_python_*` | Nightly | Smoke |
| Python operator hot reload | no automated coverage | — | Gap |
| Python builder API (programmatic dataflow construction → YAML) | `examples/python-dataflow-builder/test_builder_api.py` invoked by the `cli` job | PR | Contract |
| Python builder API → `build()` + `run()` end-to-end with hub packages (`simple_example.py`) | not covered — blocked on PyPI `dora-rs` 0.5.0 clobbering the local workspace install; re-enable after 1.0 PyPI publish (#1654) | — | Gap (infrastructural) |

## C / C++ template & examples

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| `dora new --lang c/cxx` + CMake build (Linux) | `cli` job | PR | Smoke |
| `c-dataflow`, `c++-dataflow` examples (Linux) | `examples` job | PR | Smoke |
| `c++-arrow-dataflow` (Linux / macOS) | `examples` job | PR | Smoke |
| `cmake-dataflow` (Linux) | `examples` job | PR | Smoke |
| C/C++ on macOS / Windows | not covered | — | Gap — platform-parity policy in [`testing-matrix.md`](testing-matrix.md#platform-parity) |

## ROS2 bridge

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| Rust ROS2 example | `ros2-bridge` job (ci.yml, Linux) | PR | Smoke |
| Python ROS2 example | `ros2-bridge` job | PR | Smoke |
| C++ ROS2 example | `ros2-bridge` job | PR | Smoke |
| YAML bridge (topic / service / action) | `ros2-bridge` job subset | PR | Smoke |

## Soft real-time (`--rt`, SCHED_FIFO, mlock)

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| `cpu_affinity` mask actually applied | `cpu-affinity-smoke` (Linux only) | Nightly | Contract |
| `mlockall` + SCHED_FIFO enforcement | no automated coverage | — | Manual — needs privileged execution (tracked in [#256](https://github.com/dora-rs/dora/issues/256)) |

## Supply chain / repo invariants (structural)

| Sub-capability | Test(s) | Tier | Strength |
|---|---|---|---|
| `cargo-audit` + `cargo-deny` (advisories, licenses, sources) | `audit` job | PR | Structural |
| Unwrap budget ratchet | `unwrap-budget` job | PR | Structural |
| `cargo fmt --check` | `fmt` job | PR | Structural |
| `cargo clippy -D warnings` | `clippy` job | PR | Structural |
| `cargo-deny` license check | `audit` job / `check-license` | PR | Structural |
| `typos` | `typos` job | PR | Structural |
| MSRV (workspace `rust-version` in `Cargo.toml`) | `msrv` job | PR | Structural |
| 8 target triples compile | `cross-check` matrix | PR | Structural |
| Benchmark regression vs. cached baseline | `bench` job | PR | Structural |

---

## How to use this document

- **Before changing a feature**: find its row, run the listed tests
  locally (`cargo test --test <file> <name>`).
- **Before merging a PR that touches a "Gap" row**: decide whether to
  add coverage in this PR or file a follow-up issue and link it.
- **When adding a new capability**: add a new row here in the same PR
  that ships the feature. Treat this file as a gate, not a wiki.

## Related docs

- [`testing-matrix.md`](testing-matrix.md) — tier-oriented view (what
  runs in PR CI vs nightly vs manual).
- [`testing-guide.md`](testing-guide.md) — command-oriented
  reference (how to run each test locally).
- [`qa-runbook.md`](qa-runbook.md) — deep-gate investigation playbook
  (coverage, mutants, semver).

## Open coverage gaps (quick index)

The rows labeled "Gap" are consolidated here so follow-up work can be
filed and tracked:

- Python builder `build()` + `run()` with hub packages — blocked on PyPI 1.0 publish; builder API itself is covered.
- Service pattern: timeout / retry paths — #1630 follow-up.
- Action pattern: cancellation — #1630 follow-up.
- Streaming pattern: `flush=true` + interruption — #1630 follow-up.
- `dora top` interactive TUI — needs expect harness (#215).
- Fault tolerance: full kill→respawn→kill cycle under health check —
  #1631 follow-up.
- `dora cluster up` (SSH) — manual, needs dedicated infra.
- State reconstruction: full Recovering→Running reconciliation —
  aspirational per current docstring.
- Python operator hot reload — no tests.
- C/C++ on macOS/Windows — platform-parity policy.
- `mlockall` / SCHED_FIFO — #256 (manual, privileged).
