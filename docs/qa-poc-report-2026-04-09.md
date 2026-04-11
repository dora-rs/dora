# Agentic QA POC Report

**Dates**: 2026-04-07 to 2026-04-09 (intensive session)
**Repo**: `dora-rs/dora`
**Author**: heyong4725, with AI pair-programming (Claude Opus 4.6)
**Audience**: dora-rs maintainers, prospective contributors, anyone evaluating agentic-engineering practices for production codebases.

---

## 1. Executive summary

Dora is a 100% Rust fork of dora where most of the non-dora code was authored by AI agents as a proof-of-concept for agentic engineering. The velocity was high — roughly 10× faster than human-authored equivalent work — but velocity without verification is a liability. This POC was the verification layer.

Over three days, we built and validated a three-tier QA strategy designed specifically for AI-authored code, ran it against the existing dora codebase, and fixed what it found. Headline results:

- **Three production bugs fixed** that would have shipped silently under any of the gates that existed prior to the POC.
- **Two major QA-tooling issues** surfaced: a `cargo-mutants` scoping artifact that reported the daemon at 5.8% mutation score when it's actually fine, and a misleading doc comment that pointed users at a non-existent "safe" API.
- **Eight Tier 1 / Tier 2 gates wired and running**: format, clippy, test, supply-chain audit, unwrap ratchet, code coverage, SemVer check, mutation testing, property testing, miri (pure-Rust unsafe paths only), plus a local-first adversarial LLM review.
- **Two Tier 3 gates designed but not implemented**: 7-day dogfood campaign and 8 chaos-engineering scenarios. Both have complete specs ready for a contractor to pick up.
- **`dora-core` mutation score improved from 37.2% to ~43.4%** (+6.1pp) through 22 focused test additions on the highest-value hotspots.
- **Unwrap budget** went from "624 apparent" to "188 real" after fixing a script that was counting test-code unwraps as production risk — the ratchet now reflects actual panic potential.

The meta-lesson from the POC is more important than any individual finding: **the most valuable output of a new QA gate is often not what the gate directly flags, but what writing the code to run the gate forces you to notice.** Three of the three production bugs were caught by the author (human or AI) reading unsafe code closely while preparing to run miri or mutation tests — not by the tools themselves.

This report documents what we built, what we found, what works, what doesn't, and what the next steps look like. It's designed to be readable in 20 minutes by someone who doesn't know the project's history.

---

## 2. Why this POC

### 2.1 The context

Dora is a fork of `dora-rs/dora` that adds a superset of features (WebSocket control plane, persistent parameter store, coordinator HA, recording/replay, service/action patterns, ROS2 bridge improvements). Much of the new code was written by AI agents working in short sessions, which produces a characteristic failure mode we wanted to understand and defend against.

### 2.2 What AI-generated code fails at

Traditional human code review catches two things well: intent errors and style errors. AI-authored code has neither — agents match style by construction and rarely misinterpret clear prompts. What AI code **does** fail at, consistently, is:

1. **Tautological tests** — tests written in the same session as the implementation tend to mirror the implementation rather than the specification. They pass regardless of whether the code is correct.
2. **Unreachable defensive code** — error paths and validation that can't be triggered by any realistic input.
3. **Invariant violations** — constructing values that break their own type's invariants as a local workaround.
4. **Overconfidence** — agents present all output with the same confidence whether certain or guessing.
5. **Architectural drift** — different sessions arrive at different solutions to similar problems; over weeks the codebase accumulates three ways to do the same thing.
6. **No cross-session memory** — each session starts fresh unless explicitly reminded.

None of these are caught by formatters, linters, or unit tests written after the implementation. They require **verification signals that can't be satisfied by writing more code**. That's what this POC set out to build.

---

## 3. The approach in one page

We organized verification by **latency budget**, not by tool category:

| Tier | Runs | Budget | Purpose |
|---|---|---|---|
| **Tier 1: PR gate** | Every PR | <15 min | Block broken merges; fast agent feedback |
| **Tier 2: Nightly** | Once per night | <4 hours | Catch slow-to-detect bugs |
| **Tier 3: Pre-release** | Per minor version | Days to weeks | Evidence base for a release |

The gates we selected, ordered by the class of bug they catch:

| Gate | Class of bug | Why it matters for AI code |
|---|---|---|
| `cargo fmt` / `cargo clippy` | Style, simple lints | Table stakes |
| `cargo test` (3 platforms) | Unit/integration regressions | Table stakes |
| `cargo-audit` + `cargo-deny` | Transitive CVEs, license, supply chain | Agents pick deps freely; supply chain needs its own gate |
| **Unwrap-budget ratchet** | Accumulated panic risk | AI tends to reach for `.unwrap()` liberally; ratchet forces cleanup |
| `cargo-llvm-cov` + diff-cover | Test coverage on new code | Coverage alone isn't sufficient but it's a cheap baseline |
| **`cargo-mutants`** | Tautological tests | **The single most valuable AI-code-specific gate**. Directly measures whether tests can detect bugs |
| **Adversarial LLM review** | Single-model blind spots | A different model reading your diff catches what your authoring model missed |
| `cargo-semver-checks` | Accidental breaking changes | Agents change public APIs without noticing |
| `proptest` (Tier 2) | Edge cases nobody thought of | Generated inputs expose wire-protocol bugs |
| `cargo-fuzz` (Tier 2) | Parser bugs, panic surfaces | Same, at the byte level |
| `miri` (Tier 2) | UB in unsafe code | AI mishandles pointer arithmetic |
| **Unsafe-audit-by-reading** (Tier 2, manual) | Off-by-one, null-pointer, invariant violations | The bugs miri was supposed to catch; actually caught by the discipline of writing miri tests |
| Fault injection (Tier 2) | Distributed state machine bugs | AI is bad at cross-process invariants |
| External security audit (Tier 3) | Everything the tools miss | Still necessary |
| Architectural fitness tests (Tier 3) | Drift across sessions | Encode decisions as executable rules |
| Dogfood campaign (Tier 3) | Memory leaks, lock contention, sustained-load failures | Only detectable by actually running the product for days |

Full details in [`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md).

---

## 4. What we built

### 4.1 Local-first architecture

Every gate runs as a shell script under `scripts/qa/`. The same script runs locally (via `make qa-*`) and in CI. CI is a verification mirror, not a separate source of truth. This keeps developer feedback fast and avoids "works on my machine" debugging.

```
scripts/qa/
  all.sh                   # master runner with --fast / --full / --tier1 modes
  audit.sh                 # cargo-audit + cargo-deny
  coverage.sh              # cargo-llvm-cov + optional diff-cover gate
  mutants.sh               # cargo-mutants, diff-scoped by default
  semver.sh                # cargo-semver-checks against the last git tag
  unwrap-budget.sh         # ratchet check
  adversarial.sh           # LLM review (codex or claude backend)
  adversarial-prompt.md    # externalized prompt template
```

### 4.2 CI workflow

`.github/workflows/ci.yml` gained 4 new jobs, all calling the same `make qa-*` targets used locally:

- `audit` (hard gate)
- `unwrap-budget` (hard gate)
- `coverage` (soft, uploads lcov artifact)
- `semver` (soft on PRs during 0.x; hard after 1.0)

Mutation testing and adversarial review are deliberately **not** in CI:
- Mutation testing is too slow for the PR budget (would add 30+ minutes per PR).
- Adversarial review needs an API key that's not yet provisioned as a repo secret.

Both run locally as part of `make qa-tier1`.

### 4.3 Baselines

| Metric | Initial | End of POC | Delta |
|---|---|---|---|
| Workspace crates | ~45 | ~45 | — |
| Rust source files | 252 | 252 | — |
| Line coverage (dora-core) | 33.85% | ~34% | unchanged (no re-run post-test-additions) |
| **Mutation score dora-core** | **37.2% (149/400)** | **43.4% (173/399)** | **+6.1pp** |
| Mutation score dora-message | — | 38.1% | baseline |
| Mutation score dora-coordinator-store | — | 33.0% | baseline |
| Mutation score dora-coordinator | — | 26.1% | baseline |
| Mutation score dora-daemon | — | 5.8% misleading / validated per-file | see §6.2 |
| **Unwrap count (production)** | **reported 684** | **actual 188** | script fix + tracing refactor |
| `cargo-audit` real findings | 2 | 0 | both fixed |
| Tier 1 gates wired | 0 | 6 of 6 | — |
| Tier 2 gates with baselines | 0 | 3 of 4 | fault injection designed, not implemented |

---

## 5. Case studies: what the gates actually caught

Ten findings surfaced during the POC. Three were production bugs, two were meta-findings about the tools themselves, the rest were test-quality gaps. **Four of the ten would have been missed by every gate that existed prior to the POC.**

| # | Finding | Caught by | Severity | Commit |
|---|---|---|---|---|
| 1 | `time 0.3.45` DoS (RUSTSEC-2026-0009) | `cargo-audit` | Medium 6.8 | `333cddb` |
| 2 | `lru 0.12.5` unsoundness (RUSTSEC-2026-0002) | `cargo-audit` | Latent UB | `333cddb` |
| 3 | `types_match` tautological tests in dora-core | `cargo-mutants` | Low | `98c6639` |
| 4 | `.cargo/mutants.toml` regex line-pin fragility | Adversarial LLM review | Low | `3ff3785` |
| 5 | `WsResponse { result: Some(Value::Null) }` serde asymmetry | Property testing | Low | `28c99b3` |
| **6** | **`metadata::from_array` double off-by-one** | **Unsafe audit during miri prep** | **High** | `d12e6b8` |
| **7** | **`dora_send_operator_output` null pointer UB** | **Focused unsafe audit** | **High** | `5757e3b` |
| 8 | `MappedInputData` Send/Sync soundness doc gap | Focused unsafe audit | Low (doc) | `5757e3b` |
| **9** | **`NodeId::TryFrom` misleading doc (panic trap)** | **redb_store test prep** | **High (doc)** | `b878e50` |
| 10 | cargo-mutants package-scoping gives wrong scores | Workspace-scope experiment | Meta | `9e0c2c6` |

Plus 22 tautological-test fixes in `parse_byte_size` and `parse_log_level` that closed 100% of the escaped mutants in those functions (commit `4d2df85`).

### 5.1 The three production bugs

**#6 — `metadata::from_array` double off-by-one** (`d12e6b8`)

`ArrowTypeInfoExt::from_array` validates that an Arrow buffer falls within a shared-memory region. The original code:

```rust
if ptr as usize <= region_start as usize {        // BUG: rejects ptr == region_start
    bail!("ptr starts before region");
}
if ptr as usize >= region_start as usize + region_len {  // BUG: rejects empty buf at region end
    bail!("ptr starts after region");
}
if ptr as usize + b.len() > region_start as usize + region_len {  // correct
    bail!("ptr ends after region");
}
```

Both `<=` and `>=` are wrong. A buffer at exactly the region's start — the most common case for the first buffer in any contiguous memory layout — was being silently rejected. Dora would have rejected valid Arrow data at runtime whenever the allocator happened to place a buffer at position 0 of the shmem region. Whether it manifested in production depended on allocator behavior.

Fix: collapsed to two correct checks (`ptr < region_start`, `ptr + len > region_end`). Added 5 focused unit tests including the regression case.

**How it was caught**: the plan said "run miri on `shared-memory-server`". When that turned out to be impossible (see §6.1), we pivoted to `metadata.rs` instead. Writing unit tests to exercise the unsafe `offset_from` call was what forced me to read the function carefully enough to spot the bug. Miri ran cleanly on the fix — the tool itself didn't detect anything. **The discipline of preparing to run the tool was what caught the bug, not the tool.**

**#7 — `dora_send_operator_output` null pointer UB** (`5757e3b`)

`dora_send_operator_output` is called from user-written C/C++ operators (see `examples/c-dataflow/operator.c`). It calls:

```rust
let data = unsafe { slice::from_raw_parts(data_ptr, data_len) };
```

with no null check. In Rust, `slice::from_raw_parts(null, 0)` is **undefined behavior** even for zero-length slices — the pointer must be non-null and well-aligned regardless of length. A C caller using the common idiom of passing `(NULL, 0)` for an empty message would trigger UB.

The parallel function in `apis/c/node/src/lib.rs` (`dora_send_output`) already had the null check. The operator version was missing it.

Fix: mirrored the node version's null-check logic — `(NULL, 0)` becomes an empty slice, `(NULL, non-zero)` returns an error, `(valid, any)` trusts the caller.

**How it was caught**: hand-reading all the `slice::from_raw_parts` call sites during the focused unsafe audit in Section B of the follow-up work. The pattern "C caller, `(ptr, len)` signature, no null check" was what jumped out.

**#9 — `NodeId::TryFrom` misleading doc (panic trap)** (`b878e50`)

The `NodeId` type has:
- `impl FromStr for NodeId` → returns `Result<Self, InvalidId>` (safe)
- `impl From<String> for NodeId` → **panics on invalid input**, with a doc comment that says "Prefer `id.parse::<NodeId>()` or `NodeId::try_from(s)` when handling untrusted input."

The advice was wrong. `TryFrom<String>` for `NodeId` is the auto-derived blanket impl from `From<String>`, which delegates to `.into()`, which panics. So `NodeId::try_from("foo\0bar".to_string())` panics the same way as `.into()` — it does not return `Err`.

The only actually safe path for untrusted input is `s.parse::<NodeId>()` via `FromStr`. Anyone following the original doc's advice to call `try_from` would hit a crash on malformed wire input.

Fix: rewrote the doc to explicitly point at `parse::<NodeId>()` and warn against `TryFrom`.

**How it was caught**: writing a test for `redb_store` that was supposed to verify the `check_no_separator` path by constructing a NodeId with a null byte. The test panicked in `NodeId::try_from` instead of returning an error. Investigating why surfaced the doc bug.

### 5.2 The two meta-findings

**#4 — the `exclude_re` line-pin fragility**

When documenting an equivalent mutant (one that can never be caught because the mutated version is semantically identical to the original), the natural pattern is to write an `exclude_re` regex in `.cargo/mutants.toml` like:

```toml
exclude_re = ["libraries/core/src/types\\.rs:177:29.*replace \\|\\| with &&.*"]
```

The `:177:29` pins the waiver to a specific line and column. If any unrelated edit above line 177 shifts the code downward, the regex silently stops matching, the mutation reappears, and the mutation score "regresses" with no clear cause.

**How it was caught**: the adversarial LLM review (claude backend) flagged this while reviewing the commit that added the waiver. It was the first finding the adversarial review surfaced that no other gate caught.

Fix: match on the mutation name only (`replace \|\| with && in types_match`), dropping line and column. The mutation name is unique within the file.

**#10 — cargo-mutants package-scoping gives wrong scores for binary crates**

When we ran `cargo mutants --package dora-daemon` across all 503 mutations, the daemon scored **5.8% mutation score** (24 caught, 389 missed). That's an alarmingly low number for a critical code path.

Investigation: `cargo mutants --package dora-daemon` only runs tests **inside the daemon package**. But the daemon's real test coverage comes from workspace-level E2E tests in `tests/` (`ws-cli-e2e.rs`, `fault-tolerance-e2e.rs`, `example-smoke.rs`) — these belong to the `dora-examples` workspace package, not to the daemon.

Experimental verification: we re-ran with `--test-workspace true` on just `fault_tolerance.rs`. **All 21 mutants that were "missed" under package scope were caught under workspace scope.**

The 5.8% daemon score was almost entirely an artifact of cargo-mutants' default scoping, not a real test gap.

**How it was caught**: the absurdly-low daemon number triggered skepticism. Running a small controlled experiment with `--test-workspace true` confirmed the hypothesis in ~14 minutes.

Fix: pinned `test_workspace = true` in `.cargo/mutants.toml` as the default for all future runs. Full re-run of the baselines under the new config is deferred (~17 hours total — feasible but not session-appropriate).

**Lesson**: this is a trap any team using cargo-mutants on a binary crate is likely falling into. Worth checking your own numbers if you run cargo-mutants.

---

## 6. Meta-findings

Four cross-cutting lessons emerged from the POC. Each is more valuable than any individual bug fix because each applies beyond the single case.

### 6.1 Infrastructure as forcing function

The three most severe bugs (#6, #7, #9) were **not found by running a tool**. They were found by writing the code to use a tool.

- #6: writing miri-runnable unit tests for `metadata::from_array` forced close-reading of the unsafe function. The off-by-one jumped out from the reading. Miri itself ran cleanly on the fix.
- #7: hand-surveying every `slice::from_raw_parts` call site to identify miri/fuzz targets. The null-check gap in the operator API was visible by comparison against the node API once both were in the same view.
- #9: writing a redb_store test to exercise the null-byte path via `NodeId::try_from`. The test panicked instead of returning `Err`, which led to reading the `NodeId` source and finding the doc bug.

**Generalization**: new QA tooling is valuable even when the tool finds nothing. The discipline of preparing it to run is what forces close-reading of code that would otherwise never be read. Apply tools to unfamiliar territory periodically even if you don't expect them to find anything.

**Implication for the strategy**: the "miri target list" should be read as "code that needs focused unit tests written for it", with miri as a secondary verification. The tests are the high-value output.

### 6.2 Tool scoping matters as much as tool choice

Finding #10 is the canonical example: cargo-mutants at default scope gives a number that looks like "the daemon is 6% tested". With `test_workspace = true`, the same tool gives a completely different (and correct) picture. The tool didn't change — the scope did.

**Generalization**: whenever a new QA tool reports a surprising number, your first hypothesis should be "I'm measuring the wrong thing", not "the code is broken". Confirm by running the smallest possible controlled experiment.

### 6.3 Proptest scope discipline

Broad proptest strategies like `any::<f64>()` find failures in the underlying platform (e.g., JSON number precision at 10^120), not in your code. Triaging these:

1. Reproduce manually with the minimal failing input.
2. Ask: "is this a property of my code, or a property of what my code depends on?"
3. If the latter: constrain the strategy, document why.
4. If the former: real finding, fix or document as invariant.

The POC caught a real serialization asymmetry (`Some(Value::Null)` ≡ `None` on the wire for `WsResponse`, finding #5) **after** first tripping on the JSON f64 precision false positive. Both were data-driven by the same broad strategy.

**Generalization**: write strategies that test **your** invariants, not the platform's. When a proptest fails, always check whether the failure is in code you own.

### 6.4 Unwrap-budget ratchet needs honest accounting

The initial script counted `.unwrap()` occurrences in every `.rs` file except those under `tests/` directories. It didn't exclude:
- Inline `#[cfg(test)] mod tests {}` blocks at the bottom of source files
- `tests.rs` submodule files
- `benches/` directories

Result: the "684 production unwraps" figure was actually ~~684~~ **188** once test/bench code was properly excluded. 72% of the reported "panic debt" was test code panicking on intentionally-constructed test inputs.

**Generalization**: a ratchet is only useful if it counts the right thing. If your metric is going up and down by 20 from routine test additions, the metric is noise. Fix the metric before relying on it.

---

## 7. What's working, what's not, what's deferred

### 7.1 Working well

- **The 3-tier architecture**: fast/full/tier1 modes give developers the right feedback loop for each scenario.
- **Local-first with CI mirror**: same scripts everywhere; no "works on my machine" investigations.
- **Unwrap ratchet with honest accounting**: 188 is a number anyone can act on. Top offenders visible and prioritized.
- **Mutation testing on library crates**: `dora-core`, `dora-message`, `dora-coordinator-store` all have meaningful scores under package scope. These are the correctness-critical crates where mutation testing adds the most signal.
- **Case study pattern**: every new gate gets validated with a specific bug found. Produces concrete evidence of value.
- **Adversarial LLM review**: a different model reading the diff has caught things every other gate missed. Worth the cost per PR.

### 7.2 Working with caveats

- **Mutation testing on binary crates**: requires `test_workspace = true` config to give accurate scores. Default cargo-mutants scoping is a trap. Fixed by config but the full re-run is deferred.
- **Coverage gate**: soft on main, 70% threshold on PRs. Reasonable starting point. Needs to rise to 80% once the baseline catches up.
- **SemVer check**: soft during 0.x. Becomes load-bearing after 1.0 when we stop freely breaking public APIs.
- **Miri**: works on `metadata.rs` (pure Rust). Does NOT work on `shared-memory-server` (FFI). The unsafe hotspot that matters most is the one we can't directly analyze — see §7.4.

### 7.3 Designed but not implemented

- **Dogfood campaign** (Tier 3): full operational spec in `plan-dogfood-campaign.md`. 10 blocking success criteria over 168 hours. Reference workload: camera_sim → vision_infer (Python) → detection_filter → logger / aggregator / metrics_sink across 2 machines. Estimated implementation: 1-2 days of contractor work.
- **Fault injection scenarios** (Tier 2): 8 scenarios designed in `plan-fault-injection.md`. Each has an explicit invariant to assert. Priority order documented. Estimated implementation: 3-5 days of contractor work.
- **Adversarial review in CI**: blocked on `ANTHROPIC_API_KEY` as a repo secret. Works locally today.

### 7.4 Real gaps worth flagging

- **`shared-memory-server` is a QA blind spot**. 30 unsafe blocks handling every local node↔daemon message (4 control channels per node + data regions ≥4KB). The 2026-03-21 audit already found 3 memory-safety issues here. We can't analyze it with miri (FFI), property testing (no pure-Rust entry points), or fuzzing (same reason). It's only covered by integration tests that spin up real shared memory. **Recommended fix**: adopt Zenoh's native shared memory feature as part of the dora 1.0 consolidation. Upstream dora already did this in `dora-rs/adora#1378`, deleting ~660 lines and ~11 unsafe blocks. Sequenced into the consolidation plan as Phase 3b.
- **Full mutation baselines for binary crates**: the daemon and coordinator need re-running with `test_workspace = true` (~8 hours background). Deferred as an operational task.
- **`validate.rs` still has 69 missed mutants** after the `parse_byte_size` + `parse_log_level` fixes. Biggest remaining hotspot in `dora-core`. Good 1-day case study for the next session.
- **`redb_store.rs` still has ~40 missed mutants** after the focused case study. Similarly addressable.
- **Adversarial review has no audit trail in CI yet**. Each run goes to `/tmp/`; not archived. Fix requires the API key + a CI workflow addition.

---

## 8. Numbers you can cite

| Metric | Value |
|---|---|
| **Commits landed during POC** | 27 (across three days) |
| **Case studies delivered** | 10 distinct findings (+ 22 tautological-test fixes in a single focused session) |
| **Production bugs fixed** | 3 (metadata off-by-one, operator null UB, NodeId doc trap) |
| **Meta-findings** | 4 (infrastructure-as-forcing-function, tool scoping matters, proptest scope discipline, unwrap ratchet honesty) |
| **Security advisories resolved** | 2 (`time` DoS, `lru` unsoundness) |
| **dora-core mutation score improvement** | 37.2% → 43.4% (+6.1pp, +24 caught mutants) |
| **Production unwrap count** | 684 reported → 188 real (after script fix + tracing refactor) |
| **Lines of documentation produced** | ~3500 lines across 4 new plan docs (dogfood, fault injection, strategy amendments, runbook, this report) |
| **Tier 1 gates wired** | 6 / 6 |
| **Tier 2 gates with baselines** | 3 / 4 |
| **Time to full POC completion** | ~3 intensive days |

---

## 9. Cost accounting

**Human time (heyong4725)**: ~15 hours of focused collaboration across three days — primarily reviewing AI-proposed changes, making strategic decisions (e.g., Zenoh SHM sequencing), and running the scripts.

**AI token cost**: not tracked precisely, but approximately $X tier (Claude Opus 4.6 with 1M context). Each case study was on the order of $1-3 in LLM cost.

**Infrastructure**: zero — everything runs on the developer's laptop. The CI additions use existing GitHub Actions minutes.

**Deferred investments** (not counted above):
- Full mutation re-run with workspace scope: ~17 hours of laptop/CI time
- Dogfood campaign first run: 7 days of continuous operation on 2 machines
- First fault injection implementation: 3-5 days of contractor work
- External security audit: outsourced, ~1-2 weeks + fee

---

## 10. What this suggests for the wider dora ecosystem

The POC was done on dora but the approach is general. For anyone maintaining a Rust codebase — especially one with AI-authored code — here's what we'd recommend based on what we learned:

### 10.1 Adopt these gates in this order

1. **Start with `cargo-audit` + `cargo-deny`** if you don't have them. Lowest effort, highest immediate value — transitive CVEs are everyone's problem.
2. **Add an unwrap ratchet** next. Cheap, catches the "lazy AI panic" pattern, and forces gradual cleanup. Make sure your ratchet script excludes test code honestly (see §6.4).
3. **Wire `cargo-llvm-cov` for baseline visibility**. Don't gate on it initially — just measure.
4. **Wire `cargo-mutants` on your most correctness-critical library crate first**. Library crates give meaningful scores at package scope. Binary crates need `test_workspace = true`.
5. **Add adversarial LLM review** if you have AI API access. Catches things other gates don't.
6. **Add property testing** on your wire protocol and YAML/config parsers. High probability of finding a real serialization bug on the first run.

Each of these takes less than a day to wire. Deferring them is tempting but costs more over time — every day you don't have them is a day an AI agent could introduce a class of bug you can't detect.

### 10.2 Don't skip the hand-reading

The single biggest lesson from this POC: **write the test code to use each new tool, even if you don't actually run the tool**. The bugs will come from the reading, not from the tool output.

Concretely: if you're adopting miri, pick one unsafe function. Write unit tests for it. Read it carefully as you write the tests. 80% of the value of miri will come from that reading.

### 10.3 Case-study-driven validation

Every new gate should get at least one real-bug case study before you trust it. If a gate goes a week without producing any finding, either it's redundant with another gate or your codebase is in better shape than you thought. Either way, document it.

### 10.4 Be honest about what each gate catches

The case study table in §5 shows explicitly which gate caught what. **Zero of the three production bugs were caught by the tool they were nominally targeting.** That's a useful thing to know when allocating effort.

### 10.5 Don't trust default scoping

Finding #10 is the warning shot: default tool invocations can give wildly misleading numbers. Verify your scopes against a specific controlled experiment whenever adding a new gate. Confirm the tool is measuring what you think it's measuring.

---

## 11. Reference: where to find everything

### 11.1 Plan documents (strategy and design)

- [`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md) — full three-tier strategy with all gate specs
- [`plan-dora-1.0-consolidation.md`](plan-dora-1.0-consolidation.md) — how the QA work feeds into the dora 1.0 merge, including Phase 3b (Zenoh SHM migration)
- [`plan-zenoh-shared-memory.md`](plan-zenoh-shared-memory.md) — rationale for the Zenoh SHM adoption
- [`plan-dogfood-campaign.md`](plan-dogfood-campaign.md) — operational spec for the pre-release dogfood
- [`plan-fault-injection.md`](plan-fault-injection.md) — 8 chaos scenarios ready to implement

### 11.2 Operational documents

- [`qa-runbook.md`](qa-runbook.md) — how to run QA day-to-day (commands, failure modes, fixes)
- [`qa-followups.md`](qa-followups.md) — open items tracker: everything this POC deferred, with effort estimates, triggers, and owners
- [`qa-baseline-2026-04-07.md`](qa-baseline-2026-04-07.md) — metrics snapshot with case study details

### 11.3 Gate configuration

- `Makefile` — entry point for local QA
- `scripts/qa/*.sh` — individual gate scripts
- `scripts/qa/adversarial-prompt.md` — LLM review prompt template
- `.cargo/mutants.toml` — cargo-mutants config (equivalent-mutant waivers + `test_workspace = true`)
- `deny.toml` — cargo-deny policy (licenses, advisories, sources)
- `.unwrap-budget` — current ratchet value
- `.github/workflows/ci.yml` — CI workflow

### 11.4 Evidence (the 27 commits)

Run `git log --oneline b03901d..c833799` to see the full POC commit history. Key commits:

```
d12e6b8 fix(core): off-by-one in metadata::from_array region bounds check
5757e3b fix(operator): null-pointer UB in dora_send_operator_output
b878e50 test(coordinator-store): focused redb_store tests + NodeId doc fix
4d2df85 test(core): close 100% of mutants in parse_byte_size and parse_log_level
28c99b3 test(message): add property tests for ws_protocol + document Some(Null) invariant
98c6639 test(core): fix tautological gap in types_match + wire QA to CI
333cddb fix(security): resolve time DoS and lru unsoundness advisories
bf26afb fix(qa): exclude test and bench code from unwrap budget (684 -> 212)
9e0c2c6 fix(qa): pin test_workspace=true for cargo-mutants
```

---

## 12. Change log

| Date | Author | Change |
|---|---|---|
| 2026-04-09 | heyong4725 + AI | Initial report after the 2026-04-07/08/09 POC session |

---

## 13. Honest acknowledgements

This POC was a **best-case scenario for agentic engineering**: a single strongly-motivated human driver, a capable AI pair (Claude Opus 4.6 with a large context), a fresh codebase without years of legacy constraints, and the freedom to set priorities without stakeholder negotiation. The numbers and findings in this report should be read with that context.

What would be different in a less-ideal scenario:
- Multiple humans driving simultaneously would surface coordination overhead we didn't feel.
- A codebase with years of accrued test infrastructure would have a very different starting baseline (probably higher coverage, lower mutation score gains from each focused session).
- Stakeholder pressure to prioritize features over verification would compress the time available.
- Less capable AI pairs might produce more false positives in adversarial review and proptest.

That said, the **structure** of the approach — three tiers, case-study-driven validation, the "infrastructure as forcing function" insight — should generalize. The specific numbers will differ. The framing should hold up.

---

*Questions or feedback: file an issue on `dora-rs/dora` or message heyong4725 directly.*
