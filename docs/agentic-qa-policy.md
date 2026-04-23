# Agentic QA Policy

The validation bar a change must clear before merging, **keyed to what
the change does**, not who (or what) authored it.

This document is the authoritative policy surface for
[#1634](https://github.com/dora-rs/dora/issues/1634). Operational
details — how to run each gate, how to read the output — live in
[`qa-runbook.md`](qa-runbook.md). Strategy and rationale live in
[`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md).

---

## Change classifications

Every PR falls into exactly one of these three classes. When a PR
spans multiple classes (e.g. a high-risk fix bundled with an
unrelated cosmetic change), split it or apply the **stricter**
class's requirements to the whole PR.

### Class A — Low-risk

Cosmetic or mechanical changes that can't alter observable behavior:

- Formatting, typo fixes, rewording in docs/comments.
- Pure renames (with grep-replace across the repo) when the symbol is
  already equivalent — no signature or semantics change.
- Test-only additions (new test fixture, new assertion).
- Dependency bumps that don't change semver-compatible behavior.
- CI config changes that don't change what's tested (e.g. runner image
  update, timeout bump).

### Class B — Behavior change

Anything user-visible or downstream-visible that isn't in Class C:

- Bug fixes that change an output, exit code, log line, or timing.
- New features or new CLI flags.
- Refactors that touch a public API but are meant to be
  behavior-preserving (they often aren't).
- Performance work that changes how code paths are exercised.
- Dependency bumps that change public API surface (even if
  semver-major on our side is not required).

### Class C — High-risk subsystem

Changes touching code paths where a silent regression has outsized
impact. Current list (update when a new subsystem earns it):

- `binaries/daemon/**` — lifecycle, routing, spawn loop, node kill/restart.
- `binaries/coordinator/**` — state machine, dataflow registry, recovery.
- `libraries/core/src/descriptor/**` — dataflow parsing and validation.
- `libraries/message/**` — wire protocol / enum additions / variant renames.
- `libraries/shared-memory-server/**` — transport + zero-copy path.
- Fault-tolerance paths: `restart_*`, `health_check_*`, `input_timeout`,
  `NodeRestarted` / `InputClosed` / `InputRecovered` delivery.
- Record/replay format or session semantics.
- Public Rust API of `apis/rust/node` and `apis/rust/operator`.
- Python API exports in `apis/python/node`.

---

## Required QA per class

### Class A — Low-risk

- [ ] `make qa-fast` (fmt + clippy + audit + unwrap-budget + typos).

That's the whole bar. Don't over-test cosmetic changes — reviewers can
eyeball them and CI backs it up. `typos` is now part of `qa-fast`
itself (#1657) so docs-heavy PRs don't need a separate manual step.

### Class B — Behavior change

- [ ] `make qa-fast`.
- [ ] Targeted tests for the touched crate(s): at minimum
      `cargo test -p <touched-crate>`.
- [ ] A test that demonstrates the new behavior (new feature) **or** a
      regression test for the fixed bug. If you can't write one,
      explain why in the PR description.
- [ ] If the touched feature is covered by a `smoke_*` or
      `contract_*` test in `tests/example-smoke.rs`, run the relevant
      one locally and paste the result in the PR description.
- [ ] PR validation summary (template below).

### Class C — High-risk subsystem

All of Class B, plus:

- [ ] Full `cargo test --all` (workspace-wide; Python crates excluded
      per the commands in [`CLAUDE.md`](../CLAUDE.md)).
- [ ] Fault-tolerance E2E:
      `cargo test --test fault-tolerance-e2e -- --test-threads=1`.
- [ ] Contract tests:
      `cargo test -p dora-examples --test example-smoke contract_ -- --test-threads=1`.
- [ ] Decide, and state explicitly in the PR:
  - New semantic contract test for the touched behavior (preferred), **or**
  - Thresholded diff coverage on the touched lines
    (`pip install diff-cover` then
    `DIFF_COVERAGE_FAIL_UNDER=80 make qa-coverage`; `make qa-coverage`
    alone produces a report but does not gate), **or**
  - Focused mutation-testing run on touched files
    (`make qa-mutants` scoped), **or**
  - Explicit manual-validation notes when automation is genuinely
    infeasible.

**Adversarial review** (`make qa-adversarial`) is expected for
Class C when the diff is within size limits ([see
`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md) for the
caveats). Not a hard gate — a reasoned skip in the PR description is
fine.

---

## PR validation summary template

Class B and C PRs should include a block like this in the description.
Agents authoring PRs: render this section by default so reviewers can
see the evidence without asking.

```markdown
## Validation

**Class**: B  <!-- or C -->

- `make qa-fast`: ✅
- `cargo test -p <crate>`: ✅ (N passed)
- Targeted reproduction / feature test: <test name, output excerpt>
- Contract / fault-tolerance tests (Class C): <names, passes>
- Adversarial review (Class C, when applicable): <skipped with reason | ran, notes>
```

Class A PRs don't need a formal block — the diff speaks for itself.

---

## Enforcement

- The policy is **self-enforced at PR-author level**. There's no bot
  that checks the checklist.
- Reviewers are expected to **block on missing Class C evidence** and
  to **lean-in on Class B evidence**. Class A can merge on a green
  `qa-fast` alone.
- `enforce_admins: false` on `main` lets maintainers admin-merge when
  CI is green but branch-protection review is procedurally blocked
  (e.g. self-review during the release-freeze window). Admin-merge
  does not waive the policy above — the validation evidence should
  still be in the PR description.

## Mapping to existing tools

| Policy item | Command | Runbook reference |
|---|---|---|
| Baseline fast gate | `make qa-fast` | [`qa-runbook.md` §TL;DR](qa-runbook.md#1-tldr) |
| Workspace tests | `cargo test --all --exclude <python-crates>` | [`CLAUDE.md`](../CLAUDE.md) |
| Fault-tolerance E2E | `cargo test --test fault-tolerance-e2e` | [`testing-capabilities.md`](testing-capabilities.md#fault-tolerance) |
| Semantic contracts | `cargo test -p dora-examples --test example-smoke contract_` | [`testing-capabilities.md`](testing-capabilities.md) |
| Coverage report (lcov) | `make qa-coverage` | [`qa-runbook.md`](qa-runbook.md) |
| Thresholded diff coverage (optional gate) | `pip install diff-cover` + `DIFF_COVERAGE_FAIL_UNDER=80 make qa-coverage` | `scripts/qa/coverage.sh` header |
| Mutation testing | `make qa-mutants` | [`qa-runbook.md`](qa-runbook.md) |
| Adversarial review | `make qa-adversarial` | [`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md) |

## When the classification is ambiguous

- If the PR touches any file matching the Class C list and cannot be
  characterized as strictly Class A (comments-only, for example), it
  is Class C.
- If unsure whether a Class B change is "behavior" or "refactor", ask
  whether a downstream user could write a test today that the PR
  would break. If yes → Class B. If no → Class A.
- "Fix a typo in an error message string" is Class A *unless* users
  grep for that string (CLI error-message golden files, automation
  scripts). Default to Class B if in doubt.

## Why this policy exists

Agentic coding workflows land more code per hour than a human
reviewer can fully load into their head. The traditional "trust the
author" gate degrades when the author is an LLM. This policy
re-introduces validation as an explicit, per-class expectation so:

- Low-risk changes stay fast and don't drown in ceremony.
- Behavior changes always carry a concrete demonstration of the
  behavior.
- High-risk subsystem changes always carry evidence beyond
  "compiles + unit tests pass".

See [`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md) for
the longer-form rationale and the open experiments (coverage gates,
mutants scoping, adversarial-review CI integration).
