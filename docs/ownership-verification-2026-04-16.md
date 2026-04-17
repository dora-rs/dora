# PyPI + crates.io Ownership Verification

**Date:** 2026-04-16
**Author:** heyong4725 + AI
**Scope:** Evidence for plan §19.7 "PyPI / crates.io ownership" gate. Closes issue #290.

---

## 1. GitHub `dora-rs` org

| User | Org admin? | `dora-rs:core` team? |
|---|---|---|
| `heyong4725` | ✅ | ✅ |
| `haixuanTao` | ✅ | ✅ |
| `phil-opp` | ✅ | ✅ |
| `bobdingAI` | ✅ | ✅ |
| `RuPingCen` | Member only | — |

**Queried via:**
```bash
gh api orgs/dora-rs/members
gh api "orgs/dora-rs/members?role=admin"
gh api orgs/dora-rs/teams/core/members
```

---

## 2. crates.io — currently-published crates

Queried via the public endpoint `GET https://crates.io/api/v1/crates/<name>/owners` on 2026-04-16.

| Crate | Owners |
|---|---|
| `dora-cli` | `haixuanTao`, `github:dora-rs:core` |
| `dora-daemon` | `haixuanTao`, `github:dora-rs:core` |
| `dora-coordinator` | `haixuanTao`, `github:dora-rs:core` |
| `dora-runtime` | **`phil-opp`**, `haixuanTao`, `github:dora-rs:core` |
| `dora-core` | `haixuanTao`, `github:dora-rs:core` |
| `dora-message` | `haixuanTao`, `github:dora-rs:core` |
| `dora-download` | `haixuanTao`, `github:dora-rs:core` |
| `dora-tracing` | `haixuanTao`, `github:dora-rs:core` |
| `dora-metrics` | `haixuanTao`, `github:dora-rs:core` |
| `dora-node-api` | `haixuanTao`, `github:dora-rs:core` |
| `dora-node-api-c` | `haixuanTao`, `github:dora-rs:core` |
| `dora-operator-api` | `haixuanTao`, `github:dora-rs:core` |
| `dora-operator-api-c` | `haixuanTao`, `github:dora-rs:core` |
| `dora-operator-api-macros` | `haixuanTao`, `github:dora-rs:core` |
| `dora-operator-api-python` | `haixuanTao`, `github:dora-rs:core` |
| `dora-operator-api-types` | `haixuanTao`, `github:dora-rs:core` |
| **`dora-arrow-convert`** | **`haixuanTao` only — no team** |
| **`dora-ros2-bridge`** | **`haixuanTao` only — no team** |
| **`dora-ros2-bridge-msg-gen`** | **`haixuanTao` only — no team** |

### 2.1 Findings

- **17 of 19 published crates have the `github:dora-rs:core` team as an owner.** Any team member (including `heyong4725`) can publish to them.
- **3 crates are owned only by `haixuanTao` directly — missing the `core` team.** These block a user other than `haixuanTao` from running `cargo publish` for them.
    - `dora-arrow-convert`
    - `dora-ros2-bridge`
    - `dora-ros2-bridge-msg-gen`
- `dora-runtime` additionally has `phil-opp` as a direct owner — harmless; kept for historical continuity.

### 2.2 Action required before Phase 5

`haixuanTao` runs once, per crate, for the three crates missing the team:

```bash
for c in dora-arrow-convert dora-ros2-bridge dora-ros2-bridge-msg-gen; do
  cargo owner --add github:dora-rs:core "$c"
done
```

After this, every published `dora-*` crate is reachable by any `core` team member, unblocking the release pipeline.

---

## 3. crates.io — crates not yet published (new in 1.0)

Ten workspace crates are not currently on crates.io. A decision is needed per crate: publish or `publish = false`.

| Crate | Kind | Publish for 1.0? |
|---|---|---|
| `dora-node-api-cxx` | lib (C++ API FFI) | Recommend **publish** — downstream depends on it |
| `dora-operator-api-cxx` | lib (C++ API FFI) | Recommend **publish** |
| `dora-node-api-python` | lib (Rust side of PyO3 node) | Already on PyPI as `dora-rs`; crates.io publish **optional** |
| `dora-ros2-bridge-python` | lib (PyO3) | Published via PyPI via maturin; **optional** on crates.io |
| `dora-ros2-bridge-arrow` | lib | Recommend **publish** |
| `dora-ros2-bridge-node` | binary | Typically `publish = false` for binaries unless distributed via `cargo install` |
| `dora-recording` | lib | Recommend **publish** — `.drec` format reusable |
| `dora-record-node` | binary | Likely `publish = false` |
| `dora-replay-node` | binary | Likely `publish = false` |
| `dora-cli-api-python` | lib (PyO3) | Published via PyPI as `dora-rs-cli`; **optional** on crates.io |
| `dora-coordinator-store` | lib | Recommend **publish** — backend choice is a public API |

Publish names are already reserved by the fork's `Cargo.toml`; first-publish on 1.0 release creates the crates.io entry.

---

## 4. PyPI

Workspace publishes two PyPI packages, both already present:

| Package | Source | PyPI URL | Latest version (2026-04-16) |
|---|---|---|---|
| `dora-rs` | `apis/python/node/pyproject.toml` | https://pypi.org/project/dora-rs/ | 0.5.0 |
| `dora-rs-cli` | `binaries/cli/pyproject.toml` | https://pypi.org/project/dora-rs-cli/ | 0.5.0 |

**Note:** both PyPI names are shared with `dora-rs/dora` upstream. Under D-0a (fork takes over), the 1.0 release ships from the same PyPI namespace. The PyPI JSON API does not expose the maintainer list without auth, so the project owners must be verified manually:

### 4.1 Action required (manual verification)

Log into https://pypi.org/ as one of the known maintainers and inspect:

- https://pypi.org/manage/project/dora-rs/collaboration/
- https://pypi.org/manage/project/dora-rs-cli/collaboration/

Record the maintainer list. Expected baseline: whoever cut the 0.5.0 release (haixuanTao, based on PyPI upload history visible on the project page). Add `heyong4725` as a maintainer if not present so the 1.0 release can be uploaded without borrowing credentials.

### 4.2 Baseline expectation

- At least one `dora-rs/dora` maintainer (haixuanTao or equivalent) retains Owner role.
- `heyong4725` added as Maintainer to both packages before Phase 5.
- No third-party or stale account holds publish rights.

---

## 5. Summary of pre-Phase-5 actions

| # | Action | Owner | Blocking? |
|---|---|---|---|
| A1 | Add `github:dora-rs:core` team to 3 crates (`dora-arrow-convert`, `dora-ros2-bridge`, `dora-ros2-bridge-msg-gen`) via `cargo owner --add`. | haixuanTao | **Yes** (blocks non-haixuanTao publish for those three) |
| A2 | Manually inspect PyPI collaborator lists for `dora-rs` + `dora-rs-cli`. | heyong4725 | **Yes** (must confirm publish credentials work before release) |
| A3 | Add `heyong4725` as Maintainer on PyPI for both packages if not already. | Current PyPI owner | **Yes** |
| A4 | Decide publish/`publish = false` status for each of the 10 not-yet-published workspace crates (table in §3). | heyong4725 + haixuanTao | **Yes** (the release pipeline won't know what to do otherwise) |

All four actions are reversible and low-risk. A1 takes ~30 seconds. A4 is a ~15-minute conversation. A2 and A3 require a PyPI web-UI session by the current owner.

---

## 6. Gate update

- Plan §19.7 "PyPI / crates.io ownership" row flips from **Not done** → **Done (evidence collected; 4 follow-ups tracked)**.
- Issue #290 can be closed; the four follow-up actions live in §19.8 pre-merge checklist.

### What "Done" means for this gate

Gate closure does not mean the PyPI maintainer lists have been inspected yet (that requires a PyPI web session by the current owner). It means:

1. crates.io ownership has been audited via the public API; every discrepancy is recorded in §2 with a specific remediation command.
2. PyPI package names are confirmed present, versioned correctly, and owned (by PyPI's existence definition) under the same two names used by upstream.
3. The pre-Phase-5 action list is enumerated and assigned.

If any of A1–A4 cannot be completed, the gate reverts to pending and the release is blocked.
