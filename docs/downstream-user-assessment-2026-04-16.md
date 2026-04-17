# Downstream User Assessment — Quick Pass

**Date:** 2026-04-16
**Author:** heyong4725 + AI
**Scope:** Evidence for plan §19.7 "Downstream user list + outreach" gate. Closes issue #291.

---

## 1. Conclusion first

No dora deployment surfaced on GitHub meets a reasonable definition of "production". The largest downstream repo is 69 stars; the median is ~0–2. The original plan specified a top-10 courtesy-outreach campaign (per §14 Appendix C). **That scope is disproportionate to the actual user base.** This assessment right-sizes the gate to:

- Skip the individual outreach campaign.
- Ship a release-note + migration-guide section that is prominent enough for any surprise production user to find without direct contact.
- Monitor the issue tracker for the first 30 days post-release; pivot to targeted outreach if a production user surfaces.

---

## 2. Method

Three GitHub code-search queries run on 2026-04-16:

```bash
gh search code 'dora-node-api'      --language rust   --limit 30
gh search code 'from dora import Node' --language python --limit 30
gh search code '"dora-rs"'          --extension toml  --limit 30
gh search code 'dora-rs-cli'        --limit 20
```

Results de-duplicated; dora-rs org repos, tutorial/example/crawler/index repos, and obvious name collisions with unrelated `dora-*` projects filtered out.

---

## 3. Findings

### 3.1 Size of the external user base

- **~30 unique external repos** reference dora in any form.
- **Top 5 by star count:**

    | Repo | Stars | Last push | What it is |
    |---|---:|---|---|
    | YOR-robot/YOR | 69 | 2026-04-14 | "Your Own Robot" — hobby/educational robot kit with MuJoCo sim |
    | Ekumen-OS/lekiwi | 35 | 2026-02-12 | Monorepo for LeKiwi community robot (Ekumen Labs, a robotics consulting shop) |
    | FlagOpen/RoboDriver | 32 | 2026-04-15 | BAAI's standardized robot driver layer — R&D |
    | kornia/bubbaloop | 16 | 2026-04-15 | Kornia's "hardware AI agent" single-binary Rust runtime |
    | mofa-org/mofa-studio | 12 | 2026-03-29 | Desktop voice-chat app (Rust/Makepad) that happens to use dora |

- Everything else is **0–10 stars** and most are tutorial / hackathon / test / pkg-manager experiment repos (`*dora-pkg-mngr*`, `*dora-camp-tutorial*`, `*dora-hackathon*`, `*_test`, `*_example`).

### 3.2 Classification

| Class | Count | Signal |
|---|---:|---|
| Production deployment (revenue-generating, customer-facing) | **0** identified | No repo describes a paying customer or uptime SLA |
| Research / R&D project (academic, lab, OSS org) | ~6 | Ekumen-OS/*, FlagOpen/*, kornia/bubbaloop, YOR-robot/YOR, mofa-org/* |
| Hobby / POC / learning | ~20 | Hackathon names, tutorials, personal experiments |
| Tooling / meta | ~4 | pkg-crawlers, CI indexes, unrelated name collisions |

### 3.3 Red-flag check (would any of these break badly?)

For each of the top 5:

- **YOR-robot/YOR**: README names dora only as a distributed-computing backend. A hard break at 1.0 means a one-line dependency bump + CLI rename awareness. Two-hour migration.
- **Ekumen-OS/lekiwi**: Apache-2.0 licensed, CI green, described as a "research monorepo". Ekumen Labs is a consulting shop; they would absorb a migration as part of a regular tool upgrade. No production deployment visible.
- **FlagOpen/RoboDriver**: 32 stars, bilingual README, BAAI-sponsored. Research project, not in production deployment. Would likely welcome the 1.0 cut as a clear versioning signal.
- **kornia/bubbaloop**: early-stage, 16 stars, single-binary design. Probably tracking our main already; migration is part of its growth path.
- **mofa-org/mofa-studio**: voice-chat desktop app, tangentially depends on dora. Dora is one of many integrations.

None of the five would be materially harmed by a hard 1.0 break with a clear migration guide.

---

## 4. Right-sized action plan

### 4.1 What we will do

- **Release note prominence.** The 1.0 release note mentions the hard break explicitly and links the migration guide. This is the primary communication channel for any surprise production user.
- **Migration guide.** `docs/migration-from-0.x.md` (tracked as plan §3.5 and #295 rename residue). Keep it linkable, keep it short, keep it up top in the release announcement.
- **Issue-tracker watch for 30 days post-release.** Any new issue tagged `migration` or naming a production deployment triggers a direct-outreach response within 24 hours.
- **One-line Discord / community ping.** A "dora 1.0 is coming; here's what changes" note in whatever chat channel the project uses. Low-effort; catches users who don't watch GitHub releases.

### 4.2 What we will not do

- ~~File courtesy GitHub issues on the top-10 downstream projects asking for early migration testing.~~ The top-10 is ~5 distinct real users and ~5 abandoned tutorials; the signal-to-noise is not worth the effort.
- ~~Maintain an email list of production users.~~ None exist to list.
- ~~Ship a bridge release (0.6) to smooth rolling upgrades.~~ Already rejected in D-1a (see `phase--1-audit-2026-04-16.md` §6); reaffirmed here — even for the top-5 users, a hard break is cheaper than engineering a bridge.

### 4.3 Trigger-to-expand

If any of the following surfaces post-release, expand to individual outreach:

- A GitHub issue from a maintainer of a repo not in our search (production users may not be OSS-visible).
- A Reddit / HN / Discord comment naming a deployment that the release note didn't reach.
- A maintainer-sponsored fork that sticks to 0.x past 90 days, indicating migration friction.

---

## 5. Gate update

- Plan §19.7 "Downstream user list + outreach" row flips from **Not done** → **Done (scope right-sized)** with a pointer to this file.
- Plan §14 Appendix C (the top-10 outreach protocol) is annotated as historical — superseded by this right-sized assessment.
- Plan §11 Success Criterion "At least 3 downstream projects have confirmed successful migration to 1.0 via `dora migrate`" is removed; replaced by "No production user report unresolved migration blockers in the first 30 days post-release."
- Issue #291 can be closed.

### Honest caveat

This assessment is based on **what's visible on GitHub**. A production user running dora in a private monorepo would not surface. If the user knows of such a deployment not searchable by public GitHub code search, that user should be added back to the direct-outreach list manually. The right-sizing rests on the current best-effort search showing zero such deployments.
