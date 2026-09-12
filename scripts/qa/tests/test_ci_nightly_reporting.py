#!/usr/bin/env python3
"""Tests for the nightly reporting-wiring guard.

`scripts/qa/ci-nightly-reporting.sh` is a structural check, so the only thing
that distinguishes a working one from a no-op is that a *broken* workflow makes
it red. Two review rounds found spellings that slipped past it silently --
`continue-on-error: True`, a trailing comment, a space before the colon -- and
each fix so far has been argued from a mutation table written by hand in a
commit message. The table lives here instead.

Every case mutates a minimal workflow fixture and asserts the exit code:

    0  the wiring is sound
    1  an invariant is violated
    2  the workflow could not be parsed -- never a silent pass

Run: python3 scripts/qa/tests/test_ci_nightly_reporting.py
     python3 -m unittest discover -s scripts/qa/tests
"""

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
GUARD = ROOT / "scripts/qa/ci-nightly-reporting.sh"

# Two ordinary jobs, both watched by both reporters. `beta` carries a
# *step*-level `continue-on-error`, which is legitimate and must never be
# flagged -- `smoke-suite` uses exactly this shape in the real workflow.
FIXTURE = """\
name: Nightly

on:
  schedule:
    - cron: "0 3 * * *"

env:
  RUST_VERSION: "1.97.1"

jobs:
  alpha:
    name: Alpha
    runs-on: ubuntu-latest
    steps:
      - run: echo alpha

  beta:
    name: Beta
    runs-on: ubuntu-latest
    steps:
      - name: capture the exit code rather than failing the job
        continue-on-error: true
        run: echo beta

  file-issue-on-failure:
    if: always()
    needs:
      - alpha
      - beta
    runs-on: ubuntu-latest
    steps:
      - run: echo report

  close-issue-on-success:
    if: always()
    needs:
      - alpha
      - beta
    runs-on: ubuntu-latest
    steps:
      - run: echo close
"""

ADVISORY = """\
# One job id per line. Blank lines and comments ignored.
"""


class GuardCase(unittest.TestCase):
    def run_guard(self, workflow=FIXTURE, advisory=ADVISORY, write_workflow=True):
        """Run the real guard against a throwaway tree, return (code, output)."""
        with tempfile.TemporaryDirectory() as tmp:
            tree = Path(tmp)
            (tree / "scripts/qa").mkdir(parents=True)
            (tree / ".github/workflows").mkdir(parents=True)
            shutil.copy2(GUARD, tree / "scripts/qa" / GUARD.name)
            if write_workflow:
                (tree / ".github/workflows/nightly.yml").write_text(workflow)
            (tree / ".nightly-advisory-jobs").write_text(advisory)
            proc = subprocess.run(
                ["bash", str(tree / "scripts/qa" / GUARD.name)],
                capture_output=True,
                text=True,
            )
        return proc.returncode, proc.stdout + proc.stderr

    def assertGuard(self, expected, workflow=FIXTURE, advisory=ADVISORY, **kw):
        code, out = self.run_guard(workflow, advisory, **kw)
        self.assertEqual(expected, code, f"expected exit {expected}, got {code}:\n{out}")
        return out

    @staticmethod
    def with_job_key(line, job="  alpha:\n"):
        """Insert a job-level key directly under `job`'s header."""
        assert FIXTURE.count(job) == 1
        return FIXTURE.replace(job, job + line)

    @staticmethod
    def with_extra_job(name, keys="", watched=False):
        """Add a third ordinary job, optionally into both `needs` lists."""
        anchor = "  file-issue-on-failure:\n"
        assert FIXTURE.count(anchor) == 1
        block = (f"  {name}:\n{keys}    runs-on: ubuntu-latest\n"
                 f"    steps:\n      - run: echo {name}\n\n")
        wf = FIXTURE.replace(anchor, block + anchor)
        if watched:
            assert wf.count("      - beta\n") == 2
            wf = wf.replace("      - beta\n", f"      - beta\n      - {name}\n")
        return wf


class TestSoundWiring(GuardCase):
    def test_baseline_is_green(self):
        out = self.assertGuard(0)
        self.assertIn("4 jobs", out)

    def test_step_level_continue_on_error_is_not_flagged(self):
        # `beta`'s step-level flag is in the baseline; assert the reason
        # explicitly so a parser that stopped honouring indentation fails here.
        self.assertIn("        continue-on-error: true", FIXTURE)
        self.assertGuard(0)

    def test_inline_needs_list(self):
        wf = FIXTURE.replace("    needs:\n      - alpha\n      - beta\n",
                             "    needs: [alpha, beta]\n")
        self.assertEqual(wf.count("needs: [alpha, beta]"), 2)
        self.assertGuard(0, wf)

    def test_scalar_needs_with_one_job(self):
        self.assertGuard(0, self.with_extra_job("gamma", "    needs: alpha\n",
                                                watched=True))

    def test_job_header_with_trailing_comment(self):
        self.assertGuard(0, FIXTURE.replace("  alpha:\n", "  alpha: # the first one\n"))

    def test_job_header_with_space_before_the_colon(self):
        self.assertGuard(0, FIXTURE.replace("  alpha:\n", "  alpha :\n"))

    def test_needs_with_space_before_the_colon(self):
        self.assertGuard(0, FIXTURE.replace("    needs:\n", "    needs :\n"))


class TestInvariant1AdvisoryJobs(GuardCase):
    """Job-level `continue-on-error` on a job the reporter claims to watch."""

    SPELLINGS = [
        "    continue-on-error: true\n",
        "    continue-on-error: true  # temporary, while it settles\n",
        "    continue-on-error: True\n",
        "    continue-on-error: TRUE\n",
        '    continue-on-error: "true"\n',
        "    continue-on-error: 'true'\n",
        "    continue-on-error : true\n",
        "    continue-on-error:true\n",
        "    continue-on-error: ${{ github.event_name == 'schedule' }}\n",
        "    continue-on-error:\n",
    ]

    def test_every_truthy_spelling_is_flagged(self):
        for line in self.SPELLINGS:
            with self.subTest(line=line.strip()):
                out = self.assertGuard(1, self.with_job_key(line))
                self.assertIn("- alpha (continue-on-error:", out)

    def test_the_value_is_reported_as_written(self):
        out = self.assertGuard(1, self.with_job_key("    continue-on-error: True\n"))
        self.assertIn("- alpha (continue-on-error: True)", out)

    def test_literal_false_is_the_one_value_that_passes(self):
        for line in ("    continue-on-error: false\n",
                     "    continue-on-error: False\n",
                     '    continue-on-error: "false"\n',
                     "    continue-on-error: false  # the default, spelled out\n"):
            with self.subTest(line=line.strip()):
                self.assertGuard(0, self.with_job_key(line))

    def test_an_allowlisted_job_is_exempt(self):
        wf = self.with_job_key("    continue-on-error: true\n")
        self.assertGuard(1, wf)
        out = self.assertGuard(0, wf, ADVISORY + "alpha  # flaky, tracked in #0000\n")
        self.assertIn("1 documented advisory", out)

    def test_allowlist_comments_and_blanks_are_ignored(self):
        wf = self.with_job_key("    continue-on-error: true\n")
        self.assertGuard(1, wf, ADVISORY + "\n# alpha\n   \n")


class TestInvariant2EveryJobIsWatched(GuardCase):
    def test_job_missing_from_the_reporter_is_flagged(self):
        wf = FIXTURE.replace("    needs:\n      - alpha\n      - beta\n",
                             "    needs:\n      - beta\n", 1)
        out = self.assertGuard(1, wf)
        self.assertIn("missing from 'file-issue-on-failure' needs", out)
        self.assertIn("- alpha", out)

    def test_a_new_job_nobody_watches_is_flagged(self):
        out = self.assertGuard(1, self.with_extra_job("gamma"))
        self.assertIn("- gamma", out)


class TestInvariant3TheCloserAgrees(GuardCase):
    def test_job_watched_by_the_reporter_but_not_the_closer(self):
        head, sep, tail = FIXTURE.rpartition("    needs:\n      - alpha\n      - beta\n")
        wf = head + "    needs:\n      - alpha\n" + tail
        out = self.assertGuard(1, wf)
        self.assertIn("not by 'close-issue-on-success'", out)
        self.assertIn("- beta", out)


class TestUnparseableIsFatal(GuardCase):
    """Exit 2, never 0: an unreadable workflow must not read as "all sound"."""

    def test_missing_workflow(self):
        out = self.assertGuard(2, write_workflow=False)
        self.assertIn("not found", out)

    def test_quoted_job_name(self):
        out = self.assertGuard(2, FIXTURE.replace("  alpha:\n", '  "alpha":\n'))
        self.assertIn("unreadable job header", out)
        self.assertIn('"alpha":', out)

    def test_a_job_name_the_parser_cannot_read_is_not_silently_reattributed(self):
        # The danger is not just the missing job: `needs` and
        # `continue-on-error` under an unreadable header would be read as the
        # *previous* job's, so an omission and a misattribution both pass.
        wf = FIXTURE.replace("  beta:\n", '  "beta":\n')
        self.assertGuard(2, wf)

    def test_renamed_reporter(self):
        out = self.assertGuard(2, FIXTURE.replace("  file-issue-on-failure:\n",
                                                  "  open-issue-on-failure:\n"))
        self.assertIn("no 'file-issue-on-failure' job", out)

    def test_renamed_closer(self):
        out = self.assertGuard(2, FIXTURE.replace("  close-issue-on-success:\n",
                                                  "  resolve-issue-on-success:\n"))
        self.assertIn("no 'close-issue-on-success' job", out)

    def test_reporter_with_an_empty_needs_list(self):
        wf = FIXTURE.replace("    needs:\n      - alpha\n      - beta\n",
                             "    needs: []\n", 1)
        out = self.assertGuard(2, wf)
        self.assertIn("empty 'needs:' list", out)


class TestAgainstTheRealWorkflow(GuardCase):
    def test_the_checked_in_nightly_passes(self):
        proc = subprocess.run(["bash", str(GUARD)], capture_output=True, text=True)
        self.assertEqual(0, proc.returncode, proc.stdout + proc.stderr)


if __name__ == "__main__":
    unittest.main()
