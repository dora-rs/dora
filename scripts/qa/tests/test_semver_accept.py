#!/usr/bin/env python3
"""Tests for the accepted-failure filter of the cargo-semver-checks gate.

The filter decides whether a failing run still passes, so the cases that
matter are the ones where it must say no.

Run: python3 -m unittest discover -s scripts/qa/tests
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from semver_accept import (  # noqa: E402
    ACCEPTED_SEMVER,
    parse_failures,
    split_accepted,
)

ROOT = "/work/dora"

# Output of cargo-semver-checks 0.51.0, trimmed to one crate.
OUTPUT = """\
     Checked [   0.021s] 202 checks: 200 pass, 2 fail, 0 warn, 58 skip

--- failure struct_marked_non_exhaustive: struct marked #[non_exhaustive] ---

Description:
A public struct has been marked #[non_exhaustive], which will prevent it from being constructed using a struct literal outside of its crate.
        ref: https://doc.rust-lang.org/cargo/reference/semver.html#attr-adding-non-exhaustive
       impl: https://github.com/obi1kenobi/cargo-semver-checks/tree/v0.51.0/src/lints/struct_marked_non_exhaustive.ron

Failed in:
  struct Ros2TopicConfig in /work/dora/libraries/message/src/descriptor.rs:1982
  struct Ros2BridgeConfig in /work/dora/libraries/message/src/descriptor.rs:1850

--- failure enum_variant_added: enum variant added on exhaustive enum ---

Description:
A publicly-visible enum without #[non_exhaustive] has a new variant.

Failed in:
  variant RestartPolicy:UnlessStopped in /work/dora/libraries/message/src/descriptor.rs:300

     Summary semver requires new major version: 2 major and 0 minor checks failed
"""

DESCRIPTOR = "libraries/message/src/descriptor.rs"
TABLE = {
    "v1.1.0": [
        ("struct_marked_non_exhaustive", "struct Ros2TopicConfig", DESCRIPTOR, "r"),
        ("struct_marked_non_exhaustive", "struct Ros2BridgeConfig", DESCRIPTOR, "r"),
    ]
}


class ParseFailures(unittest.TestCase):
    def test_every_failure_is_read_with_its_lint_and_relative_file(self):
        self.assertEqual(
            parse_failures(OUTPUT, ROOT),
            [
                ("struct_marked_non_exhaustive", "struct Ros2TopicConfig", DESCRIPTOR),
                ("struct_marked_non_exhaustive", "struct Ros2BridgeConfig", DESCRIPTOR),
                ("enum_variant_added", "variant RestartPolicy:UnlessStopped", DESCRIPTOR),
            ],
        )

    def test_a_line_of_another_shape_is_kept_and_not_dropped(self):
        output = "--- failure some_lint: x ---\n\nFailed in:\n  something unusual\n"
        self.assertEqual(
            parse_failures(output, ROOT), [("some_lint", "something unusual", "")]
        )

    def test_output_without_a_failure_parses_to_nothing(self):
        self.assertEqual(parse_failures("error: could not compile `x`\n", ROOT), [])


class SplitAccepted(unittest.TestCase):
    def setUp(self):
        self.failures = parse_failures(OUTPUT, ROOT)

    def test_a_failure_not_in_the_table_is_rejected(self):
        accepted, rejected = split_accepted(self.failures, "v1.1.0", TABLE)
        self.assertEqual(len(accepted), 2)
        self.assertEqual(
            rejected,
            [("enum_variant_added", "variant RestartPolicy:UnlessStopped", DESCRIPTOR)],
        )

    def test_the_same_lint_on_another_struct_is_rejected(self):
        failures = [("struct_marked_non_exhaustive", "struct Metadata", DESCRIPTOR)]
        self.assertEqual(split_accepted(failures, "v1.1.0", TABLE), ([], failures))

    def test_the_same_struct_in_another_file_is_rejected(self):
        failures = [
            ("struct_marked_non_exhaustive", "struct Ros2TopicConfig", "apis/x.rs")
        ]
        self.assertEqual(split_accepted(failures, "v1.1.0", TABLE), ([], failures))

    def test_another_baseline_accepts_nothing(self):
        for baseline in ["v1.2.0", "0123abc"]:
            accepted, rejected = split_accepted(self.failures, baseline, TABLE)
            self.assertEqual(accepted, [])
            self.assertEqual(rejected, self.failures)

    def test_shipped_entries_carry_a_reason(self):
        for entries in ACCEPTED_SEMVER.values():
            for lint, item, path, reason in entries:
                self.assertTrue(lint and item and path and reason)


if __name__ == "__main__":
    unittest.main()
