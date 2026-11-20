#!/usr/bin/env python3
"""Accepted `cargo-semver-checks` failures.

`cargo-semver-checks` has no per-item exemption: a lint is on or off for a
whole crate. `semver.sh` therefore pipes the tool's output through this script
when it fails, and the gate passes only if every reported failure is listed in
`ACCEPTED_SEMVER` for the baseline it was reported against.

Usage: semver_accept.py <baseline> <repo-root> < cargo-semver-checks-output
Exit 0 if every failure is accepted, 1 otherwise.
"""

import re
import sys

# Failures a maintainer has reviewed and accepted, keyed by the baseline they
# were reported against: (lint, item, file relative to the repo root, reason).
#
# Only for a surface outside the 1.x guarantee (docs/api-rust.md, "Stability
# scope at 1.0") that lives in a covered crate. A break in a covered surface
# is a 2.0 question, not an entry here.
#
# Keyed by baseline for the same reason as `ACCEPTED_FINDINGS` in
# breaking_changes.py: once the change ships in a release, that release is the
# baseline and the failure is gone, so an entry expires on its own.
_ROS2_EXEMPT = (
    "the `ros2:` descriptor key is outside the 1.x guarantee; "
    "#[non_exhaustive] lets it gain keys in a minor release"
)
ACCEPTED_SEMVER: dict[str, list[tuple[str, str, str, str]]] = {
    "v1.1.0": [
        (
            "struct_marked_non_exhaustive",
            f"struct {name}",
            "libraries/message/src/descriptor.rs",
            _ROS2_EXEMPT,
        )
        for name in ["Ros2BridgeConfig", "Ros2TopicConfig", "Ros2QosConfig"]
    ],
}

_FAILURE = re.compile(r"^--- failure (\w+):")
# CI runs cargo with colour forced on, so the output carries SGR escapes.
_ANSI = re.compile(r"\x1b\[[0-9;]*m")
_ITEM = re.compile(r"^\s+(.+?) in (?:file )?(\S+?):\d+(?::\d+)?$")


def parse_failures(output: str, root: str) -> list[tuple[str, str, str]]:
    """Every failure in `output` as (lint, item, file relative to `root`).

    A line of a "Failed in:" block that does not have the usual
    `<item> in <file>:<line>` shape is kept whole, with an empty file, so it
    can never match an entry and is reported instead of being dropped.
    """
    prefix = root.rstrip("/") + "/"
    failures = []
    lint = None
    in_block = False
    for line in _ANSI.sub("", output).splitlines():
        header = _FAILURE.match(line)
        if header:
            lint, in_block = header.group(1), False
        elif lint and line.strip() == "Failed in:":
            in_block = True
        elif in_block and line.startswith(" ") and line.strip():
            item = _ITEM.match(line)
            if item:
                failures.append(
                    (lint, item.group(1), item.group(2).removeprefix(prefix))
                )
            else:
                failures.append((lint, line.strip(), ""))
        elif in_block:
            in_block = False
    return failures


def split_accepted(
    failures: list[tuple[str, str, str]],
    baseline: str,
    table: dict[str, list[tuple[str, str, str, str]]] = ACCEPTED_SEMVER,
) -> tuple[list[tuple[str, str, str, str]], list[tuple[str, str, str]]]:
    """Split `failures` into (accepted, with the reason) and (rejected).

    Matched on the lint, the whole item and the file, never a substring. A
    baseline given as a commit hash matches no key, so nothing is accepted.
    """
    reasons = {entry[:3]: entry[3] for entry in table.get(baseline, [])}
    accepted = [(*f, reasons[f]) for f in failures if f in reasons]
    rejected = [f for f in failures if f not in reasons]
    return accepted, rejected


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print(__doc__, file=sys.stderr)
        return 2
    baseline, root = argv[1], argv[2]
    failures = parse_failures(sys.stdin.read(), root)
    if not failures:
        # The tool failed without a failure this parser recognises: a build
        # error, or an output format change. Neither is a pass.
        print("no failure could be parsed from the cargo-semver-checks output")
        return 1
    accepted, rejected = split_accepted(failures, baseline)
    for lint, item, path, reason in accepted:
        print(f"  ACCEPTED {lint}: {item} ({path}) -- {reason}")
    for lint, item, path in rejected:
        print(f"  BREAK    {lint}: {item}" + (f" ({path})" if path else ""))
    return 1 if rejected else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
