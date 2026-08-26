#!/usr/bin/env python3
"""Verify that a dora release actually shipped everything it should.

A release is assembled by several jobs writing to three different places
(crates.io, PyPI, the GitHub Release). Nothing previously checked that all
of them ran: `v1.0.0-rc.4` went green while `dora-rs-cli` was never
published at all, `dora-rs` shipped 4 of its 11 files, and none of the 8
C/C++ archives reached the release. This script is the gate that turns
that silence into a failure.

It checks what is *actually published*, not what CI believed it did, so it
catches a skipped job, a workflow that never triggered, and a matrix entry
that silently dropped out -- without needing to know which.

The *static* half of release correctness -- that the publish list agrees
with the workspace manifests and is ordered so no crate precedes its
dependencies -- is `scripts/qa/publish-graph.sh` (#3304), which runs in PR
CI. This script only reads back what a release actually produced.

Usage:
    scripts/release/verify-release.py --version 1.0.0-rc.5
    scripts/release/verify-release.py --version 1.0.0-rc.5 --skip github

Exits 0 when every expectation holds, 1 otherwise, naming exactly what is
missing.
"""

from __future__ import annotations

import argparse
import functools
import json
import os
import re
import sys
import time
import tomllib
import urllib.error
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REPO = "dora-rs/dora"
USER_AGENT = "dora-release-verifier"

# The two manifests `pip-release.yml` builds wheels from. The PyPI
# distribution name is read out of each, so renaming a package cannot
# silently narrow what this script checks.
PYPI_MANIFESTS = ("apis/python/node/pyproject.toml", "apis/python/cli/pyproject.toml")

# One entry per platform family in `pip-release.yml`'s build matrix
# (linux x86_64/x86/aarch64/armv7, musllinux x86_64/x86/aarch64, musleabi
# armv7, windows x64, macos aarch64). Matched as a regex against the wheel
# filename's platform tag so a deployment-target bump (macosx_11_0 ->
# macosx_12_0) does not fail the gate, while a whole missing family does.
REQUIRED_WHEEL_PLATFORMS = (
    r"manylinux_[\d_]+_x86_64",
    r"manylinux_[\d_]+_i686",
    r"manylinux_[\d_]+_aarch64",
    r"manylinux_[\d_]+_armv7l",
    r"musllinux_[\d_]+_x86_64",
    r"musllinux_[\d_]+_i686",
    r"musllinux_[\d_]+_aarch64",
    r"musllinux_[\d_]+_armv7l",
    r"win_amd64",
    r"macosx_[\d_]+_arm64",
)

# `release.yml`'s `dist` matrix plus the two install scripts.
REQUIRED_CLI_ASSETS = (
    "dora-cli-aarch64-apple-darwin.tar.gz",
    "dora-cli-aarch64-unknown-linux-gnu.tar.gz",
    "dora-cli-x86_64-unknown-linux-gnu.tar.gz",
    "dora-cli-x86_64-pc-windows-msvc.zip",
    "dora-cli-installer.sh",
    "dora-cli-installer.ps1",
)

# `publish-c-cpp-libraries.yml` stages a C and a C++ archive per target.
C_CPP_TARGETS = (
    "x86_64-unknown-linux-gnu",
    "aarch64-unknown-linux-gnu",
    "aarch64-apple-darwin",
    "x86_64-pc-windows-msvc",
)

class Report:
    """Collects failures so one run names every problem, not just the first."""

    def __init__(self) -> None:
        self.failures: list[str] = []

    def check(self, ok: bool, message: str) -> bool:
        if ok:
            print(f"  ok       {message}")
        else:
            print(f"  MISSING  {message}")
            self.failures.append(message)
        return ok

    def lookup(self, label: str, fetch):
        """Poll for a published artifact, reporting the outcome.

        Returns the fetched document, or None if it is absent or could not
        be verified — the two are reported differently because "not
        published" and "crates.io rate-limited us" call for different fixes,
        but both mean the release is not proven complete.
        """
        try:
            data = poll(fetch)
        except Transient as error:
            self.check(False, f"{label} — could not verify ({error})")
            return None
        self.check(data is not None, label)
        return data


def pep440(version: str) -> str | None:
    """Normalize a cargo version to the form maturin uploads to PyPI.

    `1.0.0-rc.5` -> `1.0.0rc5`, matching `dora-rs` 1.0.0rc4 on PyPI. The
    counter is optional because `release.yml`'s tag trigger accepts any
    `v<x>.<y>.<z>-*`, so `v1.0.0-beta` reaches here; PEP 440 spells that
    `1.0.0b0`. Returns None for a tag this cannot normalize -- the caller
    reports that as a failed check rather than exiting, because by the time
    this runs the artifacts are already published and the run still owes a
    full report.
    """
    match = re.fullmatch(r"(\d+\.\d+\.\d+)(?:-(a|b|c|alpha|beta|pre|preview|rc)\.?(\d+)?)?", version)
    if not match:
        return None
    base, kind, num = match.groups()
    if kind is None:
        return base
    kind = {"alpha": "a", "beta": "b", "c": "rc", "pre": "rc", "preview": "rc"}.get(kind, kind)
    return f"{base}{kind}{num or 0}"


class Transient(Exception):
    """A lookup that failed for a reason unrelated to what is published."""


def http_json(url: str, token: str | None = None) -> dict | None:
    """GET a JSON document.

    Returns None on 404 so callers can treat 'not published' as data. Rate
    limits and server errors raise `Transient` instead: 25 back-to-back
    crates.io reads can earn a 429, and answering that with either a
    traceback or a "MISSING" verdict would be wrong in opposite directions.
    """
    request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    if token:
        request.add_header("Authorization", f"Bearer {token}")
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        if error.code == 404:
            return None
        if error.code in (403, 408, 429) or error.code >= 500:
            raise Transient(f"HTTP {error.code} from {url}") from error
        raise
    except (urllib.error.URLError, TimeoutError) as error:
        raise Transient(f"{type(error).__name__} from {url}") from error


# One deadline shared by every polled lookup in the run, set once from
# --propagation-timeout. Per-lookup budgets would multiply: 25 crates that
# are genuinely absent at a 5-minute budget each is a two-hour job that
# reports what a two-minute job already knew.
_POLL_DEADLINE: float | None = None


def set_poll_deadline(timeout: float) -> None:
    global _POLL_DEADLINE
    _POLL_DEADLINE = time.monotonic() + timeout


def poll(fetch, interval: float = 15.0):
    """Retry `fetch` until it returns a truthy value or the shared deadline passes.

    crates.io and PyPI both serve their read APIs from caches that lag a
    publish by seconds, so a bare read right after upload is flaky. This
    makes the gate wait for propagation instead of reporting a false gap --
    but only until the run's budget is spent, after which every remaining
    lookup is a single attempt.
    """
    while True:
        transient = None
        try:
            result = fetch()
        except Transient as error:
            result, transient = None, error
        expired = _POLL_DEADLINE is None or time.monotonic() >= _POLL_DEADLINE
        if result:
            return result
        if expired:
            if transient is not None:
                raise transient
            return None
        time.sleep(min(interval, max(0.0, _POLL_DEADLINE - time.monotonic())))


@functools.cache
def publish_list() -> tuple[str, ...]:
    """The ordered crate list `release.yml` publishes.

    Parsed out of the workflow rather than duplicated here, so the two
    cannot drift -- and so a PR that adds a crate to the array is covered
    with no change to this file.
    """
    workflow = (REPO_ROOT / ".github/workflows/release.yml").read_text()
    match = re.search(r"^\s*CRATES=\((.*?)^\s*\)", workflow, re.S | re.M)
    if not match:
        sys.exit("error: could not find the CRATES=( ... ) array in release.yml")
    # Strip shell comments, matching scripts/qa/publish-graph.sh's reader so
    # the two cannot disagree about what the list contains.
    lines = [line.split("#", 1)[0] for line in match.group(1).splitlines()]
    return tuple(" ".join(lines).split())


def pypi_names() -> list[str]:
    names = []
    for manifest in PYPI_MANIFESTS:
        with (REPO_ROOT / manifest).open("rb") as handle:
            names.append(tomllib.load(handle)["project"]["name"])
    return names


def check_crates_io(report: Report, version: str) -> None:
    print(f"\ncrates.io @ {version}")
    for crate in publish_list():
        report.lookup(
            f"crates.io: {crate} {version}",
            lambda crate=crate: http_json(f"https://crates.io/api/v1/crates/{crate}/{version}"),
        )


def check_pypi(report: Report, version: str) -> None:
    pypi_version = pep440(version)
    if pypi_version is None:
        print("\nPyPI")
        report.check(False, f"PyPI: cannot normalize {version!r} to a PEP 440 version to look up")
        return
    print(f"\nPyPI @ {pypi_version}")
    for package in pypi_names():
        data = report.lookup(
            f"PyPI: {package} {pypi_version} exists",
            lambda package=package: http_json(f"https://pypi.org/pypi/{package}/{pypi_version}/json"),
        )
        if data is None:
            continue

        filenames = [url["filename"] for url in data["urls"]]
        report.check(
            any(name.endswith(".tar.gz") for name in filenames),
            f"PyPI: {package} sdist",
        )
        for platform in REQUIRED_WHEEL_PLATFORMS:
            pattern = re.compile(rf"-{platform}\.whl$")
            report.check(
                any(pattern.search(name) for name in filenames),
                f"PyPI: {package} wheel for {platform}",
            )


def check_github_release(report: Report, version: str, repo: str) -> None:
    print(f"\nGitHub Release v{version}")
    token = os.environ.get("GITHUB_TOKEN") or os.environ.get("GH_TOKEN")
    data = report.lookup(
        f"GitHub: release v{version} exists",
        lambda: http_json(f"https://api.github.com/repos/{repo}/releases/tags/v{version}", token),
    )
    if data is None:
        return

    assets = {asset["name"] for asset in data["assets"]}
    for name in REQUIRED_CLI_ASSETS:
        report.check(name in assets, f"GitHub: {name}")
    for target in C_CPP_TARGETS:
        for flavor in ("c", "cpp"):
            prefix = f"dora-{flavor}-libraries-{target}"
            report.check(
                any(name.startswith(prefix) for name in assets),
                f"GitHub: {prefix}.*",
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--version", required=True, help="workspace version, e.g. 1.0.0-rc.5")
    parser.add_argument("--repo", default=DEFAULT_REPO)
    parser.add_argument(
        "--skip",
        action="append",
        default=[],
        choices=["crates", "pypi", "github"],
        help="skip a check (repeatable)",
    )
    parser.add_argument(
        "--propagation-timeout",
        type=float,
        default=300.0,
        help="total seconds to wait for registry reads to catch up with a publish, shared across all lookups (default: 300)",
    )
    args = parser.parse_args()

    version = args.version.removeprefix("v")
    set_poll_deadline(args.propagation_timeout)
    report = Report()
    ran = []

    print(f"Verifying dora {version} is completely published")
    if "crates" not in args.skip:
        check_crates_io(report, version)
        ran.append("crates.io")
    if "pypi" not in args.skip:
        check_pypi(report, version)
        ran.append("PyPI")
    if "github" not in args.skip:
        check_github_release(report, version, args.repo)
        ran.append("GitHub Release")
    if not ran:
        sys.exit("error: every check was skipped, nothing was verified")

    print()
    if report.failures:
        print(f"INCOMPLETE RELEASE: {len(report.failures)} expectation(s) not met\n")
        for failure in report.failures:
            print(f"  - {failure}")
        print(
            "\nThe tag is published but the release is missing artifacts. Fix the job that "
            "should have produced them and re-run; every publish step is idempotent."
        )
        return 1

    # Name what was actually checked: `--skip pypi` passing does not mean
    # the wheels are there, and the summary line should not imply it.
    print(f"Release {version}: {', '.join(ran)} complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
