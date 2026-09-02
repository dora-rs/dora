#!/usr/bin/env bash
# scripts/qa/publish-graph.sh — crates.io publish-graph gate (#3304)
#
# `cargo publish` resolves every dependency of a crate against crates.io,
# so a release breaks — halfway through, after some crates are already
# irreversibly uploaded — if either half of the publish graph is wrong:
#
#   1. A published crate depends on a `publish = false` crate. That crate
#      is never uploaded, so the dependent's manifest names something the
#      registry does not have. #3304 is this: `dora-daemon` (published)
#      had an optional dependency on `dora-tensor-pool` (`publish = false`).
#
#   2. A published crate's workspace dependency is missing from — or comes
#      *after* it in — the ordered publish lists in `release.yml` /
#      `cargo-release.yml`. Marking a crate publishable is only half the
#      fix; if the lists are not updated in the same change, CI stays
#      green and the failure moves to release time.
#
# Both lists are maintained by hand and each carries a "keep in sync with
# the other" comment, so this script also checks they are identical.
#
# Which dependencies count: all of them, in every kind. `cargo package`
# strips only dev-dependencies declared without a version requirement
# (`cargo metadata` reports those as `req == "*"`). Every other
# dependency — normal, build, dev, optional, target-specific — keeps its
# requirement in the published manifest and has to resolve. This workspace
# declares its internal deps through `[workspace.dependencies]` with
# `version = "=<workspace version>"`, so nothing is stripped in practice.

set -euo pipefail

cd "$(dirname "$0")/../.."

if ! command -v python3 >/dev/null 2>&1; then
  echo "scripts/qa/publish-graph.sh requires python3 on PATH" >&2
  exit 2
fi

META="$(mktemp)"
trap 'rm -f "$META"' EXIT
cargo metadata --format-version 1 --no-deps >"$META"

python3 - \
  "$META" \
  .github/workflows/release.yml \
  .github/workflows/cargo-release.yml \
  <<'PY'
import json
import re
import sys

meta_path, release_yml, cargo_release_yml = sys.argv[1:4]

with open(meta_path, encoding="utf-8") as f:
    meta = json.load(f)

# `--no-deps` limits `packages` to workspace members, so a name lookup here
# can only ever resolve to a crate in this repository.
packages = {p["name"]: p for p in meta["packages"]}
unpublished = {n for n, p in packages.items() if p.get("publish") == []}

errors = []


def workspace_deps(pkg):
    """Deps on other workspace members that survive `cargo package`.

    `source is None` means a path dependency, which is what distinguishes a
    workspace member from a same-named crate pulled off crates.io.
    """
    for dep in pkg["dependencies"]:
        if dep["source"] is not None or dep["name"] not in packages:
            continue
        # Version-less dev-deps are dropped from the published manifest.
        if dep["kind"] == "dev" and dep["req"] == "*":
            continue
        yield dep


# --- 1. no published crate may depend on an unpublished one ---------------
for name, pkg in sorted(packages.items()):
    if name in unpublished:
        continue
    for dep in workspace_deps(pkg):
        if dep["name"] in unpublished:
            errors.append(
                f"{name} depends on {dep['name']}, which is `publish = false` "
                f"(kind: {dep['kind'] or 'normal'})"
            )


# --- 2. the two ordered publish lists must agree --------------------------
def read_release_yml(path):
    with open(path, encoding="utf-8") as f:
        block = re.search(r"^\s*CRATES=\((.*?)^\s*\)", f.read(), re.S | re.M)
    if block is None:
        sys.exit(f"could not find the CRATES=(...) list in {path}")
    # Drop shell comments, so a comment inside the array is not read as a
    # crate name (`cargo-release.yml`'s list is already comment-tolerant:
    # only `publish_if_not_exists` lines are matched there).
    lines = [ln.split("#", 1)[0] for ln in block.group(1).splitlines()]
    return " ".join(lines).split()


def read_cargo_release_yml(path):
    with open(path, encoding="utf-8") as f:
        return re.findall(r"^\s*publish_if_not_exists\s+(\S+)", f.read(), re.M)


release_list = read_release_yml(release_yml)
cargo_release_list = read_cargo_release_yml(cargo_release_yml)

if not release_list:
    sys.exit(f"{release_yml}: publish list is empty")

if release_list != cargo_release_list:
    errors.append(
        f"{release_yml} and {cargo_release_yml} publish lists differ "
        "(they must list the same crates in the same order):\n"
        f"      only in {release_yml}: "
        f"{sorted(set(release_list) - set(cargo_release_list)) or 'none'}\n"
        f"      only in {cargo_release_yml}: "
        f"{sorted(set(cargo_release_list) - set(release_list)) or 'none'}"
    )

seen = set()
for name in release_list:
    if name in seen:
        errors.append(f"{release_yml}: {name} is listed more than once")
    seen.add(name)
    if name not in packages:
        errors.append(f"{release_yml}: {name} is not a workspace member")
    elif name in unpublished:
        errors.append(f"{release_yml}: {name} is `publish = false`")


# --- 3. every dep of a listed crate must be listed, earlier ---------------
position = {name: i for i, name in enumerate(release_list)}
for name in release_list:
    pkg = packages.get(name)
    if pkg is None:
        continue
    for dep in workspace_deps(pkg):
        if dep["name"] in unpublished:
            continue  # already reported above
        if dep["name"] not in position:
            errors.append(
                f"{name} depends on {dep['name']}, which is missing from the "
                "publish lists"
            )
        elif position[dep["name"]] > position[name]:
            errors.append(
                f"{name} depends on {dep['name']}, which the publish lists "
                "publish after it"
            )

if errors:
    print("publish-graph check FAILED:")
    for e in dict.fromkeys(errors):
        print(f"  - {e}")
    print()
    print(
        "docs/qa-runbook.md section 3.5 walks through each of these; the rules "
        "and what they protect are at the top of scripts/qa/publish-graph.sh."
    )
    sys.exit(1)

print(
    f"publish-graph OK: {len(release_list)} crates, "
    f"{len(unpublished)} `publish = false` workspace members, "
    "no unpublished dependency and no out-of-order publish"
)
PY
