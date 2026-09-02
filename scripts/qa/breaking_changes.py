#!/usr/bin/env python3
"""Compare every surface frozen by the dora 1.0 guarantee against a baseline.

`docs/api-rust.md` ("Stability scope at 1.0") lists what 1.x freezes. Three
mechanisms back it today -- exact version pins, feature gates, and the
publish-graph gate -- and all three are about *cargo*. They say nothing about
the surfaces users actually touch: the C header, the cxx bridge, the dataflow
YAML schema, the `dora` command, and the postcard wire format. Those could all
break with nothing failing until someone's deployment desynchronized.

This script closes that gap without compiling anything: every surface here is
extracted from source text (or a checked-in snapshot) on both sides, so the
whole gate runs in seconds and can sit in PR CI. The compile-dependent halves
live elsewhere and are wired up by `scripts/qa/breaking-changes.sh`:
`cargo-semver-checks` for the Rust API, and the snapshot-freshness tests that
keep the checked-in inputs here honest.

Exit codes: 0 clean, 1 breaking changes found, 2 the gate could not run.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

# --------------------------------------------------------------------------
# Findings
# --------------------------------------------------------------------------

BREAK = "BREAK"  # incompatible in at least one direction -- fails the gate
WARN = "WARN"  # worth a human look, does not fail the gate


@dataclass
class Finding:
    level: str
    detail: str


@dataclass
class SurfaceResult:
    name: str
    findings: list[Finding] = field(default_factory=list)
    skipped: str | None = None
    summary: str = ""

    def breaks(self) -> list[Finding]:
        return [f for f in self.findings if f.level == BREAK]

    def warns(self) -> list[Finding]:
        return [f for f in self.findings if f.level == WARN]


# --------------------------------------------------------------------------
# Baseline access
# --------------------------------------------------------------------------


class Baseline:
    """Read-only view of the tree at the baseline ref."""

    def __init__(self, root: Path, ref: str, fallback: str | None = None):
        self.root = root
        self.ref = ref
        self.fallback = fallback

    def read_at(self, ref: str, rel: str) -> str | None:
        proc = subprocess.run(
            ["git", "-C", str(self.root), "show", f"{ref}:{rel}"],
            capture_output=True,
            text=True,
        )
        return proc.stdout if proc.returncode == 0 else None

    def read(self, rel: str) -> str | None:
        """File contents at the baseline ref, or None if it did not exist."""
        return self.read_at(self.ref, rel)

    def read_or_fallback(self, rel: str) -> tuple[str | None, str | None]:
        """As `read`, falling back to a second ref for files the release
        predates.

        A surface file added after the last release has nothing to compare
        against, so its check would sit skipped until the *next* release --
        a window in which that surface is unguarded. CI passes the PR's base
        commit as the fallback, which keeps the check live in the meantime by
        asking a narrower question: did this branch remove anything?
        """
        text = self.read(rel)
        if text is not None:
            return text, self.ref
        if self.fallback:
            text = self.read_at(self.fallback, rel)
            if text is not None:
                return text, self.fallback
        return None, None

    def list_dir(self, rel: str) -> list[str]:
        """Paths under `rel` at the baseline ref (recursive, repo-relative)."""
        proc = subprocess.run(
            ["git", "-C", str(self.root), "ls-tree", "-r", "--name-only", self.ref, rel],
            capture_output=True,
            text=True,
        )
        if proc.returncode != 0:
            return []
        return [line for line in proc.stdout.splitlines() if line]


def tag_sort_key(tag: str) -> tuple:
    """Semver ordering for a `vX.Y.Z[-pre]` tag.

    Not `git tag --sort=-v:refname`: git sorts a prerelease suffix *after* the
    release it precedes unless `versionsort.prereleaseSuffix` is configured, so
    it would call `v1.0.0-rc.5` newer than `v1.0.0` and pick an rc as the
    baseline for the release that superseded it.
    """
    match = re.fullmatch(r"v(\d+)\.(\d+)\.(\d+)(?:-(.+))?", tag)
    if not match:
        return ()
    major, minor, patch, pre = match.groups()
    # A prerelease sorts below the same version's final release.
    return (int(major), int(minor), int(patch), 0 if pre else 1, pre or "")


def grep_tracked(root: Path, ref: str | None, pattern: str) -> dict[str, str]:
    """Matching lines from tracked `Cargo.toml` files, keyed by path.

    `ref` reads the tree at that revision; None reads the working tree.
    """
    command = ["git", "-C", str(root), "grep", pattern]
    if ref:
        command.append(ref)
    command += ["--", "*Cargo.toml"]
    output = subprocess.run(command, capture_output=True, text=True).stdout
    hits: dict[str, str] = {}
    for line in output.splitlines():
        if ref and line.startswith(f"{ref}:"):
            line = line[len(ref) + 1 :]
        path, _, content = line.partition(":")
        if path:
            hits[path] = hits.get(path, "") + content + "\n"
    return hits


def resolve_baseline(root: Path) -> str:
    """The newest released tag -- what deployed users are actually running.

    Falls back to asking the remote for its tag *names* when the checkout has
    none. A CI checkout is shallow and tagless, and the repo's pack is well
    over a gigabyte, so `fetch-depth: 0` would make every PR pay a full clone
    to read one file. `ls-remote` transfers no objects; the caller then fetches
    just the one tag this returns.
    """
    proc = subprocess.run(
        ["git", "-C", str(root), "tag", "--list", "v*"],
        capture_output=True,
        text=True,
        check=True,
    )
    tags = [t for t in proc.stdout.splitlines() if t and tag_sort_key(t)]
    if not tags:
        remote = subprocess.run(
            ["git", "-C", str(root), "ls-remote", "--tags", "origin"],
            capture_output=True,
            text=True,
        )
        tags = [
            name
            for line in remote.stdout.splitlines()
            for name in [line.split("refs/tags/")[-1]]
            if not name.endswith("^{}") and tag_sort_key(name)
        ]
    if not tags:
        raise SystemExit("error: no release tags found; pass --baseline <ref> explicitly")
    return max(tags, key=tag_sort_key)


# --------------------------------------------------------------------------
# Shared text helpers
# --------------------------------------------------------------------------


def strip_c_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", " ", text)


def strip_rust_comments(text: str) -> str:
    """Drop `//` line comments (doc comments included).

    Block comments are left alone: they are rare in the files parsed here and
    stripping them naively would corrupt any `*/` inside a string literal.
    """
    return re.sub(r"//[^\n]*", "", text)


def collapse_ws(text: str) -> str:
    return re.sub(r"\s+", " ", text).strip()


def balanced_block(
    text: str, open_idx: int, open_ch: str = "{", close_ch: str = "}"
) -> tuple[str, int]:
    """Body of the bracketed group starting at `open_idx`, and the index after."""
    depth = 0
    for i in range(open_idx, len(text)):
        if text[i] == open_ch:
            depth += 1
        elif text[i] == close_ch:
            depth -= 1
            if depth == 0:
                return text[open_idx + 1 : i], i + 1
    raise ValueError(f"unbalanced {open_ch}{close_ch}")


def split_top_level(text: str, sep: str = ",") -> list[str]:
    """Split on `sep`, ignoring separators nested in (), [], <> or {}."""
    parts, depth, current = [], 0, []
    for ch in text:
        if ch in "([{<":
            depth += 1
        elif ch in ")]}>":
            depth -= 1
        if ch == sep and depth == 0:
            parts.append("".join(current))
            current = []
        else:
            current.append(ch)
    parts.append("".join(current))
    return [p.strip() for p in parts if p.strip()]


def compare_ordered(
    kind: str,
    owner: str,
    old: list[str],
    new: list[str],
    *,
    added_level: str,
    reorder_note: str,
) -> list[Finding]:
    """Compare two ordered member lists where position is part of the contract.

    Used for anything whose encoding is positional: C enum constants (ordinals),
    postcard struct fields and enum discriminants. Removal, rename and reorder
    are all breaking; whether an *addition* breaks depends on the encoding, so
    the caller passes its severity.
    """
    findings = []
    removed = [m for m in old if m not in new]
    added = [m for m in new if m not in old]
    for m in removed:
        findings.append(Finding(BREAK, f"{owner}: {kind} `{m}` removed"))
    for m in added:
        findings.append(Finding(added_level, f"{owner}: {kind} `{m}` added"))
    # Reorder: compare the members present on both sides in their two orders.
    kept_old = [m for m in old if m in new]
    kept_new = [m for m in new if m in old]
    if kept_old != kept_new and not removed:
        findings.append(
            Finding(BREAK, f"{owner}: {kind} order changed ({reorder_note})")
        )
    # An addition anywhere but the end shifts every later member's position.
    # Nothing was removed and the shared members kept their order, so `old` is
    # a subsequence of `new`: appending is exactly `new` starting with `old`.
    if added and not removed and kept_old == kept_new and new[: len(old)] != old:
        findings.append(
            Finding(
                BREAK,
                f"{owner}: {kind} inserted before the end, shifting later "
                f"{kind}s ({reorder_note})",
            )
        )
    return findings


def compare_signature_maps(
    noun: str,
    old: dict[str, str],
    new: dict[str, str],
    *,
    added_level: str = WARN,
) -> list[Finding]:
    """Compare `name -> signature` maps: the C functions and the cxx fns."""
    findings = []
    for name, signature in old.items():
        if name not in new:
            findings.append(Finding(BREAK, f"{noun} `{name}` removed"))
        elif new[name] != signature:
            findings.append(
                Finding(
                    BREAK,
                    f"{noun} `{name}` signature changed: "
                    f"`{signature}` -> `{new[name]}`",
                )
            )
    for name in new:
        if name not in old:
            findings.append(Finding(added_level, f"{noun} `{name}` added"))
    return findings


def compare_member_maps(
    noun: str,
    member_kind: str,
    old: dict[str, list[str]],
    new: dict[str, list[str]],
    *,
    added_level: str,
    reorder_note: str,
) -> list[Finding]:
    """Compare `name -> ordered members` maps: C enums, cxx structs and enums."""
    findings = []
    for name, members in old.items():
        if name not in new:
            findings.append(Finding(BREAK, f"{noun} `{name}` removed"))
            continue
        findings.extend(
            compare_ordered(
                member_kind,
                f"{noun} `{name}`",
                members,
                new[name],
                added_level=added_level,
                reorder_note=reorder_note,
            )
        )
    return findings


# --------------------------------------------------------------------------
# Surface 1: the C node API header
# --------------------------------------------------------------------------

C_HEADER = "apis/c/node/node_api.h"


def parse_c_header(text: str) -> dict:
    """Functions (name -> type signature) and enums (name -> ordered constants).

    Parameter *names* are dropped: renaming one is not a break in C, and keeping
    them would report every doc-driven rename as a signature change.
    """
    text = strip_c_comments(text)

    enums: dict[str, list[str]] = {}
    for match in re.finditer(r"\benum\s+(\w+)\s*\{([^}]*)\}", text):
        constants = []
        for entry in split_top_level(match.group(2)):
            name = entry.split("=")[0].strip()
            if name:
                constants.append(name)
        enums[match.group(1)] = constants
    # Remove the enum bodies so the function scan below cannot see into them.
    text = re.sub(r"\benum\s+\w+\s*\{[^}]*\}", " ", text)

    functions: dict[str, str] = {}
    for match in re.finditer(r"([\w\s*]+?)\s*\b(\w+)\s*\(([^)]*)\)\s*;", text):
        ret, name, params = match.group(1), match.group(2), match.group(3)
        functions[name] = f"{normalize_c_type(ret)} ({normalize_c_params(params)})"

    return {"functions": functions, "enums": enums}


C_TYPE_KEYWORDS = {
    "void", "char", "short", "int", "long", "float", "double", "signed",
    "unsigned", "size_t", "const", "enum", "struct", "union", "_Bool",
}


def normalize_c_type(text: str) -> str:
    return collapse_ws(text.replace("*", " * "))


def normalize_c_params(params: str) -> str:
    out = []
    for param in split_top_level(params):
        tokens = normalize_c_type(param).split()
        # Drop a trailing parameter name, but never the type itself.
        if len(tokens) > 1 and re.fullmatch(r"[A-Za-z_]\w*", tokens[-1]):
            if tokens[-1] not in C_TYPE_KEYWORDS:
                tokens = tokens[:-1]
        out.append(" ".join(tokens))
    return ", ".join(out)


def check_c_header(root: Path, baseline: Baseline) -> SurfaceResult:
    result = SurfaceResult(name=f"C node API ({C_HEADER})")
    old_text = baseline.read(C_HEADER)
    if old_text is None:
        result.skipped = "not present in the baseline"
        return result
    old = parse_c_header(old_text)
    new = parse_c_header((root / C_HEADER).read_text())

    result.findings += compare_signature_maps(
        "function", old["functions"], new["functions"]
    )
    # C enum constants are ABI: their ordinals are compiled into every node
    # built against the old header.
    result.findings += compare_member_maps(
        "enum",
        "constant",
        old["enums"],
        new["enums"],
        added_level=WARN,
        reorder_note="ordinals are compiled into existing nodes",
    )

    result.summary = (
        f"{len(new['functions'])} functions, {len(new['enums'])} enums"
    )
    return result


# --------------------------------------------------------------------------
# Surface 2: the C++ node API (cxx bridge)
# --------------------------------------------------------------------------

CXX_BRIDGE = "apis/c++/node/src/lib.rs"


def parse_cxx_bridge(text: str) -> dict:
    """Items declared in `#[cxx::bridge] mod ffi { ... }`.

    That block *is* the C++ API: cxx generates the header from it, so every
    struct field, enum variant and fn signature here is user-visible.
    """
    text = strip_rust_comments(text)
    match = re.search(r"#\[cxx::bridge\][\s\S]*?\bmod\s+\w+\s*\{", text)
    if not match:
        raise ValueError("no #[cxx::bridge] module found")
    body, _ = balanced_block(text, match.end() - 1)

    structs: dict[str, list[str]] = {}
    enums: dict[str, list[str]] = {}
    functions: dict[str, str] = {}

    for m in re.finditer(r"\b(?:pub\s+)?(struct|enum)\s+(\w+)\s*\{", body):
        block, _ = balanced_block(body, m.end() - 1)
        members = []
        for entry in split_top_level(block):
            entry = re.sub(r"#\[[^\]]*\]", " ", entry).strip()
            if not entry:
                continue
            if m.group(1) == "struct":
                members.append(collapse_ws(entry))
            else:
                members.append(entry.split("=")[0].split("(")[0].strip())
        if m.group(1) == "struct":
            structs[m.group(2)] = members
        else:
            enums[m.group(2)] = members

    for m in re.finditer(r"\b(?:unsafe\s+)?extern\s+\"[^\"]+\"\s*\{", body):
        block, _ = balanced_block(body, m.end() - 1)
        # Strip out any nested attribute noise, then take `;`-terminated decls.
        block = re.sub(r"#\[[^\]]*\]", " ", block)
        for decl in block.split(";"):
            decl = collapse_ws(decl)
            fn = re.match(r"(?:pub\s+)?(?:unsafe\s+)?fn\s+(\w+)\s*(.*)", decl)
            if fn:
                name, sig = fn.group(1), collapse_ws(fn.group(2))
                # `fn next` is declared once per receiver type, so the receiver
                # is part of the identity -- keyed by name alone, dropping one
                # of them would look like no change at all.
                receiver = re.search(r"\bself\s*:\s*&(?:mut\s+)?(\w+)", sig)
                key = f"{receiver.group(1)}::{name}" if receiver else name
                functions[key] = sig

    return {"structs": structs, "enums": enums, "functions": functions}


def check_cxx_bridge(root: Path, baseline: Baseline) -> SurfaceResult:
    result = SurfaceResult(name="C++ node API (cxx bridge)")
    old_text = baseline.read(CXX_BRIDGE)
    if old_text is None:
        result.skipped = "not present in the baseline"
        return result
    old = parse_cxx_bridge(old_text)
    new = parse_cxx_bridge((root / CXX_BRIDGE).read_text())

    result.findings += compare_member_maps(
        "struct",
        "field",
        old["structs"],
        new["structs"],
        # A new field changes the generated C++ struct layout.
        added_level=BREAK,
        reorder_note="the generated C++ struct layout changes",
    )
    result.findings += compare_member_maps(
        "enum",
        "variant",
        old["enums"],
        new["enums"],
        added_level=WARN,
        reorder_note="discriminants are compiled into existing nodes",
    )
    result.findings += compare_signature_maps(
        "fn", old["functions"], new["functions"]
    )

    result.summary = (
        f"{len(new['structs'])} structs, {len(new['enums'])} enums, "
        f"{len(new['functions'])} fns"
    )
    return result


# --------------------------------------------------------------------------
# Surface 3: the dataflow YAML schema
# --------------------------------------------------------------------------

# `libraries/core/dora-schema.json` is deliberately absent: it is a byte-for-byte
# copy of the root file (the generator writes both), so diffing it would report
# every schema finding twice. The freshness check in breaking-changes.sh still
# covers it -- there the point is that the copies stay in step.
SCHEMAS = ["dora-schema.json", "libraries/core/dora-node-schema.json"]


UNION_KEYS = ("oneOf", "anyOf", "allOf")


def subschema_key(node) -> str:
    """Stable identity for one alternative of a `oneOf` / `anyOf` list.

    Not its index. schemars emits one alternative per enum variant, so an
    added variant shifts every later index and an index-keyed diff then
    compares unrelated alternatives -- reporting a pile of breaks for a purely
    additive change. Keyed by content, an insertion is just an insertion.
    """
    if isinstance(node, dict):
        if "$ref" in node:
            return str(node["$ref"]).rsplit("/", 1)[-1]
        if "const" in node:
            return f"const={json.dumps(node['const'], sort_keys=True)}"
        if isinstance(node.get("enum"), list):
            return "enum=" + ",".join(sorted(json.dumps(v) for v in node["enum"]))
        if isinstance(node.get("properties"), dict):
            return "object{" + ",".join(sorted(node["properties"])) + "}"
        if "type" in node:
            t = node["type"]
            return t if isinstance(t, str) else "|".join(sorted(t))
    digest = hashlib.sha256(json.dumps(node, sort_keys=True).encode()).hexdigest()
    return f"sha:{digest[:12]}"


def union_member_keys(members: list) -> list[str]:
    """`subschema_key` per member, disambiguated if two collide."""
    keys, seen = [], {}
    for member in members:
        key = subschema_key(member)
        seen[key] = seen.get(key, 0) + 1
        keys.append(key if seen[key] == 1 else f"{key}#{seen[key]}")
    return keys


def flatten_schema(node, path: str = "#", out: dict | None = None) -> dict:
    """Map every schema location to the parts of it a dataflow can depend on.

    Keyed by JSON pointer, so `$defs` entries line up across versions even when
    the file is reordered. `$ref` is not followed -- each definition is visited
    once, where it is defined.
    """
    if out is None:
        out = {}
    if isinstance(node, dict):
        entry = {}
        if isinstance(node.get("properties"), dict):
            entry["properties"] = sorted(node["properties"])
        if isinstance(node.get("required"), list):
            entry["required"] = sorted(str(r) for r in node["required"])
        if isinstance(node.get("enum"), list):
            entry["enum"] = sorted(json.dumps(v, sort_keys=True) for v in node["enum"])
        if "type" in node:
            t = node["type"]
            entry["type"] = sorted(t) if isinstance(t, list) else [t]
        if isinstance(node.get("additionalProperties"), bool):
            entry["additionalProperties"] = node["additionalProperties"]
        unions = {}
        for key in UNION_KEYS:
            if isinstance(node.get(key), list):
                unions[key] = union_member_keys(node[key])
        if unions:
            entry["unions"] = unions
        if entry:
            out[path] = entry
        for key, value in node.items():
            if key in UNION_KEYS and isinstance(value, list):
                for member_key, member in zip(unions[key], value):
                    flatten_schema(member, f"{path}/{key}/{member_key}", out)
            else:
                flatten_schema(value, f"{path}/{key}", out)
    elif isinstance(node, list):
        for i, value in enumerate(node):
            flatten_schema(value, f"{path}/{i}", out)
    return out


def diff_schema(old: dict, new: dict) -> list[Finding]:
    findings = []
    reported_gone: list[str] = []
    for path, old_entry in old.items():
        if path not in new:
            # Only report the outermost removal: every child of a removed
            # definition is also missing, and listing them all buries it.
            if any(path.startswith(p + "/") for p in reported_gone):
                continue
            reported_gone.append(path)
            # A removed property or union alternative is already reported
            # against its parent, in the parent's own terms. Reporting the
            # vanished path too says the same thing twice, less clearly.
            parent, _, _ = path.rpartition("/")
            covered_by_parent = parent.endswith("/properties") or parent.endswith(
                UNION_KEYS
            )
            if covered_by_parent and parent.rpartition("/")[0] in new:
                continue
            findings.append(Finding(BREAK, f"{path}: removed from the schema"))
            continue
        new_entry = new[path]
        for prop in old_entry.get("properties", []):
            if prop not in new_entry.get("properties", []):
                findings.append(Finding(BREAK, f"{path}: property `{prop}` removed"))
        for req in new_entry.get("required", []):
            if req not in old_entry.get("required", []):
                findings.append(
                    Finding(
                        BREAK,
                        f"{path}: `{req}` is now required, so dataflows that "
                        "omit it stop validating",
                    )
                )
        for value in old_entry.get("enum", []):
            if value not in new_entry.get("enum", []):
                findings.append(Finding(BREAK, f"{path}: enum value {value} removed"))
        for t in old_entry.get("type", []):
            if t not in new_entry.get("type", []):
                findings.append(Finding(BREAK, f"{path}: no longer accepts type `{t}`"))
        for key, members in old_entry.get("unions", {}).items():
            now = new_entry.get("unions", {}).get(key, [])
            for member in members:
                if member not in now:
                    findings.append(
                        Finding(BREAK, f"{path}: {key} no longer accepts `{member}`")
                    )
        if new_entry.get("additionalProperties") is False and old_entry.get(
            "additionalProperties"
        ) is not False:
            findings.append(
                Finding(BREAK, f"{path}: additional properties are now rejected")
            )
    return findings


def check_schema(root: Path, baseline: Baseline) -> SurfaceResult:
    result = SurfaceResult(name="Dataflow YAML schema")
    checked = 0
    for rel in SCHEMAS:
        old_text = baseline.read(rel)
        if old_text is None:
            continue
        current = root / rel
        if not current.exists():
            result.findings.append(Finding(BREAK, f"{rel}: schema file removed"))
            continue
        old = flatten_schema(json.loads(old_text))
        new = flatten_schema(json.loads(current.read_text()))
        for finding in diff_schema(old, new):
            result.findings.append(Finding(finding.level, f"{rel} {finding.detail}"))
        checked += 1
    if checked == 0:
        result.skipped = "no schema files in the baseline"
        return result
    result.summary = f"{checked} schema files"
    return result


# --------------------------------------------------------------------------
# Surface 4: the postcard wire format (dora-message)
# --------------------------------------------------------------------------

WIRE_DIR = "libraries/message/src"


SERDE_WIRE_KEYS = (
    "rename",
    "rename_all",
    "flatten",
    "skip",
    "tag",
    "untagged",
    "content",
    "with",
)


def collect_attribute_run(text: str, i: int) -> tuple[list[str], int]:
    """The run of `#[...]` attributes starting at `i`, and the index after it."""
    attrs = []
    while True:
        while i < len(text) and text[i].isspace():
            i += 1
        if not text.startswith("#[", i):
            return attrs, i
        depth, start = 0, i
        while i < len(text):
            if text[i] == "[":
                depth += 1
            elif text[i] == "]":
                depth -= 1
                if depth == 0:
                    i += 1
                    break
            i += 1
        attrs.append(text[start:i])


def serde_wire_attrs(attrs: str) -> list[str]:
    """The serde keys in `attrs` that change what goes on the wire.

    `default`, `alias` and friends only widen decoding, so they are dropped:
    treating them as changes would fail the gate on a strictly compatible edit.
    """
    kept = []
    for attr in re.findall(r"#\[serde\(([^\]]*)\)\]", attrs):
        for part in split_top_level(attr):
            if part.split("=")[0].strip() in SERDE_WIRE_KEYS:
                kept.append(collapse_ws(part))
    return sorted(kept)


def normalize_member(entry: str) -> str:
    """A member as the wire sees it: name, type, and the serde keys that move
    bytes."""
    kept = serde_wire_attrs(entry)
    entry = collapse_ws(re.sub(r"#\[[^\]]*\]", " ", entry))
    entry = re.sub(r"^pub(\([^)]*\))?\s+", "", entry)
    # A trailing comma inside a braced variant payload is rustfmt's choice, not
    # a wire change -- normalize it away so reformatting cannot fail the gate.
    entry = re.sub(r",\s*(?=[}\)])", " ", entry)
    entry = collapse_ws(entry)
    suffix = f" [serde: {', '.join(kept)}]" if kept else ""
    return entry + suffix


def strip_cfg_test_modules(text: str) -> str:
    """Drop `#[cfg(test)] mod ... { ... }` blocks.

    Test modules define serde types of their own -- `bulk_bytes.rs` has six --
    and they are not the wire protocol. Left in, deleting a test fixture would
    be reported as a protocol break.
    """
    while True:
        match = re.search(r"#\[cfg\(test\)\]\s*(?:pub\s+)?mod\s+\w+\s*\{", text)
        if not match:
            return text
        _, end = balanced_block(text, match.end() - 1)
        text = text[: match.start()] + text[end:]


ITEM_RE = re.compile(
    r"(?:pub(?:\([^)]*\))?\s+)?(struct|enum)\s+(\w+)\s*(?:<[^>]*>)?\s*"
)


def parse_wire_types(files: dict[str, str]) -> dict[str, dict]:
    """Serde-derived types in `dora-message`, with members in declaration order.

    postcard is positional and carries no field names or type descriptors, so
    the *order* of a struct's fields and of an enum's variants is the wire
    format. `versions_compatible` is semver-caret, so a 1.0 node and a 1.5
    daemon are waved through the handshake and will then simply misparse each
    other if this drifts.

    Newtypes count: `DataId(String)` becoming `DataId(u64)` changes the bytes
    of every message carrying one. So do container attributes: `#[serde(
    untagged)]` decides whether a variant tag is written at all, and it can sit
    on either side of the derive -- which is why whole attribute *runs* are
    collected rather than scanned forward from the derive.
    """
    types: dict[str, dict] = {}
    for rel, text in sorted(files.items()):
        text = strip_cfg_test_modules(strip_rust_comments(text))
        # Full relative path, not just the stem: two `mod.rs`-style files in
        # different subdirectories would otherwise collide and one would
        # silently overwrite the other, shrinking the checked set on both
        # sides at once so the gate still reported "ok".
        module = "::".join(Path(rel).with_suffix("").parts)
        i = 0
        while True:
            at = text.find("#[", i)
            if at == -1:
                break
            attrs, after = collect_attribute_run(text, at)
            i = after
            joined = " ".join(attrs)
            derives = " ".join(re.findall(r"#\[derive\(([^)]*)\)\]", joined))
            if "Serialize" not in derives and "Deserialize" not in derives:
                continue
            item = ITEM_RE.match(text, after)
            if not item:
                continue
            kind, name = item.group(1), item.group(2)
            rest = item.end()
            if rest < len(text) and text[rest] == "{":
                body, i = balanced_block(text, rest)
                members = [
                    normalize_member(e) for e in split_top_level(body) if e.strip()
                ]
            elif rest < len(text) and text[rest] == "(":
                body, i = balanced_block(text, rest, "(", ")")
                members = [
                    f"{idx}: {normalize_member(f)}"
                    for idx, f in enumerate(split_top_level(body))
                ]
            else:
                members = []
                i = rest
            types[f"{module}::{name}"] = {
                "kind": kind,
                "members": members,
                "container": serde_wire_attrs(joined),
            }
    return types


def check_wire_format(root: Path, baseline: Baseline) -> SurfaceResult:
    result = SurfaceResult(name="Wire format (dora-message)")
    old_files = {
        rel[len(WIRE_DIR) + 1 :]: baseline.read(rel) or ""
        for rel in baseline.list_dir(WIRE_DIR)
        if rel.endswith(".rs")
    }
    if not old_files:
        result.skipped = "no dora-message sources in the baseline"
        return result
    new_files = {
        str(p.relative_to(root / WIRE_DIR)): p.read_text()
        for p in (root / WIRE_DIR).rglob("*.rs")
    }
    old = parse_wire_types(old_files)
    new = parse_wire_types(new_files)

    for name, old_type in old.items():
        if name not in new:
            result.findings.append(
                Finding(BREAK, f"`{name}` removed from the wire protocol")
            )
            continue
        new_type = new[name]
        if new_type.get("container") != old_type.get("container"):
            # `untagged`, `tag`, `from`/`into`: these decide the framing of the
            # whole type, so a change reshapes every message carrying it.
            result.findings.append(
                Finding(
                    BREAK,
                    f"`{name}`: serde container attributes changed "
                    f"({old_type.get('container') or 'none'} -> "
                    f"{new_type.get('container') or 'none'})",
                )
            )
        if new_type["kind"] != old_type["kind"]:
            result.findings.append(
                Finding(
                    BREAK,
                    f"`{name}` changed from {old_type['kind']} to {new_type['kind']}",
                )
            )
            continue
        if old_type["kind"] == "struct":
            # Every field is length-free and positional: adding one makes new
            # messages undecodable by old peers *and* old messages undecodable
            # by new ones, so additions break both directions.
            added_level = BREAK
            note = "postcard encodes fields positionally"
        else:
            # A new variant only breaks new -> old (an old peer cannot decode a
            # discriminant it has never heard of). Old messages still decode,
            # so this is reported rather than blocked -- it is how the protocol
            # is meant to grow.
            added_level = WARN
            note = "postcard encodes the variant index"
        result.findings.extend(
            compare_ordered(
                "field" if old_type["kind"] == "struct" else "variant",
                f"`{name}`",
                old_type["members"],
                new_type["members"],
                added_level=added_level,
                reorder_note=note,
            )
        )

    result.summary = f"{len(new)} serde types"
    return result


# --------------------------------------------------------------------------
# Surface 5: the `dora` command
# --------------------------------------------------------------------------

CLI_SURFACE = "binaries/cli/cli-surface.txt"


def parse_cli_surface(text: str) -> dict[str, set[str]]:
    """`command -> {items}` from the checked-in snapshot.

    The snapshot is generated from clap by `dora-cli`'s `cli_surface` test, so
    comparing two snapshots needs no build on either side -- which is the only
    reason the `dora` command can be checked in a compile-free gate at all.
    """
    surface: dict[str, set[str]] = {}
    for line in text.splitlines():
        line = line.split("#")[0].strip()
        if not line:
            continue
        command, _, item = line.partition("|")
        surface.setdefault(command.strip(), set())
        if item.strip():
            surface[command.strip()].add(item.strip())
    return surface


def check_cli(root: Path, baseline: Baseline) -> SurfaceResult:
    result = SurfaceResult(name="`dora` command surface")
    current = root / CLI_SURFACE
    old_text, used_ref = baseline.read_or_fallback(CLI_SURFACE)
    if not current.exists():
        # Deleting the snapshot must not be the way to silence this check.
        if old_text is not None:
            result.findings.append(
                Finding(BREAK, f"{CLI_SURFACE} was deleted, so the surface is unchecked")
            )
            return result
        result.skipped = f"{CLI_SURFACE} has not been generated"
        return result
    if old_text is None:
        result.skipped = (
            "no baseline snapshot (the first release carrying one becomes the "
            "baseline for every release after it)"
        )
        return result
    old = parse_cli_surface(old_text)
    new = parse_cli_surface(current.read_text())

    for command, items in old.items():
        if command not in new:
            result.findings.append(Finding(BREAK, f"`{command}` removed"))
            continue
        for item in sorted(items - new[command]):
            result.findings.append(Finding(BREAK, f"`{command}`: {item} removed"))
        for item in sorted(new[command] - items):
            result.findings.append(Finding(WARN, f"`{command}`: {item} added"))
    result.summary = f"{len(new)} commands"
    if used_ref != baseline.ref:
        result.summary += f", vs {used_ref[:12]} ({baseline.ref} predates the snapshot)"
    return result


# --------------------------------------------------------------------------
# Surface 6: the Python support floor
# --------------------------------------------------------------------------

# The two published wheels. `scripts/release/verify-release.py` keeps the same
# pair as PYPI_MANIFESTS -- a third wheel has to be added in both places.
PYPROJECTS = ["apis/python/node/pyproject.toml", "apis/python/cli/pyproject.toml"]


def python_floor(text: str) -> str | None:
    match = re.search(r"""requires-python\s*=\s*["']([^"']+)["']""", text)
    return match.group(1) if match else None


def abi3_floor(text: str) -> tuple[int, int] | None:
    """The lowest `abi3-pyXY` tag in `text`, as (major, minor).

    Parsed digit by digit rather than through `version_tuple`, which reads the
    `3` in "abi3" and returns the same answer for every tag.
    """
    tags = [
        (int(m.group(1)), int(m.group(2)))
        for m in re.finditer(r"abi3-py(\d)(\d+)", text)
    ]
    return min(tags) if tags else None


def version_tuple(spec: str) -> tuple[int, ...]:
    match = re.search(r"(\d+)(?:\.(\d+))?", spec)
    if not match:
        return ()
    return tuple(int(g) for g in match.groups() if g is not None)


def check_python_floor(root: Path, baseline: Baseline) -> SurfaceResult:
    result = SurfaceResult(name="Python support floor")
    checked = []
    for rel in PYPROJECTS:
        old_text = baseline.read(rel)
        current = root / rel
        if old_text is None or not current.exists():
            continue
        old_spec = python_floor(old_text)
        new_spec = python_floor(current.read_text())
        if old_spec is None or new_spec is None:
            continue
        checked.append(new_spec)
        if version_tuple(new_spec) > version_tuple(old_spec):
            result.findings.append(
                Finding(
                    BREAK,
                    f"{rel}: requires-python raised {old_spec} -> {new_spec}; "
                    "pip refuses to install on interpreters dora used to support",
                )
            )

    # The abi3 tag is set on the workspace `pyo3` dependency and repeated in
    # the crates that name pyo3 directly, so scan every manifest rather than
    # guessing which one holds it -- the two wheel crates inherit it and
    # mention it only in comments. Compared per manifest, not as one lowest
    # floor across all of them: a raise in the workspace dependency is what
    # actually moves the wheels, and a single unraised crate elsewhere would
    # otherwise hide it.
    old_abi3 = grep_tracked(root, baseline.ref, "abi3-py")
    new_abi3 = grep_tracked(root, None, "abi3-py")
    floors = []
    for rel, old_line in sorted(old_abi3.items()):
        old_floor = abi3_floor(old_line)
        new_floor = abi3_floor(new_abi3.get(rel, ""))
        if not old_floor:
            continue
        if not new_floor:
            if rel in new_abi3 or (root / rel).exists():
                result.findings.append(
                    Finding(
                        WARN,
                        f"{rel}: the abi3 feature is gone, so its wheel is no "
                        "longer built for a stable ABI",
                    )
                )
            continue
        floors.append(new_floor)
        if new_floor > old_floor:
            result.findings.append(
                Finding(
                    BREAK,
                    f"{rel}: abi3 floor raised py{old_floor[0]}{old_floor[1]} -> "
                    f"py{new_floor[0]}{new_floor[1]}; wheels stop loading on "
                    "interpreters dora used to support",
                )
            )
    if floors:
        checked.append("abi3-py%d%d" % min(floors))

    if not checked:
        result.skipped = "no Python manifests in the baseline"
        return result
    result.summary = ", ".join(sorted(set(checked)))
    return result


# --------------------------------------------------------------------------
# The version guard
# --------------------------------------------------------------------------


def workspace_version(text: str) -> str | None:
    match = re.search(r"^version\s*=\s*\"([^\"]+)\"", text, flags=re.M)
    return match.group(1) if match else None


def check_version_guard(root: Path, baseline: Baseline) -> SurfaceResult:
    """Stop at a major version bump, which is the one thing this cannot judge.

    Every check here asks "does this still honour what the last release
    promised". A 2.0 changes the question: the promises are being withdrawn on
    purpose, so a report of breaking changes is the expected answer rather than
    a problem, and a gate that just went green or red would be misleading
    either way. The bump therefore has to be stated (`ALLOW_MAJOR_BUMP=1`),
    which puts a human on the decision.

    Related but separate: `cargo-semver-checks` would *also* fall silent on a
    major bump, concluding breakage is permitted. `semver.sh` passes
    `--release-type minor` so it does not. This guard is not a workaround for
    that -- do not remove either one on the strength of the other.
    """
    result = SurfaceResult(name="Workspace version")
    old_text = baseline.read("Cargo.toml")
    if old_text is None:
        result.skipped = "no baseline Cargo.toml"
        return result
    old = workspace_version(old_text)
    new = workspace_version((root / "Cargo.toml").read_text())
    if not old or not new:
        result.skipped = "could not read a workspace version"
        return result
    old_major = version_tuple(old)[:1]
    new_major = version_tuple(new)[:1]
    if os.environ.get("ALLOW_MAJOR_BUMP"):
        result.summary = f"{old} -> {new} (major bump allowed)"
        return result
    if new_major > old_major and old_major >= (1,):
        result.findings.append(
            Finding(
                BREAK,
                f"major version bump {old} -> {new}: a 2.0 withdraws the "
                "promises every other check here is measuring against, so "
                "they can no longer answer anything useful. Set "
                "ALLOW_MAJOR_BUMP=1 (in the job or the shell) to say the bump "
                "is deliberate.",
            )
        )
    result.summary = f"{old} -> {new}"
    return result


# --------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------

CHECKS = [
    check_version_guard,
    check_c_header,
    check_cxx_bridge,
    check_schema,
    check_wire_format,
    check_cli,
    check_python_floor,
]


def run(root: Path, ref: str, fallback: str | None = None) -> tuple[list[SurfaceResult], int]:
    baseline = Baseline(root, ref, fallback)
    results = []
    for check in CHECKS:
        try:
            results.append(check(root, baseline))
        except Exception as error:  # noqa: BLE001 - see below
            # A surface that cannot be parsed is unchecked, and an unchecked
            # frozen surface is a failure, not a traceback that also takes the
            # remaining checks down with it. Deleting the C header or renaming
            # the cxx bridge module both land here.
            results.append(
                SurfaceResult(
                    name=check.__name__.removeprefix("check_").replace("_", " "),
                    findings=[
                        Finding(
                            BREAK,
                            f"could not be checked ({type(error).__name__}: {error}); "
                            "treated as a failure because an unchecked surface "
                            "is an unguarded one",
                        )
                    ],
                )
            )
    return results, sum(len(r.breaks()) for r in results)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--baseline",
        help="git ref to compare against (default: the latest release tag)",
    )
    parser.add_argument("--root", default=".", help="repository root")
    parser.add_argument(
        "--fallback-baseline",
        help="ref to compare surfaces the baseline predates against "
        "(CI passes the PR base commit)",
    )
    parser.add_argument(
        "--print-baseline",
        action="store_true",
        help="print the resolved baseline ref and exit (used by the shell driver)",
    )
    parser.add_argument(
        "--json", action="store_true", help="emit findings as JSON instead of text"
    )
    args = parser.parse_args()

    root = Path(args.root).resolve()
    ref = args.baseline or os.environ.get("BREAKING_BASELINE") or resolve_baseline(root)
    if args.print_baseline:
        print(ref)
        return 0
    check = subprocess.run(
        ["git", "-C", str(root), "rev-parse", "--verify", f"{ref}^{{commit}}"],
        capture_output=True,
        text=True,
    )
    if check.returncode != 0:
        print(
            f"error: baseline ref `{ref}` not found. In CI this usually means "
            "tags were not fetched (actions/checkout needs fetch-depth: 0).",
            file=sys.stderr,
        )
        return 2

    results, break_count = run(
        root, ref, args.fallback_baseline or os.environ.get("BREAKING_FALLBACK_BASELINE")
    )

    if args.json:
        print(
            json.dumps(
                {
                    "baseline": ref,
                    "surfaces": [
                        {
                            "name": r.name,
                            "skipped": r.skipped,
                            "findings": [
                                {"level": f.level, "detail": f.detail} for f in r.findings
                            ],
                        }
                        for r in results
                    ],
                },
                indent=2,
            )
        )
        return 1 if break_count else 0

    print(f"Breaking-change gate -- baseline {ref}\n")
    for result in results:
        if result.skipped:
            status = f"skipped ({result.skipped})"
        elif result.breaks():
            status = f"BREAKING ({len(result.breaks())})"
        elif result.warns():
            status = f"ok, {len(result.warns())} additions"
        else:
            status = "ok"
        detail = f" [{result.summary}]" if result.summary and not result.skipped else ""
        print(f"  {result.name:.<46} {status}{detail}")
        for finding in result.breaks():
            print(f"      BREAK  {finding.detail}")
        for finding in result.warns():
            print(f"      note   {finding.detail}")

    print()
    if break_count:
        print(f"{break_count} breaking change(s) against {ref}.")
        print(
            "dora 1.x freezes these surfaces (docs/api-rust.md, "
            "'Stability scope at 1.0'). Either keep the old surface working "
            "alongside the new one, or take the change to a 2.0 discussion."
        )
        return 1
    print(f"No breaking changes against {ref}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
