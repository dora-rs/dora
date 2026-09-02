#!/usr/bin/env python3
"""Tests for the breaking-change differ.

A gate that only ever reports "clean" is indistinguishable from a gate that
does not work -- and this one is text-driven, so a refactor upstream (a header
reformat, a serde attribute, a schema key rename) could quietly turn any
extractor into a no-op. Every check below feeds a *known* break through the
real differ and asserts it is caught.

Run: python3 -m unittest discover -s scripts/qa/tests
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from breaking_changes import (  # noqa: E402
    BREAK,
    WARN,
    compare_ordered,
    diff_schema,
    flatten_schema,
    parse_c_header,
    parse_cli_surface,
    parse_cxx_bridge,
    parse_wire_types,
    python_floor,
    tag_sort_key,
    version_tuple,
)

C_HEADER = """
#include <stddef.h>

/* a comment mentioning free_dora_context(void *x); */
void *init_dora_context_from_env();
void free_dora_context(void *dora_context);
int dora_send_output(void *ctx, const char *id, size_t id_len);

enum DoraEventType
{
    DoraEventType_Stop,
    DoraEventType_Input,
};
enum DoraEventType read_dora_event_type(void *dora_event);
"""

CXX_BRIDGE = """
#[cxx::bridge]
mod ffi {
    struct DoraNode {
        events: Box<Events>,
    }

    pub enum DoraEventType {
        Stop,
        Input,
    }

    extern "Rust" {
        type Events;
        fn init_dora_node() -> Result<DoraNode>;
        fn next(self: &mut Events) -> Box<DoraEvent>;
        fn next(self: &mut CombinedEvents) -> CombinedEvent;
    }
}
"""

WIRE = {
    "daemon_to_node.rs": """
#[derive(Debug, Serialize, Deserialize)]
pub enum NodeEvent {
    Stop,
    Input { id: DataId, data: Vec<u8> },
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Metadata {
    pub metadata_version: u16,
    pub timestamp: Timestamp,
}

#[derive(Debug, Serialize)]
pub struct DataId(String);
""",
}


def find(findings, needle, level=BREAK):
    return [f for f in findings if f.level == level and needle in f.detail]


class CHeaderTest(unittest.TestCase):
    def setUp(self):
        self.parsed = parse_c_header(C_HEADER)

    def test_extracts_every_declaration(self):
        self.assertEqual(
            sorted(self.parsed["functions"]),
            [
                "dora_send_output",
                "free_dora_context",
                "init_dora_context_from_env",
                "read_dora_event_type",
            ],
        )
        self.assertEqual(
            self.parsed["enums"]["DoraEventType"],
            ["DoraEventType_Stop", "DoraEventType_Input"],
        )

    def test_comments_are_not_declarations(self):
        # The comment above mentions a call that must not be parsed as one.
        self.assertEqual(len(self.parsed["functions"]), 4)

    def test_parameter_rename_is_not_a_signature_change(self):
        renamed = C_HEADER.replace("void *dora_context", "void *context")
        self.assertEqual(
            parse_c_header(renamed)["functions"]["free_dora_context"],
            self.parsed["functions"]["free_dora_context"],
        )

    def test_parameter_type_change_is_a_signature_change(self):
        retyped = C_HEADER.replace("size_t id_len", "int id_len")
        self.assertNotEqual(
            parse_c_header(retyped)["functions"]["dora_send_output"],
            self.parsed["functions"]["dora_send_output"],
        )


class CxxBridgeTest(unittest.TestCase):
    def setUp(self):
        self.parsed = parse_cxx_bridge(CXX_BRIDGE)

    def test_extracts_items(self):
        self.assertEqual(self.parsed["structs"]["DoraNode"], ["events: Box<Events>"])
        self.assertEqual(self.parsed["enums"]["DoraEventType"], ["Stop", "Input"])

    def test_same_named_fns_are_kept_apart_by_receiver(self):
        self.assertIn("Events::next", self.parsed["functions"])
        self.assertIn("CombinedEvents::next", self.parsed["functions"])


class WireFormatTest(unittest.TestCase):
    def setUp(self):
        self.parsed = parse_wire_types(WIRE)

    def test_extracts_braced_and_tuple_types(self):
        self.assertEqual(
            self.parsed["daemon_to_node::NodeEvent"]["members"],
            ["Stop", "Input { id: DataId, data: Vec<u8> }"],
        )
        self.assertEqual(
            self.parsed["daemon_to_node::DataId"]["members"], ["0: String"]
        )

    def test_serde_rename_is_part_of_the_member(self):
        renamed = {
            "m.rs": '#[derive(Serialize)]\npub struct S {\n'
            '    #[serde(rename = "ts")]\n    pub timestamp: u64,\n}\n'
        }
        self.assertEqual(
            parse_wire_types(renamed)["m::S"]["members"],
            ['timestamp: u64 [serde: rename = "ts"]'],
        )

    def test_default_attribute_is_not_part_of_the_member(self):
        # `#[serde(default)]` only widens decoding; treating it as a change
        # would fail the gate on a strictly compatible edit.
        widened = {
            "m.rs": "#[derive(Serialize)]\npub struct S {\n"
            "    #[serde(default)]\n    pub timestamp: u64,\n}\n"
        }
        self.assertEqual(
            parse_wire_types(widened)["m::S"]["members"], ["timestamp: u64"]
        )

    def test_cfg_test_fixtures_are_not_the_wire_protocol(self):
        # bulk_bytes.rs defines six serde types inside `#[cfg(test)]`. Counting
        # them would report deleting a test fixture as a protocol break.
        source = {
            "m.rs": "#[derive(Serialize)]\npub struct Real { pub a: u8 }\n"
            "#[cfg(test)]\nmod tests {\n"
            "    #[derive(Serialize)]\n    pub struct Fixture { pub b: u8 }\n}\n"
        }
        parsed = parse_wire_types(source)
        self.assertIn("m::Real", parsed)
        self.assertNotIn("m::Fixture", parsed)

    def test_non_serde_types_are_ignored(self):
        self.assertNotIn("m::S", parse_wire_types({"m.rs": "pub struct S { a: u8 }"}))


class SchemaTest(unittest.TestCase):
    BASE = {
        "$defs": {
            "Node": {
                "type": "object",
                "properties": {"id": {"type": "string"}, "env": {"type": "object"}},
                "required": ["id"],
            },
            "Level": {"enum": ["debug", "info"]},
        }
    }

    def diff(self, new):
        return diff_schema(flatten_schema(self.BASE), flatten_schema(new))

    def test_clean(self):
        self.assertEqual(self.diff(self.BASE), [])

    def test_a_removed_property_is_reported_once(self):
        new = {"$defs": {"Node": {"type": "object", "properties": {"id": {}}}}}
        base = {
            "$defs": {"Node": {"type": "object", "properties": {"id": {}, "env": {}}}}
        }
        findings = diff_schema(flatten_schema(base), flatten_schema(new))
        self.assertEqual(len(findings), 1, [f.detail for f in findings])

    def test_removed_property(self):
        new = {"$defs": {**self.BASE["$defs"]}}
        new["$defs"]["Node"] = {
            "type": "object",
            "properties": {"id": {"type": "string"}},
            "required": ["id"],
        }
        self.assertTrue(find(self.diff(new), "property `env` removed"))

    def test_newly_required_property(self):
        new = {"$defs": {**self.BASE["$defs"]}}
        new["$defs"]["Node"] = {**self.BASE["$defs"]["Node"], "required": ["id", "env"]}
        self.assertTrue(find(self.diff(new), "`env` is now required"))

    def test_removed_enum_value(self):
        new = {"$defs": {**self.BASE["$defs"], "Level": {"enum": ["info"]}}}
        self.assertTrue(find(self.diff(new), "enum value"))

    def test_added_property_is_not_a_break(self):
        new = {"$defs": {**self.BASE["$defs"]}}
        new["$defs"]["Node"] = {
            "type": "object",
            "properties": {
                "id": {"type": "string"},
                "env": {"type": "object"},
                "cpu": {"type": "string"},
            },
            "required": ["id"],
        }
        self.assertEqual(self.diff(new), [])

    def test_closing_additional_properties(self):
        new = {"$defs": {**self.BASE["$defs"]}}
        new["$defs"]["Node"] = {
            **self.BASE["$defs"]["Node"],
            "additionalProperties": False,
        }
        self.assertTrue(find(self.diff(new), "additional properties are now rejected"))


class OrderedComparisonTest(unittest.TestCase):
    """Positional encodings: what counts as a break is about *position*."""

    def compare(self, old, new, added_level=BREAK):
        return compare_ordered(
            "field", "T", old, new, added_level=added_level, reorder_note="n"
        )

    def test_append_is_allowed_when_additions_are_allowed(self):
        # An enum variant appended at the end only breaks new -> old, which is
        # how the protocol is meant to grow; a struct field passes BREAK here.
        findings = self.compare(["a", "b"], ["a", "b", "c"], WARN)
        self.assertEqual([f.level for f in findings], [WARN])

    def test_insert_in_the_middle_shifts_later_members(self):
        levels = self.compare(["a", "b"], ["a", "c", "b"], WARN)
        self.assertTrue(find(levels, "inserted before the end"))

    def test_reorder_is_a_break(self):
        self.assertTrue(find(self.compare(["a", "b"], ["b", "a"]), "order changed"))

    def test_removal_is_a_break(self):
        self.assertTrue(find(self.compare(["a", "b"], ["a"]), "`b` removed"))

    def test_identical_is_clean(self):
        self.assertEqual(self.compare(["a", "b"], ["a", "b"]), [])


class BaselineTagTest(unittest.TestCase):
    def test_a_prerelease_sorts_below_its_release(self):
        # git's own version sort gets this backwards without configuration,
        # which would pick an rc as the baseline for the release after it.
        tags = ["v1.0.0", "v1.0.0-rc.5", "v0.5.0", "v1.0.1"]
        self.assertEqual(max(tags, key=tag_sort_key), "v1.0.1")
        self.assertLess(tag_sort_key("v1.0.0-rc.5"), tag_sort_key("v1.0.0"))

    def test_non_version_tags_are_rejected(self):
        self.assertFalse(tag_sort_key("v0.x-final"))


class CliSurfaceTest(unittest.TestCase):
    SNAPSHOT = """
# comment
dora build | --uv
dora build | <dataflow>
dora start
"""

    def test_parse(self):
        parsed = parse_cli_surface(self.SNAPSHOT)
        self.assertEqual(parsed["dora build"], {"--uv", "<dataflow>"})
        self.assertEqual(parsed["dora start"], set())


class PythonFloorTest(unittest.TestCase):
    def test_floor_ordering(self):
        self.assertGreater(version_tuple(">=3.12"), version_tuple(">=3.11"))
        self.assertEqual(version_tuple(">=3.11"), version_tuple(">= 3.11"))
        self.assertEqual(python_floor('requires-python = ">=3.11"'), ">=3.11")


if __name__ == "__main__":
    unittest.main()
