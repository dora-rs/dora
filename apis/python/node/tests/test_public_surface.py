"""Pin the `dora` module surface that dora 1.0 freezes.

`docs/api-rust.md` puts `dora-node-api-python` in the covered tier and attaches
the guarantee to "the names importable from `dora`" rather than to a crates.io
API, because the crate ships as a wheel. Nothing enforced that: the surface
lived only in whatever the module happened to export, so a rename could reach
PyPI without anything failing first.

These tests are the enforcement. A deliberate addition means editing a set
below — which is the point, since it forces the choice of *which* set, and that
choice is the compatibility promise.
"""

import inspect

import dora

# Frozen by the 1.0 guarantee. Removing or renaming any of these needs a 2.0.
FROZEN_MODULE_NAMES = frozenset(
    {
        "DoraStatus",
        "Node",
    },
)

# Present, but explicitly outside the guarantee (`docs/api-rust.md`, "Shipped,
# but outside the guarantee"), so they may change in a minor. Optional rather
# than required: `start_runtime` is imported conditionally, and the `Ros2*`
# names come from the ros2-bridge feature.
EXEMPT_MODULE_NAMES = frozenset(
    {
        # Operator runtime + CLI entry points re-exported for convenience.
        "build",
        "run",
        "start_runtime",
        # ros2-bridge; the `ros2:` surface may change at any point.
        "Ros2Context",
        "Ros2Durability",
        "Ros2Liveliness",
        "Ros2Node",
        "Ros2NodeOptions",
        "Ros2Publisher",
        "Ros2QosPolicies",
        "Ros2Subscription",
        "Ros2Topic",
        # The native extension module. Every Python package exposes its
        # submodules as attributes; this is the language, not an API decision.
        "dora",
    },
)

# Public methods of the frozen classes. This is the half users actually call,
# so pinning only the class names would leave the real contract unguarded.
FROZEN_CLASS_MEMBERS = {
    "Node": frozenset(
        {
            "dataflow_descriptor",
            "dataflow_id",
            "drain",
            "is_empty",
            "is_restart",
            "log",
            "log_debug",
            "log_error",
            "log_info",
            "log_trace",
            "log_warn",
            "merge_external_events",
            "next",
            "node_config",
            "recv_async",
            "restart_count",
            "send_output",
            "send_output_raw",
            "send_service_request",
            "send_service_response",
            "timestamp",
            "try_recv",
        },
    ),
    "DoraStatus": frozenset({"CONTINUE", "STOP", "STOP_ALL"}),
}


def public_names(obj) -> set:
    """Names reachable by attribute access, minus the dunder/private ones."""
    return {name for name in dir(obj) if not name.startswith("_")}


def test_frozen_names_are_all_present():
    """Every name the 1.0 guarantee covers still resolves."""
    missing = FROZEN_MODULE_NAMES - public_names(dora)
    assert not missing, (
        f"{sorted(missing)} disappeared from `dora`. These are frozen for the "
        f"life of 1.x — removing or renaming one requires a 2.0."
    )


def test_no_unclassified_names_appear():
    """New public names must be deliberately placed in one bucket or the other.

    Catches accidental leaks — a stray `from enum import Enum` used to make
    `dora.Enum` importable, which 1.0 would have frozen.
    """
    unclassified = public_names(dora) - FROZEN_MODULE_NAMES - EXEMPT_MODULE_NAMES
    assert not unclassified, (
        f"`dora` exports {sorted(unclassified)}, which this test does not "
        f"classify. Add each to FROZEN_MODULE_NAMES (covered by the 1.0 "
        f"guarantee) or EXEMPT_MODULE_NAMES (may change in a minor) — or stop "
        f"exporting it, if it leaked in by accident."
    )


def test_frozen_class_members_are_all_present():
    """The methods on the frozen classes are part of the same promise."""
    for cls_name, expected in FROZEN_CLASS_MEMBERS.items():
        cls = getattr(dora, cls_name)
        missing = expected - public_names(cls)
        assert not missing, (
            f"`dora.{cls_name}` lost {sorted(missing)}. These are frozen for "
            f"the life of 1.x — removing or renaming one requires a 2.0."
        )


def test_enum_base_class_is_not_exported():
    """Regression: `DoraStatus`'s base class must not leak into the namespace."""
    assert not hasattr(dora, "Enum"), (
        "`from dora import Enum` works again — import it as `_Enum` in "
        "`dora/__init__.py` so it stays out of the public surface."
    )


def test_every_frozen_name_is_documented_as_a_class_or_callable():
    """A frozen name that is neither is almost certainly an accidental export."""
    for name in sorted(FROZEN_MODULE_NAMES):
        value = getattr(dora, name)
        assert inspect.isclass(value) or callable(value), (
            f"`dora.{name}` is a {type(value).__name__}, not a class or "
            f"callable; frozen surface should be one or the other."
        )
