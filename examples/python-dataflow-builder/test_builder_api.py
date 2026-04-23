"""Contract test for the Python DataflowBuilder API (#1654).

Exercises `dora.builder.DataflowBuilder` end-to-end at the YAML-generation
layer:

- construct a two-node dataflow programmatically,
- call `.to_yaml(path)`,
- parse the generated file,
- assert the structural contract users depend on
  (node IDs, ports, wiring between source and sink, env passthrough).

Deliberately does NOT call `build()` or `run()` — the sibling
`simple_example.py` exercises those paths, but it can't run in CI today
because its `build:` commands pull hub packages (`opencv-video-capture`,
`dora-yolo`, `opencv-plot`) which transitively install the PyPI
`dora-rs` 0.5.0 and clobber the locally-installed workspace version.
Covering the builder API without those packages restores the
coverage lane (#1654) until the hub + 1.0 PyPI story settles.

Run locally:

    uv pip install -e apis/python/node  # if not already installed
    uv pip install PyYAML
    python examples/python-dataflow-builder/test_builder_api.py
"""

from __future__ import annotations

import os
import sys
import tempfile

import yaml
from dora.builder import DataflowBuilder


def main() -> int:
    dataflow = DataflowBuilder(name="builder-contract-test")

    source = dataflow.add_node(
        id="source",
        path="source.py",
        env={"SOURCE_MODE": "test"},
    )
    source.add_input("tick", "dora/timer/millis/100")
    value_out = source.add_output("value")

    sink = dataflow.add_node(id="sink", path="sink.py")
    sink.add_input("value", value_out)

    # Write to a tempfile so the test cleans up after itself.
    fd, path = tempfile.mkstemp(suffix=".yml", prefix="builder-contract-")
    os.close(fd)
    try:
        dataflow.to_yaml(path)
        with open(path) as f:
            generated = yaml.safe_load(f)
    finally:
        os.unlink(path)

    # ---- Structural contract ----
    assert "nodes" in generated, f"no `nodes` key in generated YAML: {generated!r}"
    by_id = {n["id"]: n for n in generated["nodes"]}
    assert set(by_id) == {"source", "sink"}, (
        f"expected exactly source/sink, got {sorted(by_id)}"
    )

    src = by_id["source"]
    assert src["path"] == "source.py"
    # dora accepts `inputs` as a map of `input_id → producer_spec`.
    assert src["inputs"]["tick"] == "dora/timer/millis/100", (
        f"source tick wiring wrong: {src['inputs']}"
    )
    assert "value" in src["outputs"], f"source outputs: {src['outputs']}"
    # env passthrough must survive YAML round-trip intact.
    assert src.get("env", {}).get("SOURCE_MODE") == "test", (
        f"source env not preserved: {src.get('env')}"
    )

    snk = by_id["sink"]
    assert snk["path"] == "sink.py"
    # Core builder contract: the output handle returned by
    # `source.add_output("value")` must be resolved to the
    # "source/value" wire format in the generated YAML. A regression
    # that left the handle object un-rendered or dropped the prefix
    # would fail here.
    assert snk["inputs"]["value"] == "source/value", (
        f"sink input not wired to source/value, got {snk['inputs']!r}"
    )

    print(
        "python-dataflow-builder: SUCCESS - generated YAML has expected structure"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
