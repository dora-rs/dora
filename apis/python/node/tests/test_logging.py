"""Regression tests for the logging bridge installed by the native Node API."""

import json
import os
import subprocess
import sys

import pytest


@pytest.mark.parametrize(
    "container", ["list", "tuple", "iterator", "none", "iterator_noop"]
)
def test_basic_config_preserves_handler_iterables(tmp_path, container):
    """Accept standard handler iterables without mutating the caller's list."""
    inputs = tmp_path / "inputs.json"
    inputs.write_text(
        json.dumps(
            {
                "id": "logging-test",
                "events": [{"time_offset_secs": 0, "type": "Stop"}],
            }
        )
    )
    env = os.environ.copy()
    env.pop("DORA_NODE_CONFIG", None)
    env["DORA_TEST_WITH_INPUTS"] = str(inputs)
    env["DORA_TEST_WRITE_OUTPUTS_TO"] = str(tmp_path / "outputs.jsonl")

    # Node installs a process-wide logging bridge. A fresh interpreter exercises
    # the real constructor without affecting pytest's handlers or other tests.
    code = """
import logging
import sys

from dora import Node

node = Node()
assert node.next()["type"] == "STOP"
handler = logging.NullHandler()
kind = sys.argv[1]
handlers = {
    "list": [handler],
    "tuple": (handler,),
    "iterator": iter([handler]),
    "none": None,
    "iterator_noop": iter([handler]),
}[kind]
if kind == "iterator_noop":
    logging.basicConfig(force=True, level=logging.INFO)
    configured = list(logging.root.handlers)
    logging.basicConfig(handlers=handlers)
    assert logging.root.handlers == configured
for _ in range(2 if kind == "list" else 1):
    logging.basicConfig(handlers=handlers, force=True, level=logging.INFO)
    attached = logging.root.handlers
    assert sum(isinstance(item, logging.HostHandler) for item in attached) == 1
    assert len(attached) == (1 if kind == "none" else 2)
    if kind != "none":
        assert attached[0] is handler
    if kind == "list":
        assert handlers == [handler], "basicConfig mutated the caller's list"
logging.info("logging bridge handler iterable regression")
"""
    result = subprocess.run(
        [sys.executable, "-c", code, container],
        env=env,
        cwd=tmp_path,
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
