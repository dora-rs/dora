"""Tests for dora.testing.MockNode."""

import asyncio

import pyarrow as pa

from dora.testing import MockNode


def test_mock_node_iterates_inputs_then_stop():
    node = MockNode([("tick", pa.array([1])), ("tick", pa.array([2]))])
    events = list(node)
    assert len(events) == 3
    assert events[0]["type"] == "INPUT"
    assert events[0]["id"] == "tick"
    assert events[0]["value"].to_pylist() == [1]
    assert events[1]["type"] == "INPUT"
    assert events[1]["value"].to_pylist() == [2]
    assert events[2]["type"] == "STOP"


def test_mock_node_captures_outputs():
    node = MockNode([("data", pa.array([10]))])
    for event in node:
        if event["type"] == "INPUT":
            node.send_output("result", pa.array([event["value"].to_pylist()[0] * 2]))
    assert "result" in node.outputs
    assert len(node.outputs["result"]) == 1
    assert node.outputs["result"][0].to_pylist() == [20]


def test_mock_node_multiple_outputs():
    node = MockNode([("a", pa.array([1])), ("b", pa.array([2]))])
    for event in node:
        if event["type"] == "INPUT":
            node.send_output("out", pa.array([event["value"].to_pylist()[0]]))
    assert len(node.outputs["out"]) == 2


def test_mock_node_empty_inputs():
    node = MockNode([])
    events = list(node)
    assert len(events) == 1
    assert events[0]["type"] == "STOP"


def test_mock_node_next_method():
    node = MockNode([("tick", pa.array([0]))])
    event = node.next()
    assert event["type"] == "INPUT"
    event = node.next()
    assert event["type"] == "STOP"
    event = node.next()
    assert event is None


def test_mock_node_metadata():
    node = MockNode(
        [("tick", pa.array([0]))],
        metadata={"request_id": "abc123"},
    )
    event = next(node)
    assert event["metadata"]["request_id"] == "abc123"


def test_mock_node_metadata_not_shared():
    node = MockNode(
        [("a", pa.array([1])), ("b", pa.array([2]))],
        metadata={"key": "original"},
    )
    first = next(node)
    first["metadata"]["key"] = "mutated"
    second = next(node)
    assert second["metadata"]["key"] == "original"


def test_mock_node_recv_async():
    async def run():
        node = MockNode([("tick", pa.array([0]))])
        event = await node.recv_async()
        assert event["type"] == "INPUT"
        event = await node.recv_async()
        assert event["type"] == "STOP"
        event = await node.recv_async()
        assert event is None

    asyncio.run(run())


def test_mock_node_recv_async_loop():
    """while/recv_async loop — the canonical dora async node pattern.  Timeout guards against deadlock."""

    async def run():
        node = MockNode([("a", pa.array([10])), ("b", pa.array([20]))])
        seen_inputs = []
        while True:
            event = await asyncio.wait_for(node.recv_async(), timeout=1.0)
            if event is None or event["type"] == "STOP":
                break
            seen_inputs.append(event["id"])
        assert seen_inputs == ["a", "b"]

    asyncio.run(run())


def test_mock_node_send_output_in_async_context():
    """send_output called from an async coroutine completes synchronously."""

    async def run():
        node = MockNode([("tick", pa.array([0]))])
        while True:
            event = await node.recv_async()
            if event is None or event["type"] == "STOP":
                break
            if event["type"] == "INPUT":
                node.send_output("result", pa.array([42]))
                await asyncio.sleep(0)  # yield to let other coroutines run
        assert node.outputs["result"][0].to_pylist() == [42]

    asyncio.run(run())
