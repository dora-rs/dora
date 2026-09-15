"""Tests for dora.testing.MockNode."""

import asyncio
import datetime

import pyarrow as pa
import pytest

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


def test_mock_node_stop_event_has_all_inputs_closed_id():
    """The automatic STOP event carries the same id the real node emits."""
    node = MockNode([])
    event = next(node)
    assert event["type"] == "STOP"
    assert event["id"] == "ALL_INPUTS_CLOSED"


def test_mock_node_no_double_stop_when_raw_stop_supplied():
    """A caller-supplied STOP suppresses the automatic one."""
    node = MockNode([{"type": "STOP", "id": "MANUAL"}])
    assert list(node) == [{"type": "STOP", "id": "MANUAL"}]


def test_mock_node_accepts_raw_events():
    """Raw event dicts are forwarded as-is and can be mixed with tuples."""
    node = MockNode(
        [
            {"type": "PARAM_UPDATE", "id": "gain", "value": 2.0},
            ("tick", pa.array([0])),
        ]
    )
    events = list(node)
    assert events[0] == {"type": "PARAM_UPDATE", "id": "gain", "value": 2.0}
    assert events[1]["type"] == "INPUT"
    assert events[1]["id"] == "tick"
    assert events[2] == {"type": "STOP", "id": "ALL_INPUTS_CLOSED"}


def test_mock_node_raw_event_dict_is_copied():
    """The constructor must not retain a reference to the caller's dict."""
    raw = {"type": "ERROR", "error": "boom"}
    node = MockNode([raw])
    raw["error"] = "swapped"
    assert next(node) == {"type": "ERROR", "error": "boom"}


def test_mock_node_drain_returns_remaining_events():
    node = MockNode([("a", pa.array([1])), ("b", pa.array([2]))])
    events = node.drain()
    assert [event["type"] for event in events] == ["INPUT", "INPUT", "STOP"]
    assert node.drain() == []
    assert node.is_empty()
    assert node.next() is None


def test_mock_node_try_recv_returns_next_or_none():
    node = MockNode([("tick", pa.array([0]))])
    event = node.try_recv()
    assert event["type"] == "INPUT"
    assert node.try_recv()["type"] == "STOP"
    assert node.try_recv() is None


def test_mock_node_logs_captured_structured():
    node = MockNode([])
    node.log("info", "hello", target="my_node", fields={"k": "v"})
    node.log_error("boom")
    node.log_warn("careful")
    node.log_info("notice")
    node.log_debug("verbose")
    node.log_trace("noise")
    assert node.logs[0] == {
        "level": "info",
        "message": "hello",
        "target": "my_node",
        "fields": {"k": "v"},
    }
    assert [entry["level"] for entry in node.logs] == [
        "info",
        "error",
        "warn",
        "info",
        "debug",
        "trace",
    ]


def test_mock_node_config_getters():
    node = MockNode(
        [],
        dataflow_id="demo",
        dataflow_descriptor={"nodes": []},
        node_config={"param": 1},
        is_restart=True,
        restart_count=2,
    )
    assert node.dataflow_id() == "demo"
    assert node.dataflow_descriptor() == {"nodes": []}
    assert node.node_config() == {"param": 1}
    assert node.is_restart() is True
    assert node.restart_count() == 2


def test_mock_node_config_getters_defaults():
    node = MockNode([])
    assert node.dataflow_id() == ""
    assert node.dataflow_descriptor() == {}
    assert node.node_config() == {}
    assert node.is_restart() is False
    assert node.restart_count() == 0


def test_mock_node_timestamp_is_utc():
    node = MockNode([])
    assert node.timestamp().tzinfo is datetime.timezone.utc


def test_mock_node_service_request_delivers_queued_response():
    """A queued service response arrives back ahead of the STOP event."""
    node = MockNode([], service_responses={"reply": [pa.array([7])]})
    request_id = node.send_service_request("reply", {"cmd": "ping"})
    event = node.next()
    assert event["type"] == "INPUT"
    assert event["id"] == "reply"
    assert event["value"].to_pylist() == [7]
    assert event["metadata"]["request_id"] == request_id
    assert node.next()["type"] == "STOP"


def test_mock_node_service_request_records_request():
    """send_service_request records the request and returns a fresh id."""
    node = MockNode([])
    request_id = node.send_service_request("req", pa.array([1]), metadata={"k": "v"})
    assert len(node.service_requests) == 1
    recorded = node.service_requests[0]
    assert recorded["output_id"] == "req"
    assert recorded["metadata"] == {"k": "v"}
    assert recorded["request_id"] == request_id


def test_mock_node_service_response_records_response():
    """send_service_response records the response and extracts request_id."""
    node = MockNode([])
    data = pa.array([1])
    node.send_service_response("resp", data, {"request_id": "r1"})
    assert len(node.service_responses_sent) == 1
    recorded = node.service_responses_sent[0]
    assert recorded["output_id"] == "resp"
    assert recorded["data"] is data
    assert recorded["metadata"] == {"request_id": "r1"}
    assert recorded["request_id"] == "r1"


def test_mock_node_runtime_only_methods_raise_not_implemented():
    """Daemon-only methods fail loudly instead of disappearing."""
    node = MockNode([])
    with pytest.raises(NotImplementedError):
        node.send_output_raw("out", 8)
    with pytest.raises(NotImplementedError):
        node.merge_external_events(None)
