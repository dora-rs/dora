"""Test utilities for dora Python nodes.

Provides MockNode, a drop-in replacement for dora.Node that works
without a running daemon. Use it to unit test node logic with
deterministic inputs and captured outputs.

Usage:
    from dora.testing import MockNode
    import pyarrow as pa

    def test_my_node():
        node = MockNode([("tick", pa.array([0]))])
        for event in node:
            if event["type"] == "INPUT":
                node.send_output("result", pa.array([42]))
        assert node.outputs["result"][0].to_pylist() == [42]
"""

from __future__ import annotations

import datetime
import uuid
from collections import deque
from typing import Any

import pyarrow as pa

__all__ = ["MockNode"]

# The id attached to the automatically appended STOP event, mirroring the
# reason the real node stops when every input closes.
_ALL_INPUTS_CLOSED = "ALL_INPUTS_CLOSED"


class MockNode:
    """Drop-in replacement for dora.Node usable in unit tests.

    Every method frozen on dora.Node is present, so node code written
    against the real API runs unchanged against the mock. Methods that
    need a live runtime explicitly raise NotImplementedError instead of
    crashing with an AttributeError.

    Args:
        inputs: List of (input_id, data) tuples or raw event dicts. Tuples
            become INPUT events with an empty metadata dict; dicts are
            forwarded as-is, letting tests inject INPUT_CLOSED, PARAM_UPDATE,
            ERROR, and so on. A STOP event is appended unless one is already
            present.
        metadata: Optional dict of metadata to attach to each INPUT event
            generated from a tuple. Each event gets its own copy.
        dataflow_descriptor: Value returned by dataflow_descriptor().
            Defaults to an empty dict.
        dataflow_id: Value returned by dataflow_id(). Defaults to "".
        node_config: Value returned by node_config(). Defaults to an empty
            dict.
        is_restart: Value returned by is_restart(). Defaults to False.
        restart_count: Value returned by restart_count(). Defaults to 0.
        service_responses: Output id to list of payloads. Each
            send_service_request call pops one and delivers it back as an
            INPUT event carrying the generated request_id, letting tests
            drive the request/reply pattern deterministically.

    """

    def __init__(
        self,
        inputs: list[tuple[str, Any]] | list[dict[str, Any]],
        metadata: dict[str, Any] | None = None,
        *,
        dataflow_descriptor: dict[str, Any] | None = None,
        dataflow_id: str | None = None,
        node_config: dict[str, Any] | None = None,
        is_restart: bool = False,
        restart_count: int = 0,
        service_responses: dict[str, list[Any]] | None = None,
    ):
        events: deque[dict[str, Any]] = deque()
        for item in inputs:
            if isinstance(item, dict):
                events.append(dict(item))
            else:
                input_id, data = item
                events.append(
                    {
                        "type": "INPUT",
                        "id": input_id,
                        "value": data,
                        "metadata": dict(metadata) if metadata else {},
                    }
                )
        if not any(event.get("type") == "STOP" for event in events):
            events.append({"type": "STOP", "id": _ALL_INPUTS_CLOSED})
        self._events = events
        self._dataflow_descriptor = dataflow_descriptor or {}
        self._dataflow_id = dataflow_id or ""
        self._node_config = node_config or {}
        self._is_restart = is_restart
        self._restart_count = restart_count
        self._service_responses = {
            output_id: deque(responses)
            for output_id, responses in (service_responses or {}).items()
        }
        self.outputs: dict[str, list[Any]] = {}
        self.logs: list[dict[str, Any]] = []
        self.service_requests: list[dict[str, Any]] = []
        self.service_responses_sent: list[dict[str, Any]] = []

    def __iter__(self) -> MockNode:
        return self

    def __next__(self) -> dict[str, Any]:
        try:
            return self._events.popleft()
        except IndexError as exc:
            raise StopIteration from exc

    def next(self, timeout: float | None = None) -> dict[str, Any] | None:
        """Return the next event, or None if exhausted.

        Args:
            timeout: Ignored. Accepted for API compatibility with dora.Node.
        """
        try:
            return self._events.popleft()
        except IndexError:
            return None

    async def recv_async(self, timeout: float | None = None) -> dict[str, Any] | None:
        """Async version of next(). Returns immediately.

        Args:
            timeout: Ignored. Accepted for API compatibility with dora.Node.
        """
        return self.next(timeout=timeout)

    def try_recv(self) -> dict[str, Any] | None:
        """Return the next buffered event without blocking, or None.

        The real node returns immediately when its queue is empty. The
        deterministic mock never waits, so this behaves like next().
        """
        return self.next()

    def drain(self) -> list[dict[str, Any]]:
        """Return all currently buffered events and empty the stream."""
        remaining = list(self._events)
        self._events.clear()
        return remaining

    def is_empty(self) -> bool:
        """Return True while no events are buffered."""
        return len(self._events) == 0

    def send_output(
        self,
        output_id: str,
        data: Any,
        metadata: dict[str, Any] | None = None,
    ) -> None:
        """Capture an output for later assertion."""
        self.outputs.setdefault(output_id, []).append(data)

    def send_output_raw(
        self,
        output_id: str,
        data_length: int,
        metadata: dict[str, Any] | None = None,
    ) -> None:
        """Raise NotImplementedError; requires the native zero-copy runtime.

        The real method hands back a SampleHandler backed by the Py3.11
        native sender, which a pure-Python mock cannot provide. Use
        send_output() in unit tests.
        """
        raise NotImplementedError(
            "MockNode cannot provide the zero-copy SampleHandler that "
            "send_output_raw() returns; it needs the native dora runtime. "
            "Use send_output() in unit tests."
        )

    def send_service_request(
        self,
        output_id: str,
        data: Any,
        metadata: dict[str, Any] | None = None,
    ) -> str:
        """Send a service request and return its generated request_id.

        Records the request on service_requests for later assertion. When
        service_responses provided a queue for this output at construction,
        the next response is delivered back as an INPUT event whose metadata
        carries the request_id, mirroring the reply arriving on the response
        port ahead of the terminal STOP.
        """
        request_id = str(uuid.uuid4())
        self.service_requests.append(
            {
                "output_id": output_id,
                "data": data,
                "metadata": dict(metadata) if metadata else {},
                "request_id": request_id,
            }
        )
        responses = self._service_responses.get(output_id)
        if responses:
            self._enqueue_reply(
                {
                    "type": "INPUT",
                    "id": output_id,
                    "value": responses.popleft(),
                    "metadata": {"request_id": request_id},
                }
            )
        return request_id

    def send_service_response(
        self, output_id: str, data: Any, metadata: dict[str, Any]
    ) -> None:
        """Record a service response for later assertion.

        Mirrors the real signature: metadata is required and normally carries
        the request_id from the incoming request.
        """
        self.service_responses_sent.append(
            {
                "output_id": output_id,
                "data": data,
                "metadata": dict(metadata),
                "request_id": metadata.get("request_id"),
            }
        )

    def log(
        self,
        level: str,
        message: str,
        target: str | None = None,
        fields: dict[str, str] | None = None,
    ) -> None:
        """Record a structured log entry on self.logs.

        The real node emits a JSONL line that the daemon parses. A mock has
        no daemon, so the entry is captured instead and can be asserted on.
        """
        self.logs.append(
            {
                "level": level,
                "message": message,
                "target": target,
                "fields": dict(fields) if fields else {},
            }
        )

    def log_error(self, message: str) -> None:
        """Record an error-level log entry."""
        self.log("error", message)

    def log_warn(self, message: str) -> None:
        """Record a warn-level log entry."""
        self.log("warn", message)

    def log_info(self, message: str) -> None:
        """Record an info-level log entry."""
        self.log("info", message)

    def log_debug(self, message: str) -> None:
        """Record a debug-level log entry."""
        self.log("debug", message)

    def log_trace(self, message: str) -> None:
        """Record a trace-level log entry."""
        self.log("trace", message)

    def dataflow_descriptor(self) -> dict[str, Any]:
        """Return the descriptor passed at construction, or an empty dict."""
        return dict(self._dataflow_descriptor)

    def dataflow_id(self) -> str:
        """Return the dataflow id passed at construction, or an empty string."""
        return self._dataflow_id

    def node_config(self) -> dict[str, Any]:
        """Return the node configuration passed at construction."""
        return dict(self._node_config)

    def is_restart(self) -> bool:
        """Return whether this mock represents a restarted node."""
        return self._is_restart

    def restart_count(self) -> int:
        """Return how many times this node has been restarted."""
        return self._restart_count

    def timestamp(self) -> datetime.datetime:
        """Return the current UTC time, mirroring the HLC readout."""
        return datetime.datetime.now(datetime.UTC)

    def merge_external_events(self, subscription: Any) -> None:
        """Raise NotImplementedError; requires a live ROS2 subscription.

        The real node only supports this for a dora.ros2 bridge subscription
        merged into a running event loop, which a mock cannot provide.
        """
        raise NotImplementedError(
            "MockNode cannot merge an external event stream: "
            "merge_external_events() needs a running ROS2 core and daemon. "
            "Pass the subscription only when testing against a live runtime."
        )

    def _enqueue_reply(self, event: dict[str, Any]) -> None:
        """Queue an event ahead of the next STOP.

        A service reply must still be observable by a node that loops until
        STOP, so it is inserted before the terminal STOP rather than behind it.
        """
        for index, pending in enumerate(self._events):
            if pending.get("type") == "STOP":
                self._events.insert(index, event)
                return
        self._events.append(event)