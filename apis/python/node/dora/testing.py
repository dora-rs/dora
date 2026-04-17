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

from typing import Any

import pyarrow as pa

__all__ = ["MockNode"]


class MockNode:
    """Drop-in replacement for dora.Node usable in unit tests.

    Args:
        inputs: List of (input_id, data) tuples. Each becomes an
            INPUT event. A STOP event is automatically appended.
        metadata: Optional dict of metadata to attach to each input event.
            Each event gets its own copy.
    """

    def __init__(
        self,
        inputs: list[tuple[str, Any]],
        metadata: dict[str, Any] | None = None,
    ):
        events: list[dict[str, Any]] = []
        for input_id, data in inputs:
            events.append(
                {
                    "type": "INPUT",
                    "id": input_id,
                    "value": data,
                    "metadata": dict(metadata) if metadata else {},
                }
            )
        events.append({"type": "STOP"})
        self._events = iter(events)
        self.outputs: dict[str, list[Any]] = {}

    def __iter__(self):
        return self

    def __next__(self) -> dict[str, Any]:
        return next(self._events)

    def next(self, timeout: float = None) -> dict[str, Any] | None:
        """Return the next event, or None if exhausted.

        Args:
            timeout: Ignored. Accepted for API compatibility with dora.Node.
        """
        try:
            return next(self._events)
        except StopIteration:
            return None

    async def recv_async(self, timeout: float = None) -> dict[str, Any] | None:
        """Async version of next(). Returns immediately.

        Args:
            timeout: Ignored. Accepted for API compatibility with dora.Node.
        """
        return self.next(timeout=timeout)

    def send_output(
        self,
        output_id: str,
        data: Any,
        metadata: dict[str, Any] | None = None,
    ) -> None:
        """Capture an output for later assertion."""
        self.outputs.setdefault(output_id, []).append(data)
