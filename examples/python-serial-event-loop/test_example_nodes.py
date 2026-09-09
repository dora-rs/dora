import unittest

import pyarrow as pa

from receiver import handle_counter
from sender import run_sender
from serial_event_loop import InputEvent


class FakeSenderNode:
    def __init__(self):
        self.events = iter(
            [
                {"type": "INPUT", "id": "ignored", "value": pa.array([0])},
                {"type": "INPUT", "id": "tick", "value": pa.array([0])},
                {"type": "INPUT", "id": "tick", "value": pa.array([0])},
                {"type": "STOP"},
            ]
        )
        self.outputs = []

    def __iter__(self):
        return self

    def __next__(self):
        return next(self.events)

    def send_output(self, output_id, data):
        self.outputs.append((output_id, data))


class ExampleNodeTests(unittest.TestCase):
    def test_sender_emits_incrementing_uint64_for_ticks_only(self):
        node = FakeSenderNode()
        run_sender(node)
        self.assertEqual(
            [item[0] for item in node.outputs], ["counter", "counter"]
        )
        self.assertEqual(
            [item[1].type for item in node.outputs], [pa.uint64(), pa.uint64()]
        )
        self.assertEqual([item[1][0].as_py() for item in node.outputs], [0, 1])

    def test_receiver_prints_first_counter_value(self):
        lines = []
        handle_counter(
            InputEvent("counter", pa.array([42], type=pa.uint64())),
            emit=lines.append,
        )
        self.assertEqual(lines, ["[python-receiver] received counter=42"])

    def test_receiver_ignores_empty_arrays(self):
        lines = []
        handle_counter(
            InputEvent("counter", pa.array([], type=pa.uint64())),
            emit=lines.append,
        )
        self.assertEqual(lines, [])


if __name__ == "__main__":
    unittest.main()
