"""TODO: Add docstring."""

import pyarrow as pa
from dora import Node
from pynput import keyboard
from pynput.keyboard import Events


def main():
    """TODO: Add docstring."""
    node = Node()

    always_none = node.next(timeout=0.001) is None
    always_none = node.next(timeout=0.001) is None
    print("Always None:", always_none)
    with keyboard.Events() as events:
        while True:
            if not always_none:
                event_stream_is_none = node.next(timeout=0.001) is None
                if event_stream_is_none:
                    break
            event = events.get(1.0)
            if event is not None and isinstance(event, Events.Press):
                if hasattr(event.key, "char"):
                    if event.key.char is not None:
                        node.send_output("char", pa.array([event.key.char]))


if __name__ == "__main__":
    main()
