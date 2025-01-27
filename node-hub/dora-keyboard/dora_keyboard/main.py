import numpy as np
import pyarrow as pa
from dora import Node
from pynput import keyboard
from pynput.keyboard import Events, Key


def main():
    node = Node()

    always_none = node.next(timeout=0.001) is None
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
                else:
                    if event.key == Key.up:
                        node.send_output(
                            "action", pa.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
                        )
                    elif event.key == Key.down:
                        node.send_output(
                            "action", pa.array([-0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
                        )
                    elif event.key == Key.left:
                        node.send_output(
                            "action",
                            pa.array([0.0, 0.0, 0.0, 0.0, 0.0, np.deg2rad(30.0)]),
                        )
                    elif event.key == Key.right:
                        node.send_output(
                            "action",
                            pa.array([0.0, 0.0, 0.0, 0.0, 0.0, -np.deg2rad(30.0)]),
                        )


if __name__ == "__main__":
    main()
