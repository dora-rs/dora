from pynput import keyboard
from pynput.keyboard import Events
import pyarrow as pa
from dora import Node


def main():
    node = Node()
    with keyboard.Events() as events:
        while True:
            event = events.get(1.0)
            if event is not None and isinstance(event, Events.Press):
                if hasattr(event.key, "char"):
                    if event.key.char is not None:
                        node.send_output("char", pa.array([event.key.char]))


if __name__ == "__main__":
    main()
