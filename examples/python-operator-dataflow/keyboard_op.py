"""TODO: Add docstring."""

import pyarrow as pa
from dora import Node
from pynput import keyboard
from pynput.keyboard import Events, Key

node = Node()
buffer_text = ""
ctrl = False
submitted_text = []
cursor = 0

NODE_TOPIC = ["record", "send", "ask", "change"]

with keyboard.Events() as events:
    while True:
        dora_event = node.next(0.01)
        if (
            dora_event is not None
            and dora_event["type"] == "INPUT"
            and dora_event["id"] == "recording"
        ):
            buffer_text += dora_event["value"][0].as_py()
            node.send_output("buffer", pa.array([buffer_text]))
            continue

        event = events.get(1.0)
        if event is not None and isinstance(event, Events.Press):
            if hasattr(event.key, "char"):
                cursor = 0
                buffer_text += event.key.char
                node.send_output("buffer", pa.array([buffer_text]))
            elif event.key == Key.backspace:
                buffer_text = buffer_text[:-1]
                node.send_output("buffer", pa.array([buffer_text]))
            elif event.key == Key.esc:
                buffer_text = ""
                node.send_output("buffer", pa.array([buffer_text]))
            elif event.key == Key.enter:
                node.send_output("submitted", pa.array([buffer_text]))
                first_word = buffer_text.split(" ")[0]
                if first_word in NODE_TOPIC:
                    node.send_output(first_word, pa.array([buffer_text]))
                submitted_text.append(buffer_text)
                buffer_text = ""
                node.send_output("buffer", pa.array([buffer_text]))
            elif event.key == Key.ctrl:
                ctrl = True
            elif event.key == Key.space:
                buffer_text += " "
                node.send_output("buffer", pa.array([buffer_text]))
            elif event.key == Key.up:
                if len(submitted_text) > 0:
                    cursor = max(cursor - 1, -len(submitted_text))
                    buffer_text = submitted_text[cursor]
                    node.send_output("buffer", pa.array([buffer_text]))
            elif event.key == Key.down:
                if len(submitted_text) > 0:
                    cursor = min(cursor + 1, 0)
                    buffer_text = submitted_text[cursor]
                    node.send_output("buffer", pa.array([buffer_text]))
        elif event is not None and isinstance(event, Events.Release):
            if event.key == Key.ctrl:
                ctrl = False
