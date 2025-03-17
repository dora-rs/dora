from pynput import keyboard
from pynput.keyboard import Key, Events
import pyarrow as pa
from dora import Node


node = Node()
buffer_text = ""
space = False
submitted_text = []
cursor = -1
with keyboard.Events() as events:
    while True:
        event = events.get(0.1)
        if event is not None and isinstance(event, Events.Press):
            if event.key == Key.space and space == False:
                cursor += 1
                node.send_output("space", pa.array([cursor]))
                space = True
            

        elif event is not None and isinstance(event, Events.Release):
            if event.key == Key.space:
                node.send_output("space", pa.array([-1]))
                space = False
            elif event.key == Key.backspace:
                node.send_output("failed", pa.array([cursor]))
                

        if node.next(0.001) is None:
            break
