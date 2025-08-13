"""TODO: Add docstring."""

import pyarrow as pa
from dora import Node
import json


node = Node()

count = 0
for event in node:
    if event["type"] == "INPUT":
        if 'metadata' in event:
            data = json.loads(event["value"][0].as_py())
            name = data.get("name", "")
            match name:
                case "counter_increment":
                    count += 1
                    node.send_output("reply", pa.array([f'{{"content":[{{"type": "text", "text": "{count}"}}]}}']), metadata=event["metadata"])
                case "counter_decrement":
                    count -= 1
                    node.send_output("reply", pa.array([f'{{"content":[{{"type": "text", "text": "{count}"}}]}}']), metadata=event["metadata"])
                case "counter_get_value":
                    node.send_output("reply", pa.array([f'{{"content":[{{"type": "text", "text": "{count}"}}]}}']), metadata=event["metadata"])
                case _:
                    print(f"Unknown command: {name}")