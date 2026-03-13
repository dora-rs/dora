"""Processor node that transforms sensor readings."""

from dora import Node

node = Node()

for event in node:
    if event["type"] == "INPUT" and event["id"] == "reading":
        value = float(bytes(event["value"]))
        result = value * 1.8 + 32  # Celsius to Fahrenheit
        node.send_output("result", str(round(result, 1)).encode())
        node.log_debug(f"Converted {value}C -> {result:.1f}F")
