"""Calculator tool — dynamically added to extend agent capabilities.

Evaluates simple math expressions from tool requests.
"""

import json
import logging

import pyarrow as pa
from dora import Node


def main():
    node = Node()
    logging.info("Calculator tool started (dynamically added)")

    for event in node:
        if event["type"] == "INPUT":
            values = event["value"].to_pylist()
            for v in values:
                try:
                    request = json.loads(v)
                except json.JSONDecodeError:
                    continue

                if request.get("tool") == "calc":
                    expression = request.get("expression", "0")
                    try:
                        # Use ast.literal_eval for safe numeric parsing.
                        # For actual arithmetic, use a proper parser like simpleeval.
                        import ast
                        result = ast.literal_eval(expression)
                    except (ValueError, SyntaxError) as e:
                        result = f"error: {e}"

                    response = json.dumps({
                        "tool": "calc",
                        "expression": expression,
                        "result": str(result),
                    })
                    node.send_output("response", pa.array([response]))
                    logging.info("Calc: %s = %s", expression, result)

        elif event["type"] == "STOP":
            break


if __name__ == "__main__":
    main()
