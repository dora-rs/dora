"""Log monitoring node demonstrating in-dataflow log aggregation.

Receives structured log entries from processor via send_logs_as routing,
and sensor readings directly. Counts warnings and errors to demonstrate
that log data can be consumed and analyzed within the dataflow graph.
"""

import json
import logging

from adora import Node


def main():
    node = Node()
    warn_count = 0
    error_count = 0
    reading_count = 0

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]

            if input_id == "logs":
                # Log entries arrive as Arrow-encoded JSON strings
                arr = event["value"]
                if len(arr) == 0:
                    continue
                raw = arr[0].as_py()
                try:
                    entry = json.loads(raw) if isinstance(raw, str) else raw
                except (json.JSONDecodeError, TypeError):
                    entry = {"level": "unknown", "message": str(raw)}

                level = entry.get("level", "").lower()
                msg = entry.get("message", entry.get("msg", ""))

                if level == "error":
                    error_count += 1
                    logging.error("monitor caught error: %s", msg)
                elif level in ("warn", "warning"):
                    warn_count += 1
                    logging.warning("monitor caught warning: %s", msg)
                else:
                    logging.debug("monitor log entry: level=%s msg=%s", level, msg)

            elif input_id == "reading":
                reading_count += 1
                if reading_count % 20 == 0:
                    logging.info(
                        "monitor summary: readings=%d warnings=%d errors=%d",
                        reading_count,
                        warn_count,
                        error_count,
                    )

        elif event["type"] == "STOP":
            logging.info(
                "monitor final: readings=%d warnings=%d errors=%d",
                reading_count,
                warn_count,
                error_count,
            )
            break


if __name__ == "__main__":
    main()
