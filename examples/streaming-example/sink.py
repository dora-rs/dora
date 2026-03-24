"""Streaming sink — reassembles token streams into complete responses.

Demonstrates consuming the streaming pattern: accumulates tokens by
session_id, assembles the full response when fin=true.
"""

import logging

from adora import Node


def main():
    node = Node()
    sessions = {}  # session_id -> list of tokens

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "tokens":
                token = event["value"].to_pylist()[0]
                metadata = event.get("metadata", {})
                session_id = metadata.get("session_id", "unknown")
                fin = metadata.get("fin", "false") == "true"
                seq = metadata.get("seq", "?")

                # Accumulate tokens
                if session_id not in sessions:
                    sessions[session_id] = []
                sessions[session_id].append(token)

                if fin:
                    # Response complete — reassemble and display
                    tokens_list = sessions.pop(session_id)
                    full_response = " ".join(tokens_list)
                    logging.info(
                        "[session %s] Complete (%d tokens): %s",
                        session_id[:8],
                        len(tokens_list),
                        full_response,
                    )

        elif event["type"] == "STOP":
            break

    # Log any incomplete sessions
    for sid, tokens in sessions.items():
        logging.warning(
            "[session %s] Incomplete (%d tokens): %s",
            sid[:8],
            len(tokens),
            " ".join(tokens),
        )


if __name__ == "__main__":
    main()
