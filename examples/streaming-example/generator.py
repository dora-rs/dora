"""Token generator — simulates streaming text generation.

Demonstrates the streaming pattern with session/segment/chunk metadata:
- Each prompt starts a new session
- Tokens are sent one-by-one with sequence numbers
- Final token has fin=true to signal end of response
- Uses metadata keys: session_id, segment_id, seq, fin

In production, replace the simulated tokens with actual LLM inference.
"""

import json
import logging
import time
import uuid

import pyarrow as pa
from adora import Node

# Simulated responses (in production, these come from an LLM)
RESPONSES = {
    "Tell me about robotics": "Robotics combines mechanical engineering, electrical engineering, and computer science to create machines that can interact with the physical world.",
    "Explain zero-copy IPC": "Zero-copy IPC avoids data copies between processes by sharing memory regions directly. Adora uses shared memory for messages over 4KB.",
    "What is Apache Arrow?": "Apache Arrow is a columnar memory format for flat and hierarchical data. It enables zero-serialization data sharing between processes.",
    "How does shared memory work?": "Shared memory maps the same physical memory pages into multiple process address spaces, enabling direct data access without copying.",
}


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "prompt":
                prompt = event["value"].to_pylist()[0]
                logging.info("Generating response for: %s", prompt)

                # Find matching response or use default
                response = RESPONSES.get(prompt, f"I received: {prompt}")
                tokens = response.split()

                # Stream tokens with session/segment/chunk metadata
                session_id = str(uuid.uuid4())
                for seq, token in enumerate(tokens):
                    is_last = seq == len(tokens) - 1
                    metadata = {
                        "session_id": session_id,
                        "segment_id": "0",
                        "seq": str(seq),
                        "fin": str(is_last).lower(),
                    }
                    node.send_output(
                        "tokens",
                        pa.array([token]),
                        metadata=metadata,
                    )
                    # Simulate generation delay (50ms per token)
                    time.sleep(0.05)

                logging.info(
                    "Streamed %d tokens (session %s)",
                    len(tokens),
                    session_id[:8],
                )

        elif event["type"] == "STOP":
            break


if __name__ == "__main__":
    main()
