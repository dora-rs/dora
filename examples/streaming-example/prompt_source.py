"""Prompt source — sends prompts for text generation."""

import logging

import pyarrow as pa
from adora import Node

PROMPTS = [
    "Tell me about robotics",
    "Explain zero-copy IPC",
    "What is Apache Arrow?",
    "How does shared memory work?",
]


def main():
    node = Node()
    index = 0

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "tick":
                prompt = PROMPTS[index % len(PROMPTS)]
                node.send_output("prompt", pa.array([prompt]))
                logging.info("Sent prompt: %s", prompt)
                index += 1
        elif event["type"] == "STOP":
            break


if __name__ == "__main__":
    main()
