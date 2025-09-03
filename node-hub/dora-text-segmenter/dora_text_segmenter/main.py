#!/usr/bin/env python3
"""
Text Segmenter Node - Default implementation using queue-based segmenter.
This is the main entry point for the dora-text-segmenter node.
"""

# Import and use the queue-based segmenter as the default
from .queue_based_segmenter import main

if __name__ == "__main__":
    main()