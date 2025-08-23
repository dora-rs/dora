"""Dora Qwen3-8B LLM Node Package."""

import os

# Define the path to the README file relative to the package directory
readme_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "README.md")

# Read the content of the README file
try:
    with open(readme_path, encoding="utf-8") as f:
        __doc__ = f.read()
except FileNotFoundError:
    __doc__ = "Qwen3-8B LLM node for Dora dataflows with MLX support."

__version__ = "0.1.0"