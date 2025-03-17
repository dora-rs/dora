"""A Dora node for running Phi-4 language model inference.

This package provides functionality to run Phi-4 language model inference using Dora.
It handles loading the model, processing inputs, and generating outputs in a streaming fashion.
"""

import os

# Define the path to the README file relative to the package directory
readme_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "README.md")

# Read the content of the README file
try:
    with open(readme_path, encoding="utf-8") as f:
        __doc__ = f.read()
except FileNotFoundError:
    __doc__ = "README file not found."
