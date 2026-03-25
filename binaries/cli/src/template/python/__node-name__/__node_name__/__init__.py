"""Package initialization for the dora-rs Python node template.

This module attempts to load the package's documentation from the README.md
file located in the parent directory.
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
