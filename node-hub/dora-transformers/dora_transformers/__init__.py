"""Loads the content of the README.md file and assigns it to the module-level `__doc__`.

This script dynamically sets the module docstring by reading the README.md file
located in the package's root directory. If the README file is not found, it sets
a fallback message.

Functionality:
- Constructs the absolute path to README.md.
- Attempts to read and load its content into `__doc__`.
- Handles the case where the file does not exist by providing a default message.

Attributes
----------
- `readme_path` (str): Absolute path to the README.md file.
- `__doc__` (str): Content of the README file or a fallback message.

Exceptions:
- `FileNotFoundError`: Caught and handled if the README.md file is missing.

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
