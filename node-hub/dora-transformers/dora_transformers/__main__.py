"""Entry point for executing the `main` function from the `main` module.

This script serves as the entry point for running the `main` function.
When executed directly, it calls `main()` to start the program.

Usage:
- Run this script as the main module to execute the primary functionality.

Example:
-------
```sh
python -m package_name  # Runs the main function

"""

from .main import main

if __name__ == "__main__":
    main()
