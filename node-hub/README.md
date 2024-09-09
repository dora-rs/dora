## Dora Node Hub

This hub contains useful pre-packaged nodes for Dora.

# Structure

The structure of the node hub is as follows (please use the same structure if you need to add a new node):

## For Python

```
node-hub/
└── my-node/
    ├── README.md
    ├── pyproject.toml
    └── my_node/
        ├── __init__.py
        └── main.py
```

The idea is to make a `pyproject.toml` file that will install the required dependencies for the node **and** attach main
function of the node inside a callable script in your environment.

To do so, you will need to add a `main` function inside the `main.py` file.

```python
def main():
    pass
```

And then you will need to adapt the following `pyproject.toml` file:

```toml
[tool.poetry]
name = "my-node"
version = "0.3.6"
authors = [
    "You"
]
description = ""
readme = "README.md"

packages = [{ include = "my_node" }]

[tool.poetry.dependencies]
dora-rs = "^0.3.6"
python = "^3.7"

[tool.poetry.scripts]
my-node = "my_node.main:main"

[build-system]
requires = ["poetry-core>=1.8.0"]
build-backend = "poetry.core.masonry.api"
```

## For Rust

```
node-hub/
└── my-node/
    ├── README.md
    ├── Cargo.toml
    └── src/
        └── main.py
```

# README

The README.md file should explicit all inputs/outputs of the node and how to configure it in the YAML file.

## License

This project is licensed under Apache-2.0. Check out [NOTICE.md](../NOTICE.md) for more information.
