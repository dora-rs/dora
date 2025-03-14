## Dora Node Hub

This hub contains useful pre-built nodes for Dora.

## Add a new python node

- To work on a new node, start by:

```bash
cd node-hub
dora new your-node-name --lang python --kind node
cd ./your-node-name
uv venv --seed -p 3.11
uv pip install -e . # Install
uv run ruff check . --fix # Format
uv run ruff check . # Lint
uv run pytest . # Test
```

- To add a python dependency just do:

```bash
uv add numpy # for example
```

> The package is then added to your `pyproject.toml`

- Modify the code within `main.py` in your liking.

- Create a PR and let the CI/CD run test on it 🙋

# Structure

The structure of the node hub is as follows (please use the same structure if you need to add a new node):

```
node-hub/
└── your-node/
    ├── main.py
    ├── README.mdr
    └── pyproject.toml
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
[project]
name = "[name of the node e.g. video-encoder, with '-' to replace spaces]"
version = "0.1"
authors = [{ name = "[Pseudo/Name]", email = "[email]" }]
description = "Dora Node for []"
readme = "README.md"
license = { text = "MIT" }

dependencies = [
    "dora-rs >= 0.3.8",
]

[project.scripts]
[name of the node with '-' to replace spaces] = "[name of the node with '_' to replace spaces].main:main"

```

Finally, the README.md file should explicit all inputs/outputs of the node and how to configure it in the YAML file.

# Example

```toml
[project]
name = "opencv-plot"
version = "0.1"
authors = [
    "Haixuan Xavier Tao <tao.xavier@outlook.com>",
    "Enzo Le Van <dev@enzo-le-van.fr>"
]
description = "Dora Node for plotting data with OpenCV"
readme = "README.md"
license = { text = "MIT" }
requires-python = ">=3.7"

dependencies = [
    "dora-rs >= 0.3.8",
]
[dependency-groups]
dev = ["pytest >=8.1.1", "ruff >=0.9.1"]

[project.scripts]
opencv-plot = "opencv_plot.main:main"

```
## Adding git dependency
- If a git repository is added as submodule. Proper path should be added in `pyproject.toml` inorder to make sure that linting and testing are exempted for that dependency. 
- A very good example of how this can be done is as follows

Correct approach:
```toml
[tool.ruff]
exclude = ["dora_magma/Magma"]

[tool.black]
extend.exclude = "dora_magma/Magma"
```
Incorrect Approach:
```toml
[tool.ruff]
exclude = ["dora-magma/dora_magma/Magma"]

[tool.black]
extend.exclude = "dora_magma/Magma"
```
##### Note: 
- `dora-magma` is root folder of the node. 

## License

This project is licensed under Apache-2.0. Check out [NOTICE.md](../NOTICE.md) for more information.
