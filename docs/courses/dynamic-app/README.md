#
<p align="center">
    <img src="https://raw.githubusercontent.com/dora-rs/dora/main/docs/src/logo.svg" width="400"/>
</p>

<h2 align="center">
  <a href="https://www.dora-rs.ai">Website</a>
  |
  <a href="https://www.dora-rs.ai/docs/api/python-api">Python API</a>
  |
  <a href="https://docs.rs/dora-node-api/latest/dora_node_api/">Rust API</a>
  |
  <a href="https://www.dora-rs.ai/docs/guides/">Guide</a>
  |
  <a href="https://discord.gg/6eMGGutkfE">Discord</a>
</h2>

<div align="center">
  <a href="https://github.com/dora-rs/dora/actions">
    <img src="https://github.com/dora-rs/dora/workflows/CI/badge.svg" alt="Build and test"/>
  </a>
  <a href="https://crates.io/crates/dora-rs">
    <img src="https://img.shields.io/crates/v/dora_node_api.svg"/>
  </a>
  <a href="https://docs.rs/dora-node-api/latest/dora_node_api/">
    <img src="https://docs.rs/dora-node-api/badge.svg" alt="rust docs"/>
  </a>
  <a href="https://pypi.org/project/dora-rs/">
    <img src="https://img.shields.io/pypi/v/dora-rs.svg" alt="PyPi Latest Release"/>
  </a>
</div>

An extremely fast and simple **dataflow oriented robotic** framework to manage your projects and run complex **apps**, written in Rust.

Here’s an English version of the second course, written in a clear and structured way:

---

## Dynamic App: Course no.2

After completing Course no.1, you should feel more comfortable with `dora` and have a better understanding of how to build applications. In this course (which will be shorter than the previous one), we will make our `talker/listener` application more dynamic.

You might have noticed that `dora` currently executes our nodes in a somewhat static way, meaning we don’t have control over the process where our node is executed—**yet**.

`dora` has a feature that allows users to control the execution process of their nodes themselves, rather than relying on the `daemon`.

### Application Structure

Let’s start with the same structure from the previous course, with a `venv` already set up:

```
my_app/
├── .venv/        # Directory containing the virtual environment
├── dataflow.yml  # The YAML file representing the dataflow, to be executed by `dora`
└── nodes/        # Directory containing our simple nodes
    ├── talker.py
    └── listener.py
```

### The Dataflow

Now, let’s modify the `dataflow.yml` to enable the `dynamic` function in `dora`:

```yaml
nodes:
  - id: talker
    path: dynamic
    outputs:
      - message

  - id: listener
    path: dynamic
    inputs:
      message: talker/message
```

#### Explanation:
When the `path` parameter is set to `dynamic`, `dora` will wait for you to manually start the `dynamic` nodes.

### Python

In our applications, it’s **essential** to specify the node name when creating a node, like so: `node = Node("talker")`. This lets `dora` know which node you want to run.

Let’s make a small adjustment to our application:

```python
# talker.py
from dora import Node
import pyarrow as pa

node = Node("talker")  # Providing the node name is necessary for dynamic nodes.

text = input("Enter a message: ")
data = pa.array([text])
node.send_output("message", data)
```

Now we’re finished with the code! Let's see how to run our application using `dora`.

### Running the Application

To run this application, you’ll need to open **two** terminal windows and activate your `venv` in each. Then, execute the following:

```bash
# terminal 1
source .venv/bin/activate
dora up
dora start dataflow.yml --detach
python nodes/listener.py
```

```bash
# terminal 2
source .venv/bin/activate
python nodes/talker.py
```

In **terminal 2**, you should see the prompt `"Enter a message: "`. After entering a message, it will be displayed in **terminal 1**.

### Full Code

Here is the complete code for our application:

```YAML
# dataflow.yml
nodes:
  - id: talker
    path: dynamic
    outputs:
      - message

  - id: listener
    path: dynamic
    inputs:
      message: talker/message
```

```python
# talker.py
from dora import Node
import pyarrow as pa

node = Node("talker")

text = input("Enter a message: ")
data = pa.array([text])
node.send_output("message", data)
```

```python
# listener.py
from dora import Node

node = Node("listener")

event = node.next()

print(event)
```

---

This concludes our tutorial! You’ve successfully created a simple messaging app using `dora`. Happy coding!

## Contributing

We are passionate about supporting contributors of all levels of experience and would love to see
you get involved in the project. See the
[contributing guide](https://github.com/dora-rs/dora/blob/main/CONTRIBUTING.md) to get started.

## Discussions

Our main communication channels are:

- [Our Discord server](https://discord.gg/6eMGGutkfE)
- [Our Github Project Discussion](https://github.com/orgs/dora-rs/discussions)

Feel free to reach out on any topic, issues or ideas.

We also have [a contributing guide](CONTRIBUTING.md).

## License

This project is licensed under Apache-2.0. Check out [NOTICE.md](NOTICE.md) for more information.
