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

## Getting Started: Course no.1

You've heard about `dora` and want to give it a try? You're in the right place! Let's build a simple application in Python that allows sending a message (a string) from one app (the talker) to another (the listener).

### Installation

If you're here, it's likely that you haven't installed the necessary tools yet. Let's fix that! You’ll need two tools:
- **`dora`**, the software (CLI) that lets you run your applications.
- A Python environment manager like **`uv`** (a tool I highly recommend). However, if you're familiar with Python and know how to create virtual environments, feel free to use the tools you're comfortable with!

To install `dora`, follow these simple steps (run the following commands in a terminal):

#### On macOS and Linux
```bash
curl --proto '=https' --tlsv1.2 -sSf https://raw.githubusercontent.com/dora-rs/dora/main/install.sh | bash
```

#### On Windows
```powershell
powershell -c "irm https://raw.githubusercontent.com/dora-rs/dora/main/install.ps1 | iex"
```

To install `uv`, it's equally straightforward:

#### On macOS and Linux
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

#### On Windows
```powershell
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
```

### Application Structure

Typically, we follow this structure for `dora` applications (you’ll need to create these files yourself):

```
my_app/
├── dataflow.yml  # The YAML file representing the dataflow, to be executed by `dora`
└── nodes/        # Directory containing our simple nodes
    ├── talker.py
    └── listener.py
```

Next, let's create a Python virtual environment (`venv`) to isolate the necessary dependencies. You can do this with `uv` like so:

```bash
cd my_app
uv venv --python 3.12
```

Now, you can install the `dora` Python API:
```bash
cd my_app
uv pip install dora-rs
```

At this point, the application structure should look like this:

```
my_app/
├── .venv/        # Directory containing the virtual environment
├── dataflow.yml  # The YAML file representing the dataflow, to be executed by `dora`
└── nodes/        # Directory containing our simple nodes
    ├── talker.py
    └── listener.py
```

### The Dataflow

Now, we’ll write the structural description of our application in the `dataflow.yml` file. Our application will have two nodes: a `talker` and a `listener`. Here’s the content for `dataflow.yml`:

```yaml
nodes:
  - id: talker
    path: nodes/talker.py

  - id: listener
    path: nodes/listener.py
```

But how does `dora` know to use our `venv` to run the nodes? There are two options:
- **Option 1**: Start the `dora` daemon after activating the `venv`. For example:
```bash
source .venv/bin/activate
dora up
```
- **Option 2**: Make the `dataflow` more robust:
```yaml
nodes:
  - id: talker
    path: shell
    args: |
      source .venv/bin/activate
      python nodes/talker.py

  - id: listener
    path: shell
    args: |
      source .venv/bin/activate
      python nodes/listener.py
```
#### Windows
**For windows users it's hard to make everything works correctly**, so I recommend using the first option or this `dataflow.yml`:

```yaml
nodes:
  - id: talker
    path: ./.venv/Scripts/python
    args: nodes/talker.py

  - id: listener
    path: ./.venv/Scripts/python
    args: nodes/listener.py
```

**Note**: you may need to execute this command `Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy Bypass -Force;` as Admin

---

Now you’re almost done setting up your `dataflow.yml`.

### Inputs/Outputs

Nodes can communicate with each other through defined `inputs` and `outputs`. In our case, the `talker` will send a message to the `listener`. Here's how we can define the `inputs` and `outputs` for each node:

```yaml
nodes:
  - id: talker
    outputs:
      - message

  - id: listener
    inputs:
      message: talker/message
```

With that, we’ve completed the `dataflow.yml` setup. Now, let’s move on to programming our application using Python.

### Python

In each of our node files, we need to create a `Node` object to handle communication between the nodes in the application.

```python
from dora import Node

node = Node("talker")  # Providing the node name isn't required, but it makes the code clearer.
```

Additionally, for `talker.py`, we need to use the `pyarrow` library to send messages, as `dora` communicates via Arrow format. Add the following:

```python
import pyarrow as pa
```

Next, we’ll use two functions: `send_output` in `talker.py` and `next` in `listener.py`.

```python
# talker.py
data = pa.array(["Hello, World!", "My name is Dora"])
node.send_output("message", data)
```

```python
# listener.py
event = node.next()
print(event)
```

Now we’re finished with the code! Let's see how to run our application using `dora`.

### Running the Application

We will use the `dora start` command. However, it's essential to understand how it works. `dora start` sends a message to two processes: the `daemon` and the `coordinator`. These processes will launch and connect our nodes based on the `dataflow.yml`.

First, launch these two processes:

```bash
dora up
dora start dataflow.yml
```

Once the application starts successfully, we’ll check that `listener.py` has received the message from `talker.py`. To do this, you need the UUID of your application, which is generated every time the app is launched. After running `dora start`, you should see an output with a UUID like **0191d7cd-0dab-701e-82b8-2efbe1adf51e**. Copy this and use it to check the logs of `listener.py`:

```bash
dora logs 0191d7cd-0dab-701e-82b8-2efbe1adf51e listener
```

You should then see an output like this:

```python
{'kind': 'dora', 'id': 'message', 'value': <pyarrow.lib.StringArray object at 0x102804040>
[
  "Hello, World!",
  "My name is Dora"
], '_cleanup': <builtins.NodeCleanupHandle object at 0x102a9caf0>, 'metadata': {}, 'type': 'INPUT'}
```

As you can see, the message was successfully sent from `talker.py` to `listener.py`.

### Full Code

Here is the complete code for our application:

```YAML
# dataflow.yml | see above for windows user
nodes:
  - id: talker
    path: shell
    args: |
      source .venv/bin/activate
      python nodes/talker.py
    outputs:
      - message

  - id: listener
    path: shell
    args: |
      source .venv/bin/activate
      python nodes/listener.py
    inputs:
      message: talker/message
```

```python
# talker.py
from dora import Node
import pyarrow as pa

node = Node("talker")

node.send_output("message", pa.array(["Hello", "World"]))
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
