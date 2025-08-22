## Proposal: `dora` Pythonic Dataflow API (Final Revision)

This proposal outlines a Python API for imperatively defining `dora` dataflows, ensuring full coverage of the `dora-schema.json` specification while maintaining a Pythonic and developer-friendly interface.

### Core Concepts

The API is centered around the `Dataflow`, `Node`, and `Operator` classes. It provides multiple ways to define the graph topology to suit different developer preferences and use cases.

### API Design

#### `Dataflow` Class

The `Dataflow` class is the main entry point. It can be used as a context manager for a more Pythonic feel.

```python
from typing import Callable

class Dataflow:
    def __init__(self, name: str = "dora-dataflow"):
        ...

    # Node and Edge Definition
    def add_node(self, id: str, **kwargs) -> 'Node':
        ...

    def add_edge(self, source_node: str | 'Node', source_output: str,
                 dest_node: str | 'Node', dest_input: str,
                 queue_size: int = None):
        ...

    def add_conditional_edge(self, source_node: str | 'Node', source_output: str,
                             condition: Callable[[any], str],
                             destinations: dict[str, str | 'Node']):
        ...

    # Execution and Output
    def to_yaml(self, path: str = None) -> str | None:
        ...

    def run(self):
        ...

    def start(self):
        ...

    def stop(self):
        ...

    # Debugging and Visualization
    def visualize(self, format: str = 'png', view: bool = True):
        ...

    # Context Manager Support
    def __enter__(self):
        ...

    def __exit__(self, exc_type, exc_val, exc_tb):
        ...
```

#### `Node` Class

The `Node` class is used to configure both standard nodes (with a `path`) and runtime nodes (with operators).

```python
class Node:
    # ... (methods for path, args, env, git, etc. remain) ...

    def add_operator(self, operator: 'Operator') -> "Node":
        """
        Adds an operator to this node. If one or more operators are added,
        the node will be treated as a runtime node.
        """
        ...

    def send(self, output_id: str) -> 'Output':
        ...

class Output:
    def to(self, node: 'Node', input_id: str, queue_size: int = None):
        ...
```

#### `Operator` Class

A new `Operator` class is introduced to fully represent an operator's configuration.

```python
class Operator:
    def __init__(self, id: str, name: str = None, description: str = None,
                 build: str = None, python: str = None,
                 shared_library: str = None, send_stdout_as: str = None):
        """
        Initializes a new Operator.

        Args:
            id: The unique identifier for the operator.
            name: A human-readable name for the operator.
            description: A detailed description of the operator.
            build: The build command for the operator.
            python: The path to the Python source file for the operator.
            shared_library: The path to the shared library for the operator.
            send_stdout_as: Redirects stdout/stderr to a data output.
        """
        ...
```

### Key Features and Coverage

*   **Full Schema Coverage:** This design now covers all stable features of the `dora-schema.json`, including runtime nodes with multiple operators, full operator configuration, and input queue sizes.
*   **Graph Visualization:** The `visualize()` method provides an easy way to debug and understand the dataflow topology.
*   **Pythonic Idioms:** The context manager (`with Dataflow(...)`) makes the API feel natural to Python developers.
*   **Flexible Edge Definition:** The API supports both centralized (`dataflow.add_edge()`) and decentralized (`node.send().to()`) edge definitions.
*   **Conditional Routing:** The `add_conditional_edge` method, inspired by `LangGraph`, allows for powerful dynamic dataflows.

### Example: Runtime Node with Multiple Operators

This example shows how to define a runtime node with two operators.

```python
from dora import Dataflow, Operator

with Dataflow(name="multi-operator-dataflow") as df:
    # Define a runtime node
    runtime_node = df.add_node(id="runtime")

    # Define the operators
    operator1 = Operator(id="op1", python="operator1.py", build="pip install ...")
    operator2 = Operator(id="op2", shared_library="./op2.so")

    # Add the operators to the runtime node
    runtime_node.add_operator(operator1)
    runtime_node.add_operator(operator2)

    # ... (define other nodes and edges) ...

    df.visualize()
```

This final revision provides a comprehensive and robust API for defining `dora` dataflows in Python, laying a solid foundation for future development and extensions.
