# `doraflow`: The Pythonic API for `dora`

This document provides a comprehensive overview and reference for the `doraflow` Python API, a Pythonic interface for imperatively defining `dora` dataflows.

## Overview

The `doraflow` package offers a more familiar, imperative approach to defining `dora` dataflows, complementing the traditional declarative YAML method. This API is designed to be intuitive for Python developers and enables the creation of dynamic and complex dataflows directly within your Python code.

### Key Features

- **Pythonic Interface:** Define dataflows using familiar Python classes and methods.
- **Full Schema Coverage:** The API supports all stable features of the `dora` schema, including runtime nodes, operators, and custom queue sizes.
- **Flexible Edge Definition:** Connect nodes using either a centralized `dataflow.add_edge()` method or a more fluent `node.send().to()` syntax.
- **Execution Control:** Run dataflows in the foreground, or start and stop them in the background.
- **Visualization:** Generate visual representations of your dataflow graph to easily understand and debug the topology.

## Getting Started

Here is a simple example of how to define, run, and visualize a dataflow using the `doraflow` API.

```python
from doraflow import Dataflow

with Dataflow(name="my-dataflow") as df:
    # Define two nodes
    node1 = df.add_node(id="node1", path="path/to/your/node1.py")
    node2 = df.add_node(id="node2", path="path/to/your/node2.py")

    # Connect the nodes
    df.add_edge(node1, "output1", node2, "input1")

    # Visualize the dataflow
    df.visualize()

    # Run the dataflow in the foreground
    df.run()
```

## Core Concepts

The API is built around three core classes:

- **`Dataflow`:** The main entry point for creating and managing a dataflow.
- **`Node`:** Represents a node in the dataflow graph.
- **`Operator`:** Represents an operator within a runtime node.

## API Reference

### `Dataflow` Class

The `Dataflow` class is the primary interface for building and managing dataflows.

**`__init__(self, name: str = "dora-dataflow")`**

Initializes a new dataflow.

- **`name` (str):** The name of the dataflow.

**`add_node(self, id: str, **kwargs) -> Node`**

Adds a new node to the dataflow.

- **`id` (str):** A unique identifier for the node.
- **`**kwargs`:** Additional node configuration (see `Node` class).

**`add_edge(self, source_node: str | Node, source_output: str, dest_node: str | Node, dest_input: str, queue_size: int = None)`**

Adds a direct connection between two nodes.

- **`source_node` (str | Node):** The source node or its ID.
- **`source_output` (str):** The name of the output on the source node.
- **`dest_node` (str | Node):** The destination node or its ID.
- **`dest_input` (str):** The name of the input on the destination node.
- **`queue_size` (int, optional):** The size of the input queue.

**`to_yaml(self, path: str = None) -> str | None`**

Generates the YAML representation of the dataflow.

- **`path` (str, optional):** If provided, the YAML is written to this file path.

**`run(self)`**

Builds and runs the dataflow in the foreground. This is a blocking call.

**`start(self)`**

Starts the dataflow in the background. This method ensures the `dora` daemon is running, builds the dataflow, and starts it.

**`stop(self)`**

Stops a dataflow that was started in the background.

**`visualize(self, format: str = "png", view: bool = True)`**

Generates a visual representation of the dataflow graph using Graphviz.

- **`format` (str):** The output format (e.g., `png`, `svg`).
- **`view` (bool):** If `True`, opens the generated visualization.

### `Node` Class

The `Node` class represents a single node in the dataflow.

**`add_operator(self, operator: Operator) -> Node`**

Adds an operator to the node, making it a runtime node.

- **`operator` (Operator):** The operator to add.

**`send(self, output_id: str) -> Output`**

Specifies an output of the node to be connected to another node's input. Used for the fluent `send().to()` syntax.

- **`output_id` (str):** The name of the output.

### `Operator` Class

The `Operator` class is used to configure an operator for a runtime node.

**`__init__(self, id: str, name: str = None, description: str = None, build: str = None, python: str = None, shared_library: str = None, send_stdout_as: str = None)`**

Initializes a new operator.

- **`id` (str):** The unique identifier for the operator.
- **`name` (str, optional):** A human-readable name.
- **`description` (str, optional):** A description of the operator.
- **`build` (str, optional):** The build command for the operator.
- **`python` (str, optional):** The path to the Python source file.
- **`shared_library` (str, optional):** The path to the shared library.
- **`send_stdout_as` (str, optional):** Redirects stdout/stderr to a data output.

## Examples

For working examples, please refer to the scripts in the `examples/python-api-dataflow/` directory:

- **`main.py`:** A basic example demonstrating how to define and run a simple dataflow.
- **`advanced_dataflow.py`:** A conceptual example showing how to define a runtime node with multiple operators.
- **`translation_dataflow.py`:** A more complex example based on a real-world translation dataflow.
