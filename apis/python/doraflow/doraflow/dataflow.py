from __future__ import annotations

import subprocess
import tempfile
from typing import Callable

import yaml
import graphviz

class Dataflow:
    """A dora dataflow."""

    def __init__(self, name: str = "dora-dataflow"):
        self.name = name
        self.nodes = []

    def add_node(self, id: str, **kwargs) -> 'Node':
        """Adds a new node to the dataflow."""
        node = Node(id, **kwargs)
        self.nodes.append(node)
        return node

    def add_edge(
        self,
        source_node: str | Node,
        source_output: str,
        dest_node: str | Node,
        dest_input: str,
        queue_size: int = None,
    ):
        """Adds a direct connection between two nodes."""
        if isinstance(source_node, Node):
            source_node = source_node.id
        if isinstance(dest_node, Node):
            dest_node = dest_node.id

        for node in self.nodes:
            if node.id == dest_node:
                if "inputs" not in node.config:
                    node.config["inputs"] = {}
                source = f"{source_node}/{source_output}"
                if queue_size is not None:
                    node.config["inputs"][dest_input] = {
                        "source": source,
                        "queue_size": queue_size,
                    }
                else:
                    node.config["inputs"][dest_input] = source

            if node.id == source_node:
                if "outputs" not in node.config:
                    node.config["outputs"] = []
                if source_output not in node.config["outputs"]:
                    node.config["outputs"].append(source_output)

    def add_conditional_edge(
        self,
        source_node: str | Node,
        source_output: str,
        condition: Callable[[any], str],
        destinations: dict[str, str | Node],
    ):
        """Adds a conditional edge from a source node to multiple possible destination nodes."""
        # This is a simplified implementation.
        # A more complete implementation would create a router node.
        print("Conditional edges are not fully implemented yet.")

    def to_yaml(self, path: str = None) -> str | None:
        """Generates the YAML representation of the dataflow."""
        dataflow_spec = {"nodes": [node.to_dict() for node in self.nodes]}
        if path:
            with open(path, "w") as f:
                yaml.dump(dataflow_spec, f)
            return None
        else:
            return yaml.dump(dataflow_spec)

    def run(self):
        """Generates the YAML file and runs the dataflow in the foreground."""
        with tempfile.NamedTemporaryFile(mode="w", delete=False) as f:
            self.to_yaml(f.name)
            subprocess.run(["dora", "start", f.name], check=True)

    def start(self):
        """Generates the YAML file and starts the dataflow in the background."""
        with tempfile.NamedTemporaryFile(mode="w", delete=False) as f:
            self.to_yaml(f.name)
            subprocess.run(["dora", "start", f.name, "--detach"], check=True)

    def stop(self):
        """Stops a dataflow that was started in the background."""
        subprocess.run(["dora", "stop", "--all"], check=True)

    def visualize(self, format: str = "png", view: bool = True):
        """Generates a visual representation of the dataflow graph."""
        dot = graphviz.Digraph(self.name)
        for node in self.nodes:
            dot.node(node.id, node.id)
            if "inputs" in node.config:
                for input_id, source in node.config["inputs"].items():
                    if isinstance(source, dict):
                        source = source["source"]
                    source_node_id = source.split("/")[0]
                    dot.edge(source_node_id, node.id, label=input_id)
        dot.render(format=format, view=view)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass


class Node:
    """A node in a dora dataflow."""

    def __init__(self, id: str, **kwargs):
        self.id = id
        self.config = kwargs
        self.config["id"] = id
        self.operators = []

    def path(self, path: str) -> "Node":
        """Sets the path to the executable or script."""
        self.config["path"] = path
        return self

    def args(self, args: str) -> "Node":
        """Sets the command-line arguments for the node."""
        self.config["args"] = args
        return self

    def env(self, env: dict) -> "Node":
        """Sets the environment variables for the node."""
        self.config["env"] = env
        return self

    def build(self, build_command: str) -> "Node":
        """Sets the build command for the node."""
        self.config["build"] = build_command
        return self

    def git(
        self, url: str, branch: str = None, tag: str = None, rev: str = None
    ) -> "Node":
        """Sets the Git repository for the node."""
        self.config["git"] = url
        if branch:
            self.config["branch"] = branch
        if tag:
            self.config["tag"] = tag
        if rev:
            self.config["rev"] = rev
        return self

    def add_operator(self, operator: "Operator") -> "Node":
        """Adds an operator to this node."""
        self.operators.append(operator)
        return self

    def add_input(self, input_id: str, source: str) -> "Node":
        """Adds a user-defined input to the node."""
        if "inputs" not in self.config:
            self.config["inputs"] = {}
        self.config["inputs"][input_id] = source
        return self

    def send(self, output_id: str) -> "Output":
        """Specifies an output of the node to be connected to another node's input."""
        return Output(self, output_id)

    def to_dict(self) -> dict:
        """Returns the dictionary representation of the node."""
        config = self.config.copy()
        if self.operators:
            config["operators"] = [op.to_dict() for op in self.operators]
        return config


class Operator:
    """An operator in a dora dataflow."""

    def __init__(
        self,
        id: str,
        name: str = None,
        description: str = None,
        build: str = None,
        python: str = None,
        shared_library: str = None,
        send_stdout_as: str = None,
    ):
        self.id = id
        self.config = {"id": id}
        if name:
            self.config["name"] = name
        if description:
            self.config["description"] = description
        if build:
            self.config["build"] = build
        if python:
            self.config["python"] = python
        if shared_library:
            self.config["shared-library"] = shared_library
        if send_stdout_as:
            self.config["send_stdout_as"] = send_stdout_as

    def to_dict(self) -> dict:
        """Returns the dictionary representation of the operator."""
        return self.config


class Output:
    """An output of a node."""

    def __init__(self, node: Node, output_id: str):
        self.node = node
        self.output_id = output_id

    def to(self, node: "Node", input_id: str, queue_size: int = None):
        """Connects the output to the input of another node."""
        if "inputs" not in node.config:
            node.config["inputs"] = {}

        source = f"{self.node.id}/{self.output_id}"
        if queue_size is not None:
            node.config["inputs"][input_id] = {"source": source, "queue_size": queue_size}
        else:
            node.config["inputs"][input_id] = source

        if "outputs" not in self.node.config:
            self.node.config["outputs"] = []
        if self.output_id not in self.node.config["outputs"]:
            self.node.config["outputs"].append(self.output_id)
