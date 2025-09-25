from __future__ import annotations

import yaml


class DataflowBuilder:
    """A dora dataflow."""

    def __init__(self, name: str = "dora-dataflow"):
        self.name = name
        self.nodes = []

    def add_node(self, id: str, **kwargs) -> Node:
        """Adds a new node to the dataflow."""
        node = Node(id, **kwargs)
        self.nodes.append(node)
        return node

    def to_yaml(self, path: str = None) -> str | None:
        """Generates the YAML representation of the dataflow."""
        dataflow_spec = {"nodes": [node.to_dict() for node in self.nodes]}
        if path:
            with open(path, "w") as f:
                yaml.dump(dataflow_spec, f)
            return None
        else:
            return yaml.dump(dataflow_spec)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass


class Output:
    """Represents an output from a node."""

    def __init__(self, node: Node, output_id: str):
        self.node = node
        self.output_id = output_id

    def __str__(self) -> str:
        return f"{self.node.id}/{self.output_id}"


class Node:
    """A node in a dora dataflow."""

    def __init__(self, id: str, **kwargs):
        self.id = id
        self.config = kwargs
        self.config["id"] = id
        self.operators = []

    def path(self, path: str) -> Node:
        """Sets the path to the executable or script."""
        self.config["path"] = path
        return self

    def args(self, args: str) -> Node:
        """Sets the command-line arguments for the node."""
        self.config["args"] = args
        return self

    def env(self, env: dict) -> Node:
        """Sets the environment variables for the node."""
        self.config["env"] = env
        return self

    def build(self, build_command: str) -> Node:
        """Sets the build command for the node."""
        self.config["build"] = build_command
        return self

    def git(
        self, url: str, branch: str = None, tag: str = None, rev: str = None
    ) -> Node:
        """Sets the Git repository for the node."""
        self.config["git"] = url
        if branch:
            self.config["branch"] = branch
        if tag:
            self.config["tag"] = tag
        if rev:
            self.config["rev"] = rev
        return self

    def add_operator(self, operator: Operator) -> Node:
        """Adds an operator to this node."""
        self.operators.append(operator)
        return self

    def add_output(self, output_id: str) -> Output:
        """Adds an output to the node and returns an Output object."""
        if "outputs" not in self.config:
            self.config["outputs"] = []
        if output_id not in self.config["outputs"]:
            self.config["outputs"].append(output_id)
        return Output(self, output_id)

    def add_input(
        self, input_id: str, source: str | Output, queue_size: int = None
    ) -> Node:
        """Adds a user-defined input to the node. Source can be a string or an Output object."""
        if "inputs" not in self.config:
            self.config["inputs"] = {}

        if isinstance(source, Output):
            source_str = str(source)
            if queue_size is not None:
                self.config["inputs"][input_id] = {
                    "source": source_str,
                    "queue_size": queue_size,
                }
            else:
                self.config["inputs"][input_id] = source_str
        else:
            if queue_size is not None:
                self.config["inputs"][input_id] = {
                    "source": source,
                    "queue_size": queue_size,
                }
            else:
                self.config["inputs"][input_id] = source
        return self

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
