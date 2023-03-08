# Dataflow Specification

Dataflows are specified through a YAML file. This section presents our current draft for the file format. It only includes basic functionality for now, we will extend it later when we introduce more advanced features.

## Dataflow

Dataflows are specified through the following format:

```yaml
communication:
  zenoh:
    prefix: /example-python-no-webcam-dataflow

nodes:
    - id: foo
      # ... (see below)
    - id: bar
      # ... (see below)
```

### Inputs and Outputs

Each operator or custom node has a separate namespace for its outputs. To refer to outputs, the <operator>/<output> syntax is used. This way, there are no name conflicts between operators.

Input operands are specified using the <name>: <operator>/<output> syntax, where <data> is the internal name that should be used for the operand. The main advantage of this name mapping is that the same operator executable can be reused multiple times on different input.

## Nodes

Nodes are defined using the following format:

```yaml
nodes:
  - id: some-unique-id
    # For nodes with multiple operators
    operators:
      - id: operator-1
        # ... (see below)
      - id: operator-2
        # ... (see below)



  - id: some-unique-id-2
    custom:
      source: path/to/timestamp
      env:
        - ENVIRONMENT_VARIABLE_1: true
      working-directory: some/path

      inputs:
        input_1: operator_2/output_4
        input_2: custom_node_2/output_4
      outputs:
        - output_1
 
  # Unique operator
  - id: some-unique-id-3
    operator:
        # ... (see below)
```

Nodes must provide either a `operators` field, or a `custom` field, but not both. Nodes with an `operators` field run a dora runtime process, which runs and manages the specified operators. Nodes with a `custom` field, run a custom executable.

### Custom Nodes

Custom nodes specify the executable name and arguments like a normal shell operation through the `run` field. Through the optional `env` field, it is possible to set environment variables for the process. The optional `working-directory` field allows to overwrite the directory in which the program is started.

To integrate with the rest of the dora dataflow, custom nodes must specify their inputs and outputs, similar to operators. They can reference outputs of both operators, and other custom nodes.

## Operators

Operators are defined through the following format:

```yaml
- id: unique-operator-id
  name: Human-Readable Operator Name
  description: An optional description of the operators's purpose.

  inputs:
    input_1: source_operator_2/output_1
    input_2: custom_node_1/output_1
  outputs:
    - output_1

  ## ONE OF:
  shared_library: "path/to/shared_lib" # file extension and `lib` prefix are added automatically
  python: "path/to/python_file.py"
  wasm: "path/to/wasm_file.wasm"
```

Operators must list all their inputs and outputs. Inputs can be linked to arbitrary outputs of other operators or custom nodes.

There are multiple ways to implement an operator:

- as a C-compatible shared library
- as a Python object
- as a WebAssembly (WASM) module

Each operator must specify exactly one implementation. The implementation must follow a specific format that is specified by dora.

## Example

```yaml
{{#include ../../examples/rust-dataflow/dataflow.yml}}
```
## Communication

The mandatory `communication` key specifies how dora nodes and operators should communicate with each other. Dora supports the following backends:

- **[Zenoh](https://zenoh.io/):** The zenoh project implements a distributed publisher/subscriber system with automated routing. To communicate over zenoh, add the following key to your dataflow configuration:

  ```yaml
  communication:
    zenoh:
      prefix: /some-unique-prefix
  ```

  The specified `prefix` is added to all pub/sub topics. It is useful for filtering messages (e.g. in a logger) when other applications use `zenoh` in parallel. Dora will extend the given prefix with a newly generated UUID on each run, to ensure that multiple instances of the same dataflow run concurrently without interfering with each other.

  Zenoh is quite flexible and can be easily scaled to distributed deployment. It does not require any extra setup since it supports peer-to-peer communication without an external broker. The drawback of zenoh is that it is still in an early stage of development, so it might still have reliability and performance issues.

  _Note:_ Dora currently only supports local deployments, so interacting with remote nodes/operators is not possible yet.
  
## TODO: Integration with ROS 1/2

To integrate dora-rs operators with ROS1 or ROS2 operators, we plan to provide special _bridge operators_. These operators act as a sink in one dataflow framework and push all messages to a different dataflow framework, where they act as source.

For example, we plan to provide a `to_ros_2` operator, which takes a single `data` input, which is then published to a specified ROS 2 dataflow.

