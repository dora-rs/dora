# Design Overview

The dora framework is structured into different components:

![design diagram](overview.svg)


The following main components exist:

- **Nodes:** Dora nodes are separate, isolated processes that communicate with other nodes through the dora library. Nodes can be either a custom, user-specified program, or a dora runtime node, which allows to run dora _operators_. Nodes implement their own `main` function and thus have full control over their execution.

  Nodes use the dora _library_ to communicate with other nodes, which is available for mulitple languages (Rust, Python, WASM). Communication happens through a _communication layer_, which will be `zenoh` in our first version. We plan to add more options in the future. All communication layer implementations should be robust against disconnections, so operators should be able to keep running even if they lose the connection to the coordinator.
- **Operators:** Operators are light-weight, cooperative, library-based components that are executed by a dora runtime node. They must implement a specific interface, depending on the used language. Operators can use a wide range of advanced features provided by the dora runtime, for example priority scheduling or native deadline support.
- **Coordinator:** The coordinator is responsible for reading the dataflow from a YAML file, verifying it, and deploying the nodes and operators to the specified or automatically determined machines. It monitors the operator's health and implements high level cluster management functionality. For example, we could implement automatic scaling for cloud nodes or operator replication and restarts. The coordinator can be controlled through a command line program (CLI).


## Operators vs Custom Nodes

There are two ways to implement an operation in dora: Either as a dora operator, or as a custom nodes. Both approaches have their advantages and drawbacks, as explained below. In general, it is recommended to create dora operators and only use custom nodes when necessary.

Operators have the following advantages:

- They can use a wide range of advanced functionality provided by the dora runtime nodes. This includes special scheduling strategies and features such as deadlines. 
- They are _light-weight_, so they only occupy minimal amounts of memory. This makes it possible to run thousands of operators on the same machine.
- They can use runtime-managed state storage, for robustness or for sharing state with other operators.
- They _share the address space_ with other operators on the same node, which makes communication much faster.

Custom nodes provide a different set of advantages:

- Each node is a separate, isolated process, which can be important for security-critical operations.
- They support pinned resources. For example, a CPU core can be pinned to a custom node through the dataflow configuration file.
- They have full control over their execution, which allows to create complex input and wake-up rules.

