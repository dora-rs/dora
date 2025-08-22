from doraflow import Dataflow, Operator

# This example demonstrates some of the more advanced features of the
# dora-py API, including:
# - Defining a runtime node with multiple operators.
# - Using the send().to() syntax for defining edges.
# - Visualizing the dataflow.

with Dataflow(name="advanced-dataflow") as df:
    # Define a runtime node
    runtime = df.add_node(id="runtime")

    # Define two operators for the runtime node
    op1 = Operator(id="op1", python="operator1.py")
    op2 = Operator(id="op2", python="operator2.py")
    runtime.add_operator(op1)
    runtime.add_operator(op2)

    # Define a source node
    source = df.add_node(id="source", path="source.py")

    # Define a sink node
    sink = df.add_node(id="sink", path="sink.py")

    # Connect the nodes using the send().to() syntax
    source.send("output1").to(runtime, "input1")
    source.send("output2").to(runtime, "input2")
    # Here we imagine that op1 processes the data and sends it to op2,
    # which then sends it to the sink.
    # The internal connections between operators are not yet exposed in the API,
    # but this demonstrates the node-to-node connection.
    runtime.send("output_from_op2").to(sink, "input")

    # Visualize the dataflow
    df.visualize()

    # Generate the YAML file
    df.to_yaml("advanced_dataflow.yml")

    print("Generated advanced_dataflow.yml and visualization.")
