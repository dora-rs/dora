# Python API Dataflow Example

This example demonstrates how to use the new Pythonic API to define a dataflow.

### Motivation

Traditionally, `dora` dataflows are defined using a declarative YAML file. While this approach is powerful and easy to read, it can be unfamiliar to developers who are more accustomed to an imperative programming style.

The `doraflow` Python package provides a new way to define dataflows using a Pythonic API. This allows developers to:

- Define dataflows programmatically, which is more natural for many Python developers.
- Create dynamic dataflows that can be modified at runtime.
- Integrate `dora` more easily into existing Python applications and workflows.

This example directory showcases how to use the `doraflow` package to create simple, advanced, and complex dataflows.

## Running the Examples

1.  **Install Graphviz:** The visualization feature requires the Graphviz command-line tools to be installed on your system.

    *   **macOS:** `brew install graphviz`
    *   **Ubuntu/Debian:** `sudo apt-get install graphviz`

2.  **Install the new `doraflow` package with test dependencies:**

    ```bash
    pip install -e "apis/python/doraflow[dev]"
    ```

2.  **Run the unit tests:**

    ```bash
    pytest apis/python/doraflow/tests
    ```

3.  **Run the basic example:**

    ```bash
    python examples/python-api-dataflow/main.py
    ```

    This will generate a `dataflow.yml` file and a `yolo-dataflow.gv.png` visualization.

4.  **Run the advanced example:**

    ```bash
    python examples/python-api-dataflow/advanced_dataflow.py
    ```

    This will generate an `advanced_dataflow.yml` file and an `advanced-dataflow.gv.png` visualization.

5.  **Run the complex Reachy example:**

    ```bash
    python examples/python-api-dataflow/reachy_dataflow.py
    ```

    This will generate a `reachy_dataflow.yml` file and a `reachy-dataflow.gv.png` visualization.

6.  **Run a generated dataflow using the `dora` CLI:**

    ```bash
    dora up
    dora start examples/python-api-dataflow/dataflow.yml
    ```