# Python API

## Operator

The operator API is a framework for you to implement. The implemented operator will be managed by `dora`. This framework enable us to make optimisation and provide advanced features. It is the recommended way of using `dora`.

An operator requires an `on_input` method and requires to return a `DoraStatus` of 0 or 1, depending of it needs to continue or stop.

```python
class Operator:
    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
```

> For Python, we recommend to allocate the operator on a single runtime. A runtime will share the same GIL with several operators making those operators run almost sequentially. See: [https://docs.rs/pyo3/latest/pyo3/marker/struct.Python.html#deadlocks](https://docs.rs/pyo3/latest/pyo3/marker/struct.Python.html#deadlocks)
### Try it out!

- Create an operator python file called `object_detection.py`:
```python
{{#include ../../examples/python-dataflow/object_detection.py}}
```

- Link it in your graph as:
```yaml
{{#include ../../examples/python-dataflow/dataflow.yml:14:20}}
```

## Custom Node

The custom node API allow you to integrate `dora` into your application. It allows you to retrieve input and send output in any fashion you want.  
#### `Node()`

`Node()` initiate a node from environment variables set by `dora-coordinator` 

```python
from dora import Node

node = Node()
```

#### `.next()` or `__next__()` as an iterator

`.next()` gives you the next input that the node has received. It blocks until the next input becomes available. It will return `None` when all senders has been dropped.

```python
input_id, value, metadata = node.next()

# or

for input_id, value, metadata in node:
```

#### `.send_output(output_id, data)`

`send_output` send data from the node.

```python
node.send_output("string", b"string", {"open_telemetry_context": "7632e76"})
```

### Try it out!

- Install python node API:
```bash
pip install dora-rs
```

- Create a python file called `webcam.py`:
```python
{{#include ../../examples/python-dataflow/webcam.py}}
```

- Link it in your graph as:
```yaml
{{#include ../../examples/python-dataflow/dataflow.yml:6:12}}
```
