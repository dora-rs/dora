# Python API

## Cutom Node

The custom node API allow you to integrate `dora` into your application. It allows you to retrieve input and send output in any fashion you want.  
#### `Node()`

`Node()` initiate a node from environment variables set by `dora-coordinator` 

```python
from dora import Node()

node = Node()
```

#### `.next()` or `__next__()` as an iterator

`.next()` gives you the next input that the node has received.

```python
input_id, value = node.next()

# or

for input_id, value in node:
```

#### `.send_output(output_id, data)`

`send_output` send data from the node.

```python
node.send_output("string", b"string")
```


### Try it out!

- Install python node API:
```bash
cd apis/python/node
python3 -m venv .env
source .env/bin/activate
pip install maturin
maturin develop
```

- Create a python file called `printer.py`:
```python
{{#include ../../binaries/coordinator/examples/nodes/python/printer.py}}
```

- Link it in your graph as:
```yaml
{{#include ../../binaries/coordinator/examples/graphs/python_test.yml:12:17}}
```

## Operator

The operator API gives you a framework for operator that is going to be managed by `dora`. This framework enable us to make optimisation and provide advanced features.

An operator requires an `on_input` method and requires to return a `DoraStatus` of 0 or 1, depending of it needs to continue or stop.

```python
class Operator:
    def on_input(
        self,
        input_id: str,
        value: bytes,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
```

> For Python, we recommend to allocate the operator on a single runtime. A runtime will share the same GIL with several operators making those operators run almost sequentially. See: [https://docs.rs/pyo3/latest/pyo3/marker/struct.Python.html#deadlocks](https://docs.rs/pyo3/latest/pyo3/marker/struct.Python.html#deadlocks)
### Try it out!

- Create an operator python file called `op.py`:
```python
{{#include ../../examples/python-operator/op.py}}
```

- Link it in your graph as:
```yaml
{{#include ../../binaries/coordinator/examples/graphs/mini-dataflow.yml:67:73}}
```