# Python API

## Cutom Node

The custom node API allow you to integrate `dora` into your application. It allows you to retrieve input and send output in any fashion you want. 

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

### Try it out!

- Create an operator python file called `op.py`:
```python
{{#include ../../examples/python-operator/op.py}}
```

- Link it in your graph as:
```yaml
{{#include ../../binaries/coordinator/examples/graphs/mini-dataflow.yml:67:73}}
```