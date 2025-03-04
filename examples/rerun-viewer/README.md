# Python Dataflow Example

This examples shows how to create and connect dora to rerun.

## Getting Started

```bash
uv venv -p 3.11 --seed
uv pip install -e ../../apis/python/node
dora build dataflow.yml --uv
dora run dataflow.yml --uv
```
