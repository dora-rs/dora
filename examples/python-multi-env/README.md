# Python Dataflow Example

This examples shows how to connect two different python virtual env with python.

```bash
uv venv --seed -p 3.11 -n env_1
uv venv --seed -p 3.11 -n env_2
dora build dataflow.yml --uv
dora run dataflow.yml --uv
```
