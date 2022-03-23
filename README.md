# dora-rs
Dataflow Oriented Robotic Architecture

## Python API Design

The Python API is probably going to look as follows:
```python
@register
async def function_name(state: DoraState, message: DoraMessage):
    return outputs
``` 

The philosophy is to use async function as primary instance to:
- Mitigate the risk of running unsafe data mutations.
- Managing several run at the same time with timeout / deadline capabilities
- Using Tokio Spawn to avoid thread locks on CPU bound runs.

## Getting started

I have made a simple example that can be run with:
```
cargo run start-python app:return_1

# Running this might required some shared library as:
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/miniconda3/lib
export PYTHONPATH=$PYTHONPATH:$(pwd)
```
That is going to listen to the key_expr "a" and run the `return_1` function within the `app.py` python async.

This is still very experimental.