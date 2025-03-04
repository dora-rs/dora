# Dora openai echo example

This is a quick example to showcase how use the `dora-openai-server` to receive and send back data.

Dora Openai Server is still experimental and may change in the future.

Make sure to have, dora, uv and cargo installed.

```bash
uv venv -p 3.11 --seed
uv pip install -e ../../apis/python/node
dora build dataflow.yml --uv
dora run dataflow.yml --uv

# In a separate terminal
python openai_api_client.py
dora stop
```
