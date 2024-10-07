# Dora openai echo example

This is a quick example to showcase how use the `dora-openai-server` to receive and send back data.

Dora Openai Server is still experimental and may change in the future.

Make sure to have, dora, pip and cargo installed.

```bash
dora up
dora build dataflow.yml
dora start dataflow.yml

# In a separate terminal
python api_client.py
dora stop
```
