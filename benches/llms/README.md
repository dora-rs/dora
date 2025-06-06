# Benchmark LLM Speed

If you do not have a python virtual environment setup run

'''bash
uv venv --seed -p 3.11
'''

Then Use the following command to run the benchmark:

```bash
dora build transformers.yaml --uv
dora run transformers.yaml --uv
```

You should see a benchmark details.
