# Dora MCP Server Example

This is a quick example to showcase how use the `dora-mcp-server` to receive and send back data.

Dora MCP Server is still experimental and may change in the future.

Make sure to have, dora, uv and cargo installed.

```bash
uv venv -p 3.11 --seed
uv pip install -e ../../apis/python/node --reinstall
dora build dataflow.yml --uv
dora run dataflow.yml --uv
```

You can use mpc inspector to test:

```bash
npx @modelcontextprotocol/inspector
```