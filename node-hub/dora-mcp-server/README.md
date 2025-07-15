# Dora MCP Server

This node can provide an MCP Server, which will proxy the request to one or more other nodes in the dora application.

Dora MCP Server is still experimental and may change in the future.


## How to use

```yaml
nodes:
  - id: mcp-server
    build: cargo build -p dora-mcp-server --release
    path: ../../target/release/dora-mcp-server
    outputs:
      - counter
    inputs:
      counter_reply: counter/reply
    env:
      MCP_SERVER_CONFIG: config.toml
```

use `MCP_SERVER_CONFIG` set config file, it supports toml, json or yaml format.

An example config file:

```toml
name = "MCP Server Example"
version = "0.1.0"

# You can set your custom listen address and endpoint here.
# Default listen address is "0.0.0.0:8008" and endpoint is "mcp".
listen_addr = "0.0.0.0:8181"
endpoint = "mcp"

[[mcp_tools]]
name = "counter_decrement"
args = []
input_schema = "empty_object.json"
node_id = "counter"

[[mcp_tools]]
name = "counter_increment"
args = []
input_schema = "empty_object.json"
node_id = "counter"

[[mcp_tools]]
name = "counter_get_value"
args = []
input_schema = "empty_object.json"
node_id = "counter"
```

