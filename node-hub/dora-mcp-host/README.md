# Dora MCP Host

## Overview

Dora MCP Host is a Model Context Protocol (MCP) host implementation based on the Dora framework. It supports integration with multiple AI providers, tool calls, and local service extensions. The host is OpenAI API compatible and supports multiple models and custom local tools.

## Features

- OpenAI API compatibility (`/v1/models`, `/v1/chat/completions`)
- Multi-model, multi-provider routing
- Tool calls and local service integration

## How to use

```yaml
nodes:
  - id: dora-mcp-host
    build: cargo build -p dora-mcp-host --release
    path: ../../target/release/dora-mcp-host
    outputs:
      - text
    inputs:
      text: dora-echo/echo
    env:
      CONFIG: mcp_host.toml
```

use `CONFIG` set config file, it supports toml, json or yaml format.

An example config file:

```toml
listen_addr = "0.0.0.0:8118"
endpoint = "v1"

[[providers]]
id = "moonshot"
kind ="openai"
api_key = "env:MOONSHOT_API_KEY"
api_url = "https://api.moonshot.cn/v1"

[[providers]]
id = "gemini"
kind ="gemini"
api_key = "env:GEMINI_API_KEY"
api_url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent"

[[providers]]
id = "dora"
kind ="dora"
output = "output"

[[models]]
id = "kimi-latest"
route = { provider = "moonshot", model = "kimi-latest" }

[[models]]
id = "gemini-2.0-flash"
route = { provider = "gemini", model = "gemini-2.0-flash" }

[[mcp.servers]]
name = "amap-maps"
protocol = "stdio"
command = "npx"
args = ["-y", "@amap/amap-maps-mcp-server"]
envs = {AMAP_MAPS_API_KEY = "your_amap_maps_api_key_here"}

[[mcp.servers]]
name = "local"
protocol = "streamable"
url = "http://127.0.0.1:8228/mcp"
```

In `[[providers]]` section, if `api_key` is starts with `env:`, it will get the value from the system's environment variables.

In `[[models]]` section, we define local models that are provided for external access. These models will be routed to different AI providers in the route definition. In the above example, if the model requested by the user is kimi-latest, the request will be handled by moonshot. If the model requested is gemini-2.0-flash, the request will be handled by gemini.

In `[[mcp.servers]]` section, you can define multiple mcp servers for use.