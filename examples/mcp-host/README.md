<<<<<<< HEAD
# Dora MCP Host Example

This is a comprehensive example demonstrating how to use the Dora Model Context Protocol (MCP) Host to create a multi-node dataflow system that integrates with various AI providers and local services.

## Overview

This example showcases a complete MCP (Model Context Protocol) implementation using Dora's dataflow architecture. It includes:

- **MCP Host**: A server that exposes OpenAI-compatible API endpoints
- **MCP Server**: A backend service that handles tool calls and routing
- **Local Services**: Python nodes that provide specific functionalities
- **Multi-provider Support**: Integration with various AI providers (Moonshot, Gemini, etc.)

## Architecture

The system consists of the following components:

### Core Components

1. **dora-mcp-host** (`dora-mcp-host`)
   - Acts as an OpenAI-compatible API server
   - Listens on `0.0.0.0:8118` with `/v1` endpoint
   - Routes requests to appropriate providers and tools
   - Handles chat completions, model listings, and tool calls

2. **dora-mcp-server** (`dora-mcp-server`)
   - Central routing hub for MCP tool calls
   - Manages communication between host and local services
   - Configured via `mcp_server.toml`

3. **Local Service Nodes**:
   - **local.py**: Provides local information services
   - **telepathy.py**: Demonstrates "telepathic" capabilities

4. **dora-echo** (`dora-echo`)
   - Echo service for testing and debugging

### Configuration Files

#### mcp_host.toml
Configures the MCP host with:
- **Listen Address**: `0.0.0.0:8118`
- **API Endpoint**: `v1`
- **Providers**:
  - Moonshot (Kimi) API integration
  - Gemini API integration
  - Local Dora provider
- **Models**:
  - `kimi-latest` (Moonshot)
  - `gemini-2.0-flash` (Gemini)
- **MCP Servers**:
  - Amap Maps server (external)
  - Local streamable server

#### mcp_server.toml
Defines available tools:
- **signature_dish**: Returns signature dishes from restaurants
- **happiest_kindergarten**: Provides kindergarten information
- **telepathy**: Demonstrates user preference detection


### Data Flow

The system follows this data flow:
1. Client sends request to MCP Host (`127.0.0.1:8118/v1`)
2. MCP Host processes the request and determines routing
3. For tool calls, requests are sent to MCP Server
4. MCP Server routes to appropriate local services (local.py or telepathy.py)
5. Local services process and return responses
6. Responses flow back through the chain to the client

## Prerequisites

Make sure you have the following installed:
- **dora**: Dora framework
- **uv**: Python package manager
- **cargo**: Rust package manager
- **Python 3.11+**: For Python nodes

## Setup and Running

### 1. Environment Setup
```bash
# Create and activate Python virtual environment
uv venv -p 3.11 --seed
source .venv/bin/activate  # On Linux/macOS
# or .venv\Scripts\activate  # On Windows

# Install Python dependencies
uv pip install -e ../../apis/python/node --reinstall
```

### 2. Build the System
```bash
# Build all components
dora build dataflow.yml --uv
```

### 3. Run the System
```bash
# Start the dataflow
dora run dataflow.yml --uv
```

### 4. Test the System
In a separate terminal:
```bash
# Run the test client
uv run test_client.py
```

## API Endpoints

### Available Endpoints
- `GET /v1/models` - List available models
- `POST /v1/chat/completions` - Chat completions with tool support

### Tool Functions
- `signature_dish` - Get random signature dish
- `happiest_kindergarten` - Get kindergarten information  
- `telepathy` - Get favorite star and movie

## Provider Configuration

### Moonshot (Kimi)
- Requires `MOONSHOT_API_KEY` environment variable
- Model: `kimi-latest`

### Gemini
- Requires `GEMINI_API_KEY` environment variable  
- Model: `gemini-2.0-flash`

### Dora Local
- Uses local Dora dataflow for processing
- No external API key required

## Troubleshooting

### Common Issues
1. **Port Conflicts**: Ensure ports 8118 and 8228 are available
2. **Environment Variables**: Set required API keys for external providers
3. **Dependencies**: Ensure all Python and Rust dependencies are installed
4. **Virtual Environment**: Activate the correct Python environment

This example demonstrates the power and flexibility of Dora's MCP integration for building sophisticated AI-powered applications with multiple providers and local processing capabilities.
=======
# Dora Openai MCP Host Example

This is a quick example to showcase how use the `dora-openai-server` to receive and send back data.

Dora Openai Server is still experimental and may change in the future.

Make sure to have, dora, uv and cargo installed.

```bash
uv venv -p 3.11 --seed
uv pip install -e ../../apis/python/node --reinstall
dora build dataflow.yml --uv
dora run dataflow.yml --uv

# In a separate terminal
uv run test_client.py
dora stop
```
>>>>>>> 6dad2bdc (Add mcp-host node and mcp-server node)
