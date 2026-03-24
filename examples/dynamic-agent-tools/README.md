# Dynamic Agent Tools Example

Demonstrates how an AI agent can dynamically extend its tool capabilities
at runtime — the core use case for agentic dataflows.

## Architecture

**Initial topology (1 tool):**
```
Timer (1 Hz) --> Agent --> tool-request --> Echo Tool --> response --> Agent
```

**After adding calculator tool:**
```
                                       ┌── Echo Tool ── response ──┐
Timer (1 Hz) --> Agent -- tool-request─┤                            ├── Agent
                                       └── Calc Tool ── response ──┘
```

Both tools receive all requests via the shared `tool-request` topic.
Each tool filters for its own `"tool"` field in the JSON request.

## Nodes

**agent** (`agent.py`) — Sends JSON tool requests (`{"tool": "echo", "message": "..."}`)
and logs responses. Cycles through echo and calc tasks.

**echo-tool** (`echo_tool.py`) — Built-in. Echoes back messages for `"tool": "echo"`.

**calc-tool** (`calc_tool.py`) — Dynamically added. Evaluates math expressions
for `"tool": "calc"` requests.

## Run

### Step 1: Start with echo tool only

```bash
pip install adora-rs pyarrow
adora up
adora start examples/dynamic-agent-tools/dataflow.yml --detach --name agent-demo
```

The agent sends requests. Echo tool responds to echo requests.
Calc requests go unanswered (no calc tool yet).

### Step 2: Add calculator tool

```bash
adora node add --from-yaml examples/dynamic-agent-tools/calc-tool-node.yml --dataflow agent-demo
```

Now the agent receives responses for both echo AND calc requests.

### Step 3: Remove calculator tool

```bash
adora node remove agent-demo calc-tool
```

Back to echo-only. Calc requests go unanswered again.

### Step 4: Clean up

```bash
adora stop --all
adora down
```

## What This Demonstrates

| Feature | How It's Used |
|---------|--------------|
| Dynamic tool addition | Agent gains new capabilities at runtime |
| Fan-out to multiple tools | Same `tool-request` topic, multiple subscribers |
| Tool-specific filtering | Each tool processes only its own request type |
| Graceful removal | Removing a tool doesn't affect others |
| AI agent architecture | Matches the LLM -> Tools service pattern |

## AI Agent Pattern

This example models the LLM function-calling pattern:
1. Agent decides which tool to call (simulated by cycling through tasks)
2. Agent sends a structured request (JSON with `tool` field)
3. Tool processes and responds
4. Agent receives and logs the response

In production, the agent node would be an LLM inference node, and tools
would be specialized nodes (web search, database query, code execution, etc.)
added dynamically based on the conversation context.
