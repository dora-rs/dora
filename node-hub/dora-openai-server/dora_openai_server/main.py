"""TODO: Add docstring."""

import ast
import asyncio
from typing import List, Optional

import pyarrow as pa
import uvicorn
from dora import Node
from fastapi import FastAPI
from pydantic import BaseModel

DORA_RESPONSE_TIMEOUT = 10
app = FastAPI()


class ChatCompletionMessage(BaseModel):
    """TODO: Add docstring."""

    role: str
    content: str


class ChatCompletionRequest(BaseModel):
    """TODO: Add docstring."""

    model: str
    messages: List[ChatCompletionMessage]
    temperature: Optional[float] = 1.0
    max_tokens: Optional[int] = 100


class ChatCompletionResponse(BaseModel):
    """TODO: Add docstring."""

    id: str
    object: str
    created: int
    model: str
    choices: List[dict]
    usage: dict


node = Node()  # provide the name to connect to the dataflow if dynamic node


@app.post("/v1/chat/completions")
async def create_chat_completion(request: ChatCompletionRequest):
    """TODO: Add docstring."""
    data = next(
        (msg.content for msg in request.messages if msg.role == "user"),
        "No user message found.",
    )

    # Convert user_message to Arrow array
    # user_message_array = pa.array([user_message])
    # Publish user message to dora-echo
    # node.send_output("user_query", user_message_array)

    try:
        data = ast.literal_eval(data)
    except ValueError:
        print("Passing input as string")
    except SyntaxError:
        print("Passing input as string")
    if isinstance(data, list):
        data = pa.array(data)  # initialize pyarrow array
    elif (
        isinstance(data, str)
        or isinstance(data, int)
        or isinstance(data, float)
        or isinstance(data, dict)
    ):
        data = pa.array([data])
    else:
        data = pa.array(data)  # initialize pyarrow array
    node.send_output("v1/chat/completions", data)

    # Wait for response from dora-echo
    while True:
        event = node.next(timeout=DORA_RESPONSE_TIMEOUT)
        if event["type"] == "ERROR":
            response_str = "No response received. Err: " + event["value"][0].as_py()
            break
        if event["type"] == "INPUT" and event["id"] == "v1/chat/completions":
            response = event["value"]
            response_str = response[0].as_py() if response else "No response received"
            break

    return ChatCompletionResponse(
        id="chatcmpl-1234",
        object="chat.completion",
        created=1234567890,
        model=request.model,
        choices=[
            {
                "index": 0,
                "message": {"role": "assistant", "content": response_str},
                "finish_reason": "stop",
            },
        ],
        usage={
            "prompt_tokens": len(data),
            "completion_tokens": len(response_str),
            "total_tokens": len(data) + len(response_str),
        },
    )


@app.get("/v1/models")
async def list_models():
    """TODO: Add docstring."""
    return {
        "object": "list",
        "data": [
            {
                "id": "gpt-3.5-turbo",
                "object": "model",
                "created": 1677610602,
                "owned_by": "openai",
            },
        ],
    }


async def run_fastapi():
    """TODO: Add docstring."""
    config = uvicorn.Config(app, host="0.0.0.0", port=8000, log_level="info")
    server = uvicorn.Server(config)

    server = asyncio.gather(server.serve())
    while True:
        await asyncio.sleep(1)
        event = node.next(0.001)
        if event["type"] == "STOP":
            break


def main():
    """TODO: Add docstring."""
    asyncio.run(run_fastapi())


if __name__ == "__main__":
    asyncio.run(run_fastapi())
