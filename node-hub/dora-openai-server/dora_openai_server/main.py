"""FastAPI server with OpenAI compatibility and DORA integration,
sending text and image data on separate DORA topics.
"""

import asyncio
import base64
import time  # For timestamps
import uuid  # For generating unique request IDs
from typing import Any, List, Literal, Optional, Union

import pyarrow as pa
import uvicorn
from dora import Node
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

# --- DORA Configuration ---
DORA_RESPONSE_TIMEOUT_SECONDS = 20
DORA_TEXT_OUTPUT_TOPIC = "user_text_input"
DORA_IMAGE_OUTPUT_TOPIC = "user_image_input"
DORA_RESPONSE_INPUT_TOPIC = "chat_completion_result"  # Topic FastAPI listens on

app = FastAPI(
    title="DORA OpenAI-Compatible Demo Server (Separate Topics)",
    description="Sends text and image data on different DORA topics and awaits a consolidated response.",
)


# --- Pydantic Models ---
class ImageUrl(BaseModel):
    url: str
    detail: Optional[str] = "auto"


class ContentPartText(BaseModel):
    type: Literal["text"]
    text: str


class ContentPartImage(BaseModel):
    type: Literal["image_url"]
    image_url: ImageUrl


ContentPart = Union[ContentPartText, ContentPartImage]


class ChatCompletionMessage(BaseModel):
    role: str
    content: Union[str, List[ContentPart]]


class ChatCompletionRequest(BaseModel):
    model: str
    messages: List[ChatCompletionMessage]
    temperature: Optional[float] = 1.0
    max_tokens: Optional[int] = 100


class ChatCompletionChoiceMessage(BaseModel):
    role: str
    content: str


class ChatCompletionChoice(BaseModel):
    index: int
    message: ChatCompletionChoiceMessage
    finish_reason: str
    logprobs: Optional[Any] = None


class Usage(BaseModel):
    prompt_tokens: int
    completion_tokens: int
    total_tokens: int


class ChatCompletionResponse(BaseModel):
    id: str
    object: str = "chat.completion"
    created: int
    model: str
    choices: List[ChatCompletionChoice]
    usage: Usage
    system_fingerprint: Optional[str] = None


# --- DORA Node Initialization ---
# This dictionary will hold unmatched responses if we implement more robust concurrent handling.
# For now, it's a placeholder for future improvement.
# unmatched_dora_responses = {}

try:
    node = Node()
    print("FastAPI Server: DORA Node initialized.")
except Exception as e:
    print(
        f"FastAPI Server: Failed to initialize DORA Node. Running in standalone API mode. Error: {e}"
    )
    node = None


@app.post("/v1/chat/completions", response_model=ChatCompletionResponse)
async def create_chat_completion(request: ChatCompletionRequest):
    internal_request_id = str(uuid.uuid4())
    openai_chat_id = f"chatcmpl-{internal_request_id}"
    current_timestamp = int(time.time())

    print(f"FastAPI Server: Processing request_id: {internal_request_id}")

    user_text_parts = []
    user_image_bytes: Optional[bytes] = None
    user_image_content_type: Optional[str] = None
    data_sent_to_dora = False

    for message in reversed(request.messages):
        if message.role == "user":
            if isinstance(message.content, str):
                user_text_parts.append(message.content)
            elif isinstance(message.content, list):
                for part in message.content:
                    if part.type == "text":
                        user_text_parts.append(part.text)
                    elif part.type == "image_url":
                        if user_image_bytes:  # Use only the first image
                            print(
                                f"FastAPI Server (Req {internal_request_id}): Warning - Multiple images found, using the first one."
                            )
                            continue
                        image_url_data = part.image_url.url
                        if image_url_data.startswith("data:image"):
                            try:
                                header, encoded_data = image_url_data.split(",", 1)
                                user_image_content_type = header.split(":")[1].split(
                                    ";"
                                )[0]
                                user_image_bytes = base64.b64decode(encoded_data)
                                print(
                                    f"FastAPI Server (Req {internal_request_id}): Decoded image {user_image_content_type}, size: {len(user_image_bytes)} bytes"
                                )
                            except Exception as e:
                                print(
                                    f"FastAPI Server (Req {internal_request_id}): Error decoding base64 image: {e}"
                                )
                                raise HTTPException(
                                    status_code=400,
                                    detail=f"Invalid base64 image data: {e}",
                                )
                        else:
                            print(
                                f"FastAPI Server (Req {internal_request_id}): Warning - Remote image URL '{image_url_data}' ignored. Only data URIs supported."
                            )
            # Consider if you want to break after the first user message or aggregate all
            # break

    final_user_text = "\n".join(reversed(user_text_parts)) if user_text_parts else ""
    prompt_tokens = len(final_user_text)

    if node:
        if final_user_text:
            text_payload = {"request_id": internal_request_id, "text": final_user_text}
            arrow_text_data = pa.array([text_payload])
            node.send_output(DORA_TEXT_OUTPUT_TOPIC, arrow_text_data)
            print(
                f"FastAPI Server (Req {internal_request_id}): Sent text to DORA topic '{DORA_TEXT_OUTPUT_TOPIC}'."
            )
            data_sent_to_dora = True

        if user_image_bytes:
            image_payload = {
                "request_id": internal_request_id,
                "image_bytes": user_image_bytes,
                "image_content_type": user_image_content_type
                or "application/octet-stream",
            }
            arrow_image_data = pa.array([image_payload])
            node.send_output(DORA_IMAGE_OUTPUT_TOPIC, arrow_image_data)
            print(
                f"FastAPI Server (Req {internal_request_id}): Sent image to DORA topic '{DORA_IMAGE_OUTPUT_TOPIC}'."
            )
            prompt_tokens += len(user_image_bytes)  # Crude image token approximation
            data_sent_to_dora = True

    response_str: str
    if not data_sent_to_dora:
        if node is None:
            response_str = "DORA node not available. Cannot process request."
        else:
            response_str = "No user text or image found to send to DORA."
        print(f"FastAPI Server (Req {internal_request_id}): {response_str}")
    else:
        print(
            f"FastAPI Server (Req {internal_request_id}): Waiting for response from DORA on topic '{DORA_RESPONSE_INPUT_TOPIC}'..."
        )
        response_str = f"Timeout: No response from DORA for request_id {internal_request_id} within {DORA_RESPONSE_TIMEOUT_SECONDS}s."

        # WARNING: This blocking `node.next()` loop is not ideal for highly concurrent requests
        # in a single FastAPI worker process, as one request might block others or consume
        # a response meant for another if `request_id` matching isn't perfect or fast enough.
        # A more robust solution would involve a dedicated listener task and async Futures/Queues.
        start_wait_time = time.monotonic()
        while time.monotonic() - start_wait_time < DORA_RESPONSE_TIMEOUT_SECONDS:
            remaining_timeout = DORA_RESPONSE_TIMEOUT_SECONDS - (
                time.monotonic() - start_wait_time
            )
            if remaining_timeout <= 0:
                break

            event = node.next(
                timeout=min(1.0, remaining_timeout)
            )  # Poll with a smaller timeout

            if event is None:  # Timeout for this poll iteration
                continue

            if event["type"] == "INPUT" and event["id"] == DORA_RESPONSE_INPUT_TOPIC:
                response_value_arrow = event["value"]
                if response_value_arrow and len(response_value_arrow) > 0:
                    dora_response_data = response_value_arrow[
                        0
                    ].as_py()  # Expecting a dict
                    if isinstance(dora_response_data, dict):
                        resp_request_id = dora_response_data.get("request_id")
                        if resp_request_id == internal_request_id:
                            response_str = dora_response_data.get(
                                "response_text",
                                f"DORA response for {internal_request_id} missing 'response_text'.",
                            )
                            print(
                                f"FastAPI Server (Req {internal_request_id}): Received correlated DORA response."
                            )
                            break  # Correct response received
                        # This response is for another request. Ideally, store it.
                        print(
                            f"FastAPI Server (Req {internal_request_id}): Received DORA response for different request_id '{resp_request_id}'. Discarding and waiting. THIS IS A CONCURRENCY ISSUE."
                        )
                    # unmatched_dora_responses[resp_request_id] = dora_response_data # Example of storing
                    else:
                        response_str = f"Unrecognized DORA response format for {internal_request_id}: {str(dora_response_data)[:100]}"
                        break
                else:
                    response_str = (
                        f"Empty response payload from DORA for {internal_request_id}."
                    )
                    break
            elif event["type"] == "ERROR":
                response_str = f"Error event from DORA for {internal_request_id}: {event.get('value', event.get('error', 'Unknown DORA Error'))}"
                print(response_str)
                break
        else:  # Outer while loop timed out
            print(
                f"FastAPI Server (Req {internal_request_id}): Overall timeout waiting for DORA response."
            )

    completion_tokens = len(response_str)
    total_tokens = prompt_tokens + completion_tokens

    return ChatCompletionResponse(
        id=openai_chat_id,
        created=current_timestamp,
        model=request.model,
        choices=[
            ChatCompletionChoice(
                index=0,
                message=ChatCompletionChoiceMessage(
                    role="assistant", content=response_str
                ),
                finish_reason="stop",
            )
        ],
        usage=Usage(
            prompt_tokens=prompt_tokens,
            completion_tokens=completion_tokens,
            total_tokens=total_tokens,
        ),
    )


@app.get("/v1/models")
async def list_models():
    return {
        "object": "list",
        "data": [
            {
                "id": "dora-multi-stream-vision",
                "object": "model",
                "created": int(time.time()),
                "owned_by": "dora-ai",
                "permission": [],
                "root": "dora-multi-stream-vision",
                "parent": None,
            },
        ],
    }


async def run_fastapi_server_task():
    config = uvicorn.Config(app, host="0.0.0.0", port=8000, log_level="info")
    server = uvicorn.Server(config)
    print("FastAPI Server: Uvicorn server starting.")
    await server.serve()
    print("FastAPI Server: Uvicorn server stopped.")


async def run_dora_main_loop_task():
    if not node:
        print("FastAPI Server: DORA node not initialized, DORA main loop skipped.")
        return
    print("FastAPI Server: DORA main loop listener started (for STOP event).")
    try:
        while True:
            # This loop is primarily for the main "STOP" event for the FastAPI node itself.
            # Individual request/response cycles are handled within the endpoint.
            event = node.next(timeout=1.0)  # Check for STOP periodically
            if event is None:
                await asyncio.sleep(0.01)  # Yield control if no event
                continue
            if event["type"] == "STOP":
                print(
                    "FastAPI Server: DORA STOP event received. Requesting server shutdown."
                )
                # Attempt to gracefully shut down Uvicorn
                # This is tricky; uvicorn's server.shutdown() or server.should_exit might be better
                # For simplicity, we cancel the server task.
                for task in asyncio.all_tasks():
                    # Identify the server task more reliably if possible
                    if (
                        task.get_coro().__name__ == "serve"
                        and hasattr(task.get_coro(), "cr_frame")
                        and isinstance(
                            task.get_coro().cr_frame.f_locals.get("self"),
                            uvicorn.Server,
                        )
                    ):
                        task.cancel()
                        print(
                            "FastAPI Server: Uvicorn server task cancellation requested."
                        )
                break
            # Handle other unexpected general inputs/errors for the FastAPI node if necessary
            # elif event["type"] == "INPUT":
            #     print(f"FastAPI Server (DORA Main Loop): Unexpected DORA input on ID '{event['id']}'")

    except asyncio.CancelledError:
        print("FastAPI Server: DORA main loop task cancelled.")
    except Exception as e:
        print(f"FastAPI Server: Error in DORA main loop: {e}")
    finally:
        print("FastAPI Server: DORA main loop listener finished.")


async def main_async_runner():
    server_task = asyncio.create_task(run_fastapi_server_task())

    # Only run the DORA main loop if the node was initialized.
    # This loop is mainly for the STOP event.
    dora_listener_task = None
    if node:
        dora_listener_task = asyncio.create_task(run_dora_main_loop_task())
        tasks_to_wait_for = [server_task, dora_listener_task]
    else:
        tasks_to_wait_for = [server_task]

    done, pending = await asyncio.wait(
        tasks_to_wait_for,
        return_when=asyncio.FIRST_COMPLETED,
    )

    for task in pending:
        print(f"FastAPI Server: Cancelling pending task: {task.get_name()}")
        task.cancel()

    if pending:
        await asyncio.gather(*pending, return_exceptions=True)
    print("FastAPI Server: Application shutdown complete.")


def main():
    print("FastAPI Server: Starting application...")
    try:
        asyncio.run(main_async_runner())
    except KeyboardInterrupt:
        print("FastAPI Server: Keyboard interrupt received. Shutting down.")
    finally:
        print("FastAPI Server: Exited main function.")


if __name__ == "__main__":
    main()
