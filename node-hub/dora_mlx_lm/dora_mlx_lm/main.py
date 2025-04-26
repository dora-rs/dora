import os
import pyarrow as pa
from dora import Node
from mlx_lm import load, generate


def main():
    model_path = os.getenv("MODEL_PATH", "Qwen/Qwen2.5-Omni-7B")
    model, tokenizer = load(model_path, tokenizer_config={"eos_token": "<|im_end|>"})

    node = Node()

    for event in node:
        if event["type"] == "INPUT" and event["id"] == "prompt":
            prompt = event["value"][0].as_py()

            messages = [{"role": "user", "content": prompt}]
            formatted_prompt = tokenizer.apply_chat_template(messages, add_generation_prompt=True)

            output = generate(
                model,
                tokenizer,
                prompt=formatted_prompt,
                max_tokens=100,
                temp=0.7,
                verbose=False
            )

            node.send_output("text", pa.array([output]))

        elif event["type"] == "STOP":
            break


if __name__ == "__main__":
    main()
