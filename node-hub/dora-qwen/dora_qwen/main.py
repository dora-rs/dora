import os

import pyarrow as pa
from dora import Node
from transformers import AutoModelForCausalLM, AutoTokenizer

model_name = "Qwen/Qwen2.5-0.5B-Instruct"

model = AutoModelForCausalLM.from_pretrained(
    model_name, torch_dtype="auto", device_map="auto"
)
tokenizer = AutoTokenizer.from_pretrained(model_name)

TRIGGER_WORDS = ["you", "wh", "tu"]


def generate_response(prompt: str, history) -> str:
    history += [{"role": "user", "content": prompt}]
    text = tokenizer.apply_chat_template(
        history, tokenize=False, add_generation_prompt=True
    )
    model_inputs = tokenizer([text], return_tensors="pt").to(model.device)
    generated_ids = model.generate(**model_inputs, max_new_tokens=512)
    generated_ids = [
        output_ids[len(input_ids) :]
        for input_ids, output_ids in zip(model_inputs.input_ids, generated_ids)
    ]
    response = tokenizer.batch_decode(generated_ids, skip_special_tokens=True)[0]
    history += [{"role": "assistant", "content": response}]
    return response, history


def main():
    history = [
        {
            "role": "system",
            "content": "You are a Reachy robot, that gives extremely short answers only.",
        },
    ]
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            # Warning: Make sure to add my_output_id and my_input_id within the dataflow.
            text = event["value"][0].as_py()
            if any(word in text.lower() for word in TRIGGER_WORDS):
                response, history = generate_response(text, history)
                node.send_output(
                    output_id="text", data=pa.array([response]), metadata={}
                )


if __name__ == "__main__":
    main()
