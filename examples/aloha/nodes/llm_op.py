from dora import DoraStatus
import pylcs
import os
import pyarrow as pa
from transformers import AutoModelForCausalLM, AutoTokenizer
import torch

import gc  # garbage collect library
import re
import time

CHATGPT = False
MODEL_NAME_OR_PATH = "TheBloke/deepseek-coder-6.7B-instruct-GPTQ"

CODE_MODIFIER_TEMPLATE = """
### Instruction
Respond with one block of modified code only in ```python block. No explanation.

```python
{code}
```

{user_message}

### Response:
"""


model = AutoModelForCausalLM.from_pretrained(
    MODEL_NAME_OR_PATH,
    device_map="auto",
    trust_remote_code=True,
    revision="main",
    max_length=1024,
).to("cuda:0")


tokenizer = AutoTokenizer.from_pretrained(MODEL_NAME_OR_PATH, use_fast=True)


def extract_python_code_blocks(text):
    """
    Extracts Python code blocks from the given text that are enclosed in triple backticks with a python language identifier.

    Parameters:
    - text: A string that may contain one or more Python code blocks.

    Returns:
    - A list of strings, where each string is a block of Python code extracted from the text.
    """
    pattern = r"```python\n(.*?)\n```"
    matches = re.findall(pattern, text, re.DOTALL)
    if len(matches) == 0:
        pattern = r"```python\n(.*?)(?:\n```|$)"
        matches = re.findall(pattern, text, re.DOTALL)
        if len(matches) == 0:
            return [text]
        else:
            matches = [remove_last_line(matches[0])]

    return matches


def remove_last_line(python_code):
    """
    Removes the last line from a given string of Python code.

    Parameters:
    - python_code: A string representing Python source code.

    Returns:
    - A string with the last line removed.
    """
    lines = python_code.split("\n")  # Split the string into lines
    if lines:  # Check if there are any lines to remove
        lines.pop()  # Remove the last line
    return "\n".join(lines)  # Join the remaining lines back into a string


def calculate_similarity(source, target):
    """
    Calculate a similarity score between the source and target strings.
    This uses the edit distance relative to the length of the strings.
    """
    edit_distance = pylcs.edit_distance(source, target)
    max_length = max(len(source), len(target))
    # Normalize the score by the maximum possible edit distance (the length of the longer string)
    similarity = 1 - (edit_distance / max_length)
    return similarity


def find_best_match_location(source_code, target_block):
    """
    Find the best match for the target_block within the source_code by searching line by line,
    considering blocks of varying lengths.
    """
    source_lines = source_code.split("\n")
    target_lines = target_block.split("\n")

    best_similarity = 0
    best_start_index = 0
    best_end_index = -1

    # Iterate over the source lines to find the best matching range for all lines in target_block
    for start_index in range(len(source_lines) - len(target_lines) + 1):
        for end_index in range(start_index + len(target_lines), len(source_lines) + 1):
            current_window = "\n".join(source_lines[start_index:end_index])
            current_similarity = calculate_similarity(current_window, target_block)
            if current_similarity > best_similarity:
                best_similarity = current_similarity
                best_start_index = start_index
                best_end_index = end_index

    # Convert line indices back to character indices for replacement
    char_start_index = len("\n".join(source_lines[:best_start_index])) + (
        1 if best_start_index > 0 else 0
    )
    char_end_index = len("\n".join(source_lines[:best_end_index]))

    return char_start_index, char_end_index


def replace_code_in_source(source_code, replacement_block: str):
    """
    Replace the best matching block in the source_code with the replacement_block, considering variable block lengths.
    """
    replacement_block = extract_python_code_blocks(replacement_block)[0]
    start_index, end_index = find_best_match_location(source_code, replacement_block)
    if start_index != -1 and end_index != -1:
        # Replace the best matching part with the replacement block
        new_source = (
            source_code[:start_index] + replacement_block + source_code[end_index:]
        )
        return new_source
    else:
        return source_code


class Operator:
    def __init__(self) -> None:
        self.policy_init = False

    def on_event(
        self,
        dora_event,
        send_output,
    ) -> DoraStatus:
        global model, tokenizer
        if dora_event["type"] == "INPUT" and dora_event["id"] == "text":
            input = dora_event["value"][0].as_py()
            # Path to the current file
            current_file_path = __file__

            # Directory of the current file
            current_directory = os.path.dirname(current_file_path)
            path = current_directory + "/policy.py"

            with open(path, "r", encoding="utf8") as f:
                code = f.read()

            user_message = input
            start_llm = time.time()

            output = self.ask_llm(
                CODE_MODIFIER_TEMPLATE.format(code=code, user_message=user_message)
            )

            source_code = replace_code_in_source(code, output)
            print("response time:", time.time() - start_llm, flush=True)

            print("response: ", output, flush=True)
            with open(path, "w") as file:
                file.write(source_code)

            gc.collect()
            torch.cuda.empty_cache()

        return DoraStatus.CONTINUE

    def ask_llm(self, prompt):

        # Generate output
        # prompt = PROMPT_TEMPLATE.format(system_message=system_message, prompt=prompt))
        input = tokenizer(prompt, return_tensors="pt")
        input_ids = input.input_ids.cuda()

        # add attention mask here
        attention_mask = input.attention_mask.cuda()

        output = model.generate(
            inputs=input_ids,
            temperature=0.7,
            do_sample=True,
            top_p=0.95,
            top_k=40,
            max_new_tokens=512,
            attention_mask=attention_mask,
            eos_token_id=tokenizer.eos_token_id,
        )
        # Get the tokens from the output, decode them, print them

        # Get text between im_start and im_end
        return tokenizer.decode(output[0], skip_special_tokens=True)[len(prompt) :]


if __name__ == "__main__":
    op = Operator()

    # Path to the current file
    current_file_path = __file__

    # Directory of the current file
    current_directory = os.path.dirname(current_file_path)

    path = current_directory + "/policy.py"
    with open(path, "r", encoding="utf8") as f:
        raw = f.read()

    op.on_event(
        {
            "type": "INPUT",
            "id": "text",
            "value": pa.array(
                [
                    {
                        "path": path,
                        "user_message": "When I say suit up, get the hat and the get the food.",
                    },
                ]
            ),
            "metadata": [],
        },
        print,
    )
