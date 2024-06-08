from dora import DoraStatus
import pylcs
import os
import pyarrow as pa
from transformers import AutoModelForCausalLM, AutoTokenizer
import json

import re
import time

MODEL_NAME_OR_PATH = "TheBloke/deepseek-coder-6.7B-instruct-GPTQ"
# MODEL_NAME_OR_PATH = "hanspeterlyngsoeraaschoujensen/deepseek-math-7b-instruct-GPTQ"

CODE_MODIFIER_TEMPLATE = """
### Instruction
Respond with the small modified code only. No explanation.

```python
{code}
```

{user_message}

### Response:
"""


MESSAGE_SENDER_TEMPLATE = """
### Instruction
You're a json expert. Format your response as a json with a topic and a data field in a ```json block.  No explanation needed. No code needed.
The schema for those json are:
- line: Int[4]

The response should look like this: 
```json
  {{ "topic": "line", "data": [10, 10, 90, 10] }}
```

{user_message}

### Response:
"""

ASSISTANT_TEMPLATE = """
### Instruction
You're a helpuf assistant named dora. 
Reply with a short message. No code needed. 

User {user_message}

### Response:
"""


model = AutoModelForCausalLM.from_pretrained(
    MODEL_NAME_OR_PATH,
    device_map="auto",
    trust_remote_code=True,
    revision="main",
)


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


def extract_json_code_blocks(text):
    """
    Extracts json code blocks from the given text that are enclosed in triple backticks with a json language identifier.

    Parameters:
    - text: A string that may contain one or more json code blocks.

    Returns:
    - A list of strings, where each string is a block of json code extracted from the text.
    """
    pattern = r"```json\n(.*?)\n```"
    matches = re.findall(pattern, text, re.DOTALL)
    if len(matches) == 0:
        pattern = r"```json\n(.*?)(?:\n```|$)"
        matches = re.findall(pattern, text, re.DOTALL)
        if len(matches) == 0:
            return [text]

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

    def on_event(
        self,
        dora_event,
        send_output,
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT" and dora_event["id"] == "code_modifier":
            input = dora_event["value"][0].as_py()

            with open(input["path"], "r", encoding="utf8") as f:
                code = f.read()

            user_message = input["user_message"]
            start_llm = time.time()
            output = self.ask_llm(
                CODE_MODIFIER_TEMPLATE.format(code=code, user_message=user_message)
            )

            source_code = replace_code_in_source(code, output)
            print("response time:", time.time() - start_llm, flush=True)
            send_output(
                "modified_file",
                pa.array(
                    [
                        {
                            "raw": source_code,
                            "path": input["path"],
                            "response": output,
                            "prompt": input["user_message"],
                        }
                    ]
                ),
                dora_event["metadata"],
            )
            print("response: ", output, flush=True)
            send_output(
                "assistant_message",
                pa.array([output]),
                dora_event["metadata"],
            )
        elif dora_event["type"] == "INPUT" and dora_event["id"] == "message_sender":
            user_message = dora_event["value"][0].as_py()
            output = self.ask_llm(
                MESSAGE_SENDER_TEMPLATE.format(user_message=user_message)
            )
            outputs = extract_json_code_blocks(output)[0]
            try:
                output = json.loads(outputs)
                if not isinstance(output["data"], list):
                    output["data"] = [output["data"]]

                if output["topic"] in [
                    "line",
                ]:
                    send_output(
                        output["topic"],
                        pa.array(output["data"]),
                        dora_event["metadata"],
                    )
                else:
                    print("Could not find the topic: {}".format(output["topic"]))
            except:
                print("Could not parse json")
            # if data is not iterable, put data in a list
        elif dora_event["type"] == "INPUT" and dora_event["id"] == "assistant":
            user_message = dora_event["value"][0].as_py()
            output = self.ask_llm(ASSISTANT_TEMPLATE.format(user_message=user_message))
            send_output(
                "assistant_message",
                pa.array([output]),
                dora_event["metadata"],
            )
        return DoraStatus.CONTINUE

    def ask_llm(self, prompt):

        # Generate output
        # prompt = PROMPT_TEMPLATE.format(system_message=system_message, prompt=prompt))
        input = tokenizer(prompt, return_tensors="pt")
        input_ids = input.input_ids.cuda()

        # add attention mask here
        attention_mask = input["attention_mask"].cuda()

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

    path = current_directory + "object_detection.py"
    with open(path, "r", encoding="utf8") as f:
        raw = f.read()

    op.on_event(
        {
            "type": "INPUT",
            "id": "message_sender",
            "value": pa.array(
                [
                    {
                        "path": path,
                        "user_message": "send a star ",
                    },
                ]
            ),
            "metadata": [],
        },
        print,
    )
