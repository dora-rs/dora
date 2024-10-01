import pyarrow as pa
from dora import Node

node = Node()

file_path = 'write_down_to_translate.txt'
text_base = ""

def read_file_as_string(file_path):
    with open(file_path, 'r') as file:
        file_contents = file.read()
    return file_contents

for event in node:
    if event["type"] == "INPUT":
        to_be_translated = read_file_as_string(file_path)
        if to_be_translated != text_base:
            text_base = to_be_translated
            node.send_output("text", pa.array([text_base]))
            print(f"go for translation: " + text_base, flush=True)