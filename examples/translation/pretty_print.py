import os
import shutil


def clear_screen() -> None:
    # Clear the screen based on the operating system
    os.system("cls" if os.name == "nt" else "clear")


def print_centered(texts) -> None:
    # Get terminal size
    shutil.get_terminal_size()

    # Print newlines to move cursor to the middle vertically
    for v in texts.values():
        # Calculate horizontal padding and print the centered text
        for _l in v:
            pass


from dora import Node

node = Node("pretty-print")

previous_texts = {}

clear_screen()
for event in node:
    if event["type"] == "INPUT":
        # The sentence to be printed
        sentence = event["value"][0].as_py()
        if event["id"] not in previous_texts:

            previous_texts[event["id"]] = ["", "", sentence]
        else:
            previous_texts[event["id"]] += [sentence]
            previous_texts[event["id"]] = previous_texts[event["id"]][-3:]
        # Clear the screen
        clear_screen()

        # Print the sentence in the middle of the terminal
        print_centered(previous_texts)
