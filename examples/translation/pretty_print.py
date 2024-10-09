import os
import shutil


def clear_screen():
    # Clear the screen based on the operating system
    os.system("cls" if os.name == "nt" else "clear")


def print_centered(texts):
    # Get terminal size
    terminal_size = shutil.get_terminal_size()

    # Print newlines to move cursor to the middle vertically
    for k, v in texts.items():
        print(k)
        print("\n" * 1)
        # Calculate horizontal padding and print the centered text
        for l in v:
            print(l.center(terminal_size.columns))
        print("\n" * 1)


from dora import Node

node = Node("pretty-print")

previous_texts = {}

clear_screen()
print("Waiting for speech...")
for event in node:
    if event["type"] == "INPUT":
        # The sentence to be printed
        sentence = event["value"][0].as_py()
        if event["id"] not in previous_texts.keys():

            previous_texts[event["id"]] = ["", "", sentence]
        else:
            previous_texts[event["id"]] += [sentence]
            previous_texts[event["id"]] = previous_texts[event["id"]][-3:]
        # Clear the screen
        clear_screen()

        # Print the sentence in the middle of the terminal
        print_centered(previous_texts)
