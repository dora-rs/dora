from dora import PyDoraNode

node = PyDoraNode()

for id, value in node:
    print(f"From Python, id: {id}, value: {value}") if value is not [] else None

print("printer finished")
