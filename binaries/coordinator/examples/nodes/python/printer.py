from dora import PyDoraNode

node = PyDoraNode.init_from_env()

for i in range(100):
    value = node.next()
    print(value) if value is not [] else None

print("printer finished")
