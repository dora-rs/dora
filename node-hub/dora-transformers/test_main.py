import logging
import pyarrow as pa
from main import main as main_func  # import the function as main_func

# Create a dummy Node class to simulate events.
class DummyNode:
    def __init__(self):
        self.events = [{
            "type": "INPUT",
            "id": "text",
            "value": pa.array(["Test input: Hello world"]),
            "metadata": {}
        }]
    def __iter__(self):
        return iter(self.events)
    def send_output(self, id, value, metadata):
        print("Output for", id, ":", value.to_pylist(), "with metadata:", metadata)

# Monkey-patch the Node in our main module for testing.
import main as main_module  # import module with an alias
main_module.Node = DummyNode

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main_func()