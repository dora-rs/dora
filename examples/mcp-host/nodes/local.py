"""
<<<<<<< HEAD
This is just a simple demonstration of MCP server.

The example returns some local information about the user's request, such as 
the happiest kindergarten, each restaurant's signature dish, etc.
=======
This is just a simple demonstration of an MCP server.

The example returns some local information about the user's request, such as the tallest building, 
the happiest kindergarten, the best restaurant, etc.
>>>>>>> 6dad2bdc (Add mcp-host node and mcp-server node)
"""

import pyarrow as pa
from dora import Node
import json

import random

signature_dishes = [
    "Kung Pao Chicken",
    "Mapo Tofu",
    "Twice Cooked Pork",
    "Sweet and Sour Pork",
    "Boiled Fish in Chili Oil",
    "Peking Duck",
    "Xiaolongbao",
    "Red Braised Pork",
    "Fish-Flavored Shredded Pork",
    "Dongpo Pork",
    "White Cut Chicken",
    "Steamed Egg Custard",
    "Fish with Pickled Cabbage",
    "Saliva Chicken",
    "Spicy Beef and Ox Tongue",
    "Laziji (Spicy Diced Chicken)",
    "Steamed Sea Bass",
    "Ants Climbing a Tree",
    "Beggar's Chicken",
    "Buddha Jumps Over the Wall"
]

node = Node()

for event in node:
    if event["type"] == "INPUT":
        if 'metadata' in event:
            data = json.loads(event["value"][0].as_py())
            name = data.get("name", "")
            location = data.get("arguments", {}).get("location", "")
            match name:
                case "signature_dish":
                    random_dish = random.choice(signature_dishes)
                    node.send_output("reply", pa.array([f'{{"content":[{{"type": "text", "text": "{{\\"signature_dish\\": \\"{random_dish}\\"}}"}}]}}']), metadata=event["metadata"])
                case "happiest_kindergarten":
                    node.send_output("reply", pa.array([f'{{"content":[{{"type": "text", "text": "{{\\"kindergarten\\":\\"Golden Sun Kindergarten\\", \\"children\\": 300}}"}}]}}']), metadata=event["metadata"])
                case _:
                    print(f"Unknown command: {name}")