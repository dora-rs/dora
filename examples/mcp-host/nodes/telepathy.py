"""
<<<<<<< HEAD
This is just a simple demonstration of MCP server.
=======
This is just a simple demonstration of an MCP server.
>>>>>>> 6dad2bdc (Add mcp-host node and mcp-server node)

This MCP server has the ability of telepathy and can know who the current 
user's favorite star is and what their favorite movie is.
"""

import pyarrow as pa
from dora import Node
import json
import random

star_movie_pairs = [
    {"star": "Tom Hanks", "movie": "Forrest Gump"},
    {"star": "Leonardo DiCaprio", "movie": "Titanic"},
    {"star": "Will Smith", "movie": "Men in Black"},
    {"star": "Robert Downey Jr.", "movie": "Iron Man"},
    {"star": "Johnny Depp", "movie": "Pirates of the Caribbean"},
    {"star": "Brad Pitt", "movie": "Fight Club"},
    {"star": "Angelina Jolie", "movie": "Maleficent"},
    {"star": "Scarlett Johansson", "movie": "Black Widow"},
    {"star": "Chris Evans", "movie": "Captain America"},
    {"star": "Ryan Reynolds", "movie": "Deadpool"},
    {"star": "Emma Stone", "movie": "La La Land"},
    {"star": "Jennifer Lawrence", "movie": "The Hunger Games"},
    {"star": "Morgan Freeman", "movie": "The Shawshank Redemption"},
    {"star": "Denzel Washington", "movie": "Training Day"},
<<<<<<< HEAD
=======
    {"star": "Matt Damon", "movie": "The Martian"},
>>>>>>> 6dad2bdc (Add mcp-host node and mcp-server node)
]

node = Node()

for event in node:
    if event["type"] == "INPUT":
        if 'metadata' in event:
            data = json.loads(event["value"][0].as_py())
            name = data.get("name", "")
            location = data.get("arguments", {}).get("location", "")
            match name:
                case "telepathy":
                    random_pair = random.choice(star_movie_pairs)
                    star = random_pair["star"]
                    movie = random_pair["movie"]
                    node.send_output("reply", pa.array([f'{{"content":[{{"type": "text", "text": "{{\\"star\\":\\"{star}\\", \\"movie\\":\\"{movie}\\"}}"}}]}}']), metadata=event["metadata"])
                case _:
                    print(f"Unknown command: {name}")