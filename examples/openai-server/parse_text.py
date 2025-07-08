"""TODO: Add docstring."""

from PIL import Image
from io import BytesIO
import base64
from dora import Node
import pyarrow as pa
import numpy as np
node = Node()


for event in node:
    if event["type"] == "INPUT":
        texts = event["value"].to_numpy(zero_copy_only=False)

        for text in texts:
            if text.startswith("<|user|>\n<|vision_start|>\n"):
                # Handle the case where the text starts with <|user|>\n<|vision_start|>
                image = text.replace("<|user|>\n<|vision_start|>\n", "")
                if "base64" in image:
                    image = image.split(",", 1)[1]
                    print("image", image)
                    image = Image.open(BytesIO(base64.b64decode(image)))
                    node.send_output(
                        "image",
                        pa.array(np.array(image).ravel()),
                        metadata={
                            "encoding": "rgb8",
                            "width": image.width,
                            "height": image.height,
                        },
                    )
        print(f"Processed {len(texts)} texts.")
