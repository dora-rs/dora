import numpy as np
import pyarrow as pa
from dora import Node
import time

def main():
    node = Node()
    
    # Initialize some dummy data
    # Let's create some dummy data to send.
    # We have two images and a state vector, just like in your robotics setup.
    image1 = np.zeros((480, 640, 3), dtype=np.uint8)
    image2 = np.ones((480, 640, 3), dtype=np.uint8) * 255
    state = np.array([[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]], dtype=np.float32)

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "tick":
                # Efficiently package the arrays using PyArrow.
                # We flatten the arrays with .ravel() which is super fast and avoids expensive logic.
                # Then we wrap them in a list of length 1 to create the arrow columns.
                
                t0 = time.time()
                
                # Fast path!
                encoded_image1 = pa.array([image1.ravel()])
                encoded_image2 = pa.array([image2.ravel()])
                encoded_state = pa.array([state.ravel()])
                
                struct_data = pa.StructArray.from_arrays(
                    [encoded_image1, encoded_image2, encoded_state],
                    names=['image1', 'image2', 'state']
                )
                
                node.send_output("multi_array_msg", struct_data)
                
                proc_time = time.time() - t0
                print(f"Sent message with 3 arrays. Encoding time: {proc_time:.6f}s", flush=True)

if __name__ == "__main__":
    main()
