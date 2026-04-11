import numpy as np
from dora import Node
import time

def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "multi_array_msg":
                t0 = time.time()
                
                # The event value is our PyArrow StructArray.
                packed_data = event["value"]

                # We can access the columns by name.
                # packed_data.field('image1') gives us the column for image1.
                
                # Grab the arrow arrays for each field.
                col_image1 = packed_data.field('image1')
                col_image2 = packed_data.field('image2')
                col_state = packed_data.field('state')
                
                # Now we extract the values and convert back to numpy.
                # We take the first element since we know we sent a single batch.
                
                arr_image1_flat = col_image1[0].values.to_numpy(zero_copy_only=False)
                # Reshape it back to the original image dimensions.
                # (Ideally, you'd pass metadata for dynamic shapes, but we'll keep it simple here).
                image1 = arr_image1_flat.view(np.uint8).reshape((480, 640, 3))
                
                arr_image2_flat = col_image2[0].values.to_numpy(zero_copy_only=False)
                image2 = arr_image2_flat.view(np.uint8).reshape((480, 640, 3))
                
                arr_state_flat = col_state[0].values.to_numpy(zero_copy_only=False)
                state = arr_state_flat.view(np.float32).reshape((1, 6))
                
                proc_time = time.time() - t0
                print(f"Received and decoded. Shape1: {image1.shape}, Shape2: {image2.shape}, State: {state.shape}. Time: {proc_time:.6f}s", flush=True)

if __name__ == "__main__":
    main()
