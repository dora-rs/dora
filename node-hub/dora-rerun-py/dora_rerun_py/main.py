import numpy as np
import cv2
import rerun as rr
from dora import Node

# Global caches for image and mask data
image_cache = {}
mask_cache = {}

def process_image(event):
    """
    Processes an 'image' event.

    Expected metadata keys:
      - "width": the width of the image (int)
      - "height": the height of the image (int)
      - "encoding": the pixel encoding (e.g. "bgr8" or "rgb8")

    The event value is expected to be a PyArrow array representing a flattened image.
    If the encoding is "bgr8", the image will be converted to RGB.
    The image is then logged using Rerun using the proper `color_model` parameter.
    """
    metadata = event.get("metadata", {})
    storage = event.get("value")
    width = int(metadata.get("width", 640))
    height = int(metadata.get("height", 480))
    encoding = metadata.get("encoding", "bgr8")
    channels = 3  # Assuming a 3-channel image

    try:
        np_data = storage.to_numpy()
        image_arr = np_data.astype(np.uint8).reshape((height, width, channels))
    except Exception as e:
        print("Error processing image data:", e)
        return

    # Convert BGR to RGB if needed.
    if encoding == "bgr8":
        image_arr = cv2.cvtColor(image_arr, cv2.COLOR_BGR2RGB)
    elif encoding != "rgb8":
        print(f"Warning: unsupported image encoding '{encoding}'; proceeding without conversion.")

    # Cache the raw image bytes (if needed later)
    image_cache["camera/image"] = image_arr.tobytes()

    # Create a Rerun Image object.
    # Use the keyword `color_model` and pass the appropriate string.
    # If the image is in RGB, we pass "RGB".
    image_obj = rr.Image(image_arr, color_model="RGB")
    rr.log("camera/image", image_obj)

def process_depth(event):
    """
    Processes a 'depth' event.

    Expected metadata keys:
      - "width": width of the depth image
      - "resolution": a two-element list [width, height]
      - "focal": a two-element list [focal_x, focal_y]

    Converts the flat depth data to 3D points and logs them.
    """
    metadata = event.get("metadata", {})
    storage = event.get("value")
    width = int(metadata.get("width", 640))
    resolution = metadata.get("resolution", [640, 480])
    focal = metadata.get("focal", [605, 605])

    try:
        depth_data = storage.to_numpy().flatten()
    except Exception as e:
        print("Error processing depth data:", e)
        return

    points = []
    for i, z in enumerate(depth_data):
        u = i % width
        v = i // width
        z_val = float(z)
        x = (u - resolution[0] / 2.0) * z_val / focal[0]
        y = (v - resolution[1] / 2.0) * z_val / focal[1]
        points.append([x, y, z_val])
    points_np = np.array(points, dtype=np.float32)

    # Use a dummy white color (RGB white: 255, 255, 255) for each point.
    colors = np.full((points_np.shape[0], 3), 255, dtype=np.uint8)
    rr.log(event.get("id"), rr.Points3D(points_np, colors=colors, radii=0.5))

def process_boxes2d(event):
    """
    Processes a bounding box event.

    Expects the event value to be a PyArrow array that converts to a list containing a dictionary
    with keys:
      - "bbox": a flattened list in 'xyxy' format
      - (optional) "labels": a list of text labels.
    """
    metadata = event.get("metadata", {})
    storage = event.get("value")
    try:
        arr = storage.to_pylist()
    except Exception as e:
        print("Error converting bounding box data:", e)
        return

    if not arr:
        rr.log("object-detection/bbox", rr.Clear())
        return

    bbox_dict = arr[0]
    flat_bbox = np.array(bbox_dict.get("bbox", []), dtype=np.float32)
    if flat_bbox.size % 4 != 0:
        print("Invalid bounding box data size.")
        return

    bboxes = flat_bbox.reshape((-1, 4))
    labels = bbox_dict.get("labels", [])
    centers = []
    sizes = []
    for box in bboxes:
        min_x, min_y, max_x, max_y = box
        centers.append(((min_x + max_x) / 2.0, (min_y + max_y) / 2.0))
        sizes.append((max_x - min_x, max_y - min_y))

    boxes_obj = rr.Boxes2D(centers=centers, sizes=sizes, labels=labels if labels else None)
    rr.log("object-detection/bbox", boxes_obj)

def process_text(event):
    """
    Processes a text event.

    Expects the event value to be a PyArrow string array.
    """
    storage = event.get("value")
    try:
        texts = storage.to_pylist()
    except Exception as e:
        print("Error processing text data:", e)
        return

    for text in texts:
        if text:
            rr.log(event.get("id"), rr.TextLog(text))

def process_series(event):
    """
    Processes a series event.

    Expects the event value to be a numeric array.
    Logs each value as a scalar.
    """
    storage = event.get("value")
    try:
        series = list(storage.to_numpy())
    except Exception as e:
        print("Error processing series data:", e)
        return

    for idx, value in enumerate(series):
        rr.log(f"{event.get('id')}_{idx}", rr.Scalar(float(value)))

def process_jointstate(event):
    """
    Processes a joint state event.
    (This is a placeholder; extend it as needed for URDF-based visualizations.)
    """
    storage = event.get("value")
    try:
        positions = list(storage.to_numpy())
    except Exception as e:
        print("Error processing jointstate data:", e)
        return

    print("Received jointstate:", positions)

def process_event(event):
    """
    Dispatches the event to an appropriate handler based on the event ID.
    """
    event_id = event.get("id", "")
    if "image" in event_id:
        process_image(event)
    elif "depth" in event_id:
        process_depth(event)
    elif "boxes2d" in event_id or "bbox" in event_id:
        process_boxes2d(event)
    elif "text" in event_id:
        process_text(event)
    elif "series" in event_id:
        process_series(event)
    elif "jointstate" in event_id:
        process_jointstate(event)
    else:
        print(f"Unhandled event: {event_id}")

def main():
    # Initialize Rerun with a unique namespace and automatically spawn the viewer.
    rr.init("dora-rerun-py", spawn=True)
    print("Dora Rerun Python node started. Waiting for events...", flush=True)

    # Create a Dora Node to subscribe to dataflow events.
    node = Node("dora-rerun-py")

    # Main event loop: process each incoming event.
    for event in node:
        try:
            process_event(event)
        except Exception as err:
            print("Error processing event:", err)

if __name__ == "__main__":
    main()
