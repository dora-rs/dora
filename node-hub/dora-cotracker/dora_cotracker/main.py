"""TODO: Add docstring."""

import os

import gradio as gr
import torch
from transformers import AutoModel

MODEL_NAME_OR_PATH = os.getenv("MODEL_NAME_OR_PATH", "facebook/cotracker")

# Load the CoTracker model
model = AutoModel.from_pretrained(MODEL_NAME_OR_PATH)

def track_points(video_input, pixel_coordinates):
    """Tracks points in a video using the CoTracker model.
    
    Args:
        video_input: The input video.
        pixel_coordinates: The coordinates of the points to track.
    
    Returns:
        The tracked points.

    """
    # Create query tensor from pixel coordinates
    queries = torch.tensor(pixel_coordinates, dtype=torch.float32).unsqueeze(0)
    add_support_grid = False

    return model(video_chunk=video_input, is_first_step=True, grid_size=0, queries=queries, add_support_grid=add_support_grid)

def gradio_interface(video_input, pixel_coordinates):
    """Gradio interface function for tracking points in a video."""
    return track_points(video_input, pixel_coordinates)

def main():
    """Launch the Gradio interface."""
    iface = gr.Interface(
        fn=gradio_interface,
        inputs=["video", "text"],
        outputs="text",
        description="Track points using facebook co-tracker model"
    )
    iface.launch()

if __name__ == "__main__":
    main()