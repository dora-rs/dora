"""TODO: Add docstring."""

import pyarrow as pa
import torch
from dora import Node

# Set the device
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Load the CoTracker model from torch.hub
cotracker = torch.hub.load("facebookresearch/co-tracker", "cotracker2_online").to(device)

def track_points(video_input, pixel_coordinates):
    """Tracks points in a video using the CoTracker model.
    
    Args:
        video_input: The input video.
        pixel_coordinates: The coordinates of the points to track.
    
    Returns:
        The tracked points.

    """
    queries = torch.tensor(pixel_coordinates, dtype=torch.float32).unsqueeze(0).to(video_input.device)
    cotracker(video_chunk=video_input, is_first_step=True, grid_size=0, queries=queries, add_support_grid=True)

    # Process the video and track points
    tracked_points = []
    for ind in range(0, video_input.shape[1] - cotracker.step, cotracker.step):
        pred_tracks, pred_visibility = cotracker(
            video_chunk=video_input[:, ind : ind + cotracker.step * 2]
        )
        tracked_points.append(pred_tracks.cpu().numpy())
    
    return tracked_points

def gradio_interface(video_input, pixel_coordinates):
    """Gradio interface function for tracking points in a video."""
    return track_points(video_input, pixel_coordinates)

def main():
    """Launch the DORA node and Gradio interface."""
    node = Node()
    for event in node:
        if event["type"] == "INPUT":
            video_input = event["value"]
            pixel_coordinates = event["metadata"]["pixel_coordinates"]
            tracked_points = track_points(video_input, pixel_coordinates)
            node.send_output(
                "tracked_points",
                pa.array([tracked_points]),
                event["metadata"]
            )

if __name__ == "__main__":
    main()