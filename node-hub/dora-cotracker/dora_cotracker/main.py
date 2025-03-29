import numpy as np
import pyarrow as pa
from dora import Node
import cv2
import torch
from collections import deque

class VideoTrackingNode:
    def __init__(self):
        self.node = Node("video-tracking-node")
        
        # Initialize CoTracker
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"Using device: {self.device}")
        self.model = torch.hub.load("facebookresearch/co-tracker", "cotracker3_online")
        self.model = self.model.to(self.device)
        
        # Initialize tracking variables
        self.buffer_size = self.model.step * 2
        self.window_frames = deque(maxlen=self.buffer_size)
        self.is_first_step = True
        self.grid_size = 10  # Smaller grid for better visualization
        self.grid_query_frame = 0
        self.frame_count = 0

    def process_tracking(self, frame):
        """Process frame for tracking"""
        if len(self.window_frames) == self.buffer_size:
            try:
                # Stack frames and convert to tensor
                video_chunk = torch.tensor(
                    np.stack(list(self.window_frames)), 
                    device=self.device
                ).float()
                
                # Normalize pixel values to [0, 1]
                video_chunk = video_chunk / 255.0
                
                # Reshape to [B,T,C,H,W]
                video_chunk = video_chunk.permute(0, 3, 1, 2)[None]
                
                # Run tracking with grid parameters
                pred_tracks, pred_visibility = self.model(
                    video_chunk,
                    is_first_step=self.is_first_step,
                    grid_size=self.grid_size,
                    grid_query_frame=self.grid_query_frame
                )
                self.is_first_step = False

                if pred_tracks is not None and pred_visibility is not None:
                    # Get the latest tracks and visibility
                    tracks = pred_tracks[0, -1].cpu().numpy()
                    visibility = pred_visibility[0, -1].cpu().numpy()
                    
                    # Filter high-confidence points
                    visible_mask = visibility > 0.5
                    visible_tracks = tracks[visible_mask]
                    
                    # Send tracked points
                    if len(visible_tracks) > 0:
                        self.node.send_output(
                            "tracked_points", 
                            pa.array(visible_tracks.ravel()),
                            {
                                "num_points": len(visible_tracks),
                                "dtype": "float32",
                                "shape": (len(visible_tracks), 2)
                            }
                        )
                    
                    # Visualize tracked points
                    frame_viz = frame.copy()
                    for pt, vis in zip(tracks, visibility):
                        if vis > 0.5:  # Only draw high-confidence points
                            x, y = int(pt[0]), int(pt[1])
                            cv2.circle(frame_viz, (x, y), radius=3, 
                                     color=(0, 255, 0), thickness=-1)
                    
                    return frame, frame_viz
                else:
                    print("Debug - Model returned None values")
                
            except Exception as e:
                print(f"Error in processing: {str(e)}")
                import traceback
                traceback.print_exc()
                
        return None, None

    def run(self):
        """Main run loop"""
        try:
            for event in self.node:
                if event["type"] == "INPUT" and event["id"] == "image":
                    metadata = event["metadata"]
                    frame = event["value"].to_numpy().reshape((
                        metadata["height"],
                        metadata["width"],
                        3
                    ))
                    
                    # Add frame to tracking window
                    self.window_frames.append(frame)
                    
                    # Process tracking
                    original_frame, tracked_frame = self.process_tracking(frame)
                    
                    # Only publish when we have processed frames
                    if original_frame is not None and tracked_frame is not None:
                        self.node.send_output("image", 
                            pa.array(original_frame.ravel()), 
                            metadata
                        )
                        self.node.send_output("tracked_image", 
                            pa.array(tracked_frame.ravel()), 
                            metadata
                        )

        finally:
            pass

def main():
    tracker = VideoTrackingNode()
    tracker.run()

if __name__ == "__main__":
    main()