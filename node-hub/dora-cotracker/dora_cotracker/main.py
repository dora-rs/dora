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
        self.model = torch.hub.load("facebookresearch/co-tracker", "cotracker3_online")
        self.model = self.model.to(self.device)
        self.model.step = 8
        self.buffer_size = self.model.step * 2 
        self.window_frames = deque(maxlen=self.buffer_size)
        self.is_first_step = True
        self.clicked_points = []
        self.input_points = []

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_points.append([x, y])
            self.is_first_step = True
            # print(f"Clicked point added at: ({x}, {y})")

    def process_tracking(self, frame):
        """Process frame for tracking"""
        if len(self.window_frames) == self.buffer_size:
            all_points = self.input_points + self.clicked_points
            
            if not all_points:
                print("No points to track")
                return None, None
            
            video_chunk = torch.tensor(
                np.stack(list(self.window_frames)), 
                device=self.device
            ).float()
            video_chunk = video_chunk / 255.0
            # Reshape to [B,T,C,H,W]
            video_chunk = video_chunk.permute(0, 3, 1, 2)[None]
            query_points = torch.tensor(all_points, device=self.device).float()
            time_dim = torch.zeros(len(all_points), 1, device=self.device)
            queries = torch.cat([time_dim, query_points], dim=1).unsqueeze(0)
            # Track points
            pred_tracks, pred_visibility = self.model(
                video_chunk,
                is_first_step=self.is_first_step,
                grid_size=0,
                queries=queries,
                add_support_grid=False
            )
            self.is_first_step = False

            if pred_tracks is not None and pred_visibility is not None:
                tracks = pred_tracks[0, -1].cpu().numpy()
                visibility = pred_visibility[0, -1].cpu().numpy()
                visible_tracks = []
                for pt, vis in zip(tracks, visibility):
                    if vis > 0.5:
                        visible_tracks.append([int(pt[0]), int(pt[1])])
                visible_tracks = np.array(visible_tracks, dtype=np.float32)

                frame_viz = frame.copy()
                num_input_stream = len(self.input_points)
                # Draw input points in red
                for i, (pt, vis) in enumerate(zip(tracks[:num_input_stream], visibility[:num_input_stream])):
                    if vis > 0.5:
                        x, y = int(pt[0]), int(pt[1])
                        cv2.circle(frame_viz, (x, y), radius=3, 
                                    color=(0, 255, 0), thickness=-1)
                        cv2.putText(frame_viz, f"I{i}", (x + 5, y - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # Draw clicked points in red
                for i, (pt, vis) in enumerate(zip(tracks[num_input_stream:], visibility[num_input_stream:])):
                    if vis > 0.5:
                        x, y = int(pt[0]), int(pt[1])
                        cv2.circle(frame_viz, (x, y), radius=3, 
                                    color=(0, 0, 255), thickness=-1)
                        cv2.putText(frame_viz, f"C{i}", (x + 5, y - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                
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
                
                return frame, frame_viz

        return None, None

    def run(self):
        """Main run loop"""
        cv2.namedWindow("Raw Feed", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Raw Feed", self.mouse_callback)

        for event in self.node:
            if event["type"] == "INPUT":
                if event["id"] == "image":
                    metadata = event["metadata"]
                    frame = event["value"].to_numpy().reshape((
                        metadata["height"],
                        metadata["width"],
                        3
                    ))
                    # Add frame to tracking window
                    self.window_frames.append(frame)
                    original_frame, tracked_frame = self.process_tracking(frame)
                    if original_frame is not None and tracked_frame is not None:
                        self.node.send_output("image", 
                            pa.array(original_frame.ravel()), 
                            metadata
                        )
                        self.node.send_output("tracked_image", 
                            pa.array(tracked_frame.ravel()), 
                            metadata
                        )

                    display_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    cv2.imshow("Raw Feed", display_frame)
                    cv2.waitKey(1)
                
                if event["id"] == "points_to_track":
                    # Handle points from input_stream node
                    metadata = event["metadata"]
                    points_array = event["value"].to_numpy()
                    num_points = metadata["num_points"]
                    self.input_points = points_array.reshape((num_points, 2)).tolist()
                    self.is_first_step = True
                    print(f"Received {num_points} points from input_stream")



def main():
    tracker = VideoTrackingNode()
    tracker.run()

if __name__ == "__main__":
    main()