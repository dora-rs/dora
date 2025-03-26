import av
import cv2
import numpy as np

# Open the AV1 video file
container = av.open("video.av1")

# Get the video stream
stream = next(s for s in container.streams if s.type == "video")

# Iterate over the video frames
for frame in container.decode(stream):
    # Convert the frame to a NumPy array
    img = frame.to_ndarray(format="bgr24")

    # Display the frame
    cv2.imshow("Frame", img)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release the resources
cv2.destroyAllWindows()
