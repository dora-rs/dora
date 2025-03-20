# Image Processing Pipeline

This advanced example demonstrates a real-world image processing pipeline using Dora.

## Overview
This example shows how to:
1. Create a complex multi-node pipeline
2. Handle binary data (images)
3. Use external libraries (OpenCV)
4. Implement error handling
5. Use async processing

## Project Structure
```
image_processing/
├── dora.yml
├── requirements.txt
└── nodes/
    ├── camera.py
    ├── preprocessor.py
    ├── detector.py
    └── visualizer.py
```

## Implementation

### 1. Dependencies (requirements.txt)
```
opencv-python>=4.5.0
numpy>=1.19.0
```

### 2. Camera Node (nodes/camera.py)
```python
from dora import AsyncNode
import cv2
import numpy as np

class CameraNode(AsyncNode):
    def __init__(self):
        super().__init__()
        self.cap = None
        
    async def setup(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera")
            
    async def on_input(self, input_data):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to read frame")
            
        # Convert to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return {"image": frame_rgb.tobytes(), "shape": frame_rgb.shape}
        
    async def cleanup(self):
        if self.cap:
            self.cap.release()
```

### 3. Preprocessor Node (nodes/preprocessor.py)
```python
from dora import Node
import numpy as np

class PreprocessorNode(Node):
    def __init__(self):
        super().__init__()
        
    def on_input(self, input_data):
        # Convert bytes back to numpy array
        image = np.frombuffer(input_data["image"], dtype=np.uint8)
        image = image.reshape(input_data["shape"])
        
        # Resize image
        resized = cv2.resize(image, (640, 480))
        
        # Normalize
        normalized = resized.astype(np.float32) / 255.0
        
        return {
            "image": normalized.tobytes(),
            "shape": normalized.shape
        }
```

### 4. Detector Node (nodes/detector.py)
```python
from dora import Node
import numpy as np

class DetectorNode(Node):
    def __init__(self):
        super().__init__()
        # Load your ML model here
        self.model = None
        
    def on_input(self, input_data):
        # Convert to numpy array
        image = np.frombuffer(input_data["image"], dtype=np.float32)
        image = image.reshape(input_data["shape"])
        
        # Run detection (simulated)
        detections = self.simulate_detection(image)
        
        return {
            "detections": detections,
            "image": input_data["image"],
            "shape": input_data["shape"]
        }
        
    def simulate_detection(self, image):
        # Simulate object detection
        return [
            {"class": "person", "confidence": 0.95, "bbox": [100, 100, 200, 200]},
            {"class": "car", "confidence": 0.88, "bbox": [300, 150, 400, 250]}
        ]
```

### 5. Visualizer Node (nodes/visualizer.py)
```python
from dora import Node
import cv2
import numpy as np

class VisualizerNode(Node):
    def __init__(self):
        super().__init__()
        
    def on_input(self, input_data):
        # Convert back to numpy array
        image = np.frombuffer(input_data["image"], dtype=np.float32)
        image = image.reshape(input_data["shape"])
        
        # Convert to uint8 for display
        image = (image * 255).astype(np.uint8)
        
        # Draw detections
        for det in input_data["detections"]:
            bbox = det["bbox"]
            cv2.rectangle(image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
            cv2.putText(image, f"{det['class']} {det['confidence']:.2f}",
                       (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Display image
        cv2.imshow("Detection Results", image)
        cv2.waitKey(1)
        
        return None
```

### 6. Graph Configuration (dora.yml)
```yaml
nodes:
  camera:
    inputs: []
    outputs: ["image", "shape"]
    python: nodes/camera.py
    
  preprocessor:
    inputs: ["image", "shape"]
    outputs: ["image", "shape"]
    python: nodes/preprocessor.py
    
  detector:
    inputs: ["image", "shape"]
    outputs: ["detections", "image", "shape"]
    python: nodes/detector.py
    
  visualizer:
    inputs: ["detections", "image", "shape"]
    outputs: []
    python: nodes/visualizer.py

edges:
  - from: camera
    to: preprocessor
    data: [image, shape]
    
  - from: preprocessor
    to: detector
    data: [image, shape]
    
  - from: detector
    to: visualizer
    data: [detections, image, shape]
```

## Running the Example

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Start the application:
```bash
dora run
```

3. You should see a window showing the camera feed with detected objects.

## Understanding the Example

### Advanced Concepts
- **Async Processing**: Camera node uses async/await for non-blocking I/O
- **Binary Data Handling**: Efficient image data transfer between nodes
- **Error Handling**: Proper cleanup and error propagation
- **External Libraries**: Integration with OpenCV
- **Complex Data Flow**: Multi-stage processing pipeline

### Performance Considerations
- Zero-copy data transfer where possible
- Efficient image format conversions
- Proper resource cleanup
- Async processing for I/O operations

## Next Steps
- Add real ML model integration
- Implement frame rate control
- Add configuration options
- Implement error recovery
- Add performance monitoring 