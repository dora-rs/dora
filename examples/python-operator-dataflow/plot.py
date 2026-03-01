import cv2
import time
import numpy as np
from dora import DoraStatus
from utils import LABELS

class Operator:
    def __init__(self):
        self.bboxs = []
        self.inference_latency = 0
        self.last_time = time.time()
        self.fps = 0

    def on_event(self, dora_event, send_output):
        if dora_event["type"] == "INPUT":
            id = dora_event["id"]
            if id == "image":
                image = dora_event["value"].to_numpy().reshape((480, 640, 3)).copy()
                
                # FPS Calculation
                curr = time.time()
                self.fps = 1.0 / (curr - self.last_time)
                self.last_time = curr

                for bbox in self.bboxs:
                    [x1, y1, x2, y2, conf, lbl] = bbox
                    cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    txt = f"{LABELS[int(lbl)]}: {conf:.2f}"
                    (w, h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                    cv2.rectangle(image, (int(x1), int(y1)-h-10), (int(x1)+w, int(y1)), (0, 255, 0), -1)
                    cv2.putText(image, txt, (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

                # GSoC Performance Tooling Overlay
                cv2.putText(image, f"Total FPS: {self.fps:.1f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(image, f"AI Latency: {self.inference_latency:.1f}ms", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                cv2.imshow("frame", image)
                if cv2.waitKey(1) & 0xFF == ord("q"): return DoraStatus.STOP
            
            elif id == "bbox": self.bboxs = dora_event["value"].to_numpy().reshape((-1, 6))
            elif id == "latency": self.inference_latency = dora_event["value"].to_numpy()[0]

        return DoraStatus.CONTINUE