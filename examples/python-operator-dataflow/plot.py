import os
import cv2
import time
import numpy as np
from dora import DoraStatus
from utils import LABELS

# --- HEADLESS FIX ---
if os.environ.get('GITHUB_ACTIONS') == 'true' or os.environ.get('QT_QPA_PLATFORM') == 'offscreen':
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    IS_HEADLESS = True
else:
    IS_HEADLESS = False

class Operator:
    def __init__(self):
        self.bboxs = []
        self.inference_latency = 0
        self.last_time = time.time()
        self.fps = 0

    def on_event(self, dora_event, send_output):
        try: # Top-level safety
            if dora_event["type"] == "INPUT":
                id = dora_event["id"]
                
                # 1. IMAGE INPUT
                if id == "image":
                    try:
                        image_array = dora_event["value"].to_numpy()
                        image = image_array.reshape((-1, 640, 3)).copy()
                        
                        # FPS Calculation
                        curr = time.time()
                        self.fps = 1.0 / (curr - self.last_time) if (curr - self.last_time) > 0 else 0
                        self.last_time = curr

                        # Draw Bounding Boxes
                        for bbox in self.bboxs:
                            [x1, y1, x2, y2, conf, lbl] = bbox
                            cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            txt = f"{LABELS[int(lbl)]}: {conf:.2f}"
                            (w, h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                            cv2.rectangle(image, (int(x1), int(y1)-h-10), (int(x1)+w, int(y1)), (0, 255, 0), -1)
                            cv2.putText(image, txt, (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

                        # Overlay Data
                        cv2.putText(image, f"Total FPS: {self.fps:.1f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        cv2.putText(image, f"AI Latency: {self.inference_latency:.1f}ms", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        
                        if not IS_HEADLESS:
                            cv2.imshow("frame", image)
                            if cv2.waitKey(1) & 0xFF == ord("q"): return DoraStatus.STOP
                    except Exception as e:
                        print(f"Image processing error: {e}")

                # 2. BBOX INPUT (Protective Guard)
                elif id == "bbox":
                    try:
                        self.bboxs = dora_event["value"].to_numpy().reshape((-1, 6))
                    except Exception as e:
                        print(f"BBox reshape error: {e}")
                        self.bboxs = []

                # 3. LATENCY INPUT (Protective Guard)
                elif id == "latency":
                    try:
                        self.inference_latency = dora_event["value"].to_numpy()[0]
                    except Exception as e:
                        print(f"Latency extraction error: {e}")

        except Exception as e:
            print(f"Unexpected error in on_event: {e}")

        return DoraStatus.CONTINUE