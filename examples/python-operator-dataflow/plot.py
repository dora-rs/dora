import os
import time
import numpy as np
from dora import DoraStatus
from utils import LABELS

# HEADLESS FIX: Must be set BEFORE cv2 import
# cv2 reads QT_QPA_PLATFORM at import time, not at runtime
if os.environ.get("GITHUB_ACTIONS") == "true":
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    IS_HEADLESS = True
elif os.environ.get("QT_QPA_PLATFORM") == "offscreen":
    IS_HEADLESS = True
else:
    IS_HEADLESS = False

import cv2  # noqa: E402

CAMERA_WIDTH = 640


class Operator:
    def __init__(self):
        # Initialize as numpy array not list — safe to iterate always
        self.bboxs = np.zeros((0, 6))
        self.inference_latency = 0
        self.last_time = time.time()
        self.fps = 0

    def on_event(self, dora_event, send_output):
        try:
            if dora_event["type"] == "INPUT":
                event_id = dora_event["id"]

                # 1. IMAGE INPUT
                if event_id == "image":
                    try:
                        image_array = dora_event["value"].to_numpy()
                        # Dynamic reshape — no hardcoded height
                        image = image_array.reshape((-1, CAMERA_WIDTH, 3)).copy()

                        # FPS Calculation
                        curr = time.time()
                        elapsed = curr - self.last_time
                        self.fps = 1.0 / elapsed if elapsed > 0 else 0
                        self.last_time = curr

                        # Draw Bounding Boxes
                        for bbox in self.bboxs:
                            [x1, y1, x2, y2, conf, lbl] = bbox
                            cv2.rectangle(
                                image,
                                (int(x1), int(y1)),
                                (int(x2), int(y2)),
                                (0, 255, 0),
                                2,
                            )
                            txt = f"{LABELS[int(lbl)]}: {conf:.2f}"
                            (w, h), _ = cv2.getTextSize(
                                txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1
                            )
                            # Filled green background for readable white text
                            cv2.rectangle(
                                image,
                                (int(x1), int(y1) - h - 10),
                                (int(x1) + w, int(y1)),
                                (0, 255, 0),
                                -1,
                            )
                            cv2.putText(
                                image,
                                txt,
                                (int(x1), int(y1) - 5),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                (255, 255, 255),
                                1,
                            )

                        # Performance Overlay
                        cv2.putText(
                            image,
                            f"Total FPS: {self.fps:.1f}",
                            (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 255),
                            2,
                        )
                        cv2.putText(
                            image,
                            f"AI Latency: {self.inference_latency:.1f}ms",
                            (20, 70),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 255),
                            2,
                        )

                        if not IS_HEADLESS:
                            cv2.imshow("frame", image)
                            if cv2.waitKey(1) & 0xFF == ord("q"):
                                return DoraStatus.STOP

                    except Exception as e:
                        print(f"Image processing error: {e}")

                # 2. BBOX INPUT
                elif event_id == "bbox":
                    try:
                        arr = dora_event["value"].to_numpy()
                        # Guard against empty array — shape (0,) would crash reshape(-1,6)
                        self.bboxs = (
                            arr.reshape((-1, 6)) if arr.size > 0 else np.zeros((0, 6))
                        )
                    except Exception as e:
                        print(f"BBox reshape error: {e}")
                        self.bboxs = np.zeros((0, 6))

                # 3. LATENCY INPUT
                elif event_id == "latency":
                    try:
                        self.inference_latency = dora_event["value"].to_numpy()[0]
                    except Exception as e:
                        print(f"Latency extraction error: {e}")

        except Exception as e:
            print(f"Unexpected error in on_event: {e}")

        return DoraStatus.CONTINUE