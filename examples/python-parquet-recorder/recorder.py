"""Recorder node: buffers incoming camera frames and flushes them to Parquet files.

Sends a ready signal on startup so the camera node knows when to begin streaming.
Frames are accumulated in memory and written to disk in batches to reduce I/O overhead.
"""

import os
import time

import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq
from dora import Node

LOG_DIR = os.getenv("LOG_DIR", "logs")
BATCH_SIZE = int(os.getenv("BATCH_SIZE", 30))


def flush_batch(batch, file_index):
    """Write accumulated frames to a Parquet file and return the next file index."""
    if not batch:
        return file_index
    df = pd.DataFrame(batch)
    path = os.path.join(LOG_DIR, f"frames_{file_index:04d}.parquet")
    df.to_parquet(path, index=False)
    print(f"Saved {len(batch)} frames -> {path}", flush=True)
    return file_index + 1


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    node = Node()

    # Signal readiness so the camera node can start streaming
    node.send_output("status", pa.array(["ready"]))
    print(f"Recorder ready. batch_size={BATCH_SIZE}, log_dir={LOG_DIR}", flush=True)

    batch = []
    file_index = 0

    for event in node:
        if event["type"] == "INPUT" and event["id"] == "image":
            metadata = event["metadata"]
            frame_data = event["value"].to_numpy().tobytes()
            batch.append(
                {
                    "timestamp_ns": time.perf_counter_ns(),
                    "frame_id": metadata.get("frame_id", len(batch)),
                    "width": metadata.get("width", 640),
                    "height": metadata.get("height", 480),
                    "encoding": metadata.get("encoding", "bgr8"),
                    "data": frame_data,
                }
            )
            if len(batch) >= BATCH_SIZE:
                file_index = flush_batch(batch, file_index)
                batch = []
        elif event["type"] == "STOP":
            break

    # Flush any remaining frames before exit
    flush_batch(batch, file_index)
    print("Recorder finished.", flush=True)


if __name__ == "__main__":
    main()
