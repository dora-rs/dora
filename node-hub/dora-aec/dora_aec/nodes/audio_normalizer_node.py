#!/usr/bin/env python3
"""
Audio Normalizer Node - Converts int16 to float32
"""

import numpy as np
import pyarrow as pa
from dora import Node


def main():
    node = Node()
    
    while True:
        event = node.next()
        
        if event["type"] == "INPUT":
            if event["id"] == "audio":
                # Convert int16 to float32 [-1, 1]
                audio_int16 = event["value"].to_numpy()
                audio_float32 = audio_int16.astype(np.float32) / 32768.0
                
                # Send normalized audio
                node.send_output("normalized_audio", pa.array(audio_float32))


if __name__ == "__main__":
    main()