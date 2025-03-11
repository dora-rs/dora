"""TODO: Add docstring."""

import numpy as np
import pyarrow as pa
import sounddevice as sd
from dora import DoraStatus

# Set the parameters for recording
SAMPLE_RATE = 16000
MAX_DURATION = 5


class Operator:
    """Microphone operator that records the audio."""

    def on_event(
        self,
        dora_event,
        send_output,
    ) -> DoraStatus:
        """TODO: Add docstring."""
        if dora_event["type"] == "INPUT":
            audio_data = sd.rec(
                int(SAMPLE_RATE * MAX_DURATION),
                samplerate=SAMPLE_RATE,
                channels=1,
                dtype=np.int16,
                blocking=True,
            )

            audio_data = audio_data.ravel().astype(np.float32) / 32768.0
            if len(audio_data) > 0:
                send_output("audio", pa.array(audio_data), dora_event["metadata"])
        return DoraStatus.CONTINUE
