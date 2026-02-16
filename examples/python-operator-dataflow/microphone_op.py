"""TODO: Add docstring."""

import numpy as np
import pyarrow as pa
import sounddevice as sd
from adora import AdoraStatus

# Set the parameters for recording
SAMPLE_RATE = 16000
MAX_DURATION = 5


class Operator:
    """Microphone operator that records the audio."""

    def on_event(
        self,
        adora_event,
        send_output,
    ) -> AdoraStatus:
        """TODO: Add docstring."""
        if adora_event["type"] == "INPUT":
            audio_data = sd.rec(
                int(SAMPLE_RATE * MAX_DURATION),
                samplerate=SAMPLE_RATE,
                channels=1,
                dtype=np.int16,
                blocking=True,
            )

            audio_data = audio_data.ravel().astype(np.float32) / 32768.0
            if len(audio_data) > 0:
                send_output("audio", pa.array(audio_data), adora_event["metadata"])
        return AdoraStatus.CONTINUE
