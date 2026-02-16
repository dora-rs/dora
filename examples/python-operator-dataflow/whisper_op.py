"""TODO: Add docstring."""

import pyarrow as pa
import whisper
from adora import AdoraStatus

model = whisper.load_model("base")


class Operator:
    """Transforming Speech to Text using OpenAI Whisper model."""

    def on_event(
        self,
        adora_event,
        send_output,
    ) -> AdoraStatus:
        """TODO: Add docstring."""
        if adora_event["type"] == "INPUT":
            audio = adora_event["value"].to_numpy()
            audio = whisper.pad_or_trim(audio)
            result = model.transcribe(audio, language="en")
            send_output("text", pa.array([result["text"]]), adora_event["metadata"])
        return AdoraStatus.CONTINUE
