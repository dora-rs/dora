"""TODO: Add docstring."""

import pyarrow as pa
import whisper
from dora import DoraStatus

model = whisper.load_model("base")


class Operator:
    """Transforming Speech to Text using OpenAI Whisper model."""

    def on_event(
        self,
        dora_event,
        send_output,
    ) -> DoraStatus:
        """TODO: Add docstring."""
        if dora_event["type"] == "INPUT":
            audio = dora_event["value"].to_numpy()
            audio = whisper.pad_or_trim(audio)
            result = model.transcribe(audio, language="en")
            send_output("text", pa.array([result["text"]]), dora_event["metadata"])
        return DoraStatus.CONTINUE
