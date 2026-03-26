"""Speech-to-text operator for dora-rs dataflow using OpenAI Whisper.

This operator receives audio data, transcribes it into text using the
OpenAI Whisper 'base' model, and emits the resulting text to the "text"
output.
"""

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
        """Transcribe incoming audio data into text.

        Args:
            dora_event (dict): The event from dora-rs, expected to contain
                a raw audio buffer.
            send_output (Callable): Callback to emit the transcribed text.

        Returns:
            DoraStatus: CONTINUE to allow further audio processing.
        """
        if dora_event["type"] == "INPUT":
            audio = dora_event["value"].to_numpy()
            audio = whisper.pad_or_trim(audio)
            result = model.transcribe(audio, language="en")
            send_output("text", pa.array([result["text"]]), dora_event["metadata"])
        return DoraStatus.CONTINUE
