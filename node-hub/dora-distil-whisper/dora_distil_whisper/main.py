import torch
from transformers import AutoModelForSpeechSeq2Seq, AutoProcessor, pipeline
from dora import Node
import pyarrow as pa


device = "cuda:0" if torch.cuda.is_available() else "cpu"
torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32

model_id = "distil-whisper/distil-large-v3"

model = AutoModelForSpeechSeq2Seq.from_pretrained(
    model_id,
    torch_dtype=torch_dtype,
    low_cpu_mem_usage=True,
    use_safetensors=True,
    local_files_only=True,
)
model.to(device)

processor = AutoProcessor.from_pretrained(model_id)
pipe = pipeline(
    "automatic-speech-recognition",
    model=model,
    tokenizer=processor.tokenizer,
    feature_extractor=processor.feature_extractor,
    max_new_tokens=128,
    torch_dtype=torch_dtype,
    device=device,
)


def main():
    node = Node()
    for event in node:
        if event["type"] == "INPUT":
            audio = event["value"].to_numpy()
            result = pipe(audio)
            node.send_output("text", pa.array([result["text"]]))
