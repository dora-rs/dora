# Dora LMDeploy Node

Experimental Dora node for efficient inference using LMDeploy.

## YAML Specification

Use this node as follows:

id: dora-lmdeploy
build: pip install dora-lmdeploy
path: dora-lmdeploy
inputs:
image:
source: camera/image
queue_size: 1
text: dora-distil-whisper/text
outputs:

text
env:
MODEL_NAME: "<your-lmdeploy-model-name>"
DEFAULT_PROMPT: "Describe the image briefly."