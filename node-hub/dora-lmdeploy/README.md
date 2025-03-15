# Dora LMDeploy Node

Experimental node for using LMDeploy within dora for efficient model inference on CUDA.

## YAML Specification

This node is supposed to be used as follows:

```yaml
- id: dora-lmdeploy
  build: pip install dora-lmdeploy
  path: dora-lmdeploy
  inputs:
    image:
      source: camera/image
      queue_size: 1
    text: dora-distil-whisper/text
  outputs:
    - text
  env:
    MODEL_NAME_OR_PATH: liuhaotian/llava-v1.5-7b
    DEFAULT_QUESTION: Describe the image in a very short sentence.
    SYSTEM_PROMPT: You're a very succinct AI assistant that describes images with short sentences.
    HISTORY: False
```

## Additional documentation

- LMDeploy: https://lmdeploy.readthedocs.io/en/latest/
- GitHub: https://github.com/InternLM/lmdeploy

## Features

- Efficient inference using LMDeploy's TurboMind engine
- Support for various image formats (rgb8, bgr8, jpeg, etc.)
- Configurable system prompts and chat history
- Optimized for CUDA execution

## Examples

- Vision Language Model
  - Github: https://github.com/dora-rs/dora/blob/main/examples/vlm
  - Website: https://dora-rs.ai/docs/examples/vlm
