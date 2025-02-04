# Dora QwenVL2.5 node

Experimental node for using a VLM within dora.

## YAML Specification

This node is supposed to be used as follows:

```yaml
- id: dora-qwenvl
  build: pip install dora-qwen2-5-vl
  path: dora-qwen2-5-vl
  inputs:
    image:
      source: camera/image
      queue_size: 1
    text: dora-distil-whisper/text
  outputs:
    - text
  env:
    DEFAULT_QUESTION: Describe the image in a very short sentence.
```

## Additional documentation

- Qwenvl: https://github.com/QwenLM/Qwen-VL

## Examples

- Vision Language Model
  - Github: https://github.com/dora-rs/dora/blob/main/examples/vlm
  - Website: https://dora-rs.ai/docs/examples/vlm
