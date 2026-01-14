# dora-lmdeploy

A dora node that wraps [LMDeploy](https://github.com/InternLM/lmdeploy) for high-performance LLM and VLM inference.

This node is designed to handle both text and image inputs, making it suitable for Vision-Language Models (like Qwen2-VL, InternVL) as well as standard LLMs.

## Inputs

- `image`: (Optional) An encoded image (JPEG/PNG bytes) or compatible Arrow binary. If provided, it is stored and used for subsequent text prompts.
- `text`: (Required) A text prompt. Receiving this input triggers the inference using the latest stored image (if any).

## Outputs

- `response`: The generated text response from the model.
- `error`: Error message if inference fails.

## Configuration

You can configure the model using environment variables in your dataflow YAML.

- `MODEL_PATH`: The Hugging Face model ID or local path. Default: `Qwen/Qwen2.5-VL-7B-Instruct`.
- `BACKEND`: The inference backend (`turbomind` or `pytorch`). Default: `turbomind`.

## Example Dataflow

```yaml
nodes:
  - id: camera
    path: opencv-video-capture
    inputs:
      tick: dora/timer/millis/100
    outputs:
      - image

  - id: input-text
    path: dora-keyboard
    inputs:
      tick: dora/timer/millis/100
    outputs:
      - text

  - id: model
    build: pip install . 
    path: dora-lmdeploy
    inputs:
      image: camera/image
      text: input-text/text
    outputs:
      - response
    env:
      MODEL_PATH: Qwen/Qwen2.5-VL-7B-Instruct
  
  - id: plot
    path: dora-rerun
    inputs:
      image: camera/image
      text: model/response
```

## Testing

**Unit Tests (Mocked)**
Runs quickly without GPU.
```bash
python3 node-hub/dora-lmdeploy/tests/test_dora_lmdeploy.py
```

**Integration Tests (GPU Required)**
Requires `lmdeploy` installed and a CUDA GPU. Use a tiny model for faster testing.
```bash
export e2e_gpu_test=true
pytest node-hub/dora-lmdeploy/tests/integration_test_gpu.py
```
