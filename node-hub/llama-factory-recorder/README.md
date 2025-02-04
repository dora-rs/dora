# Dora Llama factory recorder

Experimental node for recording for training llama based model.

## Get started

- Git clone llama-factory:

```bash
git clone https://github.com/hiyouga/LLaMA-Factory --depth 1 $HOME
```

- Replace your AI model node with llama-factory-recorder node.

Example:

```yaml
- id: dora-qwenvl-recorder
  build: pip install -e ../../node-hub/llama-factory-recorder
  path: llama-factory-recorder
  inputs:
    image_right: reachy/image_right
    ground_truth: key-interpolation/text
  outputs:
    - text
  env:
    DEFAULT_QUESTION: Respond to people. # Prompt to use
    LLAMA_FACTORY_ROOT_PATH: $HOME/LLaMA-Factory # path to llama factory
```

- Run your dataflow and at the end of the run you will find your data within the llama factory folder.
- Modify `llama-factory/examples/train_lora/qwen2vl_lora_sft.yaml` so that the dataset is the one you want to use,

```yaml,diff
- dataset: mllm_demo,identity  # video: mllm_video_demo
+ dataset: dora_demo_1,identity`
```

- You can also choose the 2B model instead of the 7B model with

```yaml,diff
- model_name_or_path: Qwen/Qwen2-VL-7B-Instruct
+ model_name_or_path: Qwen/Qwen2.5-VL-3B-Instruct
```

- Then

```bash
llamafactory-cli train examples/train_lora/qwen2vl_lora_sft.yaml
```
