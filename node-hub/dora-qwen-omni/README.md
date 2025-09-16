# Rust mtmd-cli implementation

Partial port of the mtmd-cli.cpp example in the llama-cpp repository.

## Usage

### Command Line Interface

To run the mtmd example, you first need to download the model gguf file and the multimodal projection file, e.g. for Gemma3 you may use:

```sh
wget https://huggingface.co/unsloth/gemma-3-4b-it-GGUF/resolve/main/gemma-3-4b-it-Q4_K_M.gguf \
https://huggingface.co/unsloth/gemma-3-4b-it-GGUF/resolve/main/mmproj-F16.gguf
```

To then run the example on CPU, provide an image file `my_image.jpg` and run:

```sh
cargo run --release --example mtmd -- \
  --model ./gemma-3-4b-it-Q4_K_M.gguf \
  --mmproj ./mmproj-F16.gguf \
  --image my_image.jpg \
  --prompt "What is in the picture?" \
  --no-gpu \
  --no-mmproj-offload \
  --marker "<start_of_image>"
```
