# Collect data from microphone

This node will send data as soon as the microphone volume is higher than a threshold.

This is using python Sounddevice.

It detects beginning and ending of voice activity within a stream of audio and returns the parts that contains activity.

There's a maximum amount of voice duration, to avoid having no input for too long.

## Input/Output Specification

- inputs:
  - tick: This is used to detect when the dataflow is finished.
- outputs:
  - audio: 16kHz sampled audio sent by chunk

## YAML Specification

```yaml
- id: dora-vad
  description: Voice activity detection. See; <a href='https://github.com/snakers4/silero-vad'>sidero</a>
  build: pip install dora-vad
  path: dora-vad
  inputs:
    audio: dora-microphone/audio
  outputs:
    - audio
```

## Reference documentation

- dora-microphone
  - github: https://github.com/dora-rs/dora/blob/main/node-hub/dora-microphone
  - website: http://dora-rs.ai/docs/nodes/microphone
- sounddevice
  - website: https://python-sounddevice.readthedocs.io/en/0.5.1/
  - github: https://github.com/spatialaudio/python-sounddevice/tree/master

## Examples

- Speech to Text
  - github: https://github.com/dora-rs/dora/blob/main/examples/speech-to-text
  - website: https://dora-rs.ai/docs/examples/stt

## License

The code and model weights are released under the MIT License.
