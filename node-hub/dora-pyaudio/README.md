# dora-pyaudio

## Getting started

- (MacOS) Install `portaudio`

```bash
brew install portaudio
```

- (Linux) Install `portaudio`

```bash
sudo apt-get install portaudio19-dev python-all-dev
```

- Install it with pip:

```bash
pip install -e .
```

## Contribution Guide

- Format with [black](https://github.com/psf/black):

```bash
black . # Format
```

- Lint with [pylint](https://github.com/pylint-dev/pylint):

```bash
pylint --disable=C,R --ignored-modules=cv2 . # Lint
```

- Test with [pytest](https://github.com/pytest-dev/pytest)

```bash
pytest . # Test
```

## YAML Specification

## Examples

## License

dora-pyaudio's code are released under the MIT License
