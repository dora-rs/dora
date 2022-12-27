# Installation

This project is in early development, and many features have yet to be implemented with breaking changes. Please don't take for granted the current design. The installation process will be streamlined in the future.

### Download the binaries from Github


Install `dora` binaries from GitHub releases:

```bash
wget https://github.com/dora-rs/dora/releases/download/<version>/dora-<version>-x86_64-Linux.zip
unzip dora-<version>-x86_64-Linux.zip
python3 -m pip install dora-rs==<version> ## For Python API
PATH=$PATH:$(pwd):$(pwd)/iceoryx
dora --help
```

#### Or compile from Source

Build it using:
```bash
git clone https://github.com/dora-rs/dora.git
cd dora
cargo build -p dora-coordinator -p dora-runtime --release
PATH=$PATH:$(pwd)/target/release
```

If you want to use `Iceoryx`. Add `iox-roudi` to the path.
You can find `iox-roudi` with:
```bash
find target -type f -wholename "*/iceoryx-install/bin/iox-roudi"
```