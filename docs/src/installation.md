# Installation

This project is in early development, and many features have yet to be implemented with breaking changes. Please don't take for granted the current design. The installation process will be streamlined in the future.

### 1. Compile the dora-coordinator

The `dora-coordinator` is responsible for reading the dataflow descriptor file and launching the operators accordingly. 

Build it using:
```bash
git clone https://github.com/dora-rs/dora.git
cd dora
cargo build -p dora-coordinator --release
```

### 2. Compile the dora-runtime for operators

The `dora-runtime` is responsible for managing a set of operators. 
```bash
cargo build -p dora-runtime --release
```

### 3. Add those binaries to your path

This step is optional. You can also refer to the executables using their full path or copy them somewhere else.

```bash
export PATH=$PATH:$(pwd)/target/release
```