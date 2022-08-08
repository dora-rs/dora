# Installation

This project is in early development, and many features have yet to be implemented with breaking changes. Please don't take for granted the current design. The installation process will be streamlined in the future.

### 1. Compile the dora-coordinator

The `dora-coordinator` is responsible for reading the dataflow descriptor file and launching the operators accordingly. 

Build it using:
```bash
cargo build -p dora-coordinator --examples --release
```

### 2. Compile the dora-runtime for operators

The `dora-runtime` is responsible for managing a set of operators. 
```bash
cargo build -p dora-runtime --release
```
