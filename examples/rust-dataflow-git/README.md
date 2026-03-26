# Git-based Rust example

To get started:

```bash
cargo run --example rust-dataflow-git
```

This example resolves the git source to the current local checkout and its
exact `HEAD` commit before building, so the cloned nodes match the daemon and
runtime version used to run the example.
