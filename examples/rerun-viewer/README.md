# Python Dataflow Example

This examples shows how to create and connect dora to rerun.

This nodes is still experimental and format for passing Images, Bounding boxes, and text are probably going to change in the future.

## Getting Started

```bash
cargo install --force rerun-cli@0.15.1

## To install this package
git clone git@github.com:dora-rs/dora.git
cargo install --git https://github.com/dora-rs/dora dora-rerun

dora start dataflow.yml --attach
```

You will see two visualizations. One from matplotlib and one from rerun for comparaison.

## CI/CD

This example is not tested on the CI/CD as visualization is not really testable.

Please reach out in case of issues at: https://github.com/dora-rs/dora/issues
