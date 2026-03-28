# CI Workflow Templates

Ready-to-use GitHub Actions workflows for adora application projects.

## Usage

Copy the workflow file(s) you need into your project's `.github/workflows/` directory:

```bash
mkdir -p .github/workflows
cp ci-templates/rust-node-ci.yml .github/workflows/
```

## Available Templates

| Template | What it does |
|----------|-------------|
| `rust-node-ci.yml` | `cargo fmt` + `cargo clippy` + `cargo test` for Rust node crates |
| `python-node-ci.yml` | `pytest` for Python nodes (works with `MockNode` from `adora.testing`) |
| `dataflow-smoke.yml` | Installs adora CLI, builds nodes, runs dataflow with `--stop-after` |

## Customization

- **Rust version**: Change `dtolnay/rust-toolchain@stable` to `@master` with a `toolchain` input for a specific version
- **Python version**: Change `python-version` in the setup-python step
- **Dataflow path**: Set `DATAFLOW_PATH` env var in `dataflow-smoke.yml`
- **Stop timeout**: Set `STOP_AFTER` env var (e.g., `"30s"`, `"1m"`)
