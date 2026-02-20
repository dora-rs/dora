# Input and Expected Outputs for Tests

This directory stores deterministic fixtures for node integration tests.

## Record new input fixtures

Record inputs from a real dataflow run:

```bash
RECORD_DIR="$(mktemp -d /tmp/dora-it-recordings.XXXXXX)"
DORA_WRITE_EVENTS_TO="$RECORD_DIR" dora run examples/rust-dataflow/dataflow.yml
ls "$RECORD_DIR"
```

This produces files such as `inputs-node.json` and `inputs-status-node.json`.

## Regenerate expected outputs

Run each node executable directly in integration test mode:

```bash
# rust-dataflow-example-node
DORA_TEST_WITH_INPUTS="$RECORD_DIR/inputs-node.json" \
DORA_TEST_WRITE_OUTPUTS_TO="$RECORD_DIR/outputs-node.jsonl" \
DORA_TEST_NO_OUTPUT_TIME_OFFSET=1 \
cargo run -p rust-dataflow-example-node

# rust-dataflow-example-status-node
DORA_TEST_WITH_INPUTS="$RECORD_DIR/inputs-status-node.json" \
DORA_TEST_WRITE_OUTPUTS_TO="$RECORD_DIR/outputs-status-node.jsonl" \
DORA_TEST_NO_OUTPUT_TIME_OFFSET=1 \
cargo run -p rust-dataflow-example-status-node
```

By default, outputs are written next to the input file as `outputs.jsonl`.
In this workflow we set `DORA_TEST_WRITE_OUTPUTS_TO` explicitly to avoid overwriting files.

## Compare with checked-in fixtures

```bash
diff -u tests/sample-inputs/expected-outputs-rust-node.jsonl "$RECORD_DIR/outputs-node.jsonl"
diff -u tests/sample-inputs/expected-outputs-rust-status-node.jsonl "$RECORD_DIR/outputs-status-node.jsonl"
```

For more details, see:
[apis/rust/node/src/integration_testing.rs](../../apis/rust/node/src/integration_testing.rs)
