use assert2::assert;
use std::path::Path;

#[test]
fn test_rust_dataflow_example_node() {
    test_crate(
        "rust-dataflow-example-node",
        Path::new("tests/sample-inputs/inputs-rust-node.json"),
        Path::new("tests/sample-inputs/expected-outputs-rust-node.jsonl"),
    )
}

#[test]
fn test_rust_dataflow_example_status_node() {
    test_crate(
        "rust-dataflow-example-status-node",
        Path::new("tests/sample-inputs/inputs-rust-status-node.json"),
        Path::new("tests/sample-inputs/expected-outputs-rust-status-node.jsonl"),
    );
}

fn test_crate(crate_name: &str, inputs: &Path, expected_output: &Path) {
    let outputs_file = tempfile::NamedTempFile::new().unwrap();

    // run the crate in integration test mode
    let exit_status = std::process::Command::new("cargo")
        .args(["run", "-p", crate_name])
        .env("DORA_TEST_WITH_INPUTS", inputs)
        .env("DORA_TEST_NO_OUTPUT_TIME_OFFSET", "1")
        .env("DORA_TEST_WRITE_OUTPUTS_TO", outputs_file.path())
        .status()
        .unwrap();
    assert!(exit_status.success());

    // compare outputs
    let output = std::fs::read_to_string(outputs_file.path()).unwrap();
    let expected_output = std::fs::read_to_string(expected_output)
        .unwrap()
        .replace("\r\n", "\n"); // normalize line endings for Windows
    assert!(output == expected_output);
}
