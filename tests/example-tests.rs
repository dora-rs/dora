use assert2::assert;
use std::{env::consts::EXE_EXTENSION, path::Path};

#[test]
fn test_rust_dataflow_example_node() {
    test(
        "cargo build -p rust-dataflow-example-node",
        Path::new("target/debug/rust-dataflow-example-node"),
        Path::new("tests/sample-inputs/inputs-rust-node.json"),
        Path::new("tests/sample-inputs/expected-outputs-rust-node.jsonl"),
    );
}

#[test]
fn test_rust_dataflow_example_status_node() {
    test(
        "cargo build -p rust-dataflow-example-status-node",
        Path::new("target/debug/rust-dataflow-example-status-node"),
        Path::new("tests/sample-inputs/inputs-rust-status-node.json"),
        Path::new("tests/sample-inputs/expected-outputs-rust-status-node.jsonl"),
    );
}

fn test(build_command: &str, exe: &Path, inputs: &Path, expected_output: &Path) {
    let mut build_args = build_command.split(' ');
    let mut cmd = std::process::Command::new(build_args.next().unwrap());
    cmd.args(build_args);
    assert!(cmd.status().unwrap().success());

    let outputs_file = tempfile::NamedTempFile::new().unwrap();

    let exit_status = std::process::Command::new(exe.with_extension(EXE_EXTENSION))
        .env("DORA_TEST_WITH_INPUTS", inputs)
        .env("DORA_TEST_NO_OUTPUT_TIME_OFFSET", "1")
        .env("DORA_TEST_WRITE_OUTPUTS_TO", outputs_file.path())
        .status()
        .unwrap();
    assert!(exit_status.success());

    let output = std::fs::read_to_string(outputs_file.path()).unwrap();
    let expected_output = std::fs::read_to_string(expected_output).unwrap();
    assert!(output == expected_output);
}
