use assert2::assert;
use std::path::PathBuf;

#[test]
fn test_dora_test_source() {
    let outputs_file = tempfile::NamedTempFile::new().unwrap();
    let inputs_file = tempfile::NamedTempFile::new().unwrap();
    std::fs::write(&inputs_file, r#"{"id": "dora-test-source", "events": []}"#).unwrap();

    // We expect the node to parse TEST_DATA and emit the exact data "hello" as a byte array [104, 101, 108, 108, 111]
    let expected_output = r#"{"id":"test_out","data":[104,101,108,108,111],"data_type":"UInt8"}"#;

    let exit_status = std::process::Command::new("cargo")
        .args(["run", "-p", "dora-test-source"])
        .env("DORA_TEST_WITH_INPUTS", inputs_file.path())
        .env("DORA_TEST_NO_OUTPUT_TIME_OFFSET", "1")
        .env("DORA_TEST_WRITE_OUTPUTS_TO", outputs_file.path())
        .env(
            "TEST_DATA",
            r#"[{"id": "test_out", "data": "hello", "delay_ms": 0}]"#,
        )
        .status()
        .unwrap();

    assert!(exit_status.success());

    let output = std::fs::read_to_string(outputs_file.path()).unwrap();

    // The outputs file contains json lines. We verify our expected output is in the file.
    assert!(output.trim().contains(expected_output));
}
